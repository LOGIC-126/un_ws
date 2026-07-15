#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rknn_api.h>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <memory>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;

// ========== 配置参数 ==========
static const std::string RKNN_MODEL = "/home/orangepi/rknn_linker/v8.rknn";
static const int IMG_SIZE = 640;
static const float OBJ_THRESH = 0.25f;
static const float NMS_THRESH = 0.45f;
static const std::vector<std::string> CLASSES = {"elephant", "tiger", "wolf", "monkey", "peacock"};
static const int NC = CLASSES.size();
static const int REG_MAX = 16;
static const std::vector<int> STRIDES = {8, 16, 32};

// ========== 后处理工具函数 ==========

/**
 * DFL 解码：将 box_raw (形状 [1, 64, h, w], 即 4*REG_MAX 通道) 解码为边界框偏移量
 * 输出 decoded 形状为 [4 * h * w]，按顺序存储 lt_x, lt_y, rb_x, rb_y (每个平面 h*w 个值)
 */
static void dfl_decode(const float* box_raw, int h, int w, float* decoded) {
    int spatial = h * w;
    // 对每个坐标分量分别进行 softmax + 加权求和
    for (int coord = 0; coord < 4; coord++) {
        for (int j = 0; j < spatial; j++) {
            // 找到最大值用于数值稳定性
            float max_val = -1e9;
            for (int k = 0; k < REG_MAX; k++) {
                float v = box_raw[(coord * REG_MAX + k) * spatial + j];
                if (v > max_val) max_val = v;
            }
            float sum = 0.0f;
            for (int k = 0; k < REG_MAX; k++) {
                float v = box_raw[(coord * REG_MAX + k) * spatial + j];
                sum += std::exp(v - max_val);
            }
            float result = 0.0f;
            for (int k = 0; k < REG_MAX; k++) {
                float v = box_raw[(coord * REG_MAX + k) * spatial + j];
                result += (std::exp(v - max_val) / sum) * static_cast<float>(k);
            }
            decoded[coord * spatial + j] = result;
        }
    }
}

/**
 * 完整后处理：返回检测框 [x1,y1,x2,y2] 在原图尺寸下的坐标，置信度，类别ID
 */
static std::tuple<std::vector<std::vector<int>>, std::vector<float>, std::vector<int>>
post_process(const std::vector<rknn_output>& outputs,
             const std::vector<rknn_tensor_attr>& output_attrs,
             int img_w, int img_h)
{
    std::vector<std::vector<float>> all_boxes;  // [cx,cy,w,h]
    std::vector<float> all_scores;
    std::vector<int> all_cls;

    for (int i = 0; i < 3; i++) {
        int stride = STRIDES[i];

        // 获取三个输出张量：box, cls, sum
        const rknn_output& box_out   = outputs[i*3];
        const rknn_output& cls_out   = outputs[i*3+1];
        const rknn_output& sum_out   = outputs[i*3+2];

        // 获取当前输出的维度（NCHW 格式）
        const rknn_tensor_attr& box_attr = output_attrs[i*3];
        const rknn_tensor_attr& cls_attr = output_attrs[i*3+1];
        const rknn_tensor_attr& sum_attr = output_attrs[i*3+2];

        // 期望 box: [1, 64, h, w], cls: [1, NC, h, w], sum: [1, 1, h, w]
        int h = box_attr.dims[2];
        int w = box_attr.dims[3];
        int spatial = h * w;

        // DFL 解码得到 lt_x, lt_y, rb_x, rb_y 四个平面，每个平面尺寸 h*w
        std::vector<float> decoded(4 * spatial);
        dfl_decode((const float*)box_out.buf, h, w, decoded.data());

        const float* lt_x = decoded.data();
        const float* lt_y = decoded.data() + spatial;
        const float* rb_x = decoded.data() + 2 * spatial;
        const float* rb_y = decoded.data() + 3 * spatial;

        const float* cls_data = (const float*)cls_out.buf;
        const float* sum_data = (const float*)sum_out.buf;

        // 生成网格
        std::vector<float> gx(spatial), gy(spatial);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int idx = y * w + x;
                gx[idx] = x + 0.5f;
                gy[idx] = y + 0.5f;
            }
        }

        // 遍历每个位置，计算边界框并过滤
        for (int idx = 0; idx < spatial; idx++) {
            float obj_score = sum_data[idx];
            if (obj_score <= OBJ_THRESH) continue;

            // 转换为原图坐标系 (cx,cy,w,h)
            float cx = ((gx[idx] - lt_x[idx]) + (gx[idx] + rb_x[idx])) / 2.0f * stride;
            float cy = ((gy[idx] - lt_y[idx]) + (gy[idx] + rb_y[idx])) / 2.0f * stride;
            float bw = ((gx[idx] + rb_x[idx]) - (gx[idx] - lt_x[idx])) * stride;
            float bh = ((gy[idx] + rb_y[idx]) - (gy[idx] - lt_y[idx])) * stride;

            // 找出最大类别得分
            float max_cls_score = -1.0f;
            int best_cls = -1;
            for (int c = 0; c < NC; c++) {
                float score = cls_data[c * spatial + idx];
                if (score > max_cls_score) {
                    max_cls_score = score;
                    best_cls = c;
                }
            }
            if (max_cls_score <= OBJ_THRESH) continue;

            all_boxes.push_back({cx, cy, bw, bh});
            all_scores.push_back(max_cls_score);
            all_cls.push_back(best_cls);
        }
    }

    // NMS（类别无关）
    std::vector<std::vector<int>> final_boxes;
    std::vector<float> final_scores;
    std::vector<int> final_cls_ids;

    if (all_boxes.empty()) {
        return {final_boxes, final_scores, final_cls_ids};
    }

    size_t num = all_boxes.size();
    std::vector<float> x1(num), y1(num), x2(num), y2(num), areas(num);
    for (size_t i = 0; i < num; i++) {
        float cx = all_boxes[i][0], cy = all_boxes[i][1];
        float bw = all_boxes[i][2], bh = all_boxes[i][3];
        x1[i] = cx - bw / 2.0f;
        y1[i] = cy - bh / 2.0f;
        x2[i] = cx + bw / 2.0f;
        y2[i] = cy + bh / 2.0f;
        areas[i] = (x2[i] - x1[i]) * (y2[i] - y1[i]);
    }

    // 按置信度降序排序索引
    std::vector<int> order(num);
    for (int i = 0; i < (int)num; i++) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return all_scores[a] > all_scores[b];
    });

    std::vector<bool> suppressed(num, false);
    for (size_t i = 0; i < num; i++) {
        int idx_i = order[i];
        if (suppressed[idx_i]) continue;
        final_boxes.push_back({static_cast<int>(x1[idx_i]), static_cast<int>(y1[idx_i]),
                               static_cast<int>(x2[idx_i]), static_cast<int>(y2[idx_i])});
        final_scores.push_back(all_scores[idx_i]);
        final_cls_ids.push_back(all_cls[idx_i]);
        for (size_t j = i + 1; j < num; j++) {
            int idx_j = order[j];
            if (suppressed[idx_j]) continue;
            float xx1 = std::max(x1[idx_i], x1[idx_j]);
            float yy1 = std::max(y1[idx_i], y1[idx_j]);
            float xx2 = std::min(x2[idx_i], x2[idx_j]);
            float yy2 = std::min(y2[idx_i], y2[idx_j]);
            float w_intersect = std::max(0.0f, xx2 - xx1);
            float h_intersect = std::max(0.0f, yy2 - yy1);
            float iou = (w_intersect * h_intersect) / (areas[idx_i] + areas[idx_j] - w_intersect * h_intersect + 1e-6f);
            if (iou > NMS_THRESH) {
                suppressed[idx_j] = true;
            }
        }
    }

    // 映射回原始图像尺寸
    float sx = (float)img_w / IMG_SIZE;
    float sy = (float)img_h / IMG_SIZE;
    for (auto& box : final_boxes) {
        box[0] = static_cast<int>(box[0] * sx);
        box[1] = static_cast<int>(box[1] * sy);
        box[2] = static_cast<int>(box[2] * sx);
        box[3] = static_cast<int>(box[3] * sy);
    }

    return {final_boxes, final_scores, final_cls_ids};
}

// ========== 节点类 ==========
class YoloDetector : public rclcpp::Node {
public:
    YoloDetector() : Node("yolo_detector_node") {
        // 创建图像发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("detection_image", 10);

        // 初始化 RKNN
        int ret = rknn_init(&ctx_, const_cast<char*>(RKNN_MODEL.c_str()), 0, 0, nullptr);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_init failed! ret=%d", ret);
            rclcpp::shutdown();
            return;
        }

        // 查询输入输出数量
        rknn_input_output_num io_num;
        ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_query in/out num failed! ret=%d", ret);
            rclcpp::shutdown();
            return;
        }
        n_inputs_ = io_num.n_input;
        n_outputs_ = io_num.n_output;
        if (n_outputs_ != 9) {
            RCLCPP_WARN(this->get_logger(), "Expected 9 output tensors, got %d. Adjust post-process.", n_outputs_);
        }

        // 获取输入属性
        input_attrs_.resize(n_inputs_);
        for (int i = 0; i < n_inputs_; i++) {
            input_attrs_[i].index = i;
            ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attrs_[i], sizeof(rknn_tensor_attr));
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "rknn_query input attr failed!");
                rclcpp::shutdown();
                return;
            }
        }

        // 获取输出属性
        output_attrs_.resize(n_outputs_);
        for (int i = 0; i < n_outputs_; i++) {
            output_attrs_[i].index = i;
            ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(rknn_tensor_attr));
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "rknn_query output attr failed!");
                rclcpp::shutdown();
                return;
            }
        }

        // 打开摄像头
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera /dev/video0");
            rclcpp::shutdown();
            return;
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        // 定时器驱动（约 30 fps）
        timer_ = this->create_wall_timer(30ms, std::bind(&YoloDetector::process_frame, this));
        RCLCPP_INFO(this->get_logger(), "YOLOv8 detector started. Publishing to /detection_image");
    }

    ~YoloDetector() {
        if (ctx_) rknn_destroy(ctx_);
        cap_.release();
    }

private:
    void process_frame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        int img_h = frame.rows, img_w = frame.cols;

        // 预处理：BGR -> RGB, resize to 640x640, keep as uint8 NHWC
        cv::Mat rgb;
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
        cv::Mat resized;
        cv::resize(rgb, resized, cv::Size(IMG_SIZE, IMG_SIZE));

        // 设置输入
        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0;
        inputs[0].type = RKNN_TENSOR_UINT8;
        inputs[0].size = IMG_SIZE * IMG_SIZE * 3;
        inputs[0].fmt = RKNN_TENSOR_NHWC;
        inputs[0].buf = resized.data;

        int ret = rknn_inputs_set(ctx_, 1, inputs);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_inputs_set failed! ret=%d", ret);
            return;
        }

        // 推理
        ret = rknn_run(ctx_, nullptr);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_run failed! ret=%d", ret);
            return;
        }

        // 获取输出（转为 float）
        std::vector<rknn_output> outputs(n_outputs_);
        memset(outputs.data(), 0, sizeof(rknn_output) * n_outputs_);
        for (int i = 0; i < n_outputs_; i++) {
            outputs[i].want_float = 1;
        }
        ret = rknn_outputs_get(ctx_, n_outputs_, outputs.data(), nullptr);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_outputs_get failed! ret=%d", ret);
            return;
        }

        // 后处理
        auto [boxes, scores, cls_ids] = post_process(outputs, output_attrs_, img_w, img_h);

        // 绘制检测框到原始帧
        cv::Mat display_frame = frame.clone();
        for (size_t i = 0; i < boxes.size(); i++) {
            const auto& box = boxes[i];
            float score = scores[i];
            int cls_id = cls_ids[i];

            // 确保坐标在图像内
            int x1 = std::max(0, box[0]);
            int y1 = std::max(0, box[1]);
            int x2 = std::min(display_frame.cols - 1, box[2]);
            int y2 = std::min(display_frame.rows - 1, box[3]);

            // 类别标签
            std::string label = CLASSES[cls_id] + " " + std::to_string(score).substr(0, 4);

            // 绘制矩形
            cv::rectangle(display_frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

            // 绘制标签背景
            int baseline;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            cv::rectangle(display_frame,
                          cv::Point(x1, y1 - text_size.height - 5),
                          cv::Point(x1 + text_size.width + 5, y1),
                          cv::Scalar(0, 255, 0), cv::FILLED);
            // 绘制标签文字
            cv::putText(display_frame, label, cv::Point(x1 + 2, y1 - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }

        // 转换为 ROS 图像消息并发布
        auto img_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", display_frame).toImageMsg();
        img_msg->header.stamp = this->get_clock()->now();
        img_msg->header.frame_id = "camera";
        image_pub_->publish(*img_msg);

        // 释放输出
        rknn_outputs_release(ctx_, n_outputs_, outputs.data());
    }

    // 成员变量
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    rknn_context ctx_ = 0;
    int n_inputs_ = 0;
    int n_outputs_ = 0;
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloDetector>());
    rclcpp::shutdown();
    return 0;
}