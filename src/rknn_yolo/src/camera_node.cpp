/**
 * 摄像头发布节点（优化版）
 *
 * 发布话题:
 *   /image_raw             — 原始 BGR8 图像（本地消费，YOLO 等）
 *   /image_raw/compressed  — JPEG 压缩图像（图传用，每帧 ~30-50KB）
 *
 * 核心优化：
 *   1. 采集层: CAMERA_FMT=MJPG，摄像头硬件编码，带宽低
 *   2. 缓冲层: shared_ptr 交换，消除深拷贝
 *   3. 图传层: CompressedImage 发 compressed，DDS 吞吐量降低 20 倍
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

static const int   CAMERA_ID   = 0;
static const int   IMG_WIDTH   = 640;
static const int   IMG_HEIGHT  = 480;
static const int   TARGET_FPS  = 30;
static const int   JPEG_QUALITY = 75;       // 75 画质/带宽平衡，80+ 接近无损
static const char* CAMERA_FMT  = "MJPG";    // 硬件 MJPEG，别用 YUYV

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("topic_webcam_pub"), running_(true) {
        raw_pub_  = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        comp_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "image_raw/compressed", 10);

        cap_.open(CAMERA_ID);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d", CAMERA_ID);
            rclcpp::shutdown();
            return;
        }

        // 摄像头参数
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
        cap_.set(cv::CAP_PROP_FPS, TARGET_FPS);

        double w   = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double h   = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap_.get(cv::CAP_PROP_FPS);
        int fourcc = (int)cap_.get(cv::CAP_PROP_FOURCC);
        char fourcc_str[5] = {
            (char)(fourcc & 0xff), (char)((fourcc >> 8) & 0xff),
            (char)((fourcc >> 16) & 0xff), (char)((fourcc >> 24) & 0xff), '\0'
        };
        RCLCPP_INFO(this->get_logger(),
            "Camera: %dx%d @ %.0f FPS, codec=%s, JPEG quality=%d",
            (int)w, (int)h, fps, fourcc_str, JPEG_QUALITY);

        // 采集线程 + 发布定时器
        capture_thread_ = std::thread(&ImagePublisher::capture_loop, this);
        int period_ms = 1000 / TARGET_FPS;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ImagePublisher::publish_frame, this));
    }

    ~ImagePublisher() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
    }

private:
    // ── 采集线程 ──
    void capture_loop() {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            cap_ >> frame;
            if (frame.empty()) {
                std::this_thread::sleep_for(1ms);
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ = std::make_shared<cv::Mat>(std::move(frame));
            }
        }
    }

    // ── 定时发布 ──
    void publish_frame() {
        std::shared_ptr<cv::Mat> ptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ptr.swap(latest_);   // 原子取出，置空 latest_
        }
        if (!ptr || ptr->empty()) return;

        auto stamp = this->get_clock()->now();

        // --- raw (本地) ---
        auto raw_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *ptr).toImageMsg();
        raw_msg->header.stamp = stamp;
        raw_pub_->publish(std::move(raw_msg));

        // --- compressed (图传) ---
        auto comp_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        comp_msg->header.stamp = stamp;
        comp_msg->format = "jpeg";
        std::vector<uchar> buf;
        cv::imencode(".jpg", *ptr, buf,
                     {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY});
        comp_msg->data = std::move(buf);
        comp_pub_->publish(std::move(comp_msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr comp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_;

    std::mutex mutex_;
    std::shared_ptr<cv::Mat> latest_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}