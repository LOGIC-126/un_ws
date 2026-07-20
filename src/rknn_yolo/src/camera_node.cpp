#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("topic_webcam_pub"), running_(true) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        // 打开摄像头并设置参数
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            rclcpp::shutdown();
            return;
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(cv::CAP_PROP_FPS, 30);

        double w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap_.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(this->get_logger(), "Camera: %dx%d @ %.1f FPS (MJPEG)", (int)w, (int)h, fps);

        // 启动抓取线程
        capture_thread_ = std::thread(&ImagePublisher::capture_loop, this);

        // 定时器发布，周期 33ms（30 FPS）
        timer_ = this->create_wall_timer(
            33ms, std::bind(&ImagePublisher::publish_frame, this));
    }

    ~ImagePublisher() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
    }

private:
    void capture_loop() {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            cap_ >> frame;
            if (!frame.empty()) {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = frame.clone();  // 深拷贝确保数据独立
                frame_available_ = true;
            }
            // 简单控制抓取速率，避免 CPU 空转
            std::this_thread::sleep_for(1ms);
        }
    }

    void publish_frame() {
        cv::Mat frame_to_publish;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!frame_available_) return;  // 尚无新帧
            frame_to_publish = latest_frame_.clone();
            frame_available_ = false;       // 消费标志，避免重复发布同一帧
        }

        if (!frame_to_publish.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_to_publish).toImageMsg();
            publisher_->publish(*msg);
            // 降低日志频率，每 30 帧打印一次
            static int cnt = 0;
            if (++cnt % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "Published frame %d", cnt);
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_;
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    bool frame_available_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}