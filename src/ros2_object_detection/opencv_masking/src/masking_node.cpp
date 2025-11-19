#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MaskingNode : public rclcpp::Node {
public:
    MaskingNode() : Node("masking_node") {
        // Publisher untuk raw_image dan mask_image
        pub_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/raw_image", 10);
        pub_mask_ = this->create_publisher<sensor_msgs::msg::Image>("/mask_image", 10);
        
        // Buka video file
        std::string video_path = "/home/yuwand/yolov5/fourth.mp4"; // custom video path
        cap_.open(video_path);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open video: %s", video_path.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Video opened successfully!");
        
        // Timer untuk publish frame (30 FPS)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&MaskingNode::publishFrame, this));
        
        RCLCPP_INFO(this->get_logger(), "Masking Node ready! Publishing to /raw_image and /mask_image");
    }

private:
    void publishFrame() {
        cv::Mat frame;
        cap_ >> frame;
        
        if (frame.empty()) {
            // Loop video
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }
        
        // Publish RAW IMAGE (gambar asli)
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera";
        
        auto raw_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        pub_raw_->publish(*raw_msg);
        
        // ===== MASKING: Filter warna dengan NOISE REDUCTION =====
        cv::Mat hsv, mask, blur;
        
        // 1. Blur dulu untuk smooth noise
        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0);
        
        // 2. Convert ke HSV
        cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
        
        // 3. Range HSV LEBIH KETAT (kurangi noise)
        // Tuning: Naikkan lower S & V untuk filter background
        cv::Scalar lower(85, 70, 70);    // H, S, V minimum (lebih ketat)
        cv::Scalar upper(125, 255, 255); // H, S, V maximum
        
        cv::inRange(hsv, lower, upper, mask);
        
        // 4. Morphological operations LEBIH AGRESIF
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        
        // Close: Tutup hole kecil
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);
        
        // Open: Hapus noise kecil
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);
        
        // 5. Median blur untuk hapus salt-pepper noise
        cv::medianBlur(mask, mask, 5);
        
        // Publish MASK IMAGE
        auto mask_msg = cv_bridge::CvImage(header, "mono8", mask).toImageMsg();
        pub_mask_->publish(*mask_msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_, pub_mask_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaskingNode>());
    rclcpp::shutdown();
    return 0;
}
