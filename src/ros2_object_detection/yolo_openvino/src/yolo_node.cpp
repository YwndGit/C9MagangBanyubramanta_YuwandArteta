#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>

struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
};

class YOLONode : public rclcpp::Node {
public:
    YOLONode() : Node("yolo_node") {
        // Get package share directory
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("yolo_openvino");
        std::string model_path = package_share_dir + "/models/yuwandbest.xml";
        
        RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Loading OpenVINO model...");
        
        // Load model
        ov::Core core;
        auto model = core.read_model(model_path);
        compiled_model_ = core.compile_model(model, "CPU");
        infer_request_ = compiled_model_.create_infer_request();
        
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully!");
        
        // Subscribe & publish
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/raw_image", 10,
            std::bind(&YOLONode::imageCallback, this, std::placeholders::_1));
        
        publisher_text_ = this->create_publisher<std_msgs::msg::String>("/object", 10);
        publisher_image_ = this->create_publisher<sensor_msgs::msg::Image>("/detected_image", 10);
        
        // Class names - 2 classes: flares, baskom
        class_names_ = {"baskom", "flares"};
        
        RCLCPP_INFO(this->get_logger(), "YOLO Node ready with custom model (flares, baskom)!");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image → OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }
        
        cv::Mat frame = cv_ptr->image.clone();
        int img_h = frame.rows;
        int img_w = frame.cols;
        
        // Preprocess
        cv::Mat resized, blob;
        cv::resize(frame, resized, cv::Size(416, 416));
        resized.convertTo(blob, CV_32F, 1.0 / 255.0);
        
        // Convert HWC → CHW
        std::vector<cv::Mat> channels(3);
        cv::split(blob, channels);
        
        // Set input tensor
        auto input_tensor = infer_request_.get_input_tensor();
        input_tensor.set_shape({1, 3, 416, 416});
        float* input_data = input_tensor.data<float>();
        
        for (int c = 0; c < 3; c++) {
            std::memcpy(input_data + c * 416 * 416, 
                       channels[c].data, 416 * 416 * sizeof(float));
        }
        
        // Inference
        infer_request_.infer();
        
        // Get output
        auto output_tensor = infer_request_.get_output_tensor();
        const float* output_data = output_tensor.data<float>();
        auto output_shape = output_tensor.get_shape();
        
        // Parse detections (2 classes: flares=0, baskom=1)
        std::vector<Detection> detections = parseDetections(output_data, output_shape, 
                                                            img_w, img_h, 0.5, 0.4);
        
        // Draw bounding boxes
        for (const auto& det : detections) {
            cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
            
            std::string label = class_names_[det.class_id] + " " + 
                               std::to_string(det.confidence).substr(0, 4);
            
            int baseline;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                                   0.6, 2, &baseline);
            
            cv::rectangle(frame, 
                         cv::Point(det.box.x, det.box.y - label_size.height - 5),
                         cv::Point(det.box.x + label_size.width, det.box.y),
                         cv::Scalar(0, 255, 0), -1);
            
            cv::putText(frame, label, 
                       cv::Point(det.box.x, det.box.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
            
            // Publish text detection
            std_msgs::msg::String detection_msg;
            detection_msg.data = label + " at (" + std::to_string(det.box.x) + "," + 
                                std::to_string(det.box.y) + ")";
            publisher_text_->publish(detection_msg);
        }
        
        // Publish detected image
        auto detected_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        publisher_image_->publish(*detected_msg);
        
        RCLCPP_INFO(this->get_logger(), "Detected %zu objects", detections.size());
    }
    
    std::vector<Detection> parseDetections(const float* output, 
                                          const ov::Shape& shape,
                                          int img_w, int img_h,
                                          float conf_threshold,
                                          float nms_threshold) {
        std::vector<Detection> detections;
        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;
        std::vector<int> class_ids;
        
        int num_detections = shape[1];  // 10647
        int num_classes = shape[2] - 5;  // 2 classes → 2 + 5 = 7
        
        for (int i = 0; i < num_detections; i++) {
            const float* detection = output + i * (num_classes + 5);
            float objectness = detection[4];
            
            if (objectness > conf_threshold) {
                // Find max class confidence
                float max_conf = 0;
                int class_id = 0;
                for (int c = 0; c < num_classes; c++) {
                    if (detection[5 + c] > max_conf) {
                        max_conf = detection[5 + c];
                        class_id = c;
                    }
                }
                
                float confidence = objectness * max_conf;
                if (confidence > conf_threshold) {
                    // Parse bbox (center_x, center_y, w, h)
                    float cx = detection[0] * img_w / 416.0;
                    float cy = detection[1] * img_h / 416.0;
                    float w = detection[2] * img_w / 416.0;
                    float h = detection[3] * img_h / 416.0;
                    
                    int x = static_cast<int>(cx - w / 2);
                    int y = static_cast<int>(cy - h / 2);
                    
                    boxes.push_back(cv::Rect(x, y, static_cast<int>(w), static_cast<int>(h)));
                    confidences.push_back(confidence);
                    class_ids.push_back(class_id);
                }
            }
        }
        
        // NMS (Non-Maximum Suppression)
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
        
        for (int idx : indices) {
            Detection det;
            det.box = boxes[idx];
            det.confidence = confidences[idx];
            det.class_id = class_ids[idx];
            detections.push_back(det);
        }
        
        return detections;
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_text_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    std::vector<std::string> class_names_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YOLONode>());
    rclcpp::shutdown();
    return 0;
}
