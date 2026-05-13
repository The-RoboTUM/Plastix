#include <memory>
#include <librealsense2/rs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.hpp>

class D405SlamProvider : public rclcpp::Node {
public:
    D405SlamProvider() : Node("d405_slam_provider"), align_to_color_(RS2_STREAM_COLOR) {
        
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 10);
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);
        info_pub_  = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/color/camera_info", 10);

        cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        
        auto profile = pipe_.start(cfg_);
        
        auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        intrinsics_ = stream.get_intrinsics();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), 
            std::bind(&D405SlamProvider::process_frame, this));
        
        RCLCPP_INFO(this->get_logger(), "D405 SLAM Provider Node Started");
    }

private:
    void process_frame() {
        // Use poll_for_frames instead of wait_for_frames inside a timer to avoid blocking
        rs2::frameset frames;
        if (pipe_.poll_for_frames(&frames)) {
            
            frames = align_to_color_.process(frames);

            auto depth_frame = frames.get_depth_frame();
            auto color_frame = frames.get_color_frame();

            if (!depth_frame || !color_frame) return;

            auto stamp = this->now();

            // Publish Color
            cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_mat).toImageMsg();
            color_msg->header.stamp = stamp;
            color_msg->header.frame_id = "camera_link_optical";

            // Publish Depth
            cv::Mat depth_mat(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_mat).toImageMsg();
            depth_msg->header.stamp = stamp;
            depth_msg->header.frame_id = "camera_link_optical";

            // Publish Info
            auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
            info_msg->header.stamp = stamp;
            info_msg->header.frame_id = "camera_link_optical";
            info_msg->width = intrinsics_.width;
            info_msg->height = intrinsics_.height;
            info_msg->k[0] = intrinsics_.fx; info_msg->k[2] = intrinsics_.ppx;
            info_msg->k[4] = intrinsics_.fy; info_msg->k[5] = intrinsics_.ppy;
            info_msg->k[8] = 1.0;
            info_msg->p[0] = intrinsics_.fx; info_msg->p[2] = intrinsics_.ppx;
            info_msg->p[5] = intrinsics_.fy; info_msg->p[6] = intrinsics_.ppy;
            info_msg->p[10] = 1.0;

            color_pub_->publish(*color_msg);
            depth_pub_->publish(*depth_msg);
            info_pub_->publish(std::move(info_msg));
        }
    }

    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::align align_to_color_;
    rs2_intrinsics intrinsics_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D405SlamProvider>());
    rclcpp::shutdown();
    return 0;
}