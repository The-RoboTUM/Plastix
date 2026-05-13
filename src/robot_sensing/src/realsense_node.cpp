#include <memory>
#include <vector>
#include <librealsense2/rs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class D405PointCloudNode : public rclcpp::Node {
public:
    D405PointCloudNode() : Node("realsense_node") {
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/depth/color/points", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_rect_raw", 10);
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);

        // Configure D405: 480p is a good balance for processing speed
        cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        
        pipe_.start(cfg_);
        RCLCPP_INFO(this->get_logger(), "D405 PointCloud Node Started");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&D405PointCloudNode::publish_pc, this));
    }

private:
    void publish_pc() {
        rs2::frameset frames = pipe_.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        if (!depth || !color) return;

        // Publish Depth and Color images
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP)).toImageMsg();
        auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP)).toImageMsg();
        depth_msg->header.stamp = this->now();
        depth_msg->header.frame_id = "camera_depth_frame";
        color_msg->header.stamp = depth_msg->header.stamp;
        color_msg->header.frame_id = "camera_color_frame";
        
        // Calculate Points
        pc_.map_to(color);
        auto points = pc_.calculate(depth);

        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pc_msg->header.frame_id = "camera_link";
        pc_msg->header.stamp = this->now();

        // Setup PointCloud2 fields
        sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*pc_msg, "rgb");

        const rs2::vertex* vertices = points.get_vertices();
        const rs2::texture_coordinate* tex_coords = points.get_texture_coordinates();
        const uint8_t* color_ptr = reinterpret_cast<const uint8_t*>(color.get_data());

        for (size_t i = 0; i < points.size(); i++) {
            if (vertices[i].z > 0 && vertices[i].z < 1.0) { // D405 is short range (max 1m)
                *iter_x = vertices[i].x;
                *iter_y = vertices[i].y;
                *iter_z = vertices[i].z;

                int u = static_cast<int>(tex_coords[i].u * color.get_width());
                int v = static_cast<int>(tex_coords[i].v * color.get_height());
                
                if (u >= 0 && u < color.get_width() && v >= 0 && v < color.get_height()) {
                    int idx = (v * color.get_width() + u) * 3;
                    iter_rgb[0] = color_ptr[idx + 2]; // R
                    iter_rgb[1] = color_ptr[idx + 1]; // G
                    iter_rgb[2] = color_ptr[idx];     // B
                }
                ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
            }
        }
        pc_pub_->publish(*pc_msg);
        depth_pub_->publish(*depth_msg);
        color_pub_->publish(*color_msg);
    }

    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::pointcloud pc_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D405PointCloudNode>());
    rclcpp::shutdown();
    return 0;
}