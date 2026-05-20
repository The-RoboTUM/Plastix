#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace sensor_msgs::msg;
using namespace message_filters;

struct SlamPoint {
    float x, y, z;
    uint8_t r, g, b;
};

class VisualSlamNode : public rclcpp::Node {
public:
    VisualSlamNode() : Node("visual_slam_node"), is_first_frame_(true) {
        // Initialize OpenCV ORB Detector and Matcher to avoid Segfaults
        orb_detector_ = cv::ORB::create(500);
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);

        // Subscribers
        rgb_sub_.subscribe(this, "/camera/color/image_raw");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");
        info_sub_.subscribe(this, "/camera/color/camera_info");

        // Publish PointCloud2
        pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/pointcloud2", 10);
        
        // Use ApproximateTime sync since Depth and RGB might have offset
        sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_);
        sync_->registerCallback(std::bind(&VisualSlamNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "Visual Slam Node started.");
    }

private:
    void callback(const Image::ConstSharedPtr& rgb, const Image::ConstSharedPtr& depth, const CameraInfo::ConstSharedPtr& info) {
        cv_bridge::CvImagePtr cv_rgb_ptr;
        cv_bridge::CvImagePtr cv_depth_ptr;
        try {
            cv_rgb_ptr = cv_bridge::toCvCopy(rgb, "bgr8");
            cv_depth_ptr = cv_bridge::toCvCopy(depth, "16UC1");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return; 
        }
        cv::Mat rgb_image = cv_rgb_ptr->image;
        cv::Mat depth_image = cv_depth_ptr->image;

        // Camera Intrinsics
        float fx = info->k[0];
        float cx = info->k[2];
        float fy = info->k[4];
        float cy = info->k[5];

        // Check for valid intrinsics
        if(fx == 0 || fy == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Camera intrinsics not received.");
            return;
        }

        // Fill dynamic camera matrices required for PnP processing
        camera_matrix_ = (cv::Mat_<double>(3,3) << fx,  0, cx,
                                                    0, fy, cy,
                                                    0,  0,  1);
        dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);

        // Create PointCloud2 message
        auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pc2_msg->header = rgb->header;
        pc2_msg->height = rgb_image.rows;
        pc2_msg->width = rgb_image.cols;
        pc2_msg->is_dense = false;
        pc2_msg->is_bigendian = false;

        // Define PointCloud2 fields (x, y, z, r, g, b)
        sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(rgb_image.rows * rgb_image.cols);

        // Fill PointCloud2 data
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc2_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc2_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc2_msg, "b");

        // The vector matches 1-to-1 with the image pixels grid so we can do fast index lookups
        std::vector<SlamPoint> slam_points_vector(rgb_image.rows * rgb_image.cols);

        // Loop through each pixel to compute 3D point and color
        for(int v = 0; v < rgb_image.rows; ++v) {
            for(int u = 0; u < rgb_image.cols; ++u) {
                
                uint16_t depth_value = depth_image.at<uint16_t>(v, u);
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;

                // FIXED: Scope isolation fixed, processing values safely
                if(depth_value > 0 && depth_value < 10000) { 
                    z = depth_value * 0.001f; // Convert mm to meters
                    x = (u - cx) * z / fx;
                    y = (v - cy) * z / fy;
                }
                
                cv::Vec3b color = rgb_image.at<cv::Vec3b>(v, u);
                
                // FIXED: Iterators MUST update even if depth is zero to maintain matrix grid alignment
                *iter_x = x;
                *iter_y = y;
                *iter_z = z;
                *iter_r = color[2]; 
                *iter_g = color[1];
                *iter_b = color[0];
                
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;

                int index = v * rgb_image.cols + u;
                slam_points_vector[index] = {x, y, z, color[2], color[1], color[0]};
            }
        }
        
        pc2_pub_->publish(std::move(pc2_msg));
        
        // Execute feature front-end tracker pipeline
        runSlamPipeline(rgb_image, slam_points_vector);
    }

    void runSlamPipeline(const cv::Mat& rgb_image, const std::vector<SlamPoint>& slam_points_vector) {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        
        orb_detector_->detectAndCompute(rgb_image, cv::noArray(), keypoints, descriptors);

        if(is_first_frame_) {
            prev_keypoints_ = keypoints;
            prev_descriptors_ = descriptors;
            prev_points_ = slam_points_vector; // FIXED: assigned correct variable reference
            prev_width_ = rgb_image.cols;
            prev_height_ = rgb_image.rows;
            is_first_frame_ = false;
            return; // Exit early since we don't have historical frames to calculate transformations yet
        }

        if(descriptors.empty() || prev_descriptors_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No visual descriptors found to process tracking state.");
            return;
        }

        std::vector<cv::DMatch> matches;
        matcher_->match(prev_descriptors_, descriptors, matches);

        std::vector<cv::Point3f> object_points_3d; 
        std::vector<cv::Point2f> image_points_2d;  

        for (const auto& match : matches) {
            int prev_idx = match.queryIdx;
            int curr_idx = match.trainIdx;

            cv::Point2f prev_pixel = prev_keypoints_[prev_idx].pt;
            int u = static_cast<int>(prev_pixel.x);
            int v = static_cast<int>(prev_pixel.y);
            
            int spatial_vector_index = v * prev_width_ + u;

            // Safe lookup bounding check
            if (spatial_vector_index >= 0 && spatial_vector_index < static_cast<int>(prev_points_.size())) {
                SlamPoint prev_3d_point = prev_points_[spatial_vector_index];

                if (prev_3d_point.z > 0.1f) { 
                    object_points_3d.push_back(cv::Point3f(prev_3d_point.x, prev_3d_point.y, prev_3d_point.z));
                    image_points_2d.push_back(keypoints[curr_idx].pt);
                }
            }
        }

        if (object_points_3d.size() >= 8) {
            cv::Mat rvec, tvec;
            std::vector<int> inliers;
            
            bool success = cv::solvePnPRansac(
                object_points_3d, image_points_2d, camera_matrix_, dist_coeffs_, 
                rvec, tvec, false, 100, 4.0, 0.99, inliers
            );

            if (success) {
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);

                RCLCPP_INFO(this->get_logger(), 
                    "Tracking Success! Inliers: %lu/%lu. Pose Delta-T: [X: %.3f, Y: %.3f, Z: %.3f]", 
                    inliers.size(), object_points_3d.size(), tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            } else {
                RCLCPP_WARN(this->get_logger(), "PnP RANSAC Optimization failed to converge.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Insufficient tracking links (%lu/8 required).", object_points_3d.size());
        }

        // Cache historical reference frames for the next operational turn
        prev_keypoints_ = keypoints;
        prev_descriptors_ = descriptors;
        prev_points_ = slam_points_vector;
        prev_width_ = rgb_image.cols;
        prev_height_ = rgb_image.rows;
    }

    // Class properties setup variables
    bool is_first_frame_;
    cv::Ptr<cv::ORB> orb_detector_;
    cv::Ptr<cv::BFMatcher> matcher_;
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    std::vector<cv::KeyPoint> prev_keypoints_;
    cv::Mat prev_descriptors_;
    std::vector<SlamPoint> prev_points_;
    int prev_width_;
    int prev_height_;

    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> SyncPolicy;
    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    message_filters::Subscriber<CameraInfo> info_sub_;
    std::shared_ptr<Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;
}; 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualSlamNode>());
    rclcpp::shutdown();
    return 0;
}