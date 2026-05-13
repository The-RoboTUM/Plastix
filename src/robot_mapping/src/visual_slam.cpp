#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs::msg;
using namespace message_filters;

class D405_SlamNode : public rclcpp::Node{
public:
    D405_SlamNode() : Node("d405_slam_node"){
        // Subscribers
        rgb_sub_.subscribe(this, "/camera/color/image_raw");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");
        info_sub_.subscribe(this, "/camera/color/camera_info");

        // Use ApproximateTime sync since Depth and RGB might have offset
        sync_=std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_);
        sync_->registerCallback(std::bind(&D405_SlamNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "D405 Slam Node started.");
    }

private:
    void callback(const Image::ConstSharedPtr& rgb, const Image::ConstSharedPtr& depth, const CameraInfo::ConstSharedPtr& info){
        RCLCPP_INFO(this->get_logger(), "Synchronized Frame Recieved");
    }
    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> SyncPolicy;
    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    message_filters::Subscriber<CameraInfo> info_sub_;
    std::shared_ptr<Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D405_SlamNode>());
    rclcpp::shutdown();
    return 0;
}