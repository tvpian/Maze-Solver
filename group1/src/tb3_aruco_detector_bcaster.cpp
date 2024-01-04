#include <iostream>
#include <group1/tb3_aruco_detector_bcaster.h>

void RWA3::Tb3ArucoDetector::aruco_markers_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received aruco_markers message");

    // Keep only the newest marker pose
    if (msg->poses.size() > 1)
    {
        msg->poses.erase(msg->poses.begin(), msg->poses.end() - 1);
    }

    // Create a broadcaster object to broadcast the transform between the camera frame and the marker frame
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "camera_rgb_optical_frame";
    dynamic_transform_stamped.child_frame_id = "marker_frame";
    dynamic_transform_stamped.transform.translation.x = msg->poses[0].position.x;
    dynamic_transform_stamped.transform.translation.y = msg->poses[0].position.y;
    dynamic_transform_stamped.transform.translation.z = msg->poses[0].position.z;
    dynamic_transform_stamped.transform.rotation.x = msg->poses[0].orientation.x;
    dynamic_transform_stamped.transform.rotation.y = msg->poses[0].orientation.y;
    dynamic_transform_stamped.transform.rotation.z = msg->poses[0].orientation.z;
    dynamic_transform_stamped.transform.rotation.w = msg->poses[0].orientation.w;
    tf_broadcaster_->sendTransform(dynamic_transform_stamped);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RWA3::Tb3ArucoDetector>("tb3_aruco_detector");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
