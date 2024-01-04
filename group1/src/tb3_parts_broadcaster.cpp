#include <memory>
#include <chrono>
#include <group1/tb3_parts_broadcaster.h>

void RWA3::Tb3PartsDetector::parts_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // Access the parts in the message and save the color and type of the first part
  for (auto part : msg->part_poses)
  {
    // Print the part pose
    // RCLCPP_INFO(this->get_logger(), "Part Pose: [%f, %f, %f]", part.pose.position.x, part.pose.position.y,
    //             part.pose.position.z);
    // Create a broadcaster object to broadcast the transform between the logical camera frame and the part framew
    geometry_msgs::msg::TransformStamped part_tf;
    part_tf.header.stamp = this->get_clock()->now();
    part_tf.header.frame_id = "logical_camera_link";
    part_tf.child_frame_id = "part_frame";
    part_tf.transform.translation.x = part.pose.position.x;
    part_tf.transform.translation.y = part.pose.position.y;
    part_tf.transform.translation.z = part.pose.position.z;
    part_tf.transform.rotation.x = part.pose.orientation.x;
    part_tf.transform.rotation.y = part.pose.orientation.y;
    part_tf.transform.rotation.z = part.pose.orientation.z;
    part_tf.transform.rotation.w = part.pose.orientation.w;
    // Broadcast the transform
    tf_broadcaster_->sendTransform(part_tf);

    // RCLCPP_INFO(this->get_logger(), "Part Color: %d", part.part.color);
    // RCLCPP_INFO(this->get_logger(), "Part Type: %d", part.part.type);
  }
}

int main(int argc, char **argv)
{
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);
  // Create a node
  auto node = std::make_shared<RWA3::Tb3PartsDetector>("tb3_parts_broadcaster");
  // Spin the node
  rclcpp::spin(node);
  // Shutdown the node
  rclcpp::shutdown();
  return 0;
}
