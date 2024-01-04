#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <group1/listener_demo_dcomp.h>
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

// needed for the listener
#include <tf2/exceptions.h>
using namespace std::chrono_literals;

geometry_msgs::msg::Pose RWA3::ListenerDemo::listen_transform(const std::string &source_frame,
                                                              const std::string &target_frame)
{
  geometry_msgs::msg::TransformStamped t_stamped;
  geometry_msgs::msg::Pose pose_out;
  try
  {
    t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and "
                                                                               << target_frame << ": " << ex.what());
  }

  pose_out.position.x = t_stamped.transform.translation.x;
  pose_out.position.y = t_stamped.transform.translation.y;
  pose_out.position.z = t_stamped.transform.translation.z;
  pose_out.orientation = t_stamped.transform.rotation;

  return pose_out;
}

void RWA3::ListenerDemo::listen_timer_cb_()
{
}

void RWA3::ListenerDemo::aruco_markers_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  latest_marker_id_ = msg->marker_ids.back();
}

int RWA3::ListenerDemo::get_direction_from_markerid(int marker_id)
{
  std::string marker_data;

  if (marker_id == 0)
  {
    marker_data = this->get_parameter("aruco_marker_0").as_string();
    if (marker_data == "right_90")
    {
      return -1;
    }
    else if (marker_data == "left_90")
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else if (marker_id == 1)
  {
    marker_data = this->get_parameter("aruco_marker_1").as_string();
    if (marker_data == "left_90")
    {
      return 1;
    }
    else if (marker_data == "right_90")
    {
      return -1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    marker_data = this->get_parameter("aruco_marker_2").as_string();
    if (marker_data == "end")
    {
      return 0;
    }
    else if (marker_data == "left_90")
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }
}

void RWA3::ListenerDemo::move_forward_rotate()
{
  marker_pose_ = listen_transform("odom", "marker_frame");
  double distance = sqrt(pow(marker_pose_.position.x - tb3_pose_.position.x, 2) +
                         pow(marker_pose_.position.y - tb3_pose_.position.y, 2));

  // RCLCPP_INFO_STREAM(this->get_logger(), "Distance between the robot and the marker: " << distance);

  geometry_msgs::msg::Twist msg_vel;

  // If the distance is greater than 1 m, move the robot forward to reach the marker. Else, stop the robot. Do it inly
  // if marker reached is false
  if (distance > 1 && !rotate_robot_flag_ && direction != 0)
  {
    msg_vel.linear.x = 0.1;
    publisher_->publish(msg_vel);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Velocity published: " << msg_vel.linear.x);
  }
  else
  {
    if (!rotate_robot_flag_)
    {
      rotate_robot_flag_ = true;
      direction = get_direction_from_markerid(latest_marker_id_);

      geometry_msgs::msg::Quaternion q = tb3_pose_.orientation;
      double yaw = utils_ptr_->set_euler_from_quaternion(tf2::Quaternion(q.x, q.y, q.z, q.w))[2];

      // RCLCPP_INFO_STREAM(this->get_logger(), "Init Yaw:  " << yaw);

      target_yaw_ = 0.00;
      target_yaw_ = yaw + direction * M_PI / 2;
    }
  }
}

void RWA3::ListenerDemo::rotate_robot()
{
  double velocity = 0.1;

  if (rotate_robot_flag_ && direction != 0)
  {
    geometry_msgs::msg::Twist msg_vel;

    msg_vel.angular.z = direction * velocity;
    publisher_->publish(msg_vel);

    geometry_msgs::msg::Quaternion q = tb3_pose_.orientation;
    double yaw = utils_ptr_->set_euler_from_quaternion(tf2::Quaternion(q.x, q.y, q.z, q.w))[2];

    // RCLCPP_INFO_STREAM(this->get_logger(), "Robot yaw in the odom frame: " << yaw);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Direction: " << direction);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Velocity published: " << msg_vel.angular.z);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Target yaw: " << target_yaw_);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Angle reached: " << yaw);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Difference: " << abs(yaw - target_yaw_));

    if (abs(yaw - target_yaw_) < 0.035)
    {
      // RCLCPP_INFO_STREAM(this->get_logger(), "Target yaw: " << target_yaw_);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Difference: " << abs(yaw - target_yaw_));
      // RCLCPP_INFO_STREAM(this->get_logger(), "Angle reached: " << yaw);
      rotate_robot_flag_ = false;
      msg_vel.angular.z = 0;
      publisher_->publish(msg_vel);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Robot rotated by 90 degrees and stopped");
    }
  }
  else if (direction == 0)
  {
    msg_vel.linear.x = 0.0;
    publisher_->publish(msg_vel);

    // RCLCPP_INFO_STREAM(this->get_logger(), "Robot stopped & run completed");

    // Publish that the run is completed
    run_completed_.data = true;
    run_completed_publisher_->publish(run_completed_);
  }
}

void RWA3::ListenerDemo::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tb3_pose_ = msg->pose.pose;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RWA3::ListenerDemo>("listener_demo_dcomp");
  rclcpp::spin(node);
  rclcpp::shutdown();
}