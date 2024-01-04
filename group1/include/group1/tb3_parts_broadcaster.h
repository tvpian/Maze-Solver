#pragma once
#include <memory>
#include <chrono>
// #include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <rclcpp/timer.hpp>

using namespace std::chrono_literals;

namespace RWA3
{   
    /**
     * @brief Class for the Parts Detection and Broadcast
     * 
     */
    class Tb3PartsDetector : public rclcpp::Node
    {
    public:
        Tb3PartsDetector(std::string node_name) : Node(node_name)
        {
            // Create a QoS profile
            //             QoS profile:
            //   Reliability: BEST_EFFORT
            //   Durability: VOLATILE
            //   Lifespan: 9223372036854775807 nanoseconds
            //   Deadline: 9223372036854775807 nanoseconds
            //   Liveliness: AUTOMATIC
            //   Liveliness lease duration: 9223372036854775807 nanoseconds

            // Create a QoS profile
            rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
            // Set the reliability policy to best effort
            qos_profile.best_effort();
            // Set the durability policy to volatile
            qos_profile.durability_volatile();
            // Set the lifespan to 9223372036854775807 nanoseconds
            qos_profile.lifespan(std::chrono::nanoseconds(9223372036854775807));
            // Set the deadline to 9223372036854775807 nanoseconds
            qos_profile.deadline(std::chrono::nanoseconds(9223372036854775807));
            // Set the liveliness policy to automatic
            qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
            // Set the liveliness lease duration to 9223372036854775807 nanoseconds
            qos_profile.liveliness_lease_duration(std::chrono::nanoseconds(9223372036854775807));

            // Create a subscriber for the logical camera image
            parts_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/advanced_logical_camera/image", qos_profile, std::bind(&Tb3PartsDetector::parts_cb, this, std::placeholders::_1));

            // Create a dynamic broadcaster for braodcasting the tf of the detected parts in the logical camera frame
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            // Load a buffer of transforms
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_buffer_->setUsingDedicatedThread(true);

            // Print that the node is started
            RCLCPP_INFO(this->get_logger(), "Tb3 Parts Detector node started");
        }

    private:
        // Create a subscriber for the logical camera image
        /**
         * @brief Subscriber for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber_;
        // Create a callback function for the subscriber
        /**
         * @brief  Callback function for the subscriber
         *
         * @param msg // Message received from the subscriber
         */
        void parts_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
        // Create a dynamic broadcaster for braodcasting the tf of the detected parts in the logical camera frame
        /**
         * @brief  Dynamic broadcaster for braodcasting the tf of the detected parts in the logical camera frame
         *
         */
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        // Create a buffer of transforms
        /**
         * @brief Buffer of transforms
         *
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // Create a listener for obtaining the transform part_frame -> odom
        /**
         * @brief Listener for obtaining the transform part_frame -> odom
         *
         */
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_part_frame_odom_;

        // Create a buffer to store the transform part_frame -> odom for several seconds
        /**
         * @brief Buffer to store the transform part_frame -> odom for several seconds
         *
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_part_frame_odom_;
    };
}