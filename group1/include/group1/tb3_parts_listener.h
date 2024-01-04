#pragma once

#include <group1/tb3_parts_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <group1/tb3_parts_listener.h>
#include <tf2/exceptions.h>
#include <cmath>
#include <group1/utils.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <mage_msgs/msg/part.hpp>
#include <std_msgs/msg/bool.hpp>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

struct DetectedPart
{
    std::string type;
    std::string color;
    geometry_msgs::msg::Pose pose;
};
// Give the namespace RWA3
namespace RWA3
{ /**
   * @brief Class for the Parts Details Storage
   *
   */
    class Tb3PartsListener : public rclcpp::Node
    {
    public:
        Tb3PartsListener(std::string node_name) : Node(node_name)
        {
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

            RCLCPP_INFO(this->get_logger(), "Listener demo started");

            // load a buffer of transforms
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // timer to listen to the transforms
            listen_timer_ = this->create_wall_timer(1s, std::bind(&Tb3PartsListener::listen_timer_cb_, this));

            // Create a the subscriber for the advanced logical camera image
            parts_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/advanced_logical_camera/image", qos_profile, std::bind(&Tb3PartsListener::parts_cb, this, std::placeholders::_1));

            // Initialize the detected part color and type
            detected_part_color_id_ = 0;
            detected_part_type_id_ = 0;

            // Initialize the detected part color and type as strings
            detected_part_color_ = "";
            detected_part_type_ = "";

            // Initialize the detected parts vector
            detected_parts_.clear();

            // Create a subscriber to the topic /run_completed of type std_msgs::msg::Bool to check if the run is completed
            run_completed_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
                "/run_completed", 10, std::bind(&Tb3PartsListener::run_completed_cb, this, std::placeholders::_1));
        }

    private:
        /*!< Boolean variable to store the value of the parameter "listen" */
        /**
         * @brief Boolean variable to store the value of the parameter "listen"
         */
        bool param_listen_;
        /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
        /**
         * @brief Buffer that stores several seconds of transforms for easy lookup by the listener.
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        /*!< Transform listener object */
        /**
         * @brief Transform listener object
         */
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        /*!< Wall timer object */
        rclcpp::TimerBase::SharedPtr listen_timer_;

        // Create a subscriber to the advanced logical camera image
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber_;

        // Create a callback function for the subscriber
        /**
         * @brief Callback function for the subscriber to the advanced logical camera image
         *
         * @param msg // Message received from the subscriber
         */
        void parts_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create a variable to store the detected parts color and type
        /**
         * @brief Variables to store the detected parts color and type
         *
         */
        int detected_part_color_id_;
        /**
         * @brief Variables to store the detected parts color and type
         *
         */
        int detected_part_type_id_;

        // Create  variables to store the detected parts color and type as strings
        /**
         * @brief Variables to store the detected parts color and type as strings
         *
         */
        std::string detected_part_color_;
        /**
         * @brief Variables to store the detected parts color and type as strings
         *
         */
        std::string detected_part_type_;

        /**
         * @brief Listen to a transform
         *
         * @param source_frame Source frame (child frame) of the transform
         * @param target_frame Target frame (parent frame) of the transform
         */
        geometry_msgs::msg::Pose listen_transform(const std::string &source_frame, const std::string &target_frame);

        /**
         * @brief Timer to listen to the transform
         */
        void listen_timer_cb_();

        // Create a vector to store the details of the detected parts of type DetectedPart
        /**
         * @brief Vector to store the details of the detected parts of type DetectedPart
         *
         */
        std::vector<DetectedPart> detected_parts_;

        // Create a subscriber to the topic /run_completed of type std_msgs::msg::Bool to check if the run is completed
        /**
         * @brief Subscriber to the topic /run_completed of type std_msgs::msg::Bool to check if the run is completed
         *
         */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_completed_subscriber_;

        // Define the callback function for the subscriber
        /**
         * @brief Callback function for the subscriber to the topic /run_completed
         *
         * @param msg // Message received from the subscriber
         */
        void run_completed_cb(const std_msgs::msg::Bool::SharedPtr msg);
    }; // class Tb3PartsListener
} // namespace RWA3