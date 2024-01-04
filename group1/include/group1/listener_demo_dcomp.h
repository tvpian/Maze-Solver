#pragma once

#include <cmath>
#include <group1/utils.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <memory>
#include <std_msgs/msg/bool.hpp>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

namespace RWA3
{ /**
   * @brief Class for the Aruco Marker Listener and Navigation
   *
   */
    class ListenerDemo : public rclcpp::Node
    {
    public:
        ListenerDemo(std::string node_name) : Node(node_name)
        {
            RCLCPP_INFO(this->get_logger(), "Listener demo started");

            this->declare_parameter("aruco_marker_0", "right_90");
            this->declare_parameter("aruco_marker_1", "left_90");
            this->declare_parameter("aruco_marker_2", "end");

            // load a buffer of transforms
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // timer to listen to the transforms
            listen_timer_ = this->create_wall_timer(1ms, std::bind(&ListenerDemo::listen_timer_cb_, this));

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&ListenerDemo::odom_cb, this, std::placeholders::_1));

            tb3_pose_ = geometry_msgs::msg::Pose();
            marker_pose_ = geometry_msgs::msg::Pose();

            utils_ptr_ = std::make_shared<Utils>();

            move_forward_rotate_timer_ = this->create_wall_timer(100ms, std::bind(&ListenerDemo::move_forward_rotate, this));
            rotate_robot_timer_ = this->create_wall_timer(100ms, std::bind(&ListenerDemo::rotate_robot, this));

            aruco_markers_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
                "/aruco_markers", 10, std::bind(&ListenerDemo::aruco_markers_cb, this, std::placeholders::_1));

            // Create a publisher to publish that the run is completed
            run_completed_publisher_ = this->create_publisher<std_msgs::msg::Bool>("run_completed", 10);

            // Set the run_completed_ variable to false
            run_completed_.data = false;

            // Print that the listener demo has been started
            RCLCPP_INFO(this->get_logger(), "Listener demo has been started.");
        }

    private:
        /**
         * @brief Unique pointer to the buffer of transforms
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        /**
         * @brief Shared pointer to the transform listener
         */
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

        /**
         * @brief Timer to listen to the transform
         */
        rclcpp::TimerBase::SharedPtr listen_timer_;

        /**
         * @brief Listen to a transform and return the pose of the marker in the odom frame
         *
         * @param source_frame Source frame (child frame) of the transform
         * @param target_frame Target frame (parent frame) of the transform
         */
        geometry_msgs::msg::Pose listen_transform(const std::string &source_frame, const std::string &target_frame);

        /**
         * @brief Timer to listen to the transform
         *
         */
        void listen_timer_cb_();

        /**
         * @brief Publisher to publish velocity commands to the turtlebot3
         *
         */
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

        /**
         * @brief Subscriber to the topic /odom
         */
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

        /**
         * @brief Callback function for the subscriber to the topic /odom
         *
         * @param msg Message containing the pose of the turtlebot3
         */
        void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
         * @brief Variable to store the pose of the turtlebot3
         *
         * @details This variable is used to store the pose of the turtlebot3 in the odom frame
         */
        geometry_msgs::msg::Pose tb3_pose_;

        /**
         * @brief Variable to store the pose of the marker in the odom frame
         *
         * @details This variable is used to store the pose of the marker in the odom frame
         */
        geometry_msgs::msg::Pose marker_pose_;

        /**
         * @brief Shared pointer to the Utils class - used to access the utility functions
         */
        std::shared_ptr<Utils> utils_ptr_;

        /**
         * @brief Function to rotate the robot by 90 degrees in the direction specified
         */
        void rotate_robot();

        /**
        * @brief Variable to store if the marker has been reached

        * @details This variable is set to true when the marker has been reached
        */
        bool reached_marker_ = false;

        /**
        * @brief Function to get the pose of the turtlebot3

        * @details This function returns the pose of the turtlebot3 in the odom frame
        */
        geometry_msgs::msg::Pose get_tb3_pose();

        /**
        * @brief Variable to store the velocity command to be published

        * @details This variable is used to publish the velocity command to the turtlebot3
        */
        geometry_msgs::msg::Twist msg_vel;

        /**
         * @brief Function to move the robot forward by 0.5m and rotate it by 90 degrees
         */
        void move_forward_rotate();

        /**
        * @brief Variable to store if the robot should rotate

        * @details This variable is set to true when the robot should rotate by 90 degrees
        */
        bool rotate_robot_flag_ = false;

        /**
        * @brief Variable to store the target yaw angle

        * @details The target yaw angle is the yaw angle the robot should reach after rotating by 90 degrees
        */
        double target_yaw_ = 0.0;

        /**
        * @brief Variable to store the direction of rotation

        * @details 1 for clockwise and -1 for anticlockwise
        */
        int direction = 1;

        /**
         * @brief Timer to move the robot forward by 0.5m and rotate it by 90 degrees
         */
        rclcpp::TimerBase::SharedPtr move_forward_rotate_timer_;

        /**
         * @brief Timer to rotate the robot by 90 degrees in the direction specified
         */
        rclcpp::TimerBase::SharedPtr rotate_robot_timer_;

        /**
        * @brief Function to get the direction of rotation from the marker id

        * @param marker_id Marker id of the marker detected
        */
        int get_direction_from_markerid(int marker_id);

        /**
         * @brief Subscriber to the topic /aruco_markers
         */
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_markers_sub_;

        /**
        * @brief Callback function for the subscriber to the topic /aruco_markers

        * @param msg Message containing the marker ids of the markers detected
        */
        void aruco_markers_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        /**
         * @brief Variable to store the latest marker id
         */
        int latest_marker_id_ = 0;

        /**
         * @brief Publisher to publish that the run is completed
         */
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr run_completed_publisher_;

        /**
         * @brief Variable to store if the run is completed
         */
        std_msgs::msg::Bool run_completed_;
    };
}
