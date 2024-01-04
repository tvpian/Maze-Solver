#include <memory>
#include <chrono>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <group1/listener_demo_dcomp.h>
using namespace std::chrono_literals;

namespace RWA3
{
    /**
     * @brief Class for the Aruco Marker Detection and Broadcast
     * 
     */
    class Tb3ArucoDetector : public rclcpp::Node
    {
    public:
        /**
         * @brief Construct a new Tb3 Aruco Detector object
         *
         * @param node_name Name of the node
         */
        Tb3ArucoDetector(std::string node_name) : Node(node_name)
        {
            subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
                "/aruco_markers", 10, std::bind(&Tb3ArucoDetector::aruco_markers_cb, this, std::placeholders::_1));

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            tf_broadcaster_odom_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

    private:
        /**
        * @brief Callback function for /aruco_markers topic
        *
        * @param msg Message published on /aruco_markers topic
        */
        void aruco_markers_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        /**
         * @brief Subscription for /aruco_markers topic
         */
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_;

        /**
         * @brief Shared pointer to tf2_ros::Buffer
         */
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        /**
         * @brief Shared pointer to tf2_ros::TransformListener
         */
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        /**
         * @brief Shared pointer to tf2_ros::TransformBroadcaster
         */
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        /**
         * @brief Shared pointer to tf2_ros::TransformBroadcaster
         */
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_odom_;

        /**
         * @brief Timer for broadcasting odom frame
         */
        rclcpp::TimerBase::SharedPtr broadcast_timer_;
        
    }; // class Tb3ArucoDetector
} // namespace RWA3