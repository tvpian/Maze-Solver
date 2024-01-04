#pragma once
#include <group1/tb3_parts_listener.h>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

void RWA3::Tb3PartsListener::parts_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        detected_part_color_id_ = part.part.color;
        detected_part_type_id_ = part.part.type;
        break;
    }
    // Map the color and type of the detected part to the appropriate string
    if (detected_part_color_id_ == 0)
    {
        detected_part_color_ = "RED";
    }
    else if (detected_part_color_id_ == 1)
    {
        detected_part_color_ = "GREEN";
    }
    else if (detected_part_color_id_ == 2)
    {
        detected_part_color_ = "BLUE";
    }
    else if (detected_part_color_id_ == 3)
    {
        detected_part_color_ = "ORANGE";
    }
    else if (detected_part_color_id_ == 4)
    {
        detected_part_color_ = "PURPLE";
    }
    else
    {
        detected_part_color_ = "UNKNOWN";
    }

    if (detected_part_type_id_ == 10)
    {
        detected_part_type_ = "BATTERY";
    }
    else if (detected_part_type_id_ == 11)
    {
        detected_part_type_ = "PUMP";
    }
    else if (detected_part_type_id_ == 12)
    {
        detected_part_type_ = "SENSOR";
    }
    else if (detected_part_type_id_ == 13)
    {
        detected_part_type_ = "REGULATOR";
    }
    else
    {
        detected_part_type_ = "UNKNOWN";
    }
}

geometry_msgs::msg::Pose RWA3::Tb3PartsListener::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // Create a TransformStamped variable to store the transform between the source and target frames
    geometry_msgs::msg::TransformStamped t_stamped;
    // Create a Pose variable to store the pose of the target frame in the source frame
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        // RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
    }

    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;

    // RCLCPP_INFO_STREAM(this->get_logger(), target_frame << " in " << source_frame << ":\n"
    //                                                     << "x: " << pose_out.position.x << "\t"
    //                                                     << "y: " << pose_out.position.y << "\t"
    //                                                     << "z: " << pose_out.position.z << "\n"
    //                                                     << "qx: " << pose_out.orientation.x << "\t"
    //                                                     << "qy: " << pose_out.orientation.y << "\t"
    //                                                     << "qz: " << pose_out.orientation.z << "\t"
    //                                                     << "qw: " << pose_out.orientation.w << "\n");
    return pose_out;
}

void RWA3::Tb3PartsListener::listen_timer_cb_()
{
    // Create a boolean variable to check if the detected part is already in the vector detected_parts_
    bool detected_part_in_vector = false;
    // Check if the detected part is already in the vector detected_parts_
    for (const auto &part : detected_parts_)
    { // If the detected part is already in the vector detected_parts_, set the boolean variable detected_part_in_vector to true and break the loop
        // If the detected part is already in the vector detected_parts_ or any part type or color is UNKNOWN, do not add the detected part to the vector detected_parts_
        if (part.color == detected_part_color_ && part.type == detected_part_type_ || detected_part_color_ == "UNKNOWN" || detected_part_type_ == "UNKNOWN")
        {
            detected_part_in_vector = true;
            break;
        }
        else
        {
            detected_part_in_vector = false;
        }
    }
    // If the detected part is not already in the vector detected_parts_, add the detected part to the vector detected_parts_
    if (detected_part_in_vector == false)
    {
        // Create an object of the struct Part
        DetectedPart detected_part;
        // Save the detected part color and type
        detected_part.color = detected_part_color_;
        detected_part.type = detected_part_type_;
        // Save the detected part pose in the odom frame
        detected_part.pose = listen_transform("odom", "part_frame");
        // Add the detected part to the vector detected_parts_
        // Print the components of  the vector detected_parts_ in the terminal before adding the detected part

        // If either the color or the type of the detected part is UNKNOWN, do not add the detected part to the vector detected_parts_
        if (detected_part.color != "UNKNOWN" && detected_part.type != "UNKNOWN")
        {
            RCLCPP_INFO(this->get_logger(), "Robot is navigating and searching for parts\n");
            RCLCPP_INFO(this->get_logger(), ".....................................................");
            RCLCPP_INFO(this->get_logger(), "Detected new part. Adding to the Detected Parts List");
            RCLCPP_INFO(this->get_logger(), "Details of the new detected part:");
            RCLCPP_INFO(this->get_logger(), "Detected Part Color: %s", detected_part.color.c_str());
            RCLCPP_INFO(this->get_logger(), "Detected Part Type: %s", detected_part.type.c_str());
            RCLCPP_INFO(this->get_logger(), "Detected Part Position in odom frame: [%f, %f, %f]", detected_part.pose.position.x, detected_part.pose.position.y, detected_part.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Detected Part Orientation in odom frame: [%f, %f, %f, %f]", detected_part.pose.orientation.x, detected_part.pose.orientation.y, detected_part.pose.orientation.z, detected_part.pose.orientation.w);
            RCLCPP_INFO(this->get_logger(), ".....................................................\n");
            detected_parts_.push_back(detected_part);
        }
    }
}

void RWA3::Tb3PartsListener::run_completed_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    // Check if the run is completed
    if (msg->data == true)
    {
        // Print that the run is completed
        RCLCPP_INFO(this->get_logger(), "Run completed");
        // Print the output of the vector detected_parts_ in the terminal after the run is completed inthe form: Blue battery detected at xyz=[0, 0.926746, 0.25] rpy=[0, 0, 0.78539816339]
        RCLCPP_INFO(this->get_logger(), ".....................................................");
        RCLCPP_INFO(this->get_logger(), "Output of the Detected Parts List:");
        for (const auto &part : detected_parts_)
        {
            // Convert the quaternion to roll, pitch and yaw
            tf2::Quaternion q(part.pose.orientation.x, part.pose.orientation.y, part.pose.orientation.z, part.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            // Print the details of the detected part
            RCLCPP_INFO(this->get_logger(), "%s %s detected at xyz=[%f, %f, %f] rpy=[%f, %f, %f]", part.color.c_str(), part.type.c_str(), part.pose.position.x, part.pose.position.y, part.pose.position.z, roll, pitch, yaw);
        }
        RCLCPP_INFO(this->get_logger(), ".....................................................");

        // Gracefully shutdown the ROS node
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RWA3::Tb3PartsListener>("tb3_parts_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();
}