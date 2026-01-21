#ifndef SWERVE_CONTROLLER_HPP
#define SWERVE_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "swerve_module.hpp"
#include "geometry/rotation2d.hpp"
#include "constants.hpp"

using namespace controller_interface;

namespace swerve_drive {
    struct ChassisSpeeds {
        double vx_mps = 0.0;
        double vy_mps = 0.0;
        double omega_rps = 0.0;
    };
    struct Pose2d {
        double x = 0.0;
        double y = 0.0;
        Rotation2d rotation;
    };

    class SwerveController : public ControllerInterface {
        private:
        std::vector<SwerveModule> modules;
        ChassisSpeeds desired_speeds, last_speeds;
        rclcpp::Time last_time;
        Pose2d pose;
        const double drive_speed_factor = 2.5;
        const double steer_speed_factor = 2.5;
        
        // realtime tools 
        realtime_tools::RealtimeThreadSafeBox<sensor_msgs::msg::Joy> received_vel_cmd; 
        std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odom_publisher = nullptr;
        std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odom_tf_publisher = nullptr;
        // default ROS tools
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber = nullptr;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher = nullptr;
        std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odom_tf_publisher = nullptr;
        // messagges to send
        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::TransformStamped odom_transform;
        
        sensor_msgs::msg::Joy last_vel_cmd;

        public:
        SwerveController() {};

        void send_odometry_msg(tf2::Quaternion orientation, rclcpp::Time time);
        void send_transform_msg(tf2::Quaternion orientation, rclcpp::Time time);

        CallbackReturn on_init() override;

        InterfaceConfiguration command_interface_configuration() const override;

        InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_error(
            const rclcpp_lifecycle::State & previous_state) override;
    };
}

#endif