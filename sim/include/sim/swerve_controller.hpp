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
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "swerve_module.hpp"
#include "geometry/rotation2d.hpp"
#include "constants.hpp"

using namespace controller_interface;
using Twist = geometry_msgs::msg::Twist;

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
        ChassisSpeeds desired_speeds = ChassisSpeeds();
        ChassisSpeeds last_speeds = ChassisSpeeds();
        rclcpp::Time last_time;
        Pose2d pose;

        rclcpp::Subscription<Twist>::SharedPtr vel_cmd_subscriber = nullptr;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher = nullptr;
        std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster = nullptr;

        bool is_sim = true;
        
        public:
        SwerveController() {};

        void send_odometry_msg(tf2::Quaternion orientation);
        void send_transform_msg(tf2::Quaternion orientation);

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