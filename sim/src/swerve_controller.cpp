#include "sim/swerve_controller.hpp"


namespace {
    constexpr auto default_command_in_topic = "~/cmd_vel"; // ~ part is replaced with namespace.
    constexpr auto default_odometry_topic = "~/odom";
    constexpr auto default_transform_topic = "~/tf";
}

namespace swerve_drive {
/*
note:
- use Topics for streaming sensor or state data
- use Services for requests that expect quick replies (i.e. zero, get_pos)
- use Actions for tasks that require monitoring and cancellation in the middle of operation 
    (i.e. raise an arm)
Swerve drive requires frequent data reading and sending, thus we use Topics.
*/
CallbackReturn SwerveController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {
    auto node = this->get_node();

    //setup subscription to receive message from a topic
    vel_cmd_subscriber = node->create_subscription<Twist>(
        default_command_in_topic, 
        rclcpp::SystemDefaultsQoS(),
        [this](const Twist::SharedPtr msg) {
            desired_speeds.vx_mps = msg->linear.x;
            desired_speeds.vy_mps = msg->linear.y;
            desired_speeds.omega_rps = msg->angular.z;
        }); 

    //setup publishers to send message to a topic 
    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>(
        default_odometry_topic,
        rclcpp::SystemDefaultsQoS());
    odom_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    RCLCPP_DEBUG(node->get_logger(), "Subscriber and publisher are now active.");
    return CallbackReturn::SUCCESS;
}

/*
Run the teleop with keyboards via:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=swerve_controller/cmd_vel
the default topic is cmd_vel. so we need to specify.
*/
return_type SwerveController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Read controller input values from state interfaces
    // Calculate controller output values and write them to command interfaces
    const double dt = period.seconds();
    last_speeds = desired_speeds;

    // convert chassis speeds into module states
    std::vector<ModuleStates> states;
    for (int i = 0; i < 4; i++) {
        Translation2d offset = constants::module_offsets[i];
        ModuleStates state;
        double speed_x = desired_speeds.vx_mps - desired_speeds.omega_rps * offset.y;
        double speed_y = desired_speeds.vy_mps + desired_speeds.omega_rps * offset.x;
        double magnitude = std::sqrt(speed_x * speed_x + speed_y * speed_y);
        double theta = std::atan2(speed_y, speed_x); //returns 0 if both linear speeds are zero.
        state.speed_mps = magnitude;
        state.angle = Rotation2d(theta); 
        states.push_back(state);
    }

    // apply the states to modules
    modules[0].set_module_states(states[0]);
    modules[1].set_module_states(states[1]);
    modules[2].set_module_states(states[2]);
    modules[3].set_module_states(states[3]);

    // TODO: closed loop and open loop modes
    // open loop robot pose estimation: 
    double dx = last_speeds.vx_mps * dt;
    double dy = last_speeds.vy_mps * dt;
    double dheading = last_speeds.omega_rps * dt;

    if (std::fabs(dheading) < 0.01) {
        // for very small heading change, approximate to linear motion
        pose.x += dx * pose.rotation.cos() - dy * pose.rotation.sin();
        pose.y += dx * pose.rotation.sin() + dy * pose.rotation.cos();
    } else {
        // integrate motion: integral of [R(theta)] * [vx, vy] over timestamp dt. 
        Rotation2d old_heading = pose.rotation;
        pose.rotation = pose.rotation.plus(Rotation2d::from_radians(dheading));
        pose.x += (dx / dheading) * (pose.rotation.sin() - old_heading.sin()) 
                + (dy / dheading) * (-pose.rotation.cos() + old_heading.cos());
        pose.y += (dx / dheading) * (-pose.rotation.cos() + old_heading.cos())
                + (dy / dheading) * (pose.rotation.sin() - old_heading.sin());
    }

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, pose.rotation.get_radians());
    
    // update numerical odometry
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.header.stamp = get_node()->now();
    odom_msg.pose.pose.position.x = pose.x;
    odom_msg.pose.pose.position.y = pose.y;
    odom_msg.pose.pose.orientation.x = orientation.getX();
    odom_msg.pose.pose.orientation.y = orientation.getY();
    odom_msg.pose.pose.orientation.z = orientation.getZ();
    odom_msg.pose.pose.orientation.w = orientation.getW();
    odom_msg.twist.twist.linear.x = last_speeds.vx_mps;
    odom_msg.twist.twist.linear.y = last_speeds.vy_mps;
    odom_msg.twist.twist.angular.z = last_speeds.omega_rps;
    odom_publisher->publish(odom_msg);
    
    // update odometry transform
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.header.stamp = get_node()->now();
    tf.transform.translation.x = pose.x;
    tf.transform.translation.y = pose.y;
    tf.transform.rotation.x = orientation.getX();
    tf.transform.rotation.y = orientation.getY();
    tf.transform.rotation.z = orientation.getZ();
    tf.transform.rotation.w = orientation.getW();
    odom_tf_broadcaster->sendTransform(tf);

    return return_type::OK;
}

CallbackReturn SwerveController::on_init(){
    // declare and get parameters needed for controller initialization
    // allocate memory that will exist for the life of the controller
    return CallbackReturn::SUCCESS;
}

// configure commands to send. called after on_configure.
InterfaceConfiguration SwerveController::command_interface_configuration() const {
    InterfaceConfiguration config;
    config.type = interface_configuration_type::INDIVIDUAL;
    config.names = {
        "front_left_steer_joint/position",
        "front_left_wheel_joint/velocity",
        "front_right_steer_joint/position",
        "front_right_wheel_joint/velocity",
        "back_left_steer_joint/position",
        "back_left_wheel_joint/velocity",
        "back_right_steer_joint/position",
        "back_right_wheel_joint/velocity"
    };
    return config;
}

//configures feedback values to read
InterfaceConfiguration SwerveController::state_interface_configuration() const {
    //no received data for open loop
    if (is_openloop) {
        return {interface_configuration_type::NONE};
    } else {
        //closed loop setting
        InterfaceConfiguration config;
        config.type = interface_configuration_type::INDIVIDUAL;
        config.names = {
            "front_left_steer_joint/position",
            "front_left_wheel_joint/velocity",
            "front_right_steer_joint/position",
            "front_right_wheel_joint/velocity",
            "back_left_steer_joint/position",
            "back_left_wheel_joint/velocity",
            "back_right_steer_joint/position",
            "back_right_wheel_joint/velocity"
        };
       return config;
    }
}

CallbackReturn SwerveController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {
    modules.clear();

    //create swerve modules
    modules.emplace_back(command_interfaces_[0], command_interfaces_[1]); //fl
    modules.emplace_back(command_interfaces_[2], command_interfaces_[3]); //fr
    modules.emplace_back(command_interfaces_[4], command_interfaces_[5]); //bl
    modules.emplace_back(command_interfaces_[6], command_interfaces_[7]); //br

    RCLCPP_DEBUG(get_node()->get_logger(), "Initialized swerve modules.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {
    modules[0].stop();
    modules[1].stop();
    modules[2].stop();
    modules[3].stop();

    RCLCPP_DEBUG(get_node()->get_logger(), "Swerve controller deactivated.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_cleanup(
    const rclcpp_lifecycle::State & previous_state) {
    //reset
    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_error(
    const rclcpp_lifecycle::State & previous_state) {
    //reset
    return CallbackReturn::ERROR;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    swerve_drive::SwerveController,
    controller_interface::ControllerInterface
)