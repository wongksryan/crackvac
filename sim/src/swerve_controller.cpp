#include "sim/swerve_controller.hpp"
/*
Run the teleop with ps4 joystick
*/
namespace {
    constexpr auto default_joy_topic = "/joy";
    constexpr auto default_odometry_topic = "~/odom";
    constexpr auto default_transform_topic = "/tf"; //rviz listens to /tf by default.
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
    joystick_subscriber = node->create_subscription<sensor_msgs::msg::Joy>(
        default_joy_topic, 
        rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
            received_vel_cmd.set(*msg);
        }); 

    //setup publishers to send message to a topic 
    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>(
        default_odometry_topic,
        rclcpp::SystemDefaultsQoS());
    realtime_odom_publisher 
        = std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_publisher);

    odom_tf_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>(
        default_transform_topic,
        rclcpp::SystemDefaultsQoS());
    realtime_odom_tf_publisher
        = std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odom_tf_publisher);

    // configure messages
    const auto header_frame_id = "odom";
    const auto child_frame_id = "base_link"; 
    odom_msg.header.frame_id = header_frame_id;
    odom_msg.child_frame_id = child_frame_id;
    odom_transform.header.frame_id = header_frame_id;
    odom_transform.child_frame_id = child_frame_id;

    RCLCPP_DEBUG(node->get_logger(), "Subscriber and publisher are now active.");
    return CallbackReturn::SUCCESS;
}

return_type SwerveController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Read controller input values from state interfaces
    // Calculate controller output values and write them to command interfaces
    const double dt = period.seconds();
    
    std::optional<sensor_msgs::msg::Joy> vel_cmd_op = received_vel_cmd.try_get();
    if (vel_cmd_op.has_value()) {
        last_vel_cmd = vel_cmd_op.value();

        double joystick_left_x = last_vel_cmd.axes[0];
        double joystick_left_y = last_vel_cmd.axes[1];
        double joystick_right_x = last_vel_cmd.axes[3];
        
        desired_speeds.vx_mps = joystick_left_x * drive_speed_factor;
        desired_speeds.vy_mps = joystick_left_y * drive_speed_factor;
        desired_speeds.omega_rps = joystick_right_x * steer_speed_factor;
    } else {
        desired_speeds.vx_mps = 0; 
        desired_speeds.vy_mps = 0;
        desired_speeds.omega_rps = 0;
    }

    // deadband
    if (std::abs(desired_speeds.vx_mps) <= 0.1) {
        desired_speeds.vx_mps = 0;
    } 
    if (std::abs(desired_speeds.vy_mps) <= 0.1) {
        desired_speeds.vy_mps = 0;
    }
    if (std::abs(desired_speeds.omega_rps) <= 0.1) {
        desired_speeds.omega_rps = 0;
    }

    last_speeds = desired_speeds;

    // convert chassis speeds into module states
    std::vector<ModuleStates> states;
    for (int i = 0; i < 4; i++) {
        Translation2d offset = constants::module_offsets[i];
        ModuleStates state;
        double speed_x = desired_speeds.vx_mps - desired_speeds.omega_rps * offset.y;
        double speed_y = desired_speeds.vy_mps + desired_speeds.omega_rps * offset.x;
        double magnitude = std::sqrt(speed_x * speed_x + speed_y * speed_y);
        double theta = std::atan2(speed_y, speed_x);
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

    // update odometry
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, pose.rotation.get_radians());
    send_odometry_msg(orientation, time);
    send_transform_msg(orientation, time);

    return return_type::OK;
}

// update numerical odometry
void SwerveController::send_odometry_msg(tf2::Quaternion orientation, rclcpp::Time time) {
    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x = pose.x;
    odom_msg.pose.pose.position.y = pose.y;
    odom_msg.pose.pose.orientation.x = orientation.getX();
    odom_msg.pose.pose.orientation.y = orientation.getY();
    odom_msg.pose.pose.orientation.z = orientation.getZ();
    odom_msg.pose.pose.orientation.w = orientation.getW();
    odom_msg.twist.twist.linear.x = last_speeds.vx_mps;
    odom_msg.twist.twist.linear.y = last_speeds.vy_mps;
    odom_msg.twist.twist.angular.z = last_speeds.omega_rps;
    realtime_odom_publisher->try_publish(odom_msg);
}

// update transform in rviz
void SwerveController::send_transform_msg(tf2::Quaternion orientation, rclcpp::Time time) {
    odom_transform.header.stamp = time;
    odom_transform.transform.translation.x = pose.x;
    odom_transform.transform.translation.y = pose.y;
    odom_transform.transform.rotation.x = orientation.getX();
    odom_transform.transform.rotation.y = orientation.getY();
    odom_transform.transform.rotation.z = orientation.getZ();
    odom_transform.transform.rotation.w = orientation.getW();
    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(odom_transform);
    realtime_odom_tf_publisher->try_publish(tf_msg);
}

// declare and get parameters needed for controller initialization
// allocate memory that will exist for the life of the controller
CallbackReturn SwerveController::on_init() {
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
    InterfaceConfiguration config;
    config.type = interface_configuration_type::INDIVIDUAL;
    config.names = {
        "front_left_steer_joint/position",
        "front_left_steer_joint/velocity",
        "front_left_wheel_joint/position",
        "front_left_wheel_joint/velocity",

        "front_right_steer_joint/position",
        "front_right_steer_joint/velocity",
        "front_right_wheel_joint/position",
        "front_right_wheel_joint/velocity",

        "back_left_steer_joint/position",
        "back_left_steer_joint/velocity",
        "back_left_wheel_joint/position",
        "back_left_wheel_joint/velocity",

        "back_right_steer_joint/position",
        "back_right_steer_joint/velocity",
        "back_right_wheel_joint/position",
        "back_right_wheel_joint/velocity"
    };
    return config;
}

CallbackReturn SwerveController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {
    last_vel_cmd.axes.resize(8, 0);
    desired_speeds.vx_mps = 0;
    desired_speeds.vy_mps = 0;
    desired_speeds.omega_rps = 0;

    // create swerve modules
    // *** must insert in order of pos_cmd, vel_cmd, 
    //          steer_pos_state, steer_vel_cmd, drive_pos_state, drive_vel_state.
    modules.clear();
    // front left
    modules.emplace_back(command_interfaces_[0], command_interfaces_[1], 
        state_interfaces_[0], state_interfaces_[1], state_interfaces_[2], state_interfaces_[3]); 
    // front right
    modules.emplace_back(command_interfaces_[2], command_interfaces_[3],
        state_interfaces_[4], state_interfaces_[5], state_interfaces_[6], state_interfaces_[7]); 
    // back left
    modules.emplace_back(command_interfaces_[4], command_interfaces_[5],
        state_interfaces_[8], state_interfaces_[9], state_interfaces_[10], state_interfaces_[11]); 
    // back right
    modules.emplace_back(command_interfaces_[6], command_interfaces_[7],
        state_interfaces_[12], state_interfaces_[13], state_interfaces_[14], state_interfaces_[15]); 

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