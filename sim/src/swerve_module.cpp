#include "sim/swerve_module.hpp"

namespace swerve_drive {

SwerveModule::SwerveModule(
    hardware_interface::LoanedCommandInterface &vel_cmd, 
    hardware_interface::LoanedCommandInterface &pos_cmd,
    hardware_interface::LoanedStateInterface &steer_pos_state,
    hardware_interface::LoanedStateInterface &steer_vel_state,
    hardware_interface::LoanedStateInterface &drive_pos_state,
    hardware_interface::LoanedStateInterface &drive_vel_state
) : 
velocity_cmd(vel_cmd), 
position_cmd(pos_cmd),
steer_pos(steer_pos_state),
steer_vel(steer_vel_state),
drive_pos(drive_pos_state),
drive_vel(drive_vel_state) {};

void SwerveModule::set_module_states(ModuleStates state) {
    //optimize anlgle
    double target = state.angle.get_radians();
    double diff = target - get_steer_position_feedback();
    diff = std::atan2(std::sin(diff), std::cos(diff)); //wrap diff between -pi and pi.
    
    if (std::abs(diff) > M_PI / 2.0) {
        state.speed_mps *= -1.0; //invert direction
        if (diff < 0) {
            target += M_PI;
        } else {
            target -= M_PI;
        }
    }

    set_position(target);
    set_velocity(state.speed_mps);
}

// returns radians
double SwerveModule::get_drive_position_feedback() {
    return drive_pos.get().get_value();
}

double SwerveModule::get_drive_velocity_feedback() {
    return drive_vel.get().get_value();
}

double SwerveModule::get_steer_position_feedback() {
    return steer_pos.get().get_value();
}

double SwerveModule::get_steer_velocity_feedback() {
    return steer_vel.get().get_value();
}

double SwerveModule::get_drive_distance() {
    double theta = get_drive_position_feedback();
    return theta * constants::wheel_radius_meter;
}

//set steer position
bool SwerveModule::set_position(double pos) {
   return position_cmd.get().set_value(pos);
}

//set drive velocity
bool SwerveModule::set_velocity(double vel) {
   return velocity_cmd.get().set_value(vel);
}

void SwerveModule::stop() {
    double current_pos = get_steer_position_feedback();
    set_position(current_pos);
    set_velocity(0.0);
}

}