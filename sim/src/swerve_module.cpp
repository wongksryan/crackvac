#include "sim/swerve_module.hpp"

namespace swerve_drive {

SwerveModule::SwerveModule(
    hardware_interface::LoanedCommandInterface &vel_cmd, 
    hardware_interface::LoanedCommandInterface &pos_cmd
) : 
velocity_cmd(vel_cmd), 
position_cmd(pos_cmd) {
    //TODO: encoder zero: encoder.get() - encoder_offset
};

void SwerveModule::set_module_states(ModuleStates state) {
    //optimize anlgle
    double target = state.angle.get_radians();
    double diff = target - get_position_feedback().value_or(0.0);
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

std::optional<double> SwerveModule::get_position_feedback() {
    return position_cmd.get().get_optional();
}

std::optional<double> SwerveModule::get_velocity_feedback() {
    return velocity_cmd.get().get_optional();
}

bool SwerveModule::set_position(double pos) {
   return position_cmd.get().set_value(pos);
}

bool SwerveModule::set_velocity(double vel) {
   return velocity_cmd.get().set_value(vel);
}

void SwerveModule::zero() {
    set_position(0.0);
}

void SwerveModule::stop() {
    set_velocity(0.0);
}

void SwerveModule::reset() {
    set_position(0.0);
    set_velocity(0.0);
}

}