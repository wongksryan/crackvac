#ifndef SWERVE_MODULE_HPP
#define SWERVE_MODULE_HPP

#include <string.h>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "geometry/rotation2d.hpp"
#include "constants.hpp"

namespace swerve_drive {
    struct ModuleStates {
        double speed_mps = 0.0;
        Rotation2d angle = 0.0;
    };

    class SwerveModule {
        public:
        SwerveModule(
            hardware_interface::LoanedCommandInterface &vel_cmd, 
            hardware_interface::LoanedCommandInterface &pos_cmd,
            hardware_interface::LoanedStateInterface &steer_pos_state,
            hardware_interface::LoanedStateInterface &steer_vel_state,
            hardware_interface::LoanedStateInterface &drive_pos_state,
            hardware_interface::LoanedStateInterface &drive_vel_state
        );

        double get_drive_position_feedback();
        double get_drive_velocity_feedback();
        double get_steer_position_feedback();
        double get_steer_velocity_feedback();
        double get_drive_distance();

        void set_module_states(ModuleStates state);
        bool set_position(double pos);
        bool set_velocity(double vel);
        void stop();

        private:
        // commands sent to hardwares
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_cmd; //for drive
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_cmd; //for steer

        // states receieved from hardwares
        std::reference_wrapper<hardware_interface::LoanedStateInterface> steer_pos;
        std::reference_wrapper<hardware_interface::LoanedStateInterface> steer_vel;
        std::reference_wrapper<hardware_interface::LoanedStateInterface> drive_pos;
        std::reference_wrapper<hardware_interface::LoanedStateInterface> drive_vel;
    };
}
#endif