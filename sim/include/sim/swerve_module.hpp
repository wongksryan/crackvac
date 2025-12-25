#ifndef SWERVE_MODULE_HPP
#define SWERVE_MODULE_HPP

#include <string.h>
#include "hardware_interface/loaned_command_interface.hpp"
#include "geometry/rotation2d.hpp"

namespace swerve_drive {
    struct ModuleStates {
        double speed_mps = 0.0;
        Rotation2d angle = 0.0;
    };

    class SwerveModule {
        public:
        SwerveModule(
            hardware_interface::LoanedCommandInterface &vel_cmd, 
            hardware_interface::LoanedCommandInterface &pos_cmd);

        std::optional<double> get_position_feedback();
        std::optional<double> get_velocity_feedback();

        void set_module_states(ModuleStates state);
        bool set_position(double pos);
        bool set_velocity(double vel);
        void zero();
        void stop();
        void reset();

        private:
        //TODO: use relative encoder
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_cmd; //for drive
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_cmd; //for steer
    };
}
#endif