#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "geometry/translation2d.hpp"

namespace constants {
    const double wheel_radius_meter = 0.04;   
    const double wheel_base_half_meter = 0.4064 / 2; //wheel base: 16 inch
    const Translation2d module_offsets[4] = {
        Translation2d(-wheel_base_half_meter, wheel_base_half_meter),  //front left
        Translation2d(wheel_base_half_meter, wheel_base_half_meter),  //front right
        Translation2d(-wheel_base_half_meter, -wheel_base_half_meter),  //back left
        Translation2d(wheel_base_half_meter, -wheel_base_half_meter)   //back right
    };
}

#endif