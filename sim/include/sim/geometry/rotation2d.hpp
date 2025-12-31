#ifndef ROTATION_2D_HPP
#define ROTATION_2D_HPP

#include <cmath>

//rotation in a 2d coordinate frame represented by a point on the unit circle.
struct Rotation2d 
{
    double cos_theta = 1.0;
    double sin_theta = 0.0;

    constexpr static double kEpsilon = 1e-9;
    Rotation2d() = default;
    Rotation2d(double x, double y, bool normalize) {
        if (normalize) {
            double hypot = std::hypot(x, y);
            if (hypot > kEpsilon) {
                cos_theta = x / hypot;
                sin_theta = y / hypot;
            } else {
                cos_theta = 1;
                sin_theta = 0;
            }
        } else {
            cos_theta = x;
            sin_theta = y;
        }
    }
    Rotation2d(double x, double y) : Rotation2d(x, y, true) {}
    Rotation2d(double radians) : Rotation2d(std::cos(radians), std::sin(radians), false) {}

    static Rotation2d from_degrees(double degrees) {
        double radians = degrees * M_PI / 180.0;
        return Rotation2d(radians);
    }

    static Rotation2d from_radians(double radians) {
        return Rotation2d(radians);
    }

    static Rotation2d from_rotations(double rotations) {
        double radians = rotations * 2.0 * M_PI;
        return Rotation2d(radians);
    }

    double cos() {
        return cos_theta; 
    }

    double sin() {
        return sin_theta; 
    }

    double tan() {
        if (std::abs(cos_theta) < kEpsilon) {
            if (sin_theta >= 0.0) {
                return 1.0 / 0.0; //positive infinity
            } else {
                return -1.0 / 0.0; //negative infinity
            }
        } else {
            return sin_theta / cos_theta;
        }
    }

    double get_radians() {
        return std::atan2(sin_theta, cos_theta); 
    }

    double get_degrees() {
        return get_radians() * 180 / M_PI;
    }

    double get_rotations() {
        return get_radians() / (2.0 * M_PI);
    }

    Rotation2d minus(Rotation2d other) {
        double theta = get_radians() - other.get_radians();
       return Rotation2d(theta); 
    }

    //adds two rotations together, with the result being bounded between -π and π.
    Rotation2d plus(Rotation2d other) {
        double theta = get_radians() + other.get_radians();
        while(theta > M_PI) theta -= 2.0 * M_PI;
        while(theta < -M_PI) theta += 2.0 * M_PI;
        return Rotation2d(theta);
    }

    //adds the new rotation to the current rotation using a rotation matricos_theta.
    Rotation2d rotate_by(Rotation2d other) {
        return Rotation2d(
            cos_theta * other.cos_theta - sin_theta * other.sin_theta, 
            sin_theta * other.cos_theta + cos_theta * other.sin_theta
        );
    }

    Rotation2d times(double scalar) {
        double theta = get_radians() * scalar;
        return Rotation2d(theta);
    }

    Rotation2d div(double scalar) {
        double theta = get_radians() / scalar;
        return Rotation2d(theta);
    }

    Rotation2d unary_minus() {
        return Rotation2d(-get_radians());
    }
};

#endif