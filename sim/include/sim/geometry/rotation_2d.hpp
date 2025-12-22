#ifndef ROTATION_2D_HPP
#define ROTATION_2D_HPP

#include <cmath>

//rotation in a 2d coordinate frame represented by a point on the unit circle.
class Rotation2D 
{
protected:
    double cos_theta;
    double sin_theta;
public:
    const static double kEpsilon = 1e-9;

    Rotation2D(double x, double y, bool normalize) {
        if (normalize) {
            double hypot = std::hypot(cos_theta, sin_theta);
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
    Rotation2D() : Rotation2D(1, 0, false) {}
    Rotation2D(double x, double y) : Rotation2D(x, y, true) {}
    Rotation2D(double radians) : Rotation2D(std::cos(radians), std::sin(radians), false) {}

    static Rotation2D from_degrees(double degrees) {
        double radians = degrees * M_PI / 180.0;
        return Rotation2D(radians);
    }

    static Rotation2D from_radians(double radians) {
        return Rotation2D(radians);
    }

    static Rotation2D from_rotations(double rotations) {
        double radians = rotations * 2.0 * M_PI;
        return Rotation2D(radians);
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

    Rotation2D minus(Rotation2D other) {
        double theta = get_radians() - other.get_radians();
       return Rotation2D(theta); 
    }

    //adds two rotations together, with the result being bounded between -π and π.
    Rotation2D plus(Rotation2D other) {
        double theta = get_radians() + other.get_radians();
        while(theta > M_PI) theta -= 2.0 * M_PI;
        while(theta < -M_PI) theta += 2.0 * M_PI;
        return Rotation2D(theta);
    }

    //adds the new rotation to the current rotation using a rotation matricos_theta.
    Rotation2D rotate_by(Rotation2D other) {
        return Rotation2D(
            cos_theta * other.cos_theta - sin_theta * other.sin_theta, 
            sin_theta * other.cos_theta + cos_theta * other.sin_theta
        );
    }

    Rotation2D times(double scalar) {
        double theta = get_radians() * scalar;
        return Rotation2D(theta);
    }

    Rotation2D div(double scalar) {
        double theta = get_radians() / scalar;
        return Rotation2D(theta);
    }

    Rotation2D unary_minus() {
        return Rotation2D(-get_radians());
    }
};

#endif