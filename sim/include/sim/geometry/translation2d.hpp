#ifndef TRANSLATION_2D_HPP
#define TRANSLATION_2D_HPP

#include <cmath>

struct Translation2d {
    double x = 0.0;
    double y = 0.0;
    Translation2d() = default;
    Translation2d(double x_, double y_) {
        this->x = x_;
        this->y = y_;
    }

    double get_angle() {
        return std::atan2(y, x);  
    }

    double get_norm() {
        return std::sqrt(x * x + y * y);
    }

    double get_distance(Translation2d other) {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }

    bool equals(Translation2d other) {
        return x == other.x && y == other.y;
    }

    Translation2d plus(Translation2d other) {
        return Translation2d(x + other.x, y + other.y);
    }

    Translation2d minus(Translation2d other) {
        return Translation2d(x - other.x, y - other.y);
    }

    Translation2d times(double scalar) {
        return Translation2d(x * scalar, y * scalar);
    }

    Translation2d div(double scalar) {
        return Translation2d(x / scalar, y / scalar);
    }

    Translation2d unary_minus() {
        return Translation2d(-x, -y);
    }
};

#endif