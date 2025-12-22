#ifndef TRANSLATION_2D_HPP
#define TRANSLATION_2D_HPP

#include <cmath>

class Translation2D {
    private:
    double x = 0.0;
    double y = 0.0;

    public:
    Translation2D();
    Translation2D(double x, double y) :x(x), y(y) {}
    Translation2D(double distance, double angle) {
        x = std::acos(angle) * distance;
        y = std::asin(angle) * distance;
    }

    double x() {
        return x;
    }
    double y() {
        return y;
    }

    double get_angle() {
        return std::atan2(y, x);  
    }

    double get_norm() {
        return std::sqrt(x * x + y * y);
    }

    double get_distance(Translation2D other) {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }

    bool equals(Translation2D other) {
        return x == other.x && y == other.y;
    }

    Translation2D plus(Translation2D other) {
        return Translation2D(x + other.x, y + other.y);
    }

    Translation2D minus(Translation2D other) {
        return Translation2D(x - other.x, y - other.y);
    }

    Translation2D times(double scalar) {
        return Translation2D(x * scalar, y * scalar);
    }

    Translation2D div(double scalar) {
        return Translation2D(x / scalar, y / scalar);
    }

    Translation2D unary_minus() {
        return Translation2D(-x, -y);
    }
};

#endif