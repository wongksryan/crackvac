#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <assert.h>

class PIDController {
    private:
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double dt = 0.0; // period in seconds
    double error_tolerance = 0.0;
    double error_derivative_tolerance = std::numeric_limits<double>::infinity();
    double minimum_input = 0.0;
    double maximum_input = 0.0;
    double minimum_integral = 0.0;
    double maximum_integral = 0.0;
    double i_zone = 0.0;
    bool is_continuous = false;
    bool has_setpoint = false;
    bool has_measurement = false;

    double measurement = 0.0; // input
    double setpoint = 0.0; // target
    double error = 0.0;
    double prev_error = 0.0;
    double error_accumulated = 0.0; 
    double error_derivative = 0.0;

    public:
    PIDController(double kP, double kI, double kD, double dt) 
        : kP(kP), kI(kI), kD(kD), dt(dt) {};
    
    PIDController(double kP, double kI, double kD) 
        : PIDController(kP, kI, kD, 0.0) {};

    void set_tolerances(double error_tolerance, 
        double error_derivative_tolerance = std::numeric_limits<double>::infinity()) {
        this->error_tolerance = error_tolerance;
        this->error_derivative_tolerance = error_derivative_tolerance;
    }

    // when the absolute value of position error is greater than i_zone,
    // the total accumulated error is reset to zero.
    // disables the integral gain until the absolute value of position error is less than i_zone.
    void set_i_zone(double i_zone) {
        assert(i_zone >= 0);
        this->i_zone = i_zone;
    }

    void enable_continuous_input(double min_input, double max_input) {
        is_continuous = true;
        minimum_input = min_input;
        maximum_input = max_input;
    }

    void disable_continuous_input() {
        is_continuous = false;
    }

    void set_integral_range(double min_integral, double max_integral) {
        minimum_integral = min_integral;
        maximum_integral = max_integral;
    }

    double calculate(double measurement) {
        this->measurement = measurement;
        prev_error = error;
        has_measurement = true;

        if (is_continuous) {
            double error_bound = (maximum_input - minimum_input) / 2.0;
            double min = -error_bound;
            double max = error_bound;
            double range = max - min;
            double error = std::fmod(error - min, range);
            if (error < 0) {
                error += range;
            }
            error + min; 
        } else {
            error = setpoint - measurement;
        }

        error_derivative = (error - prev_error) / dt;

        if (std::abs(error) > i_zone) {
            error_accumulated = 0;
        } else if (kI != 0) {
            double value = error_accumulated + error * dt;
            double low = minimum_integral / kI;
            double high = maximum_integral / kI;
            error_accumulated = std::clamp(value, low, high);
        }

        return kP * error + kI * error_accumulated + kD * error_derivative;
    }

    double calculate(double measurement, double setpoint) {
        this->setpoint = setpoint;
        has_setpoint = true;
        return calculate(measurement);
    }

    void reset() {
        error = 0;
        prev_error = 0;
        error_accumulated = 0;
        error_derivative = 0;
        has_measurement = false;
    }

    bool at_setpoint() {
        return has_measurement && has_setpoint && 
            std::abs(error) < error_tolerance && 
            std::abs(error_derivative) < error_derivative_tolerance;
    }

    double get_error() {
        return error;
    }
};

#endif