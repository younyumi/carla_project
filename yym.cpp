#include <iostream>
#include <cmath>
#include <algorithm>
#include <tuple>

class KinematicBicycleModel {
public:
   
    KinematicBicycleModel(float wheelbase, float max_steer, float delta_time = 0.05)
        : wheelbase_(wheelbase), max_steer_(max_steer), delta_time_(delta_time) {}

    std::tuple<float, float, float, float, float, float> update(float x, float y, float yaw, float velocity, float acceleration, float steering_angle) {
        float new_velocity = velocity + delta_time_ * acceleration;

        steering_angle = std::max(-max_steer_, std::min(max_steer_, steering_angle));

        float angular_velocity = new_velocity * std::tan(steering_angle) / wheelbase_;

        float new_x = x + velocity * std::cos(yaw) * delta_time_;
        float new_y = y + velocity * std::sin(yaw) * delta_time_;
        float new_yaw = normalise_angle(yaw + angular_velocity * delta_time_);

        return std::make_tuple(new_x, new_y, new_yaw, new_velocity, steering_angle, angular_velocity);
    }

private:
    float wheelbase_;
    float max_steer_;
    float delta_time_;

    float normalise_angle(float angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};