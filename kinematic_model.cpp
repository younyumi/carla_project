#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

struct VehicleState{
    float x;
    float y;
    float yaw;
    float velocity;
    float steering_angle;
    float angular_velocity;
};

class KinematicBicycleModel {
public:
    KinematicBicycleModel(float wheelbase, float max_steer, float delta_time = 0.05)
        : wheelbase_(wheelbase), max_steer_(max_steer), delta_time_(delta_time) {}

    VehicleState update(float x, float y, float yaw, float velocity, float acceleration, float steering_angle) {
        float new_velocity = velocity + delta_time_ * acceleration;    //10m/s + 0.1s*0.1m/s^2 = 10.01m/s 

        steering_angle = max(-max_steer_, min(max_steer_, steering_angle));   //steering_angle = 1rad

        float angular_velocity = new_velocity * tan(steering_angle) / wheelbase_;  //10.01m/s * tan(0.1) /3 = 0.334

        float new_x = x + velocity * cos(yaw) * delta_time_;        // 0 + 10m/s * cos(0) * 0.1s = 1m  
        float new_y = y + velocity * sin(yaw) * delta_time_;        // 0 + 10m/s * sin(0) * 0.1s = 0m  
        float new_yaw = normalise_angle(yaw + angular_velocity * delta_time_);  // 0 + 0.334 * 0.1 = 0.0334m

        return VehicleState{new_x, new_y, new_yaw, new_velocity, steering_angle, angular_velocity};
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

int main(){
    float wheelbase = 3;     
    float max_steer = 1;    
    float delta_time = 0.1;  
    KinematicBicycleModel model(wheelbase, max_steer, delta_time);

    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
    float velocity = 10.0; 
    float steering_angle = 0.1;  
    float acceleration = 0.1;    
    float total_time = 10.0;    //simulation time
    int steps = total_time / delta_time;

    for(int i = 0; i < steps ; i++) {
        VehicleState state = model.update(x,y,yaw,velocity,acceleration, steering_angle);
        x = state.x;
        y = state.y;
        yaw = state.yaw;
        velocity = state.velocity;
        steering_angle = state.steering_angle;

        cout << fixed;
        cout.precision(4);


        cout << "Time: " << i * delta_time << " s | ";
        cout << "x: " << x << " | ";
        cout << "y: " << y << " | ";
        cout << "yaw: " << yaw << " | ";
        cout << "velocity: " << velocity << " | ";
        cout << "steering_angle: " << steering_angle << " | ";
        cout << "angular_velocity: " << state.angular_velocity << endl;
    }

    return 0;
}



    