//class 
#include "KinematicBicycle.h"
#include <cmath>

KinematicBicycle::KinematicBicycle(double x, double y, double yaw, double vel) 
    : x(x), y(y), yaw(yaw), vel(vel) {}

void KinematicBicycle::update(double acc, double steer, double dT, double L) {
    double beta = atan(0.35 * tan(steer));  
    x += vel * cos(yaw + beta) * dT;
    y += vel * sin(yaw + beta) * dT;
    yaw += (vel / L) * cos(beta) * tan(steer) * dT;
    vel += acc * dT;
}