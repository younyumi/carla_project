#ifndef KINEMATICBICYCLE_H  
#define KINEMATICBICYCLE_H

class KinematicBicycle {
public:
    double x, y, yaw, vel;
    KinematicBicycle(double x, double y, double yaw, double vel);
    void update(double acc, double steer, double dT, double L);
};

#endif 