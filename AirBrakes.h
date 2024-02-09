/*
  AirBrakes.h - Library for calculatin air brake force/area/servo
  angle bassed on a PID system.
  Created by Filipe Cavalheiro, February, 2024.
  MIT LICENSE APPLIES.
*/
#ifndef AIRBRAKES_h
#define AIRBRAKES_h

#include "Arduino.h"

class Parameters {
public:
    Parameters(int setpoint, int Kp, int Ki, int Kd, double rocket_area, double AB_area, double rocket_CD, double AB_CD);
    int _setpoint;
    int _Kp;
    int _Ki;
    int _Kd;
    double _rocket_area;
    double _AB_area;
    double _rocket_CD;
    double _AB_CD;
};

class Rocket {
public:
    Rocket(double height, double velocity, double vertVelocity, double mass);
    double _height;
    double _velocity;
    double _vertVelocity;
    double _mass;
};

class Miscellaneous {
public:
    Miscellaneous(int step, double inter, double diff, double prev_err, double prev_predApogee, double tau);
    int _step;
    double _inter;
    double _diff;
    double _prev_err;
    double _prev_predApogee;
    double _tau;
};

class AirBrakes {
public:
    double airbrakeForce(Rocket rocket, Parameters param, Miscellaneous misc);
    double airBrakeArea(double drag, double velocity, double airDensity, double AB_CD);
    int calculateServoAngle(Parameters param, double area);
    double airDensity(double altitude);
  private:
    double dragSurface(double altitude, double velocity, double area, double C_drag);
    double requiredDrag(Rocket rocket, Parameters param, Miscellaneous misc);
};

#endif /*AIRBRAKES*/