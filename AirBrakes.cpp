#include "AirBrakes.h"
#include "Arduino.h"

Parameters::Parameters(int setpoint, int Kp, int Ki, int Kd, double rocket_area, double AB_area, double rocket_CD, double AB_CD) {
  _setpoint = setpoint;
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _rocket_area = rocket_area;
  _AB_area = AB_area;
  _rocket_CD = rocket_CD;
  _AB_CD = AB_CD;
}

Rocket::Rocket(double height, double velocity, double vertVelocity, double mass) {
    _height = height;
    _velocity = velocity;
    _vertVelocity = vertVelocity;
    _mass = mass;
}

Miscellaneous::Miscellaneous(int step, double inter, double diff, double prev_err, double prev_predApogee, double tau) {
    _step = step;
    _inter = inter;
    _diff = diff;
    _prev_err = prev_err;
    _prev_predApogee = prev_predApogee;
    _tau = tau;
}

double AirBrakes::airDensity(double altitude)
/**
 * Using a linear approximation (Y = -1.05^{-4} + 1.225) finds the air density for the given altitude.
 * 
 * @param altitude
 * 
 * @return The air density*/{
    return -0.000105 * altitude + 1.225;
}

double AirBrakes::dragSurface(double altitude, double velocity, double area, double C_drag)
/**
 * Finds the drag force of a surfaces given the current velocity, 
 * altitude and area.
 * 
 * @return The drag given altitude, velocity, and area.*/{
    return (C_drag * airDensity(altitude) * velocity * velocity * area / 2);
}

double AirBrakes::requiredDrag(Rocket rocket, Parameters param, Miscellaneous misc)
/**
 * Computes required drag using a PID controller.
 * 
 * @param rocket structure that contains rocket variables;
 * @param param parameters of the simulation;
 * @param misc miscellaneous variables to be updates TODO: categorize correctly
 * 
 * @return required drag
 */{
  double out = 0;
  double alt = rocket._height;
  double velocity = rocket._velocity;
  double vertVelocity = rocket._vertVelocity;
  double mass = rocket._mass;

  double gravity = 9.80665;
  double refArea = param._rocket_area;

  double termVelocity = sqrt((2 * mass * gravity) / (param._rocket_CD * refArea * 1.225));
  double predApogee = alt + (((pow(termVelocity, 2)) / (2 * gravity)) * log((pow(vertVelocity, 2) + pow(termVelocity, 2)) / (pow(termVelocity, 2))));

  double minInte = dragSurface(predApogee, velocity, param._rocket_area, 0.5);
  double maxInte = dragSurface(predApogee, velocity, param._AB_area, 1.17) + minInte;

  double err = param._setpoint - predApogee;

  double prop = -param._Kp * err;

  misc._inter += 0.5 * param._Ki * misc._step * (err + misc._prev_err);

  misc._diff = (-2 * param._Kd * (predApogee - misc._prev_predApogee) + (2 * misc._tau - misc._step) * misc._diff) / (2 * misc._tau + misc._step);

  if (misc._inter > maxInte)
    misc._inter = maxInte;
  else if (misc._inter < minInte)
    misc._inter = minInte;

  out = prop + misc._inter + misc._diff;

  misc._prev_err = err;
  misc._prev_predApogee = predApogee;

  return out;
}

double AirBrakes::airBrakeArea(double drag, double velocity, double airDensity, double AB_CD)
/**
 * Using the drag formula we can calculate the area of the air brake
 * 
 * @return air brake Area*/{
  return (drag * 2 / (velocity * velocity * airDensity * AB_CD));
}

double AirBrakes::airbrakeForce(Rocket rocket, Parameters param, Miscellaneous misc) {
  double reqDrag = requiredDrag(rocket, param, misc);
  double maxDrag = dragSurface(rocket._height, rocket._velocity, param._AB_area, param._AB_CD);
  if (reqDrag > maxDrag) {
    reqDrag = maxDrag;
  } else if (reqDrag < 0) {
    reqDrag = 0;
  }
  return reqDrag;
}

int AirBrakes::calculateServoAngle(Parameters param, double area)
/**
 * Using a linear approximation (Y = 40142.73 * area) finds the servo angle for the given area.
 * 
 * @param area AB area
 * 
 * @return servo angle for the given area*/{
    if (area < 0)
        return 0;
    if (area > param._rocket_area)
        return 180;
    return 40142.73 * area;
}
