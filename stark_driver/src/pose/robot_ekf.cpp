#include "robot_ekf.hpp"
#include <cmath>
#include <iostream>

using namespace std;

RobotEKF::RobotEKF() 
{
  setDim(3, 3, 3, 3, 3);
  proc_gain_x = 0;
  proc_gain_y = 0;
  proc_gain_theta = 0;
  proc_var_x = 0;
  proc_var_y = 0;
  proc_var_theta = 0;
  meas_gain_x = 0;
  meas_gain_y = 0;
  meas_gain_theta = 0;
  meas_var_x = 0;
  meas_var_y = 0;
  meas_var_theta = 0;
}

void RobotEKF::makeA()
{
  A(1,1) = 1.0;
  A(1,2) = 0.0;
  A(1,3) = 0.0;
  
  A(2,1) = 0.0;
  A(2,2) = 1.0;
  A(2,3) = 0.0;
  
  A(3,1) = 0.0;
  A(3,2) = 0.0;
  A(3,3) = 1.0;
}
// W*Q process noise
void RobotEKF::makeW()
{
  W(1,1) = proc_gain_x;
  W(1,2) = 0.0;
  W(1,3) = 0.0;

  W(2,1) = 0.0;
  W(2,2) = proc_gain_y;
  W(2,3) = 0.0;

  W(3,1) = 0.0;
  W(3,2) = 0.0;
  W(3,3) = proc_gain_theta;
}

// process noise variance (should be low)
void RobotEKF::makeQ()
{
  Q(1,1) = proc_var_x;
  Q(1,2) = 0.0;
  Q(1,3) = 0.0;

  Q(2,1) = 0.0;
  Q(2,2) = proc_var_y;
  Q(2,3) = 0.0;

  Q(3,1) = 0.0;
  Q(3,2) = 0.0;
  Q(3,3) = proc_var_theta;
}

void RobotEKF::makeH()
{
  H(1,1) = 1.10;
  H(1,2) = 0.0;
  H(1,3) = 0.0;

  H(2,1) = 0.0;
  H(2,2) = 1.10;
  H(2,3) = 0.0;

  H(3,1) = 0.0;
  H(3,2) = 0.0;
  H(3,3) = 1.10;
}

// observation noise  V*R
void RobotEKF::makeV()
{
  V(1,1) = meas_gain_x;
  V(1,2) = 0.0;
  V(1,3) = 0.0;

  V(2,1) = 0.0;
  V(2,2) = meas_gain_y;
  V(2,3) = 0.0;

  V(3,1) = 0.0;
  V(3,2) = 0.0;
  V(3,3) = meas_gain_theta;
}

// observation noise variance (should be large except theta)
void RobotEKF::makeR()
{
  R(1,1) = meas_var_x;
  R(1,2) = 0.0;
  R(1,3) = 0.0;

  R(2,1) = 0.0;
  R(2,2) = meas_var_y;
  R(2,3) = 0.0;

  R(3,1) = 0.0;
  R(3,2) = 0.0;
  R(3,3) = meas_var_theta;
}

void RobotEKF::makeProcess()
{
  // there is no process. Robot is stationary without control
  /*
  Vector x_(x.size());
  x_(1) = x(1) + dx;
  x_(2) = x(2) + dy;
  x_(3) = x(3) + dtheta;
  x.swap(x_);
  */
}

void RobotEKF::makeMeasure()
{
  // our measured values should be our state values
  z(1) = x(1);
  z(2) = x(2);
  z(3) = x(3);
}

void RobotEKF::SetProcGain(double x, double y, double theta)
{
  proc_gain_x = x;
  proc_gain_y = y;
  proc_gain_theta = theta;
}

void RobotEKF::SetProcVar(double x, double y, double theta)
{
  proc_var_x = x;
  proc_var_y = y;
  proc_var_theta = theta;
}

void RobotEKF::SetMeasGain(double x, double y, double theta)
{
  meas_gain_x = x;
  meas_gain_y = y;
  meas_gain_theta = theta;
}

void RobotEKF::SetMeasVar(double x, double y, double theta)
{
  meas_var_x = x;
  meas_var_y = y;
  meas_var_theta = theta;
}

void Update(double meas_x, double meas_y, double meas_theta,
	    double ctrl_x, double ctrl_y, double ctrl_theta)
{
  RobotEKF::Vector u(3);
  RobotEKF::Vector z(3);
  
  u(1) = ctrl_x;
  u(2) = ctrl_y;
  u(3) = ctrl_theta;
  z(1) = meas_x;
  z(2) = meas_y;
  z(3) = meas_theta;
  //step(u,z);
}
