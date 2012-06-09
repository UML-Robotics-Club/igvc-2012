#ifndef ROBOT_EKF_HPP
#define ROBOT_EKF_HPP

#include <kalman/ekfilter.hpp>

class  RobotEKF: public Kalman::EKFilter<double,1> {
public:
  RobotEKF();
  void SetProcGain(double x, double y, double theta);
  void SetProcVar(double x, double y, double theta);
  void SetMeasGain(double x, double y, double theta);
  void SetMeasVar(double x, double y, double theta);
  void Update(double meas_x, double meas_y, double meas_theta,
	      double ctrl_x, double ctrl_y, double ctrl_theta);
protected:
  void makeA();
  void makeH();
  void makeV();
  void makeR();
  void makeW();
  void makeQ();
  void makeProcess();
  void makeMeasure();
private:
  double proc_gain_x, proc_gain_y, proc_gain_theta;
  double proc_var_x, proc_var_y, proc_var_theta;
  double meas_gain_x, meas_gain_y, meas_gain_theta;
  double meas_var_x, meas_var_y, meas_var_theta;
};

//typedef RobotEKF::Vector Vector;
//typedef RobotEKF::Matrix Matrix;

#endif
