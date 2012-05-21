/* -*- c++ -*- */
#ifndef MCP_DRIVER_HH
#define MCP_DRIVER_HH

#include <vector>
using std::vector;

#include <deque>
using std::deque;

#include <string>
using std::string;

#include "MotorController.hh"
#include "croak.hh"

class MotorDriver {
  public:
    MotorDriver(std::string dev);
    ~MotorDriver();

    void SetSpeed(double speed, double turn);
    inline void SetAux(bool enable) {
        if (motor) 
            motor->set_aux(enable);
    }

  private:    
    void ReadCompass();
    void SendMotorCommand();
    double SpeedToPow(double speed);

    double speed;
    double rotsp;
    
    double min_power;

    deque<double> speed_history;
    double last_speed;

    MotorController* motor;
    std::string motor_dev;
};

#endif

