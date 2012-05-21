
#include <cmath>
#include <cstring>
#include <iostream>

#include "fsleep.hh"
#include "MotorDriver.hh"

const double MAX_POWER      = 1.0;
const double MAX_SPEED      = 1.58;
const double MIN_POWER_BASE = 0.2;
const double WHEEL_WIDTH    = 0.5;
const double TURN_FACTOR    = WHEEL_WIDTH * 0.50;
const double TURN_POWER     = 1.6; 

const double BREAK_THRESHOLD = 0.5;

const int SPEED_HISTORY_SIZE = 10;

MotorDriver::MotorDriver(std::string dev)
    : min_power(MIN_POWER_BASE), speed_history(SPEED_HISTORY_SIZE, 0.0), 
      last_speed(0.0), motor_dev(dev)
{
    motor = new MotorController(motor_dev);
}

MotorDriver::~MotorDriver() 
{
    delete motor;
}

void
MotorDriver::SetSpeed(double speed, double turn)
{
    this->speed = speed;
    this->rotsp = turn;
    SendMotorCommand();
}

inline static 
double 
fsign(double a) 
{
    return (a > 0) ? 1.0 : -1.0;
}

void 
MotorDriver::SendMotorCommand() 
{
    //Experimental (awesome) active breaking stuff
    speed_history.pop_front();
    speed_history.push_back(last_speed);
    
    double sum = 0;
    
    for (int i = 1; i <= SPEED_HISTORY_SIZE; i++) {
        sum += speed_history[i - 1];
    }
    
    double avg = sum / (double)SPEED_HISTORY_SIZE;
    
    if (avg - speed > BREAK_THRESHOLD && avg > 0.0) {
        std::cout << "Breaking by " << (avg - speed) / 1.0 << " avg : " << avg << std::endl;
        speed = speed - (avg - speed) / 1.0;
    }
    
    last_speed = speed;
    
    // The left and right wheels are about 0.5 meters apart.
    // So if we want to turn left at 1.0 rad/sec, the right
    // wheels need to be going (1.0 * 0.5 =) 0.5 m/s faster
    // than the left wheels.
    
    // To go straight, both sides go at the same speed.
    double left_speed = speed;
    double right_speed = speed;
    
    // To turn in direction D, we slow down the wheels
    // on the D side and speed up the wheels on the other.
    // 
    // Positive rotsp is a left turn.
    
    double turn_delta = rotsp * TURN_FACTOR * 
        pow(fabs(speed) + 1.0, TURN_POWER);
    
    if(fabs(turn_delta) > MAX_SPEED)
        turn_delta = fsign(turn_delta) * MAX_SPEED;
    
    right_speed += turn_delta;
    left_speed  -= turn_delta;
    
    // rescale if over max speed
    if(fabs(right_speed) > MAX_SPEED || fabs(left_speed) > MAX_SPEED) {
        double excess = fmax(
            fabs(right_speed) / MAX_SPEED, 
            fabs(left_speed)  / MAX_SPEED
        );
        right_speed /= excess;
        left_speed  /= excess;
    }
    
    double left_pow  = SpeedToPow(left_speed);
    double right_pow = SpeedToPow(right_speed);
    
    //printf("For speed, rotsp: %.02f, %.02f\n", speed, rotsp);
    //printf("Goal speeds L, R: %.02f, %.02f\n", left_speed, right_speed);
    //printf("Set Power: %.02f, %.02f\n", left_pow, right_pow);
    motor->set_power(left_pow, right_pow);    
}

double 
MotorDriver::SpeedToPow(double speed) 
{
    // Motor power is approximated as follows:
    // - There is some minimum power min_power that
    //   breaks friction with the ground.
    // - There is some maximum power MAX_POWER that
    //   causes the robot to go the maximum speed.
    // - Those values are determined experimentally.
    // - In the middle, a linear approximation
    //   is used.

    // Zero speed, no power.
    if(fabs(speed) < 0.01)
        return 0.0;
    
    // Otherwise, proportional power in the valid range.
    double range = MAX_POWER - min_power;
    double value = speed / MAX_SPEED;
    
    if(speed > 0)
        return min_power + range * value;
    else
        return -min_power + range * value;
}
