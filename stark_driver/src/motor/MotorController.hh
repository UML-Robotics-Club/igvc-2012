#ifndef MOTOR_CONTROLLER_HH
#define MOTOR_CONTROLLER_HH

#include <string>
#include <boost/thread.hpp>

class MotorController {
  public:
    MotorController(std::string dev);
    ~MotorController();
    
    void set_power(double left, double right);
    void set_aux(bool enable);
    // getting encoder tick count resets the counter
    int get_left_ticks(){return get_ticks('4');}
    int get_right_ticks(){return get_ticks('5');}
  private:
    std::string device_path;
    int fd;
    
    std::string speed_command(int chan, double value);
    void send_command(std::string cmd, std::string resp);
    void send_char(char c);
    int get_ticks(char wheel);

    boost::mutex lock;
    boost::thread flasher_thread;
    bool light_state;
    bool estop;

    void flasher_thread_main();
    void toggle_light();
    bool check_estop();
    int read_digit();
};

#endif
