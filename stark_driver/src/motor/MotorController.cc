
#include <iostream>
using std::cout;
using std::endl;

#include <ios>
#include <sstream>
#include <cstdio>
#include <iomanip>
#include <exception>
#include <cmath>
#include <cstdlib>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <signal.h>

#include "croak.hh"
#include "MotorController.hh"

#include "qc_serial.hh"
#include "thread_fsleep.hh"

int motor_fd;
MotorController* mc_instance = 0;;

void 
got_signal(int n, siginfo_t*, void*) 
{
    printf("Got SIGINT, cleanup.\n");
    if (mc_instance) {
        mc_instance->set_power(0.0, 0.0);
        mc_instance->set_aux(false);
    }
    close(motor_fd);
    exit(0);
}

MotorController::MotorController(std::string dev)
    : device_path(dev), light_state(false), estop(false)
{
    boost::mutex::scoped_lock lock_(lock);

    if (mc_instance)
        throw new std::exception();
    mc_instance = this;

    struct sigaction sa;
    sa.sa_sigaction = got_signal;

    sigaction(SIGINT, &sa, 0);
    sigaction(SIGTERM, &sa, 0);

    // real code
    printf("Opening motor controller on %s\n", device_path.c_str());
    fd = open(device_path.c_str(), O_SYNC | O_RDWR | O_NDELAY);
    check_syscall("open", fd);

    motor_fd = fd;
    printf("(1) fd = %d\n", fd);

    int n = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, n & ~O_NDELAY);

    qc_setserial(fd, 9600, 7, "Even", "1", false, false);

    check_syscall("tcdrain", tcdrain(fd));
	check_syscall("tcflush", tcflush(fd, TCIOFLUSH));

    send_command("\r\r\r\r\r\r\r\r\r\r", "OK");
    send_command("!c", "+");

    flasher_thread = boost::thread(
        boost::bind(&MotorController::flasher_thread_main, this));

}

MotorController::~MotorController()
{
    boost::mutex::scoped_lock lock_(lock);
    flasher_thread.interrupt();
    close(fd);
}

void 
MotorController::set_power(double left, double right) 
{
    boost::mutex::scoped_lock lock_(lock);

    std::string ch1 = speed_command(1, left);
    std::string ch2 = speed_command(2, right);

    send_command(ch1, "+");
    send_command(ch2, "+");

}

void 
MotorController::set_aux(bool enable) 
{
    boost::mutex::scoped_lock lock_(lock);

    if (enable) {
        send_command("!C", "+");
    }
    else {
        send_command("!c", "+");
    }
}

// The encoder uses this compression scheme
// that's supposed to reduce the number of bytes
// sent on the serial port. For some stupid reason
// the encoder wastes 4 bytes repeating your command
// -avardaro
int 
MotorController::get_ticks(char wheel)
{
  boost::mutex::scoped_lock lock_(lock);
  char resp;

  int val = 0;
  bool is_neg = false;
  int nibbles_read = 0;
  std::stringstream ss;
  // query resets tick count
  send_char('?');
  send_char('q');
  send_char(wheel);
  send_char('\r');
  send_char('\n');
  
  // get the echo
  // Ok, so sometimes it prepends the echo with a '\r' and
  // sometimes it doesn't.
  
  check_syscall("read", read(fd, &resp, 1)); // '?' or '\r'
  if(resp == '\r')
    check_syscall("read", read(fd, &resp, 1)); // '?'
  check_syscall("read", read(fd, &resp, 1)); // 'q''
  check_syscall("read", read(fd, &resp, 1)); // wheel
  check_syscall("read", read(fd, &resp, 1)); // '\r'

  // Read the first value. Unless we use all 8 digits, 
  // the first value tells you if the
  // number is positive or negetive.
  // 0-7: The ticks are positive
  // 8-F: The ticks are negetive
  check_syscall("read", read(fd, &resp, 1));
  ss << std::hex << resp;
  if(resp >='8')
    is_neg = true;
  
  // we already read in one nibble so we start at 1
  // we want to make sure we read the terminating '\r',
  // so we read in 8 more nibbles
  for(nibbles_read = 1; nibbles_read < 9; nibbles_read++){
    check_syscall("read", read(fd, &resp, 1));
    if(resp == '\r')
      break;
    ss << resp;
  }
  ss >> val;
  if(is_neg) // set the rest of the bits high
    val = val | (-1 << (4*(nibbles_read)));
  return val;
}

std::string 
MotorController::speed_command(int chan, double value)
{
    int  v = int(fabs(127 * value));
    char c = 'A' + chan - 1;

    if(value < 0)
        c = tolower(c);

    std::ostringstream cmd;
    cmd << "!" << c;
    cmd << std::hex << std::setw(2) << std::setfill('0');
    cmd << std::uppercase << v;
    return cmd.str();
}

void 
MotorController::send_char(char cc) 
{
    check_syscall("write", write(fd, &cc, 1))
    //check_syscall("tcdrain", tcdrain(fd));
}

void
MotorController::send_command(std::string cmd, std::string resp) 
{
    unsigned int i, p = 0;
    const char* cs = cmd.c_str();
    const char* rs = resp.c_str();
    char c;

    // Send the command.
    printf("(Motors) Sending command: %s\n", cmd.c_str());

    for(i = 0; i < cmd.size(); ++i) {
        send_char(cs[i]);
    }
    send_char('\r');
    send_char('\n');
    
    // Wait for the response.
    while(p < resp.size()) {
        check_syscall("read", read(fd, &c, 1));
        //printf("(motor) got byte: %d (%c)\n", c, c);

        if(c == rs[p] || c == 'W')
            p += 1;
    }
}

void 
MotorController::flasher_thread_main()
{
    while (1) {
        if (check_estop()) {
            cout << "estop: on" << endl;
            light_state = true;
            set_aux(light_state);
        }
        else {
            cout << "estop: off" << endl;
            toggle_light();
        }
        
        thread_fsleep(0.5); 
    }
}

int
MotorController::read_digit()
{
    char ch[2];
	ch[0] = 0; ch[1] = 0;

    while (1) {
        check_syscall("read", read(fd, ch, 1));
        
        if (isdigit(ch[0])) {
            return atoi(ch);
        }
    }
}

bool
MotorController::check_estop()
{
    boost::mutex::scoped_lock lock_(lock);

    send_char('?');
    send_char('i');
    send_char('\r');
    send_char('\n');

    int nums[6];
    for (int ii = 0; ii < 6; ++ii) {
        nums[ii] = read_digit();
    }

    return nums[5] == 0;
}

void
MotorController::toggle_light()
{
    light_state = !light_state;
    set_aux(light_state);
}

