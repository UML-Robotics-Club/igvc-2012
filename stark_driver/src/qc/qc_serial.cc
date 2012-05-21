#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>

#include "qc_serial.hh"

void qc_setserial(int m_fd, int baudrate, int databits, const string& parity, 
	const string& stop, bool softwareHandshake, bool hardwareHandshake)
{
   struct termios newtio;

   if (tcgetattr(m_fd, &newtio)!=0)
   {
      std::cerr<<"tcgetattr() 3 failed"<<std::endl;
   }

   speed_t _baud=0;
   switch (baudrate)
   {
#ifdef B0
   case      0: _baud=B0;     break;
#endif
   
#ifdef B50
   case     50: _baud=B50;    break;
#endif
#ifdef B75
   case     75: _baud=B75;    break;
#endif
#ifdef B110
   case    110: _baud=B110;   break;
#endif
#ifdef B134
   case    134: _baud=B134;   break;
#endif
#ifdef B150
   case    150: _baud=B150;   break;
#endif
#ifdef B200
   case    200: _baud=B200;   break;
#endif
#ifdef B300
   case    300: _baud=B300;   break;
#endif
#ifdef B600
   case    600: _baud=B600;   break;
#endif
#ifdef B1200
   case   1200: _baud=B1200;  break;
#endif
#ifdef B1800
   case   1800: _baud=B1800;  break;
#endif
#ifdef B2400
   case   2400: _baud=B2400;  break;
#endif
#ifdef B4800
   case   4800: _baud=B4800;  break;
#endif
#ifdef B7200
   case   7200: _baud=B7200;  break;
#endif
#ifdef B9600
   case   9600: _baud=B9600;  break;
#endif
#ifdef B14400
   case  14400: _baud=B14400; break;
#endif
#ifdef B19200
   case  19200: _baud=B19200; break;
#endif
#ifdef B28800
   case  28800: _baud=B28800; break;
#endif
#ifdef B38400
   case  38400: _baud=B38400; break;
#endif
#ifdef B57600
   case  57600: _baud=B57600; break;
#endif
#ifdef B76800
   case  76800: _baud=B76800; break;
#endif
#ifdef B115200
   case 115200: _baud=B115200; break;
#endif
#ifdef B128000
   case 128000: _baud=B128000; break;
#endif
#ifdef B230400
   case 230400: _baud=B230400; break;
#endif
#ifdef B460800
   case 460800: _baud=B460800; break;
#endif
#ifdef B576000
   case 576000: _baud=B576000; break;
#endif
#ifdef B921600
   case 921600: _baud=B921600; break;
#endif
   default:
//   case 256000:
//      _baud=B256000;
      break;
   }
   cfsetospeed(&newtio, (speed_t)_baud);
   cfsetispeed(&newtio, (speed_t)_baud);

   /* We generate mark and space parity ourself. */
   if (databits == 7 && (parity=="Mark" || parity == "Space"))
   {
      databits = 8;
   }
   switch (databits)
   {
   case 5:
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS5;
      break;
   case 6:
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS6;
      break;
   case 7:
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS7;
      break;
   case 8:
   default:
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
      break;
   }
   newtio.c_cflag |= CLOCAL | CREAD;

   //parity
   newtio.c_cflag &= ~(PARENB | PARODD);
   if (parity == "Even")
   {
      newtio.c_cflag |= PARENB;
   }
   else if (parity== "Odd")
   {
      newtio.c_cflag |= (PARENB | PARODD);
   }

   //hardware handshake
/*   if (hardwareHandshake)
      newtio.c_cflag |= CRTSCTS;
   else
      newtio.c_cflag &= ~CRTSCTS;*/
   newtio.c_cflag &= ~CRTSCTS;

   //stopbits
   if (stop=="2")
   {
      newtio.c_cflag |= CSTOPB;
   }
   else
   {
      newtio.c_cflag &= ~CSTOPB;
   }

//   newtio.c_iflag=IGNPAR | IGNBRK;
   newtio.c_iflag=IGNBRK;
//   newtio.c_iflag=IGNPAR;

   //software handshake
   if (softwareHandshake)
   {
      newtio.c_iflag |= IXON | IXOFF;
   }
   else
   {
      newtio.c_iflag &= ~(IXON|IXOFF|IXANY);
   }

   newtio.c_lflag=0;
   newtio.c_oflag=0;

   newtio.c_cc[VTIME]=1;
   newtio.c_cc[VMIN]=60;

//   tcflush(m_fd, TCIFLUSH);
   if (tcsetattr(m_fd, TCSANOW, &newtio)!=0)
   {
      std::cerr<<"tcsetattr() 1 failed"<<std::endl;
   }

   int mcs=0;
   ioctl(m_fd, TIOCMGET, &mcs);
   mcs |= TIOCM_RTS;
   ioctl(m_fd, TIOCMSET, &mcs);

   if (tcgetattr(m_fd, &newtio)!=0)
   {
      std::cerr<<"tcgetattr() 4 failed"<<std::endl;
   }

   //hardware handshake
   if (hardwareHandshake)
   {
      newtio.c_cflag |= CRTSCTS;
   }
   else
   {
      newtio.c_cflag &= ~CRTSCTS;
   }
/*  if (on)
     newtio.c_cflag |= CRTSCTS;
  else
     newtio.c_cflag &= ~CRTSCTS;*/
   if (tcsetattr(m_fd, TCSANOW, &newtio)!=0)
   {
      std::cerr<<"tcsetattr() 2 failed"<<std::endl;
   }

}

