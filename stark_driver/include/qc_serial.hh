#ifndef QC_SERIAL_HH
#define QC_SERIAL_HH

#include <string>
using std::string;

void qc_setserial(int m_fd, int baudrate, int databits, const string& parity, 
	const string& stop, bool softwareHandshake, bool hardwareHandshake);

#endif
