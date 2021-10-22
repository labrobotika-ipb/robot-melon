/*
 * Serial.h
 *
 *  Created on: Oct 7, 2021
 *      Author: sujiwo
 */

#ifndef INCLUDE_SERIALPORT_H_
#define INCLUDE_SERIALPORT_H_

#include <string>
#include <cstdint>

class SerialPort
{
public:
	SerialPort();
	virtual ~SerialPort();

	bool init
	(const std::string &port);

	void read();
	void write(uint8_t *data, size_t len);

private:
	int uniFd = -1;
	std::string m_port;
	int m_baud;
	int m_dataBits;
	int m_stopBits;
	bool m_parity;
	bool m_hardwareControl;

};

#endif /* INCLUDE_SERIALPORT_H_ */
