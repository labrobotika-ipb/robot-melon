/*
 * Serial.cpp
 *
 *  Created on: Oct 7, 2021
 *      Author: sujiwo
 */

#include <unistd.h>
#include <fcntl.h>
#include "SerialPort.h"

using namespace std;


SerialPort::SerialPort()
{}


SerialPort::~SerialPort()
{
	if (uniFd!=-1)
		close(uniFd);
}


bool SerialPort::init(const string &portname)
{
	m_port = portname;
	uniFd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

}
