#include <iostream>
#include <cstring>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "arm_commands.h"

using namespace ArmCommands;
using namespace std;

int main(int argc, char *argv[])
{
	boost::system::error_code ignored_error;

	boost::asio::io_context io_ctx;
	boost::asio::serial_port serial(io_ctx, "/dev/ttyUSB0");
	serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
	serial.set_option(boost::asio::serial_port_base::character_size(8));
	serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

	SetWorktableCoordinateCommand work;
	auto s1 = work.serialize();

	DelayCommand delay;
	delay.delay = 14;

	LinearInterpAccelCommand moveAccel;
	moveAccel.Xt = 209.9;
	moveAccel.Yt = 0.0;
	moveAccel.Zt = 424.7;
	moveAccel.b0 = 0.0;
	moveAccel.b1 = 29.22;
	moveAccel.w = 0.03;
	moveAccel.accelOpt = LinearInterpAccelCommand::ACCEL_THEN_DECEL;
	moveAccel.speed1 = 4000;
	moveAccel.accel = 1200;

	LinearInterpCommand move;
	move.Xt = 209.9;
	move.Yt = 0.0;
	move.Zt = 424.7;
	move.b0 = 0.0;
	move.b1 = 29.22;
	move.w = 0.03;
	move.speed = 4000;
	move.pwm = 2500;

	boost::asio::write(serial,
		boost::asio::buffer(work.serialize(), CommandLength),
		ignored_error);
	boost::asio::write(serial,
		boost::asio::buffer(delay.serialize(), CommandLength),
		ignored_error);
	boost::asio::write(serial,
		boost::asio::buffer(moveAccel.serialize(), CommandLength),
		ignored_error);
	boost::asio::write(serial,
		boost::asio::buffer(move.serialize(), CommandLength),
		ignored_error);

	return 0;
}
