#include "arm_commands.h"

namespace ArmCommands
{

/*
 * XXX: These two functions clearly assume that data is in
 * host byte order (ie. little-endian)
 */
void ArmCommand::convertFLoatToByte(const float &v, uint8_t t[4])
{
    uint8_t *ptr = (uint8_t*)&v;
    for (auto i=0; i<4; i++) {
        t[i] = ptr[i];
    }
}

float ArmCommand::convertByteToFloat(uint8_t t[4])
{
    float *ptr = (float*)t;
    return *ptr;
}


command_t 
ArmCommand::serialize()
{
    command_t cm;
    cm.fill(0);
    cm[0] = head;
    cm[47] = tail;
    return cm;
}


command_t
Move3DCommand::serialize()
{
    auto cmdserialized = ArmCommand::serialize();
    cmdserialized[1] = this->cmd;
    cmdserialized[2] = this->subcmd;
    convertFLoatToByte(Xt, &cmdserialized[3]);
    convertFLoatToByte(Yt, &cmdserialized[7]);
    convertFLoatToByte(Zt, &cmdserialized[11]);
    convertFLoatToByte(b0, &cmdserialized[15]);
    convertFLoatToByte(b1, &cmdserialized[19]);
    convertFLoatToByte(w, &cmdserialized[23]);

    return cmdserialized;
}


command_t
LinearInterpCommand::serialize()
{
	this->subcmd = pwmChannel;
	auto cmdser = Move3DCommand::serialize();
    convertFLoatToByte(pwm, &cmdser[27]);
    convertFLoatToByte(speed, &cmdser[43]);

    return cmdser;
}


command_t
LinearInterpAccelCommand::serialize()
{
	this->subcmd = (uint8_t)accelOpt;
	auto cmdser = Move3DCommand::serialize();
	convertFLoatToByte(speed1, &cmdser[27]);
	convertFLoatToByte(accel, &cmdser[39]);
	convertFLoatToByte(speed2, &cmdser[43]);

	return cmdser;
}


command_t
Arc3DInterpCommand::serialize()
{
	this->subcmd = which;
	auto cmds = Move3DCommand::serialize();
	convertFLoatToByte(pwm, &cmds[27]);
	convertFLoatToByte(R, &cmds[39]);
	convertFLoatToByte(speed, &cmds[43]);

	return cmds;
}


DelayCommand::DelayCommand() :
	ArmCommand()
{ head = 0xee; tail = 0xff; }


command_t
DelayCommand::serialize()
{
	auto cmds = ArmCommand::serialize();
	cmds[1] = '6';
	convertFLoatToByte(delay, &cmds[3]);
	return cmds;
}


SetWorktableCoordinateCommand::SetWorktableCoordinateCommand()
{
	head = 0xfc; tail = 0xfd;
}

command_t SetWorktableCoordinateCommand::serialize()
{
	auto cmds = ArmCommand::serialize();
	cmds[1] = a1;
	cmds[2] = a2;
	convertFLoatToByte(X0, &cmds[3]);
	convertFLoatToByte(Y0, &cmds[7]);
	convertFLoatToByte(Z0, &cmds[11]);
	return cmds;
}

} // namespace ArmCommands
