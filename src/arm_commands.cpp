#include "arm_commands.h"

namespace ArmCommand
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
LinearInterpCommand::serialize()
{
    auto cmd = ArmCommand::serialize();
    cmd[1] = this->cmd;
    cmd[2] = this->pwmChannel;
    convertFLoatToByte(Xt, &cmd[3]);
    convertFLoatToByte(Yt, &cmd[7]);
    convertFLoatToByte(Zt, &cmd[11]);
    convertFLoatToByte(B0, &cmd[15]);
    convertFLoatToByte(B1, &cmd[19]);
    convertFLoatToByte(W, &cmd[23]);
    convertFLoatToByte(pwm, &cmd[27]);
    convertFLoatToByte(speed, &cmd[43]);

    return cmd;
}

} // namespace ArmCommands
