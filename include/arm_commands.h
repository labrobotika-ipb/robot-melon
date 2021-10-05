#include "unistd.h"
#include <cstdint>
#include <array>


namespace ArmCommand {


typedef std::array<uint8_t,48> command_t;

class ArmCommand
{
    public:
    ArmCommand() {}
    command_t serialize();

    protected:
    /*
     * Fields
     */
    char 
        head = 0xfc,
        tail = 0xfd;

    static void convertFLoatToByte(const float &v, uint8_t *t);
};


/*
 * The linear interpolation command moves the end of the robot arm move in 
 * a straight line from the current position to the position and posture 
 * specified by the command data.
 * This corresponds to G1 command
 */
class LinearInterpCommand: public ArmCommand
{
public:

protected:
    /*
     * Fields
     */
    const char 
        cmd = '1';

public:
    /*
     * PWM channels to be enabled in the end effector.
     * By default it is 0 or 1.
     * Other valid values are 2, 3, 4.
     */
    uint8_t
        pwmChannel = 0;
    /*
     * Coordinates of the end point, measured in mm
     */
    float 
        Xt = 0,
        Yt = 0,
        Zt = 0;
    /*
     * Orientation of the end point
     */
    float
        B0 = 0,
        B1 = 0,
        W = 0;
    /*
     * Closing/opening the claw
     * XXX: valid values are 500-2500 ?
     */
    float pwm = 0;
    /*
     * How fast to move the end point,
     * in mm/min
     */
    float speed = 0;

};

}