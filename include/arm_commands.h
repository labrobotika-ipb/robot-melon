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

    static void convertFLoatToByte(const float &v, uint8_t t[4]);
    static float convertByteToFloat(uint8_t t[4]);
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
    LinearInterpCommand() : ArmCommand()
    { head = 0xee; tail = 0xef; }

    command_t serialize();

protected:
    const char 
        cmd = '1';

/*
 * Fields
 */
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


/*
 * Linear interpolation positioning with acceleration & deceleration
 * (corresponding to G220, G222 and G230)
 */
class LinearInterpAccelCommand: public ArmCommand
{
public:

protected:
    const char
        cmd = '2';

// Fields
public:
    enum accelOpt {
        SINGLE,             // Accelerate/decelerate from 0 to speed1.
                            // To decelerate, specify negative value to accel
        ACCEL_THEN_DECEL,   // Accelerate from 0 to speed1 then decelerate to 0.
                            // accel must be positive.
        ABSOLUTE            // Accelerate from speed1 to speed2.
                            // accel must be positive
    };

    float
        // Coordinates of the end point, measured in mm
        Xt = 0,
        Yt = 0,
        Zt = 0;
    float
        // Orientation of the end point
        B0 = 0,
        B1 = 0,
        W = 0;

    float
        speed1 = 0,         // initial speed (mm/s ?)
        accel = 0,          // acceleration. range is [-3200,+3200] (mm/s^2)
        speed2 = 0;         // final speed (mm/s ?)
};


/*
 * 3D arc interpolation command
 * (corresponding to G300, G301)
 */

}