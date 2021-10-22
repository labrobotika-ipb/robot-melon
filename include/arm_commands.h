#include "unistd.h"
#include <cstdint>
#include <array>


namespace ArmCommands {


const int CommandLength = 48;


typedef std::array<uint8_t,CommandLength> command_t;

class ArmCommand
{
    public:
    ArmCommand() {}
    virtual command_t serialize();

    protected:
    /*
     * Fields
     */
    unsigned char
        head = 0xfc,
        tail = 0xfd;

    static void convertFLoatToByte(const float &v, uint8_t t[4]);
    static float convertByteToFloat(uint8_t t[4]);
};


/*
 * Base command for 3D movement
 */
class Move3DCommand : public ArmCommand
{
public:
	Move3DCommand() :
		ArmCommand()
	{ head = 0xee; tail = 0xff; }

	virtual command_t serialize();

protected:
	// Derived classes must define these two fields;
	// otherwise undefined behaviour
	char cmd = '0';
	// Each command will have different interpretation
	// regarding this field
	uint8_t subcmd = 0;

public:
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
		b0 = 0,
        b1 = 0,
        w = 0;

};

/*
 * The linear interpolation command moves the end of the robot arm move in 
 * a straight line from the current position to the position and posture 
 * specified by the command data.
 * This corresponds to G1 command
 */
class LinearInterpCommand: public Move3DCommand
{
public:
    LinearInterpCommand() : Move3DCommand()
    { cmd = '1'; subcmd = 1; }

    command_t serialize();

public:
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
class LinearInterpAccelCommand: public Move3DCommand
{
public:
	LinearInterpAccelCommand(): Move3DCommand()
	{ cmd = '2'; }

	command_t serialize();

// Fields
public:
    enum {
        SINGLE,             // Accelerate/decelerate from 0 to speed1.
                            // To decelerate, specify negative value to accel
        ACCEL_THEN_DECEL,   // Accelerate from 0 to speed1 then decelerate to 0.
                            // accel must be positive.
        ABSOLUTE            // Accelerate from speed1 to speed2.
                            // accel must be positive
    } accelOpt = SINGLE;

    float
        speed1 = 0,         // initial speed (mm/s ?)
        accel = 0,          // acceleration. range is [-3200,+3200] (mm/s^2)
        speed2 = 0;         // final speed (mm/s ?)
};


/*
 * 3D arc interpolation command
 * (corresponding to G300, G301).
 * This command must be sent in pair;
 * first by setting which to 0 (G300) -> the specified coordinate is
 *  an intermediate point
 * and then to 1 (G301) -> the specified coordinate is end point
 */
class Arc3DInterpCommand: public Move3DCommand
{
public:
	Arc3DInterpCommand(): Move3DCommand()
	{ subcmd = '4'; }
	command_t serialize();

public:
	uint8_t
		which = 0;
	float
		pwm = 0,
		R = 0,
		speed = 0;
};


/*
 * Delay command
 * (corresponding to G4)
 */
class DelayCommand : public ArmCommand
{
public:
	DelayCommand();
	command_t serialize();

public:
	float delay = 1;	// in millisecond
};


/*
 * Set worktable
 * (corresponding to G54)
 */
class SetWorktableCoordinateCommand : public ArmCommand
{
public:
	SetWorktableCoordinateCommand();
	command_t serialize();

protected:
	const uint8_t a1 = 20, a2=1;
public:
	// Worktable coordinate, in mm
	float
		X0=0,
		Y0=0,
		Z0=0;
};

}	// namespace ArmCommands
