/*
 * RobotArm.h
 *
 *  Created on: Oct 7, 2021
 *      Author: sujiwo
 */

#ifndef INCLUDE_ROBOTARM_H_
#define INCLUDE_ROBOTARM_H_

#include <vector>
#include <cstring>
#include <Eigen/Eigen>
#include "util.h"
#include "SerialPort.h"

#define S6H4D_CMD_N 48
#define S6H4D_MOV_BEGIN 0xee
#define S6H4D_MOV_END 0xef
#define S6H4D_CTRL_BEGIN 0xfc
#define S6H4D_CTRL_END 0xfd
#define S6H4D_STAT_BEGIN 0xce
#define S6H4D_STAT_END 0xcf


typedef Eigen::Vector2f vector2f;
typedef Eigen::Vector3f vector3f;
typedef Eigen::Vector4f vector4f;




struct S6H4D_CMD_MOV
{
	uint8_t m_b[S6H4D_CMD_N];

	void init(uint8_t a, uint8_t b)
	{
		memset(m_b, 0, S6H4D_CMD_N);
		m_b[0] = S6H4D_MOV_BEGIN;
		m_b[1] = a;
		m_b[2] = b;
		m_b[47] = S6H4D_MOV_END;
	}

	void set(const vector3f &vP, const vector3f &vR, float spd)
	{
		f2b(&m_b[3], vP.x());
		f2b(&m_b[7], vP.y());
		f2b(&m_b[11], vP.z());
		f2b(&m_b[15], vR.x());
		f2b(&m_b[19], vR.y());
		f2b(&m_b[23], vR.z());
		f2b(&m_b[27], 1500); //pwm
		f2b(&m_b[31], 0);
		f2b(&m_b[35], 0);
		f2b(&m_b[39], 0);
		f2b(&m_b[43], spd);
	}
};

struct S6H4D_CMD_CTRL
{
	uint8_t m_b[S6H4D_CMD_N];

	void init(void)
	{
		memset(m_b, 0, S6H4D_CMD_N);
		m_b[0] = S6H4D_CTRL_BEGIN;
		m_b[47] = S6H4D_CTRL_END;
	}
};

struct S6H4D_CMD_STATE
{
	uint8_t m_pB[9];
	int m_iB;

	void init(void)
	{
		m_iB = 0;
	}
};

enum S6H4D_VOL_TYPE
{
	vol_box = 0,
	vol_ball = 1,
	vol_cylinder = 2,
};

struct S6H4D_VOL
{
	S6H4D_VOL_TYPE m_type = vol_ball;
	bool	m_bInside = false;	//true: inside valid

	vector2f m_vX{0,0},
		m_vY{0,0}, m_vZ{0,0};

	vector3f m_vC{0,0,0};
	vector2f m_vR{0,0,0};

	bool bValid(const vector3f& vP)
	{
		bool bInside = true;

		if(m_type == vol_box)
		{
			if(vP.x() < m_vX.x())bInside = false;
			if(vP.x() > m_vX.y())bInside = false;
			if(vP.y() < m_vY.x())bInside = false;
			if(vP.y() > m_vY.y())bInside = false;
			if(vP.z() < m_vZ.x())bInside = false;
			if(vP.z() > m_vZ.y())bInside = false;
		}
		else if(m_type == vol_ball)
		{
			float r = (vP-m_vC).norm();
			if(r < m_vR.x()) bInside = false;
			if(r > m_vR.y()) bInside = false;
		}
		else if(m_type == vol_cylinder)
		{
			vector3f vR = vP - m_vC;
			vR.z() = 0.0;
			float r = vR.norm();
			if(r < m_vR.x())bInside = false;
			if(r > m_vR.y())bInside = false;
		}

		if(m_bInside != bInside) return false;
		return true;
	}
};


class RobotArm {
public:
	RobotArm();
	virtual ~RobotArm();

	// Commands
	void pause(void);
	void reset(void);
	void ctrlReset(void);
	void aGotoPos(const vector3f &vP, const vector3f& vR, float speed);
	void setOrigin(const vector3f &vP);
	void setMode(int mode);

	// Analog joystick commands
	void stickSpeed(vector3f &vM);
	void stickRot(int iAxis, float r);
	void stickStop(void);
	void stickRelease(void);


private:
	SerialPort mPort;
	S6H4D_CMD_STATE m_state;

	bool isLittleEndian=true;
	int m_mode=1;
	vector2f m_vSpeedRange{0, 5000};	//mm/m
	float m_speed=2000;
	vector3f m_vOriginTarget{0,0,0};
	vector3f m_vOrigin{std::numeric_limits::max()};
	vector3f m_vLastValidP{0,0,0};

	vector3f m_vPgoing{0,0,0};
	float m_pErr;

	std::vector<S6H4D_VOL> m_vForbiddenArea;

};

#endif /* INCLUDE_ROBOTARM_H_ */
