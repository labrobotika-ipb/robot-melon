/*
 * RobotArm.cpp
 *
 *  Created on: Oct 7, 2021
 *      Author: sujiwo
 */

#include "RobotArm.h"


RobotArm::RobotArm()
{
	// TODO Auto-generated constructor stub
}

RobotArm::~RobotArm()
{
	// TODO Auto-generated destructor stub
}

void
RobotArm::reset()
{
	S6H4D_CMD_CTRL cmd;
	cmd.init();
	cmd.m_b[1] = 12;
	cmd.m_b[2] = 3;
	mPort.write(cmd.m_b, S6H4D_CMD_N);
}

void
RobotArm::pause(void)
{
	S6H4D_CMD_CTRL cmd;
	cmd.init();
	cmd.m_b[1] = 30;
	cmd.m_b[2] = 3;
	cmd.m_b[3] = 2;
	mPort.write(cmd.m_b, S6H4D_CMD_N);
}

void
RobotArm::ctrlReset(void)
{
	S6H4D_CMD_CTRL cmd;
	cmd.init();
	cmd.m_b[1] = 12;
	cmd.m_b[2] = 6;
	cmd.m_b[3] = 0;
	mPort.write(cmd.m_b, S6H4D_CMD_N);
}

void
RobotArm::aGotoPos(const vector3f &vP, const vector3f& vR, float speed)
{
	S6H4D_CMD_MOV cmd;
	cmd.init('1', 0);
	cmd.set(vP, vR, speed);
	mPort.write(cmd.m_b, S6H4D_CMD_N);
}

void
RobotArm::setOrigin(const vector3f &vP)
{
	S6H4D_CMD_CTRL cmd;
	cmd.init();
	cmd.m_b[1] = 20;
	cmd.m_b[2] = 1;
	f2b(&cmd.m_b[3], vP.x() * 10.0);
	f2b(&cmd.m_b[7], vP.y() * 10.0);
	f2b(&cmd.m_b[11], vP.z() * 10.0);
	mPort.write(cmd.m_b, S6H4D_CMD_N);
}

void
RobotArm::setMode(int mode)
{
	S6H4D_CMD_CTRL cmd;
	cmd.init();
	cmd.m_b[1] = 30;
	cmd.m_b[2] = 9;
	cmd.m_b[3] = (mode == 0) ? 0 : 9;
	mPort.write(cmd.m_b, S6H4D_CMD_N);
}


void
RobotArm::stickSpeed(vector3f &vM)
{
//	S6H4D_CMD_CTRL cmd;
//	cmd.init();
//	cmd.m_b[1] = 30;
//	cmd.m_b[2] = 7;
//	cmd.m_b[3] = 100;
//	cmd.m_b[4] = (vM.x >= 0.0) ? constrain<float>(128 + (vM.x - 0.5) * 255, 0, 255) : 128;
//	cmd.m_b[5] = (vM.y >= 0.0) ? constrain<float>(128 + (vM.y - 0.5) * 255, 0, 255) : 128;
//	cmd.m_b[6] = (vM.z >= 0.0) ? constrain<float>(128 + (vM.z - 0.5) * 255, 0, 255) : 128;
//	mPort.write(cmd.m_b, S6H4D_CMD_N);
}
