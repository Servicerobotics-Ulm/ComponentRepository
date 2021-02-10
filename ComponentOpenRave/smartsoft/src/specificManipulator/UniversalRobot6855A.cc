//--------------------------------------------------------------------------
//  Copyright (C) 2012 Timo Hegele
//
//        hegele@mail.hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartOpenRave component".
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//---------------------------------------------------------------------

#include <math.h>
#include "UniversalRobot6855A.hh"
#include "util/MessageHandler.hh"
#include "ComponentOpenRave.hh"

namespace SpecificManipulator
{

UniversalRobot6855A::UniversalRobot6855A()
{
	//offset of the manipulator between the real manipulator origin and the open rave origin
	this->manipulatorCoord_x = 0.0;
	this->manipulatorCoord_y = 0.0;
	this->manipulatorCoord_z = 0.0;


	this->closedAngleOpenRave = 0.05;
	this->openAngleOpenRave = 0.0;
	this->closedAngleReal = 0.365483;
	this->openAngleReal = 2.11552;

	// This calculations has to been done, to scale the real angle to the OpenRave angle from the gripper.
	// The range and the scale of the real angle of the gripper is bigger than OpenRave.
	this->center = abs(closedAngleReal) - (abs(closedAngleReal) + abs(openAngleReal)) / 2;
	this->offset = abs(closedAngleOpenRave) - (abs(closedAngleOpenRave) + abs(openAngleOpenRave)) / 2; //= centerOpenrave
	//this->factor = (-1.0 * closedAngleReal + center) / (closedAngleOpenRave + offset);
	this->factor = (-1.0 * closedAngleReal + center) / (closedAngleOpenRave + offset);

	this->robotURI = "robots/UR6SchunkBigger.robot.xml";
	this->manipulatorName = "arm";
	this->openRaveName = "UR6Schunk";
	this->manipulatorType = "UR6855A";

	activeArmDofs.push_back(0);
	activeArmDofs.push_back(1);
	activeArmDofs.push_back(2);
	activeArmDofs.push_back(3);
	activeArmDofs.push_back(4);
	activeArmDofs.push_back(5);

	// TODO: 20.04

	activeGripperDofs.push_back(6);

	activeDofs.insert(activeDofs.end(), activeArmDofs.begin(), activeArmDofs.end());
	activeDofs.insert(activeDofs.end(), activeGripperDofs.begin(), activeGripperDofs.end());

}

bool UniversalRobot6855A::convertRealAnglesToOpenRaveAngles(const std::vector<double>& realAngles, std::vector<double>& openRaveAngles)
{
	if (realAngles.size() != openRaveAngles.size())
	{
		MessageHandler::handleMessage(
				"realAngles vector and openRaveAngles vector is not equal. [convertOpenRaveAnglesToRealAngles in Katana]\n",
				MessageHandler::WARNING);
		return false;
	}
	// Only joint angles
	if (realAngles.size() >= 6)
	{
		openRaveAngles[0] = realAngles[0];// + M_PI_4;
		openRaveAngles[1] = realAngles[1];// + M_PI_2;
		openRaveAngles[2] = realAngles[2];
		openRaveAngles[3] = realAngles[3];// + M_PI_2;
		openRaveAngles[4] = realAngles[4];
		openRaveAngles[5] = realAngles[5];
	}
	// With gripper angles
	if (realAngles.size() == 7)
	{
		openRaveAngles[6] = -1 * (((realAngles[6] / 1000) / 2) - 0.05); // realAngles[6] / 1000 --> conversion from mm to m
	}
	//Only gripper
	if (realAngles.size() == 1)
	{
		openRaveAngles[0] = -1 * (((realAngles[0] / 1000) / 2) - 0.05);
	}
	return true;
}

bool UniversalRobot6855A::convertOpenRaveAnglesToRealAngles(const std::vector<double>& openRaveAngles, std::vector<double>& realAngles)
{
//	if (realAngles.size() != openRaveAngles.size())
//	{
//		MessageHandler::handleMessage(
//				"realAngles vector and openRaveAngles vector is not equal. [convertOpenRaveAnglesToRealAngles in Universal]\n",
//				MessageHandler::WARNING);
//		return false;
//	}
	realAngles.resize(openRaveAngles.size());
	// Only joint angles
	if (openRaveAngles.size() >= 6)
	{
		realAngles[0] = openRaveAngles[0];// - M_PI_4;
		realAngles[1] = openRaveAngles[1];// - M_PI_2;
		realAngles[2] = openRaveAngles[2];
		realAngles[3] = openRaveAngles[3];// - M_PI_2;
		realAngles[4] = openRaveAngles[4];
		realAngles[5] = openRaveAngles[5];
	}
	// With gripper angles
	if (openRaveAngles.size() == 7)
	{
		realAngles[6] = (((-1 * openRaveAngles[6]) + 0.05) * 2.0) * 1000;
	}
	// Gripper Only
	if (openRaveAngles.size() == 1)
	{
		realAngles[0] = (((-1 * openRaveAngles[0]) + 0.05) * 2.0) * 1000;
	}
	return true;
}

const vector<int>& UniversalRobot6855A::getActiveDofs()
{
 return this->activeDofs;
}

const vector<int>& UniversalRobot6855A::getActiveArmDofs()
{
	return this->activeArmDofs;
}

const vector<int>& UniversalRobot6855A::getActiveGripperDofs()
{
	return this->activeGripperDofs;
}

bool UniversalRobot6855A::removeDummyJointAngles(std::vector<double>& angles)
{
	//Model of UR6855A got no dummy joints
	return true;
}

arma::mat UniversalRobot6855A::getOffsetRealManipulatortoOpenRaveManipulator()
{
	arma::mat mat = arma::eye<arma::mat>(4, 4);
	double angle = (M_PI - M_PI_4);
	mat(0, 0) = cos(angle);
	mat(0, 1) = -sin(angle);
	mat(1, 0) = sin(angle);
	mat(1, 1) = cos(angle);
	mat(0, 3) = this->manipulatorCoord_x;
	mat(1, 3) = this->manipulatorCoord_y;
	mat(2, 3) = this->manipulatorCoord_z;

	return mat;
}

arma::mat UniversalRobot6855A::getSimulationTransform()
{
	arma::mat mat = arma::eye<arma::mat>(4, 4);
	mat(0, 0) = 0;	mat(0, 1) = -0.7071;	mat(0, 2) = -0.7071;	mat(0, 3) = 0.13;
	mat(1, 0) = 0;	mat(1, 1) = -0.7071;	mat(1, 2) = 0.7071;		mat(1, 3) = -0.07;
	mat(2, 0) = -1;	mat(2, 1) = 0;			mat(2, 2) = 0;			mat(2, 3) = 0.352;
	mat(3, 0) = 0;	mat(3, 1) = 0;			mat(3, 2) = 0;			mat(3, 3) = 1;

	return mat;
}

void UniversalRobot6855A::getSimulationJointAngles(std::vector<double>& angles)
{
	angles.resize(this->activeArmDofs.size());
	angles[0] = 2.652;
	angles[1] = -2.499;
	angles[2] = 2.497;
	angles[3] = 0.006;
	angles[4] = 1.5707;
	angles[5] = -3.1415;
}

void UniversalRobot6855A::parseIterParameters(ORUtil::PlanningParameter& params, ORUtil& orutil)
{
	//params.azimuth += 90;
	//params.elevation += 90;
}

UniversalRobot6855A::~UniversalRobot6855A()
{
}


double UniversalRobot6855A::wrapToPi(double a)
{
	return wrapTo2Pi(a + M_PI) - M_PI;
}

double UniversalRobot6855A::wrapTo2Pi(double a)
{
	bool was_neg = a < 0;
	a = fmod(a, 2 * M_PI);

	if (was_neg)
		a += 2 * M_PI;

	return a;
}

}
