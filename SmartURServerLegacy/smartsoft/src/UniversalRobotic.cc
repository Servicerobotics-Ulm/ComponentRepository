//------------------------------------------------------------------------
//
//  Copyright (C) 2011-2020 Manuel Wopfner, Matthias Lutz
//
//        lutz@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartURServer component".
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
//--------------------------------------------------------------------------



#include "UniversalRobotic.hh"

#include "SmartURServerLegacy.hh"

#include <CommBasicObjects/CommPose3d.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>

#include <armadillo.hh>
#include <EulerTransformationMatrices.hh>

#include <iostream>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <bitset>

#include <thread>



//TODO
// typedef ACE_Guard<ACE_Recursive_Thread_Mutex> SmartRecursiveGuard;

using namespace CommManipulatorObjects;

UniversalRobotic* UniversalRobotic::_instance = NULL;

//////////////////////////////////////////////////
//
//	public methods
//
//////////////////////////////////////////////////

UniversalRobotic* UniversalRobotic::instance()
{
	if (_instance == NULL)
	{
		COMP->URMutex.acquire();
		if (_instance == NULL)
		{
			_instance = new UniversalRobotic();
		}
		COMP->URMutex.release();
	}
	return _instance;
}

UniversalRobotic::~UniversalRobotic()
{
	if (manipulator != 0)
	{
		delete manipulator;
	}
}

bool UniversalRobotic::init()
{
	try
	{
		std::cout << "[UniversalRobotic] Connecting to manipulator.\n";
		manipulator = new URManipulator(COMP->getGlobalState().getNetwork().getIp_adress());
		std::cout << "[UniversalRobotic] Successfully connected to manipulator.\n";
		std::cout << "[UniversalRobotic] READY ...\n";
		return true;
	} catch (URManipulator::ConnectionException ex)
	{
		handleException(ex.what());
		return false;
	}
}

void UniversalRobotic::calibrate()
{
	std::cout << "---------------- [UniversalRobotic::calibrate] ----------------\n";
	// wait until there is controller data
	while (!manipulator->hasControllerData())
	{
		usleep(500000);
	}

	std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = manipulator->getControllerData();

	while (data->robotModeData.robotMode == PrimaryAndSecondaryClientInterfaceParser::ROBOT_NO_POWER_MODE)
	{
		std::cout << "robot in no power mode\n";
		manipulator->powerOn();
		data = manipulator->getControllerData();
		usleep(500000);
	}

	////////////////////////////////
	// setup terminal for key input
	////////////////////////////////
	char c;
	int kfd = 0;
	struct termios cooked, raw;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	////////////////////////////////
	// calibrate
	////////////////////////////////
	std::cout << "------------------------\n";
	std::cout << "Robot: \"w\" or \"s\" \t" << PrimaryAndSecondaryClientInterfaceParser::robotModeToString(data->robotModeData.robotMode)
			<< "\n";
	std::cout << "Base: \"e\" or \"d\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[0].jointMode)
			<< "\n";
	std::cout << "Shoulder: \"r\" or \"f\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[1].jointMode)
			<< "\n";
	std::cout << "Elbow: \"t\" or \"g\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[2].jointMode)
			<< "\n";
	std::cout << "Wrist 1: \"z\" or \"h\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[3].jointMode)
			<< "\n";
	std::cout << "Wrist 2: \"u\" or \"j\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[4].jointMode)
			<< "\n";
	std::cout << "Wrist 3: \"i\" or \"k\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[5].jointMode)
			<< "\n";

	manipulator->startCalibration();

	while (data->robotModeData.robotMode == PrimaryAndSecondaryClientInterfaceParser::ROBOT_INITIALIZING_MODE)
	{
		data = manipulator->getControllerData();

		if (data->jointData.size() != 6)
		{
			usleep(500000);
			continue;
		}

		if (read(kfd, &c, 1) < 0)
		{
			std::cerr << "error during key read" << std::endl;
			continue;
		}

		std::cout << "------------------------\n";
		std::cout << "Robot: \"w\" or \"s\" \t"
				<< PrimaryAndSecondaryClientInterfaceParser::robotModeToString(data->robotModeData.robotMode) << "\n";
		std::cout << "Base: \"e\" or \"d\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[0].jointMode)
				<< "\n";
		std::cout << "Shoulder: \"r\" or \"f\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(
				data->jointData[1].jointMode) << "\n";
		std::cout << "Elbow: \"t\" or \"g\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(data->jointData[2].jointMode)
				<< "\n";
		std::cout << "Wrist 1: \"z\" or \"h\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(
				data->jointData[3].jointMode) << "\n";
		std::cout << "Wrist 2: \"u\" or \"j\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(
				data->jointData[4].jointMode) << "\n";
		std::cout << "Wrist 3: \"i\" or \"k\" \t" << PrimaryAndSecondaryClientInterfaceParser::jointModeToString(
				data->jointData[5].jointMode) << "\n";

		manipulator->calibrate(c);
	}

	manipulator->stopCalibration();

	// send empty program
	stringstream program;
	program << "def emptyProg():\n";
	program << "end\n";
	manipulator->sendProgram(program.str());

	std::cout << "---------------------------------------------------------------\n";

	tcsetattr(kfd, TCSANOW, &cooked);
}

void UniversalRobotic::performTrajectory(const CommManipulatorObjects::CommManipulationTrajectory &trajectory)
{
	std::cout<<"GO"<<std::endl;
	// first apply global parameters
//	applyGlobalParameters();

	SmartACE::SmartRecursiveGuard guard(mutex);

	resetGoalAngles();

	// send event that manipulator is now moving
	CommManipulatorObjects::CommManipulatorEventState state;
	state.setEvent(CommManipulatorObjects::ManipulatorEvent::GOAL_NOT_REACHED);
	std::cout<<"Put event "<<std::endl;
	COMP->manipulatorEventServiceOut->put(state);
	std::cout<<"Put event DONE "<<std::endl;

	switch (trajectory.get_valid_values())
	{
	case CommManipulatorObjects::ManipulatorTrajectoryFlag::JOINT_ANGLES:
		performTrajectoryJointAngles(trajectory);
		break;
	case CommManipulatorObjects::ManipulatorTrajectoryFlag::POSE_TCP:
		performTrajectoryPoseTCP(trajectory);
		break;
	default: 
		std::cout<<"ERROR this should never happen!"<<std::endl;
	}

}

std::vector<bool> UniversalRobotic::getDigitalInputValues(){
	std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = manipulator->getControllerData();
	std::vector<bool> out;

	//int16_t
	std::bitset<16> input(data->masterboardData.digitalInputBits);

	//std::cout<<__FUNCTION__<<" "<<input<<std::endl;

	for(unsigned int i=0;i<16;++i){
		out.push_back(input[i]);
	}
	return out;
}


std::vector<float> UniversalRobotic::getAnalogInputValues(){
	std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = manipulator->getControllerData();

	std::vector<float> out;
	out.push_back(data->masterboardData.analogInput0);
	out.push_back(data->masterboardData.analogInput1);
	out.push_back(data->toolData.analogInput2);
	out.push_back(data->toolData.analogInput3);

	return out;
}


void UniversalRobotic::setDigitalOutputValue(unsigned int outputNumber, bool outputValue){
	manipulator->setDigitalOutput(outputNumber,outputValue);
}

void UniversalRobotic::setAnalogOutputValue(unsigned int outputNumber, double outputValue){
	manipulator->setAnalogOutput(outputNumber, (float)outputValue);
}


void UniversalRobotic::getCurrentState(CommManipulatorObjects::CommManipulatorState &state)
{
	std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = manipulator->getControllerData();
//	std::cout << "DEBUG MATTHIAS"<<std::endl;
//	std::cout << data.asString() << std::endl;

	// check state of manipulator and send events
	checkState(data);

	// set id for the state
	state.set_id(stateIdCounter++);

	state.set_pose_manipulator(COMP->getGlobalState().getManipulator().getX(), COMP->getGlobalState().getManipulator().getY(), COMP->getGlobalState().getManipulator().getZ(), COMP->getGlobalState().getManipulator().getAzimuth(),
			COMP->getGlobalState().getManipulator().getElevation(), COMP->getGlobalState().getManipulator().getRoll());

	// read values from manipulator
	state.set_joint_count(data->jointData.size());
	for (size_t i = 0; i < data->jointData.size(); ++i)
	{
		state.set_joint_angle(i, data->jointData[i].q_actual);
	}

	double angle_axis_rot_angle; //angle to turn around the vector
	double nx,ny,nz; //normalized vector
	double yaw,pitch,roll; //converted euler angles

	EulerTransformationMatrices::normalize_angle_axis(data->cartesianInfo.rx, data->cartesianInfo.ry, data->cartesianInfo.rz, angle_axis_rot_angle, nx, ny,nz);
	std::cout << "rx,y,z :"<<data->cartesianInfo.rx << ", "<< data->cartesianInfo.ry << ", "<<data->cartesianInfo.rz<<std::endl;

	EulerTransformationMatrices::angle_axis_to_euler_angels_ZYX(nx, ny, nz, angle_axis_rot_angle, yaw, pitch, roll);

/*
	//////////////////////////////
	// TESTING
	{


		double nrx,nry,nrz,angle, rx,ry,rz;
		EulerTransformationMatrices::euler_angels_ZYX_to_angle_axis(yaw, pitch, roll, nrx, nry, nrz,angle);
		EulerTransformationMatrices::denormalize_angle_axis(nrx,nry,nrz,angle, rx, ry,rz);

		std::cout<<"Input : "<<data.cartesianInfo.rx<<" "<<data.cartesianInfo.ry<<" "<<data.cartesianInfo.rz<<std::endl;
		std::cout<<"Output: "<<rx<<" "<<ry<<" "<<rz<<std::endl;
		std::cout<<"In Euler: "<<yaw<<" "<<pitch<<" "<<roll<<std::endl<<std::endl;


	}
	/////////////////////////////
*/
	arma::mat tcp_ur;
	tcp_ur.set_size(4, 4);
	EulerTransformationMatrices::create_zyx_matrix(yaw, pitch, roll, tcp_ur);
	tcp_ur(0, 3) = data->cartesianInfo.x;
	tcp_ur(1, 3) = data->cartesianInfo.y;
	tcp_ur(2, 3) = data->cartesianInfo.z;
	tcp_ur(3, 3) = 1;

	arma::mat trans_tcp_smartsoft;
	trans_tcp_smartsoft.set_size(4, 4);
	trans_tcp_smartsoft.zeros();
	EulerTransformationMatrices::create_zyx_matrix(M_PI_2, -M_PI_2, 0, trans_tcp_smartsoft);

	trans_tcp_smartsoft(0, 3) = 0;
	trans_tcp_smartsoft(1, 3) = 0;
	trans_tcp_smartsoft(2, 3) = 0;
	trans_tcp_smartsoft(3, 3) = 1;

	arma::mat tcp_smartsoft;
	tcp_smartsoft.set_size(4, 4);
	tcp_smartsoft = tcp_ur * trans_tcp_smartsoft;  //transform TCP from manip to robot system coords (gipper allong X axis)

	double x,y,z;
	EulerTransformationMatrices::zyx_from_matrix(tcp_smartsoft, x, y, z, yaw, pitch,roll);

	state.set_pose_TCP_manipulator(x, y, z, yaw, pitch, roll, 1.0);

	state.set_valid(true);
}

//void UniversalRobotic::setParameter(const CommManipulatorObjects::CommManipulatorParameter &param)
//{
//
//
//	try
//	{
//		CommManipulatorObjects::ManipulatorParameterMode tag(CommManipulatorObjects::ManipulatorParameterMode::TRAJECTORY);
//		int p1;
//
//		param.get(tag, p1);
//
//		CHS::SmartGuard guard(COMP->ParameterMutex);
//		switch (tag)
//		{
//		case CommManipulatorObjects::ManipulatorParameterMode::SET_ALL_JOINT_VELOCITY_LIMITS:
//			{
//					std::cout<<"FIXME PARAMETER USED FALSE!"<<std::endl;
//					std::cout <<"SET_ALL_JOINT_VELOCITY_LIMITS: triggered --> perform saved trajectory\n";
//					std::cout<<"p1: "<<p1<<std::endl;
//
//					CommManipulatorObjects::CommManipulatorTrajectory trajectory;
//					ifstream myfile("URTrajectory_saved.xml");
//					if (myfile.is_open())
//					{
//							trajectory.load_xml(myfile);
//							std::cout<<"Loaded Trajectory"<<std::endl;
//							myfile.close();
//					}
//
//					performTrajectory(trajectory);
//
//			}
//
//		}
//		guard.release();
//	} catch (...)
//	{
//		handleException("Unhandled exception in UniversalRobotic::setParameters()");
//	}
//
//}

//const UniversalRobotic::UniversalRoboticParameters UniversalRobotic::getParameters() const
//{
//	return globalParameters;
//}

void UniversalRobotic::switchToRun()
{
	try
	{
		// lock manipulator
		SmartACE::SmartGuard guard(COMP->URMutex);
		manipulator->setMode(URManipulator::MODE_RUN);
		guard.release();
	} catch (...)
	{
		handleException("Unhandled exception in UniversalRobotic::switchToFreedrive()");
	}
}

void UniversalRobotic::switchToFreedrive()
{
	try
	{
		// lock manipulator
		SmartACE::SmartGuard guard(COMP->URMutex);
		manipulator->setMode(URManipulator::MODE_FREEDRIVE);
		guard.release();
	} catch (...)
	{
		handleException("Unhandled exception in UniversalRobotic::switchToFreedrive()");
	}
}

void UniversalRobotic::resetGoalAngles()
{
	SmartACE::SmartRecursiveGuard guard(mutex);
	target_joint_angles.clear();
	goal_reached = false;
}

void UniversalRobotic::start()
{
	try
	{
		activated = true;

		// lock manipulator
		SmartACE::SmartGuard guard(COMP->URMutex);
		manipulator->setProgramMode(URManipulator::PROGRAM_RUN);
		guard.release();
	} catch (...)
	{
		handleException("Unhandled exception in UniversalRobotic::start()");
	}
}

void UniversalRobotic::stop()
{
	try
	{
		activated = false;

		// lock manipulator
		SmartACE::SmartGuard guard(COMP->URMutex);
		manipulator->setProgramMode(URManipulator::PROGRAM_STOP);
		guard.release();
	} catch (...)
	{
		handleException("Unhandled exception in UniversalRobotic::stop()");
	}
}

void UniversalRobotic::shutdown()
{
	manipulator->shutdown();
}

//////////////////////////////////////////////////
//
//	private methods
//
//////////////////////////////////////////////////

UniversalRobotic::UniversalRobotic()
{
	stateIdCounter = 0;
	pointCloudIdCounter = 0;
	manipulator = 0;
	goal_reached = false;
}

void UniversalRobotic::performTrajectoryJointAngles(const CommManipulatorObjects::CommManipulationTrajectory& trajectory)
{
	std::cout<<"UniversalRobotic::performTrajectoryJointAngles"<<std::endl;
	if (trajectory.get_trajectory_size() > 0 && trajectory.get_joint_count() == manipulator->getJointCount())
	{
		// set target_joint_angles
		target_pose.clear();
		target_joint_angles.clear();
		for (size_t i = 0; i < trajectory.get_joint_count(); ++i)
		{
			target_joint_angles.push_back(trajectory.get_joint_angle(trajectory.get_trajectory_size() - 1, i));
		}

		// read values from manipulator
		std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = manipulator->getControllerData();

		int counter = 1000;

		std::list<TrajElement> traj;
		for (size_t i = 0; i < trajectory.get_trajectory_size(); ++i)
		{
			TrajElement el;
			for (size_t j = 0; j < trajectory.get_joint_count(); ++j)
			{
				el.joint_angles.push_back(trajectory.get_joint_angle(i, j));
			}
			el.id = counter++;
			el.time = trajectory.get_joint_time(i);
			traj.push_back(el);
		}

		// TODO set values from params
		//manipulator->setPayload(0.5);
		//manipulator->setSpeed(0.5);
		manipulator->setSpeed(1.0); //speed values from openrave no prescale!!

		// send program
		stringstream program;
		program << "def moveRobot():\n";

		for (std::list<TrajElement>::iterator it = traj.begin(); it != traj.end(); it++)
		{
			program << "	servoj([";
			for (size_t j = 0; j < it->joint_angles.size(); ++j)
			{
				// write angle positions
				program << it->joint_angles[j];

				if (j < it->joint_angles.size() - 1)
					program << ", ";

			}
			program << "],t=" << it->time << ")\n";
		}

		// to enforce an accurate end position send the last target again with movej
		std::list<TrajElement>::iterator iter_last = traj.end();
		iter_last--;
		program << "	movej([";
		for (size_t j = 0; j < iter_last->joint_angles.size(); ++j)
		{
			// write angle positions
			program << iter_last->joint_angles[j];

			if (j < iter_last->joint_angles.size() - 1)
				program << ", ";

		}
		program << "])\n";


		program << "end\n";
		manipulator->sendProgram(program.str());
		std::cout << "[UniversalRobotic] perform trajectory from joint angles.\n";

		std::cout << program.str() << std::endl;
	} else
	{
		std::cerr << "[UniversalRobotic] There are not enough joint values in the send trajectory!\n";
	}
}

void UniversalRobotic::sendProgram(const std::string& program){
	manipulator->sendProgram(program);
}

void UniversalRobotic::performTrajectoryPoseTCP(const CommManipulatorObjects::CommManipulationTrajectory& trajectory)
{
	target_pose.clear();
	target_joint_angles.clear();

	arma::mat offset_manipulator;
	arma::mat offset_manipulator_inv;
	arma::mat tcp_robot;
	double x, y, z, azimuth, elevation, roll;

	trajectory.get_first_pose_TCP_robot(x, y, z, azimuth, elevation, roll, 0.001); // get pose in mm

	EulerTransformationMatrices::create_zyx_matrix(x, y, z, azimuth, elevation, roll, tcp_robot);

	EulerTransformationMatrices::create_zyx_matrix(COMP->getGlobalState().getManipulator().getX(), COMP->getGlobalState().getManipulator().getY(), COMP->getGlobalState().getManipulator().getZ(),
			COMP->getGlobalState().getManipulator().getAzimuth(), COMP->getGlobalState().getManipulator().getElevation(), COMP->getGlobalState().getManipulator().getRoll(), offset_manipulator);

	offset_manipulator_inv = arma::inv(offset_manipulator);
	arma::mat tcp_manipulator_smartsoft = offset_manipulator_inv * tcp_robot;

	arma::mat trans_tcp_smartsoft;
	trans_tcp_smartsoft.set_size(4, 4);
	trans_tcp_smartsoft.zeros();
	EulerTransformationMatrices::create_zyx_matrix(M_PI_2, -M_PI_2, 0, trans_tcp_smartsoft);

	trans_tcp_smartsoft(0, 3) = 0;
	trans_tcp_smartsoft(1, 3) = 0;
	trans_tcp_smartsoft(2, 3) = 0;
	trans_tcp_smartsoft(3, 3) = 1;

	arma::mat trans_tcp_smartsoft_inv;
	trans_tcp_smartsoft_inv.set_size(4, 4);
 	trans_tcp_smartsoft_inv = arma::inv(trans_tcp_smartsoft);
	
	arma::mat tcp_manipulator;
	tcp_manipulator.set_size(4, 4);
	tcp_manipulator = tcp_manipulator_smartsoft * trans_tcp_smartsoft_inv;  //transform TCP from robot to  manip coords system

	CommBasicObjects::CommPose3d tcp_manip(tcp_manipulator);

	target_pose.push_back(tcp_manip.get_x(1.0));
	target_pose.push_back(tcp_manip.get_y(1.0));
	target_pose.push_back(tcp_manip.get_z(1.0));
	double nrx,nry,nrz,angle, rx,ry,rz;
	if(!EulerTransformationMatrices::euler_angels_ZYX_to_angle_axis(tcp_manip.get_azimuth(), tcp_manip.get_elevation(), tcp_manip.get_roll(), nrx, nry, nrz,angle))
	{
		std::cout<<__FILE__<<" "<<__FUNCTION__<<": Error in transformation from euler angles to angle axis: "<<tcp_manip.get_azimuth()<<" "<<tcp_manip.get_elevation()<<" "<<tcp_manip.get_roll()<<std::endl;
	}
	EulerTransformationMatrices::denormalize_angle_axis(nrx,nry,nrz,angle, rx, ry,rz);

	target_pose.push_back(rx);
	target_pose.push_back(ry);
	target_pose.push_back(rz);


	std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = manipulator->getControllerData();
	std::cout<<"Pose before: "<<data->cartesianInfo.x<<" "<<data->cartesianInfo.y<<" "<<data->cartesianInfo.z<<" "<<data->cartesianInfo.rx<<" "<<data->cartesianInfo.ry<<" "<<data->cartesianInfo.rz<<std::endl;
	std::cout<<"Pose new: "<<target_pose[0]<<" "<<target_pose[1]<<" "<<target_pose[2]<<" "<<target_pose[3]<<" "<<target_pose[4]<<" "<<target_pose[5]<<std::endl;


	// TODO set values from params
	manipulator->setPayload(0.5);
	manipulator->setSpeed(0.5);

	// send program
	stringstream program;
	program << "def moveRobot():\n";

	program << "	movel(get_inverse_kin(p[";
	program << tcp_manip.get_x(1.0) << ", ";
	program << tcp_manip.get_y(1.0) << ", ";
	program << tcp_manip.get_z(1.0) << ", ";
	program << rx << ", ";
	program << ry << ", ";
	program << rz << "])";

	program << ", a=1.2, v=0.3)\n";

	program << "end\n";
	manipulator->sendProgram(program.str());
	std::cout << "[UniversalRobotic] perform trajectory from tcp pose.\n";

//	std::cout << program.str() << std::endl;
}

void UniversalRobotic::adjustTrajectory(std::list<TrajElement>& trajectory)
{
	const double maxJointSpeed = M_PI; // [rad/sec]
	const double acc = M_PI / 15; // [rad/sec^2]
	const double interval = 0.05; // [sec]

	const int steps = (maxJointSpeed / acc) / interval; // number of steps for acceleration

	double curTime = interval;
	double restTime = curTime;
	std::list<TrajElement>::iterator it = trajectory.end();
	it--;

	std::list<TrajElement>::iterator nit = it;
	nit--;

	double ang = 0.5 * acc * curTime * curTime;

	std::cout << "Steps: " << steps << std::endl;

	// check current trajectory and insert new interpolated points inbetween if needed
	// we start from the back and move to the front
	std::list<TrajElement>::iterator end = trajectory.begin();
	end--;
	for (int i = 1; i <= steps && nit != end;)
	{
		double maxAngle = getMaxAngle(it->joint_angles, nit->joint_angles);

		//		std::cout << "-----------------------" << std::endl;
		//		std::cout << "step: " << i << std::endl;
		//		std::cout << "curTime: " << curTime << std::endl;
		//		std::cout << "restTime: " << restTime << std::endl;
		//		std::cout << "ang: " << ang << std::endl;
		//		std::cout << "maxAngle: " << maxAngle << std::endl;
		//		std::cout << "it: " << it->id << std::endl;
		//		std::cout << "nit: " << nit->id << std::endl;
		//		std::cout << "list: ";
		//		for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
		//		{
		//			std::cout << iter->id << ", ";
		//		}
		//		std::cout << std::endl;
		//		std::cout << "time: ";
		//		for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
		//		{
		//			std::cout << iter->time << ", ";
		//		}
		//		std::cout << std::endl;

		// the next (previous) point is nearer than the determined interpolation point
		if (maxAngle <= ang)
		{
			//			std::cout << ">>>>> maxAngle <= ang" << std::endl;

			it->time = (restTime * maxAngle) / ang; // set time for the next point
			it->done = true;

			restTime -= it->time;
			ang -= maxAngle;

			//			std::cout << "time: ";
			//			for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
			//			{
			//				std::cout << iter->time << ", ";
			//			}
			//			std::cout << std::endl;

			it--; // move to the next point
			nit--;
		}
		// the next (previous) point is further away than the determined interpolation point
		// thus a new interpolated point must be added
		else
		{
			//			std::cout << ">>>>> maxAngle > ang" << std::endl;

			std::vector<double> diff;
			getAngleDiff(it->joint_angles, nit->joint_angles, diff);

			double factor = ang / maxAngle;

			// create interpolation point
			it->time = restTime;
			it->done = true;

			//			std::cout << "time: ";
			//			for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
			//			{
			//				std::cout << iter->time << ", ";
			//			}
			//			std::cout << std::endl;

			TrajElement el;
			el.joint_angles.resize(diff.size());
			for (size_t j = 0; j < diff.size(); ++j)
			{
				el.joint_angles[j] = diff[j] * factor + it->joint_angles[j];
			}
			el.id = i;

			it = trajectory.insert(it, el); // insert interpolated point and adjust iterators
			nit = it;
			nit--;

			curTime += interval;
			restTime = curTime;

			ang = 0.5 * acc * curTime * curTime;
			i++;

		}

		//		std::cout << "-----------------------" << std::endl;
	}

	//	std::cout << "-----------------------" << std::endl;
	//	std::cout << "list: ";
	//	for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
	//	{
	//		std::cout << iter->id << ", ";
	//	}
	//	std::cout << std::endl;
	//	std::cout << "time: ";
	//	for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
	//	{
	//		std::cout << iter->time << ", ";
	//	}
	//	std::cout << std::endl;
	//	std::cout << "-----------------------" << std::endl;

	std::vector<double> lastAngles;
	for (std::list<TrajElement>::iterator it = trajectory.begin(); it != trajectory.end(); it++)
	{
		if (!it->done)
		{
			it->time = getMaxAngle(lastAngles, it->joint_angles) / maxJointSpeed;
			it->done = true;
		}
		lastAngles = it->joint_angles;
	}

	//	std::cout << "-----------------------" << std::endl;
	//	std::cout << "list: ";
	//	for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
	//	{
	//		std::cout << iter->id << ", ";
	//	}
	//	std::cout << std::endl;
	//	std::cout << "time: ";
	//	for (std::list<TrajElement>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
	//	{
	//		std::cout << iter->time << ", ";
	//	}
	//	std::cout << std::endl;
	//	std::cout << "-----------------------" << std::endl;

}

double UniversalRobotic::getMaxAngle(const std::vector<double>& angles1, const std::vector<double>& angles2) const
{
	double maxMoveAngle = 0; // [rad]

	for (size_t j = 0; j < angles1.size(); ++j)
	{
		// check for new maxMoveAngle
		if (maxMoveAngle < fabs(angles1[j] - angles2[j]))
			maxMoveAngle = fabs(angles1[j] - angles2[j]);
	}

	return maxMoveAngle;
}

void UniversalRobotic::getAngleDiff(const std::vector<double>& angles1, const std::vector<double>& angles2, std::vector<double>& diff) const
{
	diff.resize(angles1.size());
	for (size_t j = 0; j < angles1.size(); ++j)
	{
		diff[j] = angles2[j] - angles1[j];
	}
}

void UniversalRobotic::checkState(std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data)
{
	SmartACE::SmartRecursiveGuard guard(mutex);

	bool goalReached = false;

	// check goal reached
	if (!data->robotModeData.isProgramRunning && (data->jointData.size() == target_joint_angles.size() || target_pose.size() == 6))
	{

		// use joint angles for target test
		if (target_joint_angles.size() > 0)
		{
		        goalReached = true;
			for (size_t i = 0; i < target_joint_angles.size(); ++i)
			{
				goalReached = (goalReached && (fabs(data->jointData[i].q_actual - target_joint_angles[i]) < 0.01));
				//			std::cout << ">> " << goalReached << ", " << target_joint_angles[i] << ", " << data.jointData[i].q_actual << ", "
				//					<< data.jointData[i].q_target << ", " << fabs(data.jointData[i].q_actual - target_joint_angles[i]) << "\n";
			}
			//		std::cout << "--------------\n";
		}
		// use pose for target test
		else if (target_pose.size() > 0)
		{
		        goalReached = true;

			goalReached = (goalReached && (fabs(data->cartesianInfo.x - target_pose[0]) < 0.001));
//			std::cout << ">> " << goalReached << ", " << target_pose[0] << ", " << data.cartesianInfo.x << ", " << fabs(data.cartesianInfo.x - target_pose[0]) << "\n";

			goalReached = (goalReached && (fabs(data->cartesianInfo.y - target_pose[1]) < 0.001));
//			std::cout << ">> " << goalReached << ", " << target_pose[1] << ", " << data.cartesianInfo.y << ", " << fabs(data.cartesianInfo.y - target_pose[1]) << "\n";

			goalReached = (goalReached && (fabs(data->cartesianInfo.z - target_pose[2]) < 0.001));
//			std::cout << ">> " << goalReached << ", " << target_pose[2] << ", " << data.cartesianInfo.z << ", " << fabs(data.cartesianInfo.z - target_pose[2]) << "\n";

			goalReached = (goalReached && (fabs(data->cartesianInfo.rx - target_pose[3]) < 0.001));
//			std::cout << ">> " << goalReached << ", " << target_pose[3] << ", " << data.cartesianInfo.rx << ", " << fabs(data.cartesianInfo.rx - target_pose[3]) << "\n";

			goalReached = (goalReached && (fabs(data->cartesianInfo.ry - target_pose[4]) < 0.001));
//			std::cout << ">> " << goalReached << ", " << target_pose[4] << ", " << data.cartesianInfo.ry << ", " << fabs(data.cartesianInfo.ry - target_pose[4]) << "\n";

			goalReached = (goalReached && (fabs(data->cartesianInfo.rz - target_pose[5]) < 0.001));
//			std::cout << ">> " << goalReached << ", " << target_pose[5] << ", " << data.cartesianInfo.rz << ", " << fabs(data.cartesianInfo.rz - target_pose[5]) << "\n";

//			std::cout << "--------------\n";
		}else{

			std::cout<<__FILE__<<__LINE__<<"This case may be an error!"<<std::endl;
			std::cout<<"goalReached: "<<goalReached<<" this->goal_reached: "<<this->goal_reached<<std::endl;
		}


	}

		if (goalReached == true && this->goal_reached == false)
		{
			this->goal_reached = true;

			CommManipulatorEventState state;
			state.setEvent(ManipulatorEvent::GOAL_REACHED);
			COMP->manipulatorEventServiceOut->put(state);
		}

	// check mode
	switch (data->robotModeData.robotMode)
	{
	case PrimaryAndSecondaryClientInterfaceParser::ROBOT_EMERGENCY_STOPPED_MODE:
	{
		std::cout << "[UniversalRobotic::checkState] emergency stopped\n";
		break;
	}

	case PrimaryAndSecondaryClientInterfaceParser::ROBOT_SECURITY_STOPPED_MODE:
	{
		std::cout << "[UniversalRobotic::checkState] security stopped\n";

		CommManipulatorEventState state;
		state.setEvent(ManipulatorEvent::COLLISION);
		COMP->manipulatorEventServiceOut->put(state);

		manipulator->setMode(URManipulator::MODE_RUN);
		break;
	}
	}
}

//void UniversalRobotic::applyGlobalParameters()
//{
//	//-------- copy parameters --------
////	CHS::SmartGuard parGuard(COMP->ParameterMutex);
//
////	if (globalParameters.modified)
////	{
//
//		// copy global parameters for local use which must not be synchronized
////		globalParameters.modified = false;
////		localParameters = globalParameters;
////		parGuard.release();
//
//		//-------- set parameters to UniversalRobotic --------
//		CHS::SmartGuard guard(COMP->URMutex);
//
//		guard.release();
////	}
//}

void UniversalRobotic::handleException(const std::string& message)
{
	activated = false;
	std::cout << "[UniversalRobotic] " << message << "\n";
}

void UniversalRobotic::handleException(const std::string& message, CommManipulatorObjects::ManipulatorEvent event)
{
	handleException(message);

	// send event to other components
	CommManipulatorObjects::CommManipulatorEventState state;
	state.setEvent(event);
	COMP->manipulatorEventServiceOut->put(state);
}
