/*
 * URManipulator.cpp
 *
 *  Created on: Jan 26, 2011
 *      Author: wopfner
 *  Updates on: 2017
 *     Authors: Lutz
 *  Updates on 2020
 *     Authors: Lutz
 */

#include "URManipulator.hh"

#include <sstream>
#include <iomanip>
#include <ace/ACE.h>

URManipulator::URManipulator(const std::string& host, bool activateControllerTask, bool activateRealtimeTask, bool debug) :
	controllerTask(*this), realtimeTask(*this), calibrateTimeoutTask(*this)
{
	debug_on = debug;
	hasControllerDataVal.store(false);
	hasRealtimeDataVal.store(false);

	// controller task
	if (activateControllerTask)
	{
		if (!controllerSocket.connect(host, 30002))
		{
			throw ConnectionException("Connection to " + host + ":30002 failed");
		}
		controllerTask.start();
	}

	// realtime task
	if (activateRealtimeTask)
	{
		if (!realtimeSocket.connect(host, 30003))
		{
			throw ConnectionException("Connection to " + host + ":30003 failed");
		}
		realtimeTask.start();
	}
}

URManipulator::~URManipulator()
{
}

void URManipulator::setReal()
{
	std::string send = "set real\n";
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::setSim()
{
	std::string send = "set sim\n";
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::setSpeed(float speed)
{
	if (speed < 0.0)
	{
		speed = 0.0;
	} else if (speed > 1.0)
	{
		speed = 1.0;
	}

	stringstream p;
	p << "set speed ";
	p << std::fixed << std::setprecision(2) << speed;
	p << "\n";
	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::setMode(URManipulator::Mode mode)
{
	std::string send;

	switch (mode)
	{
	case MODE_FREEDRIVE:
		send = "set robotmode freedrive\n";
		break;
	case MODE_RUN:
		send = "set robotmode run\n";
		break;
	case MODE_READY:
		send = "set robotmode ready\n";
		break;
	case MODE_BACKDRIVE:
		send = "set robotmode backdrive\n";
		break;
	}

	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::setProgramMode(URManipulator::Program program)
{
	std::string send;

	switch (program)
	{
	case PROGRAM_RUN:
		send = "run program\n";
		break;
	case PROGRAM_STOP:
		send = "stop program\n";
		break;
	case PROGRAM_PAUSE:
		send = "pause program\n";
		break;
	case PROGRAM_RESUME:
		send = "resume program\n";
		break;
	}

	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::powerOn()
{
	std::string send = "power on\n";
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::powerOff()
{
	std::string send = "power off\n";
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::shutdown()
{
	std::string send = "powerdown()\n";
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::sendProgram(const std::string& program)
{
	controllerSocket.write(program.c_str(), program.size());
}

void URManipulator::setAnalogInputRange(int port, URManipulator::InputRange range){

	stringstream p;

	p << "run program\n";
	p << "def init_robot():\n";
	p << "	set_analog_inputrange(";
	p << port << ", ";
	p << (int) range << ")\n";
	p << "end\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
	ACE_OS::sleep(ACE_Time_Value(0,50000));
}

void URManipulator::setDigitalOutput(int port, bool level){
	stringstream p;

	p << "set_digital_out(";
	p << port << ",";
	if(level){
		p << "True" << ")\n";
	}else{
		p << "False" << ")\n";
	}

	//std::cout<<__FUNCTION__<<": "<<p.str()<<std::endl;
	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
	//0.05 sec
	ACE_OS::sleep(ACE_Time_Value(0,50000));
}

void URManipulator::setAnalogOutput(int port, float level){
	stringstream p;
	p << "set_analog_out(";
	p << port << ", ";
	p << level << ")\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
	ACE_OS::sleep(ACE_Time_Value(0,50000));
}

void URManipulator::setPayload(double mass)
{
	stringstream p;

	p << "set_payload(";
	p << mass;
	p << ")\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::setGravity(double x, double y, double z)
{
	stringstream p;

	p << "set_gravity([";
	p << x << ", ";
	p << y << ", ";
	p << z;
	p << "])\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::setTCP(double x, double y, double z, double yaw, double pitch, double roll)
{
	stringstream p;

	p << "set_tcp([";
	p << x << ", ";
	p << y << ", ";
	p << z << ", ";
	p << yaw << ", ";
	p << pitch << ", ";
	p << roll << "]";
	p << ")\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::moveToTCP(double x, double y, double z, double yaw, double pitch, double roll, double a, double v)
{
	stringstream p;

	p << "movej([";
	p << x << ", ";
	p << y << ", ";
	p << z << ", ";
	p << yaw << ", ";
	p << pitch << ", ";
	p << roll << "], ";
	p << "a=" << a << ", ";
	p << "v=" << v;
	p << ")\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::moveLinearToTCP(double x, double y, double z, double yaw, double pitch, double roll, double a, double v)
{
	stringstream p;

	p << "movel([";
	p << x << ", ";
	p << y << ", ";
	p << z << ", ";
	p << yaw << ", ";
	p << pitch << ", ";
	p << roll << "], ";
	p << "a=" << a << ", ";
	p << "v=" << v;
	p << ")\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::moveToJoints(const std::vector<double>& jointPos, double time)
{
	stringstream p;

	p << "servoj([";
	for (size_t i = 0; i < jointPos.size(); i++)
	{
		p << jointPos[i];
		if (i < jointPos.size() - 1)
		{
			p << ", ";
		}
	}
	p << "], ";
	p << "t=" << time;
	p << ")\n";

	std::string send = p.str();
	controllerSocket.write(send.c_str(), send.size());
}

void URManipulator::calibrate(char c)
{
	stringstream p;
	p << "run program\n";
	p << "def do_speedj_init():\n";
	p << "	speedj_init([";

	switch (c)
	{
	case 'w':
	{
		std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = getControllerData();
		for (size_t i = 0; i < data->jointData.size(); i++)
		{
			if (data->jointData[i].jointMode == PrimaryAndSecondaryClientInterfaceParser::JOINT_INITIALISATION_MODE)
				p << "0.1";
			else
				p << "0.0";

			if (i < data->jointData.size() - 1)
				p << ", ";
		}
		break;
	}

	case 's':
	{
		std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = getControllerData();
		for (size_t i = 0; i < data->jointData.size(); i++)
		{
			if (data->jointData[i].jointMode == PrimaryAndSecondaryClientInterfaceParser::JOINT_INITIALISATION_MODE)
				p << "-0.1";
			else
				p << "0.0";

			if (i < data->jointData.size() - 1)
				p << ", ";
		}
		break;
	}

	case 'e':
	{
		p << "0.1, 0.0, 0.0, 0.0, 0.0, 0.0";
		break;
	}

	case 'd':
	{
		p << "-0.1, 0.0, 0.0, 0.0, 0.0, 0.0";
		break;
	}

	case 'r':
	{
		p << "0.0, 0.1, 0.0, 0.0, 0.0, 0.0";
		break;
	}

	case 'f':
	{
		p << "0.0, -0.1, 0.0, 0.0, 0.0, 0.0";
		break;
	}

	case 't':
	{
		p << "0.0, 0.0, 0.1, 0.0, 0.0, 0.0";
		break;
	}

	case 'g':
	{
		p << "0.0, 0.0, -0.1, 0.0, 0.0, 0.0";
		break;
	}

	case 'z':
	{
		p << "0.0, 0.0, 0.0, 0.1, 0.0, 0.0";
		break;
	}

	case 'h':
	{
		p << "0.0, 0.0, 0.0, -0.1, 0.0, 0.0";
		break;
	}

	case 'u':
	{
		p << "0.0, 0.0, 0.0, 0.0, 0.1, 0.0";
		break;
	}

	case 'j':
	{
		p << "0.0, 0.0, 0.0, 0.0, -0.1, 0.0";
		break;
	}

	case 'i':
	{
		p << "0.0, 0.0, 0.0, 0.0, 0.0, 0.1";
		break;
	}

	case 'k':
	{
		p << "0.0, 0.0, 0.0, 0.0, 0.0, -0.1";
		break;
	}

	}

	p << "], 1.5, 100.0)\n";
	p << "end\n";

	calibSemaphore.set_value();
	sendProgram(p.str());

}
