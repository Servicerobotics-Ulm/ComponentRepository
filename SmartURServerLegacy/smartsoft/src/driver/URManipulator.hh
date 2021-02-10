/*
 * URManipulator.h
 *
 *  Created on: Jan 26, 2011
 *      Author: wopfner
 *  Updates on: 2017
 *     Authors: Lutz
 *  Updates on 2020
 *     Authors: Lutz
 */

#ifndef URMANIPULATOR_H_
#define URMANIPULATOR_H_

#include "Socket.hh"
#include "PrimaryAndSecondaryClientInterfaceParser.hh"
#include "RealTimeInterfaceParser.hh"

#include <iostream>
#include <thread>
#include <atomic>
#include <future>
#include <cstring>

//#ifdef WITH_MRPT_2_0_VERSION
//
//#else
//#include <mrpt/system.h>
//#include <mrpt/synch/CSemaphore.h>
//#include <mrpt/synch/CThreadSafeVariable.h>
//#endif

class URManipulator
{

public:
	enum Mode
	{
		MODE_FREEDRIVE, MODE_RUN, MODE_READY, MODE_BACKDRIVE
	};

	enum Program
	{
		PROGRAM_RUN, PROGRAM_STOP, PROGRAM_PAUSE, PROGRAM_RESUME
	};

	enum InputRange
	{
		MODE_0V_5V = 0, MODE_n5V_5V = 1, MODE_0V_10V = 2, MODE_n10V_10V = 3
	};

	class ConnectionException: public std::exception
	{
	private:
		std::string message;

	public:
		ConnectionException(const std::string& message) throw()
		{
			this->message = message;
		}

		~ConnectionException() throw()
		{
		}

		virtual const char* what() const throw()
		{
			return this->message.c_str();
		}
	};

private:
	class ControllerUpdateTask
	{

	private:
		URManipulator& manipulator;

	public:
		ControllerUpdateTask(URManipulator& m) :
			manipulator(m)
		{
		}

		void svc()
		{
			const size_t buf_size = 2000;
			char buf[buf_size];

			size_t left_over = 0;
			char* copy_pos = buf;

			while (manipulator.controllerSocket.isConnected())
			{
				size_t len = manipulator.controllerSocket.read(copy_pos, buf_size - left_over);
				std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data = std::make_shared<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData>();
				char* ptr = PrimaryAndSecondaryClientInterfaceParser::parse(data, buf, left_over + len);
				std::atomic_store(&manipulator.controllerData, data);
				manipulator.hasControllerDataVal.store(true);

				left_over = buf + buf_size - ptr;
				copy_pos = buf + left_over;
				memcpy(buf, ptr, left_over);
			}
		}

		void start()
		{
			std::thread(&ControllerUpdateTask::svc, this).detach();
			//mrpt::system::createThreadFromObjectMethod(this, &ControllerUpdateTask::svc, 0);
		}
	};

	class RealtimeUpdateTask
	{

	private:
		URManipulator& manipulator;

	public:
		RealtimeUpdateTask(URManipulator& m) :
			manipulator(m)
		{
		}

		void svc()
		{
			const size_t buf_size = 2000;
			char buf[buf_size];

			size_t left_over = 0;
			char* copy_pos = buf;

			while (manipulator.realtimeSocket.isConnected())
			{
				size_t len = manipulator.realtimeSocket.read(copy_pos, buf_size - left_over);

				std::shared_ptr<RealTimeInterfaceParser::RealtimeImterfaceData> data = std::make_shared<RealTimeInterfaceParser::RealtimeImterfaceData>();
				char* ptr = RealTimeInterfaceParser::parse(data, buf, left_over + len);
				std::atomic_store(&manipulator.realtimeData, data);
				manipulator.hasRealtimeDataVal.store(true);

				left_over = buf + buf_size - ptr;
				copy_pos = buf + left_over;
				memcpy(buf, ptr, left_over);
			}
		}

		void start()
		{
			std::thread(&RealtimeUpdateTask::svc, this).detach();
//			mrpt::system::createThreadFromObjectMethod(this, &RealtimeUpdateTask::svc, 0);
		}

	};

	class CalibrateTimeoutTask
	{
	private:
		URManipulator& manipulator;
		bool run;

	public:
		CalibrateTimeoutTask(URManipulator& m) :
			manipulator(m)
		{
		}

		void svc()
		{
			while (run)
			{


				if (manipulator.calibSemaphore.get_future().wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
//					if (!manipulator.calibSemaphore.waitForSignal(100))
				{
					stringstream p;
					p << "run program\n";
					p << "def do_speedj_init():\n";
					p << "	speedj_init([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.5, 100.0)\n";
					p << "end\n";
					manipulator.sendProgram(p.str());
				}
			}
		}

		void start()
		{
			run = true;
			std::thread(&CalibrateTimeoutTask::svc, this).detach();
//			mrpt::system::createThreadFromObjectMethod(this, &CalibrateTimeoutTask::svc, 0);
		}

		void stop()
		{
			run = false;
		}
	};

private:
	Socket controllerSocket;
	Socket realtimeSocket;

	Socket dashBoardSocket;

//	mrpt::synch::CThreadSafeVariable<bool> hasControllerDataVal;
//	mrpt::synch::CThreadSafeVariable<bool> hasRealtimeDataVal;

	std::atomic<bool> hasControllerDataVal;
	std::atomic<bool> hasRealtimeDataVal;

	std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> controllerData = std::make_shared<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData>();
	std::shared_ptr<RealTimeInterfaceParser::RealtimeImterfaceData> realtimeData  = std::make_shared<RealTimeInterfaceParser::RealtimeImterfaceData>();

//	mrpt::synch::CThreadSafeVariable<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> controllerData;
//	mrpt::synch::CThreadSafeVariable<RealTimeInterfaceParser::RealtimeImterfaceData> realtimeData;

	ControllerUpdateTask controllerTask;
	RealtimeUpdateTask realtimeTask;

	CalibrateTimeoutTask calibrateTimeoutTask;
//	mrpt::synch::CSemaphore calibSemaphore;
	std::promise<void> calibSemaphore;

	bool debug_on;

public:
	URManipulator(const std::string& host, bool activateControllerTask = true, bool activateRealtimeTask = false, bool debug = false);
	virtual ~URManipulator();

	void setReal();
	void setSim();

	/**
	 * set the speed percentage [0;1]
	 */
	void setSpeed(float speed);
	void setMode(URManipulator::Mode mode);

	void powerOn();
	void powerOff();
	void shutdown();

	void setProgramMode(URManipulator::Program program);
	void sendProgram(const std::string& program);

	void setDigitalOutput(int port, bool level);
	void setAnalogOutput(int port, float level);

	/**
	 * set the input range of the analog inputs
	 */
	void setAnalogInputRange(int port, InputRange range);

	/**
	 * mass in [kg]
	 */
	void setPayload(double mass);

	/**
	 * 3d vector of the gravity direction [m/s^2]
	 */
	void setGravity(double x = 0.0, double y = 0.0, double z = 9.81);

	/**
	 * Set the tool center point from the output flange.
	 * x, y, z in [m]
	 * yaw, pitch, roll in [rad]
	 */
	void setTCP(double x, double y, double z, double yaw, double pitch, double roll);

	/**
	 * x, y, z in [m]
	 * yaw. pitch, roll in [rad]
	 * a in m/s^2
	 * v in m/s
	 */
	void moveToTCP(double x, double y, double z, double yaw, double pitch, double roll, double a = 1.0, double v = 1.0);

	/**
	 * x, y, z in [m]
	 * yaw. pitch, roll in [rad]
	 * a in m/s^2
	 * v in m/s
	 */
	void moveLinearToTCP(double x, double y, double z, double yaw, double pitch, double roll, double a = 1.0, double v = 1.0);

	/**
	 * joint positions in [rad]
	 * time in [sec]
	 */
	void moveToJoints(const std::vector<double>& jointPos, double time = 0.0);

	void startCalibration()
	{
		calibrateTimeoutTask.start();
	}

	/**
	 * Automatically stops the movement if no new command
	 * is set within 10ms.
	 *
	 * w/s - robot
	 * e/d - base
	 * r/f - shoulder
	 * t/g - elbow
	 * z/h - wrist 1
	 * u/j - wrist 2
	 * i/k - wrist 3
	 */
	void calibrate(char c);

	void stopCalibration()
	{
		calibrateTimeoutTask.stop();
	}

	bool hasControllerData() const
	{
		return hasControllerDataVal.load();
	}

	bool hasRealtimeData() const
	{
		return hasRealtimeDataVal.load();
	}

	std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> getControllerData()
	{
		return atomic_load(&controllerData);
	}

	std::shared_ptr<RealTimeInterfaceParser::RealtimeImterfaceData> getRealtimeData()
	{
		return atomic_load(&realtimeData);
	}

	uint32_t getJointCount() const
	{
		return 6;
	}

};

#endif /* URMANIPULATOR_H_ */
