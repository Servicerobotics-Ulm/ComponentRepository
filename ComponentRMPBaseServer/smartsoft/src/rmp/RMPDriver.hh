/*
 * RMPWrapper.h
 *
 *  Created on: Aug 4, 2010
 *      Author: wopfner
 *  Updates on: Jan, 2018
 *     Authors: Lutz
 */

// includes

// remove define LPSECURITY_ATTRIBUTES because this
// name is used as a variable in WinTypes.h
#define RMP_DRIVER_TEMP LPSECURITY_ATTRIBUTES
#undef LPSECURITY_ATTRIBUTES

#include "rmp_frame.h"
#include "canio_rmpusb.h"

// restore the define
#define LPSECURITY_ATTRIBUTES RMP_DRIVER_TEMP

#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include <fstream>

#ifndef RMPWRAPPER_H_
#define RMPWRAPPER_H_

class RMPDriver
{

public:
	enum OperationalMode
	{
		DISABLED = 0, TRACTOR = 1, BALANCE = 2, POWER_DOWN = 3
	};

	enum BalanceMode
	{
		BALANCE_ALLOWED = 0, LOCKOUT_BALANCE = 1
	};

	struct OdometryData
	{
		double x, y, yaw;
		double vel_x, vel_yaw;
	};

	struct RMPStatusData
	{
		double right_torque, left_torque;
		double power_battery, ui_battery;
		double time_active;

		RMPDriver::OperationalMode mode;

		uint16_t gain, frames;
		uint16_t velocity_cmd, turn_cmd;
	};

	struct IMUData
	{
		double pitch, roll;
		double vel_yaw, vel_pitch, vel_roll;
	};

	struct RMPParameters
	{
		RMPParameters()
		{
			SPEED_TIMEOUT = 500;
			INITIALIZATION_SLEEP_TIME = 2000;

			COUNT_PER_M = 40181;
			COUNT_PER_DEG = 7.8;
			COUNT_PER_M_PER_S = 401;
			COUNT_PER_M_PER_S_CMD = 328;
			COUNT_PER_DEG_PER_S = 7.8;
			COUNT_PER_DEG_PER_S_CMD = 7.8; //TODO
			COMMAND_COUNT_PER_M_PER_S = 401;
			COMMAND_COUNT_PER_DEG_PER_S = 7.8;
			COUNT_PER_REV = 117031;
			COUNT_PER_NM = 1463;

			COUNT_DRIFT_OFFSET = 10;

			POWERBASE_BATTERY = 4;
			USER_INTERFACE_BATTERY = 4;

			MAX_TRANS_VEL_COUNT = 1176;
			MAX_ROT_VEL_COUNT = 1024;

		}

		uint32_t COUNT_PER_M;
		double COUNT_PER_DEG;
		uint32_t COUNT_PER_M_PER_S;     // count/m/s
		uint32_t COUNT_PER_M_PER_S_CMD; // count/m/s command
		double COUNT_PER_DEG_PER_S;
		double COUNT_PER_DEG_PER_S_CMD;
		uint32_t COMMAND_COUNT_PER_M_PER_S;
		double COMMAND_COUNT_PER_DEG_PER_S;
		uint32_t COUNT_PER_REV;
		uint32_t COUNT_PER_NM;

		int16_t COUNT_DRIFT_OFFSET;

		// time between two speed commands in msec.
		// if the last speed command is older than this time, the robot is stopped.
		uint32_t SPEED_TIMEOUT;

		// in msecs
		uint32_t INITIALIZATION_SLEEP_TIME;

		double POWERBASE_BATTERY;
		double USER_INTERFACE_BATTERY;

		// max values are in counts
		int32_t MAX_TRANS_VEL_COUNT;
		int32_t MAX_ROT_VEL_COUNT;
	};

private:
	enum RMPActiveState
	{
		NEUTRAL, INITIALIZING, ACTIVE
	};

	std::string serial;
	CANIO *canio;

	bool verbose, debug;

	// 1 = front is forward, -1 = back is forward
	int driveDirection;

	OdometryData odometry_data;
	IMUData imu_data;
	RMPStatusData base_status;

	float max_xspeed, max_yawspeed;
	float des_xspeed, des_yawspeed;	// xspeed, yawspeed that should be reached
	float curr_xspeed, curr_yawspeed;
	double time_active;

	// For handling rollover
	uint32_t last_raw_yaw, last_raw_left, last_raw_right, last_raw_foreaft;
	uint16_t last_frames_raw;

	// save configuration commands
	uint16_t max_vel_scale_factor, max_acc_scale_factor, max_turn_scale_factor, limit_scale_factor;

	// Odometry calculation
	double odom_x, odom_y, odom_yaw;

	RMPActiveState activeState;
	bool motor_enabled;
	bool firstread;

	struct timeval lastSpeedUpdate;
	struct timeval lastVelocityCommand;

	RMPParameters params;

	// values for velocity controller
	float error_trans, error_rot; // velocity error
	int esum_trans, esum_rot; // correction for longterm error

	// Accelerations in m/s^2
	float forwardAcc, forwardDec, backwardAcc, backwardDec;
	float leftwardAcc, leftwardDec, rightwardAcc, rightwardDec;

	// debug
	//std::ofstream outputFile;
	//int16_t tmp_trans;


public:
	RMPDriver();

	virtual ~RMPDriver();

	inline void setVerbose(bool v)
	{
		this->verbose = v;
	}

	inline void setDebug(bool d)
	{
		this->debug = d;
	}

	bool setup(const std::string& serial, int driveDirection = 1, const RMPParameters& parameters = RMPParameters());

	// set acceleration, deceleration values [m/s^2]
	void setAccelerations(double forwardAcc, double forwardDec, double backwardAcc, double backwardDec,
			              double rightwardAcc, double rightwardDec, double leftwardAcc, double leftwardDec );

	void shutdown();
	void resetOdometry();
	bool update();

	bool isActive()
	{
		return this->activeState == ACTIVE;
	}

	const OdometryData& getOdometryData() const
	{
		return this->odometry_data;
	}

	const IMUData& getIMUData() const
	{
		return this->imu_data;
	}

	const RMPStatusData& getStatusData() const
	{
		return this->base_status;
	}

	inline void enableMotors(bool enabled)
	{
		this->motor_enabled = enabled;
	}

	void setSpeed(float xspeed, float yawspeed);

	// set speed in x direction [m/s]
	void setXSpeed(float xspeed);

	// set rotational speed [rad/s]
	void setYawSpeed(float yawspeed);

	// set maximum x speed in m/sec
	inline void setMaxXSpeed(float max)
	{
		this->max_xspeed = max;
	}

	// set maximum yaw speed in rad/sec
	inline void setMaxYawSpeed(float max)
	{
		this->max_yawspeed = max;
	}

	// Maximum velocity scale factor
	bool setMaxVelocityScaleFactor(uint16_t vel);

	// Maximum acceleration scale factor
	bool setMaxAccelerationScaleFactor(uint16_t acc);

	// Maximum turn rate scale factor
	bool setMaxTurnRateScaleFactor(uint16_t turn);

	// Gain schedule
	bool setGainSchedule(uint16_t gain);

	// Current limit scale factor
	bool setLimitScaleFactor(uint16_t scale);

	// Balance mode lockout
	bool setBalanceModeLockout(BalanceMode mode);

	// Operational mode
	bool setOperationalMode(OperationalMode mode);

private:
	void init();

	void updateData(rmp_frame_t *data_frame);

	// helper to read a cycle of data from the RMP
	int read();

	// helper to write a packet
	int write(CanPacket& pkt);

	// helper to create a status command packet from the given args
	void makeStatusCommand(CanPacket* pkt, uint16_t cmd, uint16_t val);

	// helper to take make a velocity command
	void makeVelocityCommand(CanPacket* pkt);

	int16_t makeVelocityCommand_transvel(double dT);

	int16_t makeVelocityCommand_rotvel(double dT);

	// Calculate the difference between two raw counter values, taking care of rollover.
	int diffUint32(uint32_t from, uint32_t to, bool first);

	// Calculate the difference between two raw counter values, taking care of rollover.
	int diffUint16(uint16_t from, uint16_t to, bool first);

	inline double DTOR(double degree)
	{
		return degree * (M_PI / 180.0);
	}

	inline double RTOD(double rad)
	{
		return rad * (180.0 / M_PI);
	}

};

#endif /* RMPWRAPPER_H_ */
