/*
 * RMPWrapper.cpp
 *
 *  Created on: Aug 4, 2010
 *      Author: wopfner
 *  Updates on: Jan, 2018
 *     Authors: Lutz
 */

#include "RMPDriver.hh"

#include <iostream>

RMPDriver::RMPDriver() :
	canio(NULL)
{
	verbose = false;
	driveDirection = 1;
	esum_rot = 0;
	esum_trans = 0;

	forwardAcc = 0.2;
	forwardDec = 1;
	backwardAcc = 0.2;
	backwardDec = 0.2;

	rightwardAcc = 0.3;
	rightwardDec = 0.3;
	leftwardAcc = 0.3;
	leftwardDec = 0.3;

//	outputFile.open("b.mat");
}

RMPDriver::~RMPDriver()
{
	this->shutdown();
}

bool RMPDriver::setup(const std::string& serial, int driveDirection, const RMPParameters& parameters)
{

	std::cout << "[RMPDriver] setup\n";

	this->serial = serial;
	this->driveDirection = driveDirection;
	this->params = parameters;
	this->motor_enabled = false;

	this->odom_x = this->odom_y = this->odom_yaw = 0.0;

	this->max_vel_scale_factor = 16;
	this->max_acc_scale_factor = 16;
	this->max_turn_scale_factor = 16;
	this->limit_scale_factor = 256;

	// set max speeds
	this->max_xspeed = 2.5;
	this->max_yawspeed = 0.7;

	this->shutdown();
	this->canio = new CANIOrmpusb(this->serial.c_str());
	if (this->canio->Init() != FT_OK)
	{
		std::cout << "[RMP Driver] error on CAN Init\n";
		return false;
	}

	// set state to initializing
	this->activeState = INITIALIZING;

	return true;
}

void RMPDriver::setAccelerations(double forwardAcc, double forwardDec, double backwardAcc, double backwardDec,
		                         double rightwardAcc, double rightwardDec, double leftwardAcc, double leftwardDec )
{
	this->forwardAcc = forwardAcc;
	this->forwardDec = forwardDec;
	this->backwardAcc = backwardAcc;
	this->backwardDec = backwardDec;

	this->rightwardAcc = rightwardAcc;
	this->rightwardDec = rightwardDec;
	this->leftwardAcc = leftwardAcc;
	this->leftwardDec = leftwardDec;
}

void RMPDriver::shutdown()
{
	if (canio != 0)
	{
		// send zero velocities, for some semblance of safety
		CanPacket pkt;

		this->des_xspeed = this->des_yawspeed = 0;
		this->curr_xspeed = this->curr_yawspeed = 0;
		makeVelocityCommand(&pkt);
		if (write(pkt) != FT_OK)
		{
			std::cerr << "[RMP Driver] Init speed command not sent!\n";
		}

		// shutdown the device
		canio->Shutdown();
		delete canio;
		canio = NULL;
	}
}

bool RMPDriver::update()
{
	CanPacket pkt;

	if (activeState == INITIALIZING)
	{
		// wait till rmp setup is done
		usleep(params.INITIALIZATION_SLEEP_TIME * 1000);
		init();
	}

	// Read from the RMP
	if (read() != FT_OK)
	{
		std::cerr << "[RMP Driver] Read() errored; bailing\n";
		return false;
	}

	// check to see how long since the last command
	struct timeval curr;
	// get the current time
	gettimeofday(&curr, NULL);

	// calculate how long since the last command
	double msecs = (curr.tv_sec - this->lastSpeedUpdate.tv_sec) * 1000 + (curr.tv_usec - this->lastSpeedUpdate.tv_usec) / 1000;

	if (this->activeState == ACTIVE && msecs > this->params.SPEED_TIMEOUT)
	{
		if (this->debug || this->des_xspeed > 0 || this->des_yawspeed > 0)
		{
			std::cerr << "[RMP Driver] timeout exceeded: " << (int) msecs << " msecs since last command stopping robot\n";
			this->des_xspeed = 0;
			this->des_yawspeed = 0;
		}
	}

	if (!motor_enabled || this->activeState != ACTIVE)
	{
		this->des_xspeed = 0;
		this->des_yawspeed = 0;
	}

	// make a velocity command... could be zero
	makeVelocityCommand(&pkt);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] error on write\n";
		return false;
	}

	//	if (fabs(this->curr_xspeed) < fabs(this->odometry_data.vel_x) && this->params.COMMAND_COUNT_PER_M_PER_S > 0)
	//	{
	//		this->params.COMMAND_COUNT_PER_M_PER_S -= 1;
	//	} else if (fabs(this->curr_xspeed) > fabs(this->odometry_data.vel_x) && this->params.COMMAND_COUNT_PER_M_PER_S < 10000)
	//	{
	//		this->params.COMMAND_COUNT_PER_M_PER_S += 1;
	//	}
	//
	//	//	std::cout << this->params.COMMAND_COUNT_PER_M_PER_S << "\n";

	return true;
}

void RMPDriver::resetOdometry()
{
	if (this->verbose)
	{
		std::cout << "[RMP Driver] resetting odometry ...\n";
	}

	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_RST_INT, (uint16_t) RMP_CAN_RST_ALL);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to reset odometry\n";
	}

	this->odom_x = this->odom_y = this->odom_yaw = 0.0;

	this->activeState = INITIALIZING;
}

void RMPDriver::setSpeed(float xspeed, float yawspeed)
{
	setXSpeed(xspeed);
	setYawSpeed(yawspeed);
	gettimeofday(&(this->lastSpeedUpdate), NULL);
}

void RMPDriver::setXSpeed(float xspeed)
{
	if (xspeed > this->max_xspeed)
	{
		printf("xspeed thresholded! (%4.2f > %4.2f)\n", xspeed, this->max_xspeed);
		this->des_xspeed = this->max_xspeed;
	} else if (xspeed < -this->max_xspeed)
	{
		printf("xspeed thresholded! (%4.2f < %4.2f)\n", xspeed, -this->max_xspeed);
		this->des_xspeed = -this->max_xspeed;
	} else
	{
		this->des_xspeed = xspeed;
	}

	gettimeofday(&(this->lastSpeedUpdate), NULL);
}

void RMPDriver::setYawSpeed(float yawspeed)
{
	if (yawspeed > this->max_yawspeed)
	{
		printf("yawspeed thresholded! (%4.2f > %4.2f)\n", yawspeed, this->max_yawspeed);
		this->des_yawspeed = this->max_yawspeed;
	} else if (yawspeed < -this->max_yawspeed)
	{
		printf("yawspeed thresholded! (%4.2f < %4.2f)\n", yawspeed, -this->max_yawspeed);
		this->des_yawspeed = -this->max_yawspeed;
	} else
	{
		this->des_yawspeed = yawspeed;
	}

	gettimeofday(&(this->lastSpeedUpdate), NULL);
}

bool RMPDriver::setMaxVelocityScaleFactor(uint16_t vel)
{
	this->max_vel_scale_factor = vel;

	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_MAX_VEL, vel);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to set maximum velocity scale factor to " << vel << "\n";
		return false;
	}
	return true;
}

bool RMPDriver::setMaxAccelerationScaleFactor(uint16_t acc)
{
	this->max_acc_scale_factor = acc;

	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_MAX_ACCL, acc);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to set maximum acceleration scale factor to " << acc << "\n";
		return false;
	}
	return true;
}

bool RMPDriver::setMaxTurnRateScaleFactor(uint16_t turn)
{
	this->max_turn_scale_factor = turn;

	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_MAX_TURN, turn);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to set maximum turn rate scale factor to " << turn << "\n";
		return false;
	}
	return true;
}

bool RMPDriver::setGainSchedule(uint16_t gain)
{
	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_GAIN_SCHED, gain);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to set gain schedule to " << gain << "\n";
		return false;
	}
	return true;
}

bool RMPDriver::setLimitScaleFactor(uint16_t scale)
{
	this->limit_scale_factor = scale;

	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_CURR_LIMIT, scale);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to set current limit scale factor to " << scale << "\n";
		return false;
	}
	return true;
}

bool RMPDriver::setBalanceModeLockout(BalanceMode mode)
{
	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_ENABLE_MODE, (uint16_t) mode);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to set balance mode lockout to " << mode << "\n";
		return false;
	}
	return true;
}

bool RMPDriver::setOperationalMode(OperationalMode mode)
{
	CanPacket pkt;
	makeStatusCommand(&pkt, (uint16_t) RMP_CAN_CMD_SET_MODE, (uint16_t) mode);
	if (write(pkt) != FT_OK)
	{
		std::cerr << "[RMP Driver] Failed to set operational mode to " << mode << "\n";
		return false;
	}
	return true;
}

////////////////////////////////////////////////
// private methods
////////////////////////////////////////////////

void RMPDriver::init()
{
	std::cout << "[RMP Driver] initializing RMP ...\n";

	this->firstread = true;

	this->last_frames_raw = 0;
	this->time_active = 0;

	this->last_raw_yaw = this->last_raw_left = 0;
	this->last_raw_right = this->last_raw_foreaft = 0;

	this->des_xspeed = this->des_yawspeed = 0.0;
	this->curr_xspeed = this->curr_yawspeed = 0.0;

	setMaxVelocityScaleFactor(this->max_vel_scale_factor);
	setMaxAccelerationScaleFactor(this->max_acc_scale_factor);
	setMaxTurnRateScaleFactor(this->max_turn_scale_factor);
	setLimitScaleFactor(this->limit_scale_factor);

	// set the initial time
	gettimeofday(&(this->lastSpeedUpdate), NULL);
	gettimeofday(&(this->lastVelocityCommand), NULL);

	// set state to active
	this->activeState = ACTIVE;
}

void RMPDriver::updateData(rmp_frame_t *data_frame)
{
	int delta_lin_raw, delta_ang_raw, delta_frames_raw;
	double delta_lin, delta_ang;
	double tmp;

	// Get the new linear and angular encoder values and compute odometry.
	delta_lin_raw = diffUint32(this->last_raw_foreaft, data_frame->foreaft, this->firstread);
	this->last_raw_foreaft = data_frame->foreaft;

	delta_ang_raw = diffUint32(this->last_raw_yaw, data_frame->yaw, this->firstread);
	this->last_raw_yaw = data_frame->yaw;

	delta_lin = (double) delta_lin_raw / (double) params.COUNT_PER_M;
	delta_ang = DTOR((double) delta_ang_raw / (double) params.COUNT_PER_REV * (double) 360.0);

	// First-order odometry integration
	this->odom_x += delta_lin * cos(this->odom_yaw);
	this->odom_y += delta_lin * sin(this->odom_yaw);
	this->odom_yaw += delta_ang;

	// Normalize yaw in [0, 360]
	this->odom_yaw = atan2(sin(this->odom_yaw), cos(this->odom_yaw));
	if (this->odom_yaw < 0)
		this->odom_yaw += (double) 2 * M_PI;

	///////////////////////////////////////////////////
	// set Odometry data
	///////////////////////////////////////////////////
	this->odometry_data.x = this->driveDirection * this->odom_x;
	this->odometry_data.y = this->driveDirection * this->odom_y;
	if (this->odom_yaw > M_PI)
	{
		this->odometry_data.yaw = (this->odom_yaw - ((double) 2 * M_PI));
	} else
	{
		this->odometry_data.yaw = this->odom_yaw;
	}

	// combine left and right wheel velocity to get forward velocity change from counts/s into mm/s
	double comb_vel = ((double) data_frame->left_dot + (double) data_frame->right_dot) / (double) 2.0;
	//	outputFile << "   " << this->tmp_trans << "   " << data_frame->left_dot << "   " << data_frame->right_dot << "   " << comb_vel << "\n";
	this->odometry_data.vel_x = this->driveDirection * (comb_vel / (double) params.COUNT_PER_M_PER_S);

	//this->odometry_data.vel_x = this->driveDirection * data_frame->foreaft / (double) params.COUNT_PER_M_PER_S;

	// This one uses left_dot and right_dot, which comes from odometry
	this->odometry_data.vel_yaw = DTOR((((double) data_frame->right_dot - (double) data_frame->left_dot)
			/ ((double) params.COUNT_PER_DEG_PER_S)));

	if (this->debug)
	{
		std::cout << "[RMP Driver] fore/aft displacement=" << data_frame->foreaft << ", yaw displacement=" << data_frame->yaw << "\n";
		std::cout << "[RMP Driver] vel_x=" << this->odometry_data.vel_x << " m/s, vel_yaw=" << this->odometry_data.vel_yaw << " rad/s \n";
	}

	//outputFile 	<< "   " << this->curr_xspeed << "   " << (double) data_frame->left_dot / 1000.0 << "   "
	//		<< (double) data_frame->right_dot / 1000.0 << "   " << this->odometry_data.vel_x << "\n";

	///////////////////////////////////////////////////
	// set IMU data
	///////////////////////////////////////////////////

	// normalize angles to [0,360]
	tmp = DTOR((double) data_frame->pitch / (double) params.COUNT_PER_DEG);
	if (tmp > M_PI)
		tmp -= 2 * M_PI;
	this->imu_data.pitch = tmp;

	// normalize angles to [0,360]
	tmp = DTOR((double) data_frame->roll / (double) params.COUNT_PER_DEG);
	if (tmp > M_PI)
	{
		tmp -= 2 * M_PI;
	}
	this->imu_data.roll = tmp;

	this->imu_data.vel_yaw = DTOR((double) data_frame->yaw_dot / (double) params.COUNT_PER_DEG_PER_S);
	this->imu_data.vel_roll = DTOR((double) data_frame->roll_dot / (double) params.COUNT_PER_DEG_PER_S);
	this->imu_data.vel_pitch = DTOR((double) data_frame->pitch_dot / (double) params.COUNT_PER_DEG_PER_S);

	///////////////////////////////////////////////////
	// set additionally RMP status information
	///////////////////////////////////////////////////

	this->base_status.power_battery = (double) data_frame->power_battery / this->params.POWERBASE_BATTERY;
	this->base_status.ui_battery = ((double) data_frame->ui_battery / this->params.USER_INTERFACE_BATTERY);

	this->base_status.mode = (RMPDriver::OperationalMode) data_frame->mode;
	this->base_status.gain = data_frame->gain;
	this->base_status.frames = data_frame->frames;

	delta_frames_raw = diffUint16(this->last_frames_raw, data_frame->frames, this->firstread);
	this->last_frames_raw = data_frame->frames;
	this->time_active += (double) delta_frames_raw * 0.01;
	this->base_status.time_active = this->time_active;

	if (data_frame->right_torque > 0x7FFF)
	{
		this->base_status.right_torque = ((double) data_frame->right_torque - (double) 2 * (double) 0x7FFF) / (double) params.COUNT_PER_NM;
	} else
	{
		this->base_status.right_torque = (double) data_frame->right_torque / (double) params.COUNT_PER_NM;
	}

	if (data_frame->left_torque > 0x7FFF)
	{
		this->base_status.left_torque = ((double) data_frame->left_torque - (double) 2 * (double) 0x7FFF) / (double) params.COUNT_PER_NM;
	} else
	{
		this->base_status.left_torque = (double) data_frame->left_torque / (double) params.COUNT_PER_NM;
	}

	if (this->debug)
	{
		std::cout << "[RMP Driver] power_bat=" << this->base_status.power_battery << " V, user_bat=" << this->base_status.ui_battery
				<< "\n";
		std::cout << "[RMP Driver] time_active=" << this->base_status.time_active << " s, right_torque=" << this->base_status.right_torque
				<< " Nm, left_torque=" << this->base_status.left_torque << " Nm\n";
	}

	this->base_status.velocity_cmd = data_frame->velocity_cmd;
	this->base_status.turn_cmd = data_frame->turn_cmd;

	// Calculate velocity error
	this->error_trans = this->curr_xspeed - this->odometry_data.vel_x;
	this->error_rot = this->curr_yawspeed - this->odometry_data.vel_yaw;

	firstread = false;
}

int RMPDriver::read()
{
	CanPacket pkt;
	rmp_frame_t data_frame;

	int ret = 0;
	data_frame.ready = 0;

	// before each read set read start to current time
	struct timeval startRead;
	gettimeofday(&(startRead), NULL);

	// read until we've gotten all 8 packets for this cycle on this channel
	while ((ret = canio->ReadPacket(&pkt)) >= 0)
	{
		// then we got a new packet...

		data_frame.AddPacket(pkt);

		// if data has been filled in, then let's make it the latest data
		if (data_frame.IsReady())
		{
			if (this->activeState == ACTIVE)
			{
				updateData(&data_frame);
				data_frame.ready = 0;
			} else if (this->activeState == NEUTRAL)
			{
				this->activeState = INITIALIZING;
			}
			break;
		}

		// check to see how long since the last command
		struct timeval curr;
		// get the current time
		gettimeofday(&curr, NULL);

		// calculate how long since the last command
		double msecs = (curr.tv_sec - startRead.tv_sec) * 1000 + (curr.tv_usec - startRead.tv_usec) / 1000;

		if (msecs > this->params.SPEED_TIMEOUT)
		{
			this->activeState = NEUTRAL;

			if (this->debug)
			{
				std::cerr << "[RMP Driver] timeout exceeded: " << (int) msecs << " msecs since last read.\n";
			}

			break;
		}
	}

	if (ret < 0)
	{
		std::cerr << "[RMP Driver] error (" << ret << ") reading packet\n";
	}

	return 0;
}

int RMPDriver::write(CanPacket& pkt)
{
	return canio->WritePacket(pkt);
}

void RMPDriver::makeStatusCommand(CanPacket* pkt, uint16_t cmd, uint16_t val)
{

	// set velocity command to packet
	makeVelocityCommand(pkt);

	pkt->id = RMP_CAN_ID_COMMAND;
	pkt->PutSlot(2, cmd); //2

	// it was noted in the windows demo code that they
	// copied the 8-bit value into both bytes like this
	// UPDATE: Changed according to preliminary datasheet
	pkt->PutSlot(3, val);

	if (cmd)
	{
		printf("SEGWAYIO: STATUS: cmd: %02x val: %02x pkt: %s\n", cmd, val, pkt->toString());
	}
}


int16_t RMPDriver::makeVelocityCommand_transvel(double dT)
{

	////////////////////////////////////////////////////////////////////
	// calculate velocity based on the given acceleration, deceleration

	bool accelerate = (fabs(this->des_xspeed) > fabs(this->curr_xspeed));
	bool decelerate = (fabs(this->des_xspeed) < fabs(this->curr_xspeed));

	bool forward = (this->des_xspeed > 0.0) || (this->des_xspeed == 0.0 && this->curr_xspeed > 0);
	bool backward = (this->des_xspeed < 0.0) || (this->des_xspeed == 0.0 && this->curr_xspeed < 0);



	// move forward -> accelerate
	if (forward && accelerate)
	{
		float givenVel = fabs(this->des_xspeed - this->curr_xspeed);
		float maxVel = 	this->forwardAcc * dT;

		this->curr_xspeed += (givenVel > maxVel) ? maxVel : givenVel;
		this->curr_xspeed = (this->curr_xspeed > this->des_xspeed) ? this->des_xspeed : this->curr_xspeed;
	}
	// move backward -> accelerate
	else if (backward && accelerate)
	{
		float givenVel = fabs(this->des_xspeed - this->curr_xspeed);
		float maxVel = 	this->backwardAcc * dT;

		this->curr_xspeed -= (givenVel > maxVel) ? maxVel : givenVel;
		this->curr_xspeed = (this->curr_xspeed < this->des_xspeed) ? this->des_xspeed : this->curr_xspeed;
	}
	// move forward -> decelerate
	else if (forward && decelerate)
	{
		float givenVel = fabs(this->curr_xspeed - this->des_xspeed);
		float maxVel = 	this->forwardDec * dT;

		this->curr_xspeed -= (givenVel > maxVel) ? maxVel : givenVel;
		this->curr_xspeed = (this->curr_xspeed < 0) ? 0 : this->curr_xspeed;
	}
	// move backward -> decelerate
	else if (backward && decelerate)
	{
		float givenVel = fabs(this->curr_xspeed - this->des_xspeed);
		float maxVel = 	this->backwardDec * dT;

		this->curr_xspeed += (givenVel > maxVel) ? maxVel : givenVel;
		this->curr_xspeed = (this->curr_xspeed > 0) ? 0 : this->curr_xspeed;
	} else
	{
		this->curr_xspeed = this->des_xspeed;
	}

//	std::cout << ">>> dT, des_xspeed, curr_xspeed: " << dT << ", " << this->des_xspeed << ", " <<  this->curr_xspeed << std::endl;
//	outputFile << this->des_xspeed << ", " <<  this->curr_xspeed << "\n";
//	outputFile.flush();

	////////////////////////////////////////////////////////////////////

	int ctrans = this->curr_xspeed * (double) params.COUNT_PER_M_PER_S_CMD;

	////////////////////////////////////////////
	// integral controler --- to move robot when low velocities are comanded
	if (this->error_trans > 0)
	{
		this->esum_trans += 2;
	} else if (this->error_trans < 0)
	{
		this->esum_trans -= 2;
	}

	if (this->esum_trans + ctrans > params.MAX_TRANS_VEL_COUNT)
	{
		this->esum_trans = params.MAX_TRANS_VEL_COUNT - ctrans;
	} else if (this->esum_trans + ctrans < -params.MAX_TRANS_VEL_COUNT)
	{
		this->esum_trans = -params.MAX_TRANS_VEL_COUNT - ctrans;
	}

	// if ctrans is zero also esum is zero
	if (ctrans == 0)
	{
		this->esum_trans = 0;
	}

	ctrans += this->esum_trans;
	//std::cout << "esum: " << this->esum_trans << ", ctrans: " << ctrans << "\n";
	////////////////////////////////////////////

	int16_t trans = (int16_t) rint(this->driveDirection * ctrans);

	// limit trans value to allowed range
	if (trans > params.MAX_TRANS_VEL_COUNT)
		trans = params.MAX_TRANS_VEL_COUNT;
	else if (trans < -params.MAX_TRANS_VEL_COUNT)
		trans = -params.MAX_TRANS_VEL_COUNT;

	return trans;
}

int16_t RMPDriver::makeVelocityCommand_rotvel(double dT)
{

	//////////////////////////////////////////////////////////////////////////




	////////////////////////////////////////////////////////////////////
		// calculate velocity based on the given acceleration, deceleration

		bool accelerate = (fabs(this->des_yawspeed) > fabs(this->curr_yawspeed));
		bool decelerate = (fabs(this->des_yawspeed) < fabs(this->curr_yawspeed));

//forward
		bool leftwards = (this->des_yawspeed > 0.0) || (this->des_yawspeed == 0.0 && this->curr_yawspeed > 0);
//backward
		bool rightwards = (this->des_yawspeed < 0.0) || (this->des_yawspeed == 0.0 && this->curr_yawspeed < 0);


//		std::cout << ">>> accelerate("<<accelerate<<") decelerate("<<decelerate<<") leftwards("<<leftwards<<") rightwards("<<rightwards<<")"<< std::endl;

		// move leftward -> accelerate
		if (leftwards && accelerate)
		{
			float givenVel = fabs(this->des_yawspeed - this->curr_yawspeed);
			float maxVel = 	this->leftwardAcc * dT;

			this->curr_yawspeed += (givenVel > maxVel) ? maxVel : givenVel;
			this->curr_yawspeed = (this->curr_yawspeed > this->des_yawspeed) ? this->des_yawspeed : this->curr_yawspeed;
		}
		// move rightward -> accelerate
		else if (rightwards && accelerate)
		{

			float givenVel = fabs(this->des_yawspeed - this->curr_yawspeed);
			float maxVel = 	this->rightwardAcc * dT;

//			std::cout << ">>> this->rightwardAcc("<<this->rightwardAcc<<") rightwards && accelerate givenVel:"<<givenVel<<" maxVel: "<<maxVel<<") leftwards("<<leftwards<<") rightwards("<<rightwards<<")"<< std::endl;
			this->curr_yawspeed -= (givenVel > maxVel) ? maxVel : givenVel;
			this->curr_yawspeed = (this->curr_yawspeed < this->des_yawspeed) ? this->des_yawspeed : this->curr_yawspeed;
		}
		// move leftward -> decelerate
		else if (leftwards && decelerate)
		{
			float givenVel = fabs(this->curr_yawspeed - this->des_yawspeed);
			float maxVel = 	this->leftwardDec * dT;

			this->curr_yawspeed -= (givenVel > maxVel) ? maxVel : givenVel;
			this->curr_yawspeed = (this->curr_yawspeed < 0) ? 0 : this->curr_yawspeed;
		}
		// move rightward -> decelerate
		else if (rightwards && decelerate)
		{
			float givenVel = fabs(this->curr_yawspeed - this->des_yawspeed);
			float maxVel = 	this->rightwardDec * dT;

			this->curr_yawspeed += (givenVel > maxVel) ? maxVel : givenVel;
			this->curr_yawspeed = (this->curr_yawspeed > 0) ? 0 : this->curr_yawspeed;
		} else
		{
			this->curr_yawspeed = this->des_yawspeed;
		}

//		std::cout << ">>> dT, des_yawspeed, curr_yawspeed: " << dT << ", " << this->des_yawspeed << ", " <<  this->curr_yawspeed << std::endl;
	//	outputFile << this->des_xspeed << ", " <<  this->curr_xspeed << "\n";
	//	outputFile.flush();

		////////////////////////////////////////////////////////////////////

		int crot = RTOD((double) this->curr_yawspeed) * (double) params.COUNT_PER_DEG_PER_S_CMD;
//		std::cout << "crot: " << crot << "\n";

		////////////////////////////////////////////
		// integral controler --- to move robot when low velocities are comanded
		if (this->error_rot > 0)
		{
			this->esum_rot += 2;
		} else if (this->error_rot < 0)
		{
			this->esum_rot -= 2;
		}

		if (this->esum_rot + crot > params.MAX_ROT_VEL_COUNT)
		{
			this->esum_rot = params.MAX_ROT_VEL_COUNT - crot;
		} else if (this->esum_rot + crot < -params.MAX_ROT_VEL_COUNT)
		{
			this->esum_rot = -params.MAX_ROT_VEL_COUNT - crot;
		}

		// if crot is zero also esum is zero
		if (crot == 0)
		{
			this->esum_rot = 0;
		}

		crot += this->esum_rot;
//		std::cout << "esum: " << this->esum_rot << ", crot: " << crot << "\n";
		////////////////////////////////////////////


	///////////////////////////////////////////////////////////


//	float yawspeed = this->curr_yawspeed = this->des_yawspeed;

	// rotational RMP command \in [-1024, 1024]
	// this is ripped from rmi_demo... to go from deg/s to counts
	// deg/s -> count = 1/0.013805056
//	int16_t rot = (int16_t) rint(RTOD((double) yawspeed) * (double) params.COMMAND_COUNT_PER_DEG_PER_S);

	int16_t rot = (int16_t) rint(this->driveDirection * crot);

	if (rot > params.MAX_ROT_VEL_COUNT)
		rot = params.MAX_ROT_VEL_COUNT;
	else if (rot < -params.MAX_ROT_VEL_COUNT)
		rot = -params.MAX_ROT_VEL_COUNT;

	return rot;

}



void RMPDriver::makeVelocityCommand(CanPacket* pkt)
{
	pkt->id = RMP_CAN_ID_COMMAND;
	pkt->PutSlot(2, (uint16_t) RMP_CAN_CMD_NONE);


	// we only care about cmd.xspeed and cmd.yawspeed
	// translational velocity is given to RMP in counts
	// [-1176,1176] ([-8mph,8mph])

	////////////////////////////////////////////////////////////////////
	// calculate velocity based on the given acceleration, deceleration


	// calculate deltaT
	struct timeval curr;
	gettimeofday(&curr, NULL); // get the current time

	// calculate how long since the last command
	double dT = (curr.tv_sec - this->lastVelocityCommand.tv_sec) + (curr.tv_usec - this->lastVelocityCommand.tv_usec) / 1000000.0;
	dT = (dT > 0.1) ? 0 : dT; // if there was no velocity command in the last 100 ms set dT to zero
	this->lastVelocityCommand = curr;

	int16_t trans = this->makeVelocityCommand_transvel(dT);

	int16_t rot = this->makeVelocityCommand_rotvel(dT);

	if (this->debug)
	{
		std::cout << "[RMP Driver] commanded trans=" << trans << " enc, commanded rot=" << rot << " enc\n";
	}

	//	this->tmp_trans = trans;
	//std::cout << trans << std::endl;

	pkt->PutSlot(0, (int16_t) trans);
	pkt->PutSlot(1, (int16_t) rot);
}

// Calculate the difference between two raw counter values, taking care of rollover.
int RMPDriver::diffUint32(uint32_t from, uint32_t to, bool first)
{
	int diff1, diff2;
	static uint32_t max = (uint32_t) pow(2, 32) - 1;

	// if this is the first time, report no change
	if (first)
		return (0);

	diff1 = to - from;

	/* find difference in two directions and pick shortest */
	if (to > from)
		diff2 = -(from + max - to);
	else
		diff2 = max - from + to;

	if (abs(diff1) < abs(diff2))
		return (diff1);
	else
		return (diff2);
}

// Calculate the difference between two raw counter values, taking care of rollover.
int RMPDriver::diffUint16(uint16_t from, uint16_t to, bool first)
{
	int diff1, diff2;
	static uint16_t max = (uint16_t) pow(2, 16) - 1;

	// if this is the first time, report no change
	if (first)
		return (0);

	diff1 = to - from;

	/* find difference in two directions and pick shortest */
	if (to > from)
		diff2 = -(from + max - to);
	else
		diff2 = max - from + to;

	if (abs(diff1) < abs(diff2))
		return (diff1);
	else
		return (diff2);
}
