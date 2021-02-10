//------------------------------------------------------------------------
//
//  Copyright (C) 2010 Manuel Wopfner
//
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//      
//  This file is part of the "SmartRMPBaseServer component".
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

#include "Robot.hh"

#include "ComponentRMPBaseServer.hh"

Robot::Robot()
{
	setupDone = false;
	//	outputFile.open("m.mat");
}

Robot::~Robot()
{
}

void Robot::setup(bool enableMotors)
{
	this->enableMotors = enableMotors;

	// read parameters
	std::cout << "[Robot] Read parameters ...\n";

	// TODO: Check if correct
	//CHS::SmartParameter parameter;
	SmartACE::SmartIniParameter parameter;

	parameter.addFile(COMP->getGlobalState().getRobot().getRmp_config_file());

	parameter.getString("rmp", "serial_number", serialNumber);

	parameter.getInteger("times", "speed_timeout", rmpParams.SPEED_TIMEOUT);
	parameter.getInteger("times", "initialization_sleep_time", rmpParams.INITIALIZATION_SLEEP_TIME);

	parameter.getInteger("counts", "count_per_m", rmpParams.COUNT_PER_M);
	parameter.getDouble("counts", "count_per_deg", rmpParams.COUNT_PER_DEG);
	parameter.getInteger("counts", "count_per_m_per_s", rmpParams.COUNT_PER_M_PER_S);
	parameter.getInteger("counts", "count_per_m_per_s_cmd", rmpParams.COUNT_PER_M_PER_S_CMD);
	parameter.getDouble("counts", "count_per_deg_per_s", rmpParams.COUNT_PER_DEG_PER_S);
	parameter.getDouble("counts", "count_per_deg_per_s_cmd", rmpParams.COUNT_PER_DEG_PER_S_CMD);
	parameter.getInteger("counts", "command_count_per_m_per_s", rmpParams.COMMAND_COUNT_PER_M_PER_S);
	parameter.getDouble("counts", "command_count_per_deg_per_s", rmpParams.COMMAND_COUNT_PER_DEG_PER_S);
	parameter.getInteger("counts", "count_per_rev", rmpParams.COUNT_PER_REV);
	parameter.getInteger("counts", "count_per_nm", rmpParams.COUNT_PER_NM);
	parameter.getInteger("counts", "count_drift_offset", rmpParams.COUNT_DRIFT_OFFSET);

	parameter.getDouble("battery", "powerbase_battery", rmpParams.POWERBASE_BATTERY);
	parameter.getDouble("battery", "user_interface_battery", rmpParams.USER_INTERFACE_BATTERY);

	parameter.getInteger("maxima", "max_trans_vel_count", rmpParams.MAX_TRANS_VEL_COUNT);
	parameter.getInteger("maxima", "max_rot_vel_count", rmpParams.MAX_ROT_VEL_COUNT);

	driveDirection = (COMP->getGlobalState().getRobot().getBack_is_front()) ? -1 : 1;

	// RMP setup
	{
		SmartACE::SmartGuard rmpGuard(mutexRMP);
		initRMP();
		rmpGuard.release();
	}

	// reset position
	resetPosition();

	setupDone = true;
}

void Robot::shutdown()
{
	std::cout << "[Robot] shutting down ...\n";

	SmartACE::SmartGuard rmpGuard(mutexRMP);
	{
		rmp.shutdown();
	}
	rmpGuard.release();
}

void Robot::update()
{

	if (!setupDone)
	{
		// setup robot
		setup(COMP->getGlobalState().getRobot().getEnable_motors());
	}

	SmartACE::SmartGuard posGuard(mutexRobotPos);
	{
		{
			SmartACE::SmartGuard rmpGuard(mutexRMP);
			if (rmp.update())
			{
				this->odoData = rmp.getOdometryData();
				this->statusData = rmp.getStatusData();

				if (!rmp.isActive())
				{
					std::cout << "robot is not active\n";
				}
			} else
			{
				initRMP();
			}
			rmpGuard.release();
		}

		double deltaX = this->odoData.x - this->rawPos.get_x(1);
		double deltaY = this->odoData.y - this->rawPos.get_y(1);
		// TODO: Check if correct
		//double deltaA = piToPiRad(this->odoData.yaw - this->rawPos.get_base_alpha());
		double deltaA = piToPiRad(this->odoData.yaw - this->rawPos.get_base_azimuth());

		//////////////////////////////////////////////////////////////////
		// calculation of new robotPos
		// TODO: Check if correct
		//double dAlpha = piToPiRad(this->robotPos.get_base_alpha() - rawPos.get_base_alpha());
		double dAlpha = piToPiRad(this->robotPos.get_base_azimuth() - rawPos.get_base_azimuth());
		double a1 = cos(dAlpha);
		double a2 = (-1.0) * sin(dAlpha);
		double a3 = 0;

		double a4 = sin(dAlpha);
		double a5 = cos(dAlpha);
		double a6 = 0;

		double a7 = 0;
		double a8 = 0;
		double a9 = 1;

		this->robotPos.set_x(this->robotPos.get_x(1) + (a1 * deltaX + a2 * deltaY + a3 * deltaA), 1);
		this->robotPos.set_y(this->robotPos.get_y(1) + (a4 * deltaX + a5 * deltaY + a6 * deltaA), 1);
		// TODO: Check if correct
		//this->robotPos.set_base_alpha(piToPiRad(this->robotPos.get_base_alpha() + (a7 * deltaX + a8 * deltaY + a9 * deltaA)));
		this->robotPos.set_base_azimuth(piToPiRad(this->robotPos.get_base_azimuth() + (a7 * deltaX + a8 * deltaY + a9 * deltaA)));

		//////////////////////////////////////////////////////////////////
		// calculation of new rawPos

		this->rawPos.set_x(this->odoData.x, 1);
		this->rawPos.set_y(this->odoData.y, 1);
		// TODO: Check if correct
		//this->rawPos.set_base_alpha(piToPiRad(this->odoData.yaw));
		this->rawPos.set_base_azimuth(piToPiRad(this->odoData.yaw));

		//////////////////////////////////////////////////////////////////
		// update CovMatrix

		updateCovMatrix(this->oldPos, this->robotPos);
		this->oldPos = this->robotPos;

	}
	posGuard.release();

	//outputFile << "   " << xspeed << "   " << getV() << "\n";
}

void Robot::setV(double v)
{
	SmartACE::SmartGuard rmpGuard(mutexRMP);
	{
		rmp.setXSpeed(v);
		xspeed = v;
	}
	rmpGuard.release();
}

double Robot::getV()
{
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	{
		return this->odoData.vel_x;
	}
	posGuard.release();
}

void Robot::setOmega(double angle)
{
	SmartACE::SmartGuard rmpGuard(mutexRMP);
	{
		rmp.setYawSpeed(angle);
	}
	rmpGuard.release();
}

double Robot::getOmega()
{
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	{
		return this->odoData.vel_yaw;
	}
	posGuard.release();
}

double Robot::getBatteryVoltage()
{
	return this->statusData.power_battery;
}

void Robot::setParameters(double maxVel, double maxRotVel, double maxAcc, double maxRotAcc)
{
	SmartACE::SmartGuard rmpGuard(mutexRMP);
	{

	}
	rmpGuard.release();
}

void Robot::resetPosition()
{
	std::cout << "[Robot] resetting position ...\n";

	// Uncertainty of robot
	this->lamdaSigmaD = (COMP->getGlobalState().getKalman_filter().getLamda_sigma_d() * COMP->getGlobalState().getKalman_filter().getLamda_sigma_d()) / 1000.0;

	this->lamdaSigmaDeltaAlpha = ((COMP->getGlobalState().getKalman_filter().getLamda_sigma_delta_alpha() * COMP->getGlobalState().getKalman_filter().getLamda_sigma_delta_alpha())
			/ 360.0) / 180.0 * M_PI;

	this->lamdaSigmaDeltaBeta
			= ((COMP->getGlobalState().getKalman_filter().getLamda_sigma_delta_beta() * COMP->getGlobalState().getKalman_filter().getLamda_sigma_delta_beta()) / 1000.0) / 180.0 * M_PI;

	SmartACE::SmartGuard posGuard(mutexRobotPos);
	{
		this->robotPos.set_x(0.0);
		this->robotPos.set_y(0.0);
		// TODO: Check if correct
		//this->robotPos.set_base_alpha(0.0);
		this->robotPos.set_base_azimuth(0.0);

		this->rawPos.set_x(0.0);
		this->rawPos.set_y(0.0);
		//this->rawPos.set_base_alpha(0.0);
		this->rawPos.set_base_azimuth(0.0);

		this->oldPos.set_x(0.0);
		this->oldPos.set_y(0.0);
		//this->oldPos.set_base_alpha(0.0);
		this->rawPos.set_base_azimuth(0.0);

		// initialize covariance matrix
		this->oldPos.set_cov(0, 0, 50* 50 );
		this->oldPos.set_cov(1,1, 50*50);
		this->oldPos.set_cov(2,2, 5*5/180.0*M_PI);
		this->robotPos.set_cov(0,0, oldPos.get_cov(0,0) );
		this->robotPos.set_cov(1,1, oldPos.get_cov(1,1) );
		this->robotPos.set_cov(2,2, oldPos.get_cov(2,2) );
	}
	posGuard.release();

	SmartACE::SmartGuard rmpGuard(mutexRMP);
	{
		rmp.resetOdometry();
	}
	rmpGuard.release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::updatePosition(CommBasicObjects::CommBasePositionUpdate update)
{
	bool isCovUpdated = false;

	CommBasicObjects::CommBasePose upd_oldPos = update.get_old_position();
	CommBasicObjects::CommBasePose upd_correctedPos = update.get_corrected_position();
	CommBasicObjects::CommBasePose newCorrectedPos;

	// robot motion between current position and the position the laserscan (selfloc-scan) was taken
	double deltaX = this->getBasePosition().get_x() - upd_oldPos.get_x();
	double deltaY = this->getBasePosition().get_y() - upd_oldPos.get_y();

	// TODO: Check if correct
	//double deltaA = piToPiRad(this->getBasePosition().get_base_alpha()) - piToPiRad(upd_oldPos.get_base_alpha());
	double deltaA = piToPiRad(this->getBasePosition().get_base_azimuth()) - piToPiRad(upd_oldPos.get_base_azimuth());
	deltaA = piToPiRad(deltaA);

	// TODO: Check if correct
	//double dAlpha = piToPiRad(upd_correctedPos.get_base_alpha()) - piToPiRad(upd_oldPos.get_base_alpha());
	double dAlpha = piToPiRad(upd_correctedPos.get_base_azimuth()) - piToPiRad(upd_oldPos.get_base_azimuth());

	double a1 = cos(dAlpha);
	double a2 = (-1.0) * sin(dAlpha);
	double a3 = 0;

	double a4 = sin(dAlpha);
	double a5 = cos(dAlpha);
	double a6 = 0;

	double a7 = 0;
	double a8 = 0;
	double a9 = 1;

	newCorrectedPos.set_x(upd_correctedPos.get_x() + (a1 * deltaX + a2 * deltaY + a3 * deltaA));
	newCorrectedPos.set_y(upd_correctedPos.get_y() + (a4 * deltaX + a5 * deltaY + a6 * deltaA));

	// TODO: Check if correct
	//newCorrectedPos.set_base_alpha(piToPiRad(upd_correctedPos.get_base_alpha() + (a7 * deltaX + a8 * deltaY + a9 * deltaA)));
	newCorrectedPos.set_base_azimuth(piToPiRad(upd_correctedPos.get_base_azimuth() + (a7 * deltaX + a8 * deltaY + a9 * deltaA)));

	// calculate covMatrix
	isCovUpdated = updateCovMatrix(upd_correctedPos, newCorrectedPos);
	// now in newCorrectedPos exists the new covMatrix

	// robot motion once more. this is because of the motion while calculating covM
	//<hochdorfer,lutz>
	deltaX = this->getBasePosition().get_x() - upd_oldPos.get_x();
	deltaY = this->getBasePosition().get_y() - upd_oldPos.get_y();

	// TODO: Check if correct
	// deltaA = piToPiRad(this->getBasePosition().get_base_alpha()) - piToPiRad(upd_oldPos.get_base_alpha());
	deltaA = piToPiRad(this->getBasePosition().get_base_azimuth()) - piToPiRad(upd_oldPos.get_base_azimuth());

	deltaA = piToPiRad(deltaA);

	// TODO: Check if correct
//	cout << "new: " << newCorrectedPos.get_x() << " " << newCorrectedPos.get_y() << " " << newCorrectedPos.get_base_alpha() << endl;
//	cout << "old: " << upd_correctedPos.get_x() << " " << upd_correctedPos.get_y() << " " << piToPiRad(piToPiRad(
//			upd_correctedPos.get_base_alpha()) ) << endl;

	std::cout << "new: " << newCorrectedPos.get_x() << " " << newCorrectedPos.get_y() << " " << newCorrectedPos.get_base_azimuth() << std::endl;
	std::cout << "old: " << upd_correctedPos.get_x() << " " << upd_correctedPos.get_y() << " " << piToPiRad(piToPiRad(
				upd_correctedPos.get_base_azimuth()) ) << std::endl;

	//  newCorrectedPos.set_base_alpha( correctedPos.get_base_alpha() + deltaA );


	//</hochdorfer,lutz>
	newCorrectedPos.set_cov_invalid(upd_correctedPos.get_cov_invalid());

	// update of the robot Position
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	{
		this->robotPos = newCorrectedPos;
		this->oldPos = this->robotPos;
	}
	posGuard.release();

	// TODO: Check if correct
//	printf("this Pos        (cnt=%lu) ( %9.4f %9.4f %6.2f deg)\n", upd_correctedPos.get_update_count(), this->getBasePosition().get_x(),
//			this->getBasePosition().get_y(), this->getBasePosition().get_base_alpha() / M_PI * 180.0);

	printf("this Pos        (cnt=%lu) ( %9.4f %9.4f %6.2f deg)\n", upd_correctedPos.get_update_count(), this->getBasePosition().get_x(),
			this->getBasePosition().get_y(), this->getBasePosition().get_base_azimuth() / M_PI * 180.0);

	printf("corrected covM  (0,0)(1,1)(2,2): %8.1f; %8.1f; %8.1f \n", upd_correctedPos.get_cov(0, 0), upd_correctedPos.get_cov(1, 1),
			upd_correctedPos.get_cov(2, 2));

	printf("this covM       (0,0)(1,1)(2,2): %8.1f; %8.1f; %8.1f \n", this->getBasePosition().get_cov(0, 0),
			this->getBasePosition().get_cov(1, 1), this->getBasePosition().get_cov(2, 2));
	std::cout << "Set Cov invalid: " << this->getBasePosition().get_cov_invalid() << std::endl;
}

/////////////////////////////////////////////////////
//
//		private
//
/////////////////////////////////////////////////////

void Robot::initRMP()
{
	std::cout << "[Robot] setting up robot ...\n";
	std::cout.flush();

	while (!rmp.setup(serialNumber, driveDirection, rmpParams))
	{
		usleep(50000);
	}

	ParameterStateStruct::robotType robot = COMP->getGlobalState().getRobot();

	rmp.setAccelerations(robot.getAccelerationForward() / 1000.0, robot.getDecelerationFoward() / 1000.0, robot.getAccelerationBackward() / 1000.0, robot.getDecelerationBackward() / 1000.0,
			             robot.getAccelerationLeftward(), robot.getDecelerationLeftward(), robot.getAccelerationRightward() , robot.getDecelerationRightward());

	rmp.enableMotors(enableMotors);
	rmp.setVerbose(COMP->getGlobalState().getRobot().getVerbose());
	rmp.setDebug(COMP->getGlobalState().getRobot().getDebug());

	rmp.setMaxXSpeed(COMP->getGlobalState().getRobot().getMax_vel() / 1000.0);
	rmp.setMaxYawSpeed((COMP->getGlobalState().getRobot().getMax_rot_vel() / 10.0) * (M_PI / 180.0));

	// set scale factors
	rmp.setMaxVelocityScaleFactor(COMP->getGlobalState().getRobot().getMax_velocity_scale_factor());
	rmp.setMaxAccelerationScaleFactor(COMP->getGlobalState().getRobot().getMax_acceleration_scale_factor());
	rmp.setMaxTurnRateScaleFactor(COMP->getGlobalState().getRobot().getMax_turn_rate_scale_factor());
	rmp.setGainSchedule(COMP->getGlobalState().getRobot().getGain_schedule());
	rmp.setLimitScaleFactor(COMP->getGlobalState().getRobot().getLimit_scale_factor());

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this function just calculates the covMatric for pos1
bool Robot::updateCovMatrix(CommBasicObjects::CommBasePose pos0, CommBasicObjects::CommBasePose &pos1)
{
	//< probabilitic (Andreas Steck 07/2008)

	// maybe some of the piToPiRad() calls are not necessary.

	// robot orientation (radiant) !!!

	// TODO: Check if correct
	//double phiK = pos0.get_base_alpha();
	//double phiK1 = pos1.get_base_alpha();

	double phiK = pos0.get_base_azimuth();
	double phiK1 = pos1.get_base_azimuth();

	phiK = piToPiRad(phiK);
	phiK1 = piToPiRad(phiK1);

	// robot motion
	double deltaX = pos1.get_x() - pos0.get_x();
	double deltaY = pos1.get_y() - pos0.get_y();
	double d = sqrt((deltaX * deltaX) + (deltaY * deltaY)); // euclidic distance betwenn old and new position

	double delta = 0.0;
	double deltaAlpha1 = 0.0;
	double deltaAlpha2 = 0.0;

	// delta exists only if d > 0
	if (deltaX > 0 && deltaY > 0)
	{
		delta = atan(deltaY / deltaX); // angle of d (angular average of robot motion)
	}
	delta = piToPiRad(delta);

	// both in radiant
	deltaAlpha1 = delta - phiK; // angular difference
	deltaAlpha2 = phiK1 - delta; // angular difference

	// calculate new covariance matrix just if there is some motion !!!
	// if covM will be calculated and there is no motion covM will be increased, but should be constant;
	// this is because of the fabs() in the calculation of sigmaDeltaAlpha1 and sigmaDeltaAlpha2
	// the problem occurs by calculating sigmaQ2; sigmaQ2(1,1) and sigmaQ2(2,2) will be positive and thus increasing covM
	// without the fabs() covM will decrease in some situations, although this should never happen
	if (d > 0.0 || fabs(phiK1 - phiK) > 0.0)
	{
		boost::numeric::ublas::matrix<double> covM(3, 3);
		covM(0, 0) = pos0.get_cov(0, 0);
		covM(0, 1) = pos0.get_cov(0, 1);
		covM(0, 2) = pos0.get_cov(0, 2);
		covM(1, 0) = pos0.get_cov(1, 0);
		covM(1, 1) = pos0.get_cov(1, 1);
		covM(1, 2) = pos0.get_cov(1, 2);
		covM(2, 0) = pos0.get_cov(2, 0);
		covM(2, 1) = pos0.get_cov(2, 1);
		covM(2, 2) = pos0.get_cov(2, 2);

		deltaAlpha1 = piToPiRad(deltaAlpha1);
		deltaAlpha2 = piToPiRad(deltaAlpha2);

		double sigmaD = fabs(d) * lamdaSigmaD;
		double sigmaDeltaAlpha1 = fabs(deltaAlpha1) * lamdaSigmaDeltaAlpha;
		double sigmaDeltaAlpha2 = fabs(deltaAlpha2) * lamdaSigmaDeltaAlpha;
		double sigmaDeltaBeta = fabs(d) * lamdaSigmaDeltaBeta;

		boost::numeric::ublas::matrix<double> nablaFq1(3, 3);
		nablaFq1(0, 0) = 1;
		nablaFq1(0, 1) = 0;
		nablaFq1(0, 2) = -d * sin(phiK + deltaAlpha1);
		nablaFq1(1, 0) = 0;
		nablaFq1(1, 1) = 1;
		nablaFq1(1, 2) = d * cos(phiK + deltaAlpha1);
		nablaFq1(2, 0) = 0;
		nablaFq1(2, 1) = 0;
		nablaFq1(2, 2) = 1;

		boost::numeric::ublas::matrix<double> nablaFq2(3, 4);
		nablaFq2(0, 0) = cos(phiK + deltaAlpha1);
		nablaFq2(0, 1) = -d * sin(phiK + deltaAlpha1);
		nablaFq2(0, 2) = 0;
		nablaFq2(0, 3) = 0;
		nablaFq2(1, 0) = sin(phiK + deltaAlpha1);
		nablaFq2(1, 1) = d * cos(phiK + deltaAlpha1);
		nablaFq2(1, 2) = 0;
		nablaFq2(1, 3) = 0;
		nablaFq2(2, 0) = 0;
		nablaFq2(2, 1) = 1;
		nablaFq2(2, 2) = 1;
		nablaFq2(2, 3) = 1;

		boost::numeric::ublas::matrix<double> sigmaQ1(3, 3);
		sigmaQ1 = covM;

		boost::numeric::ublas::matrix<double> sigmaQ2(4, 4);
		sigmaQ2(0, 0) = sigmaD;
		sigmaQ2(0, 1) = 0;
		sigmaQ2(0, 2) = 0;
		sigmaQ2(0, 3) = 0;

		sigmaQ2(1, 0) = 0;
		sigmaQ2(1, 1) = sigmaDeltaAlpha1;
		sigmaQ2(1, 2) = 0;
		sigmaQ2(1, 3) = 0;

		sigmaQ2(2, 0) = 0;
		sigmaQ2(2, 1) = 0;
		sigmaQ2(2, 2) = sigmaDeltaAlpha2;
		sigmaQ2(2, 3) = 0;

		sigmaQ2(3, 0) = 0;
		sigmaQ2(3, 1) = 0;
		sigmaQ2(3, 2) = 0;
		sigmaQ2(3, 3) = sigmaDeltaBeta;

		boost::numeric::ublas::matrix<double> tmp3x3(3, 3);
		boost::numeric::ublas::matrix<double> tmp3x3_2(3, 3);
		boost::numeric::ublas::matrix<double> tmp3x4(3, 4);
		boost::numeric::ublas::matrix<double> nablaFq1_T(3, 3);
		boost::numeric::ublas::matrix<double> nablaFq2_T(4, 3);

		nablaFq1_T = boost::numeric::ublas::trans(nablaFq1);
		nablaFq2_T = boost::numeric::ublas::trans(nablaFq2);

		boost::numeric::ublas::axpy_prod(nablaFq1, sigmaQ1, tmp3x3, true); // tmp =  nablaFq1 * sigmaQ1
		boost::numeric::ublas::axpy_prod(tmp3x3, nablaFq1_T, tmp3x3_2, true); // tmp =  tmp * nablaFq1_T

		covM.clear();
		covM = tmp3x3_2;

		boost::numeric::ublas::axpy_prod(nablaFq2, sigmaQ2, tmp3x4, true);
		boost::numeric::ublas::axpy_prod(tmp3x4, nablaFq2_T, tmp3x3, true);

		covM += tmp3x3;

		pos1.set_cov(0, 0, covM(0, 0));
		pos1.set_cov(0, 1, covM(0, 1));
		pos1.set_cov(0, 2, covM(0, 2));
		pos1.set_cov(1, 0, covM(1, 0));
		pos1.set_cov(1, 1, covM(1, 1));
		pos1.set_cov(1, 2, covM(1, 2));
		pos1.set_cov(2, 0, covM(2, 0));
		pos1.set_cov(2, 1, covM(2, 1));
		pos1.set_cov(2, 2, covM(2, 2));

		return true;

	}
	// if there was no motion the above calculations will not be done - thus set the old valuse
	// no motion --> no change in covMatrix
	else
	{
		pos1.set_cov(0, 0, pos0.get_cov(0, 0));
		pos1.set_cov(0, 1, pos0.get_cov(0, 1));
		pos1.set_cov(0, 2, pos0.get_cov(0, 2));
		pos1.set_cov(1, 0, pos0.get_cov(1, 0));
		pos1.set_cov(1, 1, pos0.get_cov(1, 1));
		pos1.set_cov(1, 2, pos0.get_cov(1, 2));
		pos1.set_cov(2, 0, pos0.get_cov(2, 0));
		pos1.set_cov(2, 1, pos0.get_cov(2, 1));
		pos1.set_cov(2, 2, pos0.get_cov(2, 2));

		return false;
	}

	//>
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -PI <= a <= PI
double Robot::piToPiRad(double a)
{
	a += M_PI;
	bool was_neg = a < 0;
	a = fmod(a, 2*M_PI);

	if (was_neg)
		a += 2*M_PI;

	a -= M_PI;
	return a;
}

