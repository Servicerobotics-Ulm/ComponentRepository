//--------------------------------------------------------------------------
//
//  Copyright (C) 2003 Christian Schlegel, Andreas Steck
//                2012 Matthias Hörger
//
//        schlegel@hs-ulm.de
//        steck@hs-ulm.de
//        hoerger@hs-ulm.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
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
//
//--------------------------------------------------------------------------

//----------------------------------------------------------------------------
// This software makes use of Boost. License available at:
// http://www.boost.org/LICENSE_1_0.txt
//----------------------------------------------------------------------------


#include "Robot.hh"
#include "aceSmartSoft.hh"
#include <chrono>
#include <ctime>
#include <ratio>

#include "CommBasicObjects/CommLaserSafetyEventState.hh"

#include "ComponentRobotinoBaseServer.hh"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Robot::Robot( )
: _ignoreOdometryEvent(false)
{

	digitalInputArray.setComId(robotinoCom.id());
	analogInputArray.setComId(robotinoCom.id());
	digitalOutput.setComId(robotinoCom.id());
	relay.setComId(robotinoCom.id());

	powerOutput.setComId(robotinoCom.id());

	robotinoOdom.setComId(robotinoCom.id());
	robotinoBumper.setComId(robotinoCom.id());
	robotinoDrive.setComId(robotinoCom.id());

	robotinoM1.setComId(robotinoCom.id());
	robotinoM2.setComId(robotinoCom.id());
	robotinoM3.setComId(robotinoCom.id());

	power.setComId(robotinoCom.id());

	// reset position
	initVariables();


	// for possible other robotino models
	robotinoM1.setMotorNumber(0);
	robotinoM2.setMotorNumber(1);
	robotinoM3.setMotorNumber(2);

	// uncertainity of robot
	lamdaSigmaD = 50*50/1000.0;
	lamdaSigmaDeltaAlpha = (5*5/360.0) /180.0 * M_PI;
	lamdaSigmaDeltaBeta = (2*2/1000.0) /180.0 * M_PI;

	this->robotinoBumper.setTimoutConfiguration(COMP->getGlobalState().getBumper().getBumperTimeOutSec(),COMP->getGlobalState().getBumper().getBumperTimeOutMSec());



	generateLaserSafetyFieldEvents = COMP->getGlobalState().getLaserSafetyField().getGenerateLaserSafetyFieldEvents();
	laserSafetyFieldIOBit = 0;
	laserSafetyFieldTimerId = -1;
	laserSafetyFieldTimeoutSec = COMP->getGlobalState().getLaserSafetyField().getLaserSafetyfFieldTimeOutSec();
	laserSafetyFieldTimeoutMsec = COMP->getGlobalState().getLaserSafetyField().getLaserSafetyfFieldTimeOutMSec();
	laserSafetyFieldLastState = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Robot::~Robot()
{
	this->closeSerial();
	rec::robotino::api2::shutdown();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Robot::openSerial( std::string daemonIp )
{
	// set up connection to robotino daemon (either direct on Robotino using localhost or via WLAN)
	this->hostIP = daemonIp;
	this->robotinoCom.setAddress(this->hostIP.c_str());

	std::cout << "Connecting to deamon..." << std::endl;

	try{
		robotinoCom.connectToServer(true);
	}catch(rec::robotino::api2::RobotinoException &e){
		std::cout<<e.what()<<std::endl;
	}

	if ( false == robotinoCom.isConnected() )
	{
		std::cout << std::endl << "Could not connect to Robotino daemon " << robotinoCom.address()
				<< std::endl;
		//rec::robotino::api2::shutdown();
		return 1;
	}
	else
	{
		std::cout << "Robotino daemon connection successful." << std::endl;
		// set odometry and wait till it's ready. otherwise the first readings contain a false sequence nr
		while( false == robotinoOdom.set(0.0, 0.0, 0.0, 100) )
		{
			std::cout << "Setting the Odometry timed out!!! --> Retry..." << std::endl;
		}
		std::cout << "Odometry set." << std::endl;
		return 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::closeSerial()
{
	robotinoCom.disconnectFromServer();
	std::cout << "Disconnected from server." << std::endl;
}


template <typename T,unsigned S>
inline unsigned arraysize(const T (&v)[S]) { return S; }

void Robot::processEvents()
{

	//call into the robotino api to provide resources for the callbacks of the api using classes
	robotinoCom.processEvents();

	//fetch state of digital and analog inputs
	CommRobotinoObjects::CommDigitalInputEventState state;
	SmartACE::SmartGuard posGuard(lockIO);
	{
	digitalInputArray.values(digitalInputs);
	analogInputArray.values(analogInputs);

	//pass values to the event server!

	for(unsigned int i=0;i<arraysize(digitalInputs);i++){
		state.getDigitalInputValuesRef().push_back(digitalInputs[i]);
	}

	}



	////////////////////////////////////////////////////////////////
	//robotino laser safety field state evalution
	//this should be realized within a laser server component
	//due to firmware issues with the sick s300 laser sever the evaluation of the lasersafety fields
	//is done using the digital io of the robotino base
	//This will only work if the laser is configured to switch the io and connected to the digital ios of the robotino base!
	if(generateLaserSafetyFieldEvents == true){

		int laserSafetyFieldCurrentState = digitalInputs[0];

		if(laserSafetyFieldCurrentState != laserSafetyFieldLastState){

			if(laserSafetyFieldCurrentState == 0){
				//we need some timeout here. If station is invisible for more than x seconds, we abort this task
				//COMP->ini.laser.noStationVisibleTimeout
				if(laserSafetyFieldTimerId == -1){
					std::cout << "[Robot::processEvents()] laserSafety Event scheduleTimer relative time: " << laserSafetyFieldTimeoutSec << " : " << laserSafetyFieldTimeoutMsec << std::endl;

					std::chrono::seconds sec(laserSafetyFieldTimeoutSec);
					std::chrono::milliseconds msec(laserSafetyFieldTimeoutMsec);
					laserSafetyFieldTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,NULL,sec+msec);
				} else {
					std::cout<<__FUNCTION__<<":"<<__LINE__<<"ERROR: this should never had happened!"<<std::endl;
				}

			} else {
				//abort timer
				if(laserSafetyFieldTimerId != -1)
				{
					std::cout << "[Robot::processEvents()] laserSafety Event cancelTimer!"<< std::endl;
					COMP->getComponentImpl()->getTimerManager()->cancelTimer(laserSafetyFieldTimerId);
					laserSafetyFieldTimerId = -1;
				} else {
					std::cout<<__FUNCTION__<<":"<<__LINE__<<" ERROR: this should never had happened!"<<std::endl;
				}

				//send free state immediately
				CommBasicObjects::CommLaserSafetyEventState state;
				state.setProtectiveState(CommBasicObjects::SafetyFieldState::FREE);
				COMP->laserSafetyEventServiceOut->put(state);
			}

			laserSafetyFieldLastState = laserSafetyFieldCurrentState;
		} else {
			//state not changed this is only used for output!
			if (laserSafetyFieldCurrentState == 0){
				if(COMP->getGlobalState().getGeneral().getVerbose()){
					std::cout << "LaserSafety blocked!" << std::endl;
				}
				if(laserSafetyFieldTimerId == -1){
					std::cout << "[Robot::processEvents()] laserSafety Event scheduleTimer relative time: " << laserSafetyFieldTimeoutSec << " : " << laserSafetyFieldTimeoutMsec << std::endl;
					std::chrono::seconds sec(laserSafetyFieldTimeoutSec);
					std::chrono::milliseconds msec(laserSafetyFieldTimeoutMsec);
					laserSafetyFieldTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,NULL, sec+msec);
				}
			}
		}
	}
	////////////////////////////////////////////////////////////////

	COMP->digitalInputEventOut->put(state);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Robot::update(double newxpos, double newypos, double newalpha, double vx, double vy, double omega, unsigned int sequence)
{
	//std::cout << "Robot::update -- readings x: " << newxpos << "\ty: " << newypos << "\tomega: " << newalpha << std::endl;
	//std::cout << "sequence new: " << this->sequenceNew << " sequence old: " << this->sequenceOld << std::endl;


	// update pos with new readings
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	
	if(false == this->_ignoreOdometryEvent)
	{
		robotVel.set_vX(vx,1);
		robotVel.set_vY(vy,1);
		robotVel.set_WZ_base(omega);


		double deltaX = newxpos - this->rawPos.get_x(1);
		double deltaY = newypos - this->rawPos.get_y(1);
		double deltaA = piToPiRad(newalpha - this->rawPos.get_base_azimuth());


		//////////////////////////////////////////////////////////////////
		// calculation of new robotPos

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

		CommBasicObjects::CommTimeStamp time_stamp;
		time_stamp.set_now(); // Set the timestamp to the current time

		this->robotPos.set_x(this->robotPos.get_x(1) + (a1 * deltaX + a2 * deltaY + a3 * deltaA), 1);
		this->robotPos.set_y(this->robotPos.get_y(1) + (a4 * deltaX + a5 * deltaY + a6 * deltaA), 1);
		this->robotPos.set_base_azimuth(piToPiRad(this->robotPos.get_base_azimuth() + (a7 * deltaX + a8 * deltaY + a9 * deltaA)));
		this->robotPos.setTimeStamp(time_stamp);
		

	//	std::cout << "robotPos " << this->robotPos.get_x() << " " << this->robotPos.get_y() << " " << this->robotPos.get_base_azimuth() << std::endl;

		//////////////////////////////////////////////////////////////////
		// calculation of new rawPos

		this->rawPos.set_x(newxpos, 1);
		this->rawPos.set_y(newypos, 1);
		this->rawPos.set_base_azimuth(piToPiRad(newalpha));
		this->rawPos.setTimeStamp(time_stamp);

		//////////////////////////////////////////////////////////////////
		// update CovMatrix

		updateCovMatrix(this->oldPos, this->robotPos);
		this->oldPos = this->robotPos;

		double deltaDistance = sqrt((deltaX * deltaX) + (deltaY * deltaY));
		totalDistance += deltaDistance;	
	}
	
	posGuard.release();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this function just calculates the covMatric for pos1
bool Robot::updateCovMatrix( CommBasicObjects::CommBasePose pos0,
		CommBasicObjects::CommBasePose &pos1 )
{
	// maybe some of the piToPiRad() calls are not necesarry.

	// robot orientation (radiant) !!!
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

	// delta existst only if d > 0
	if ( deltaX > 0 && deltaY > 0 )
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
	// the problem occurs by calculating sigmaQ2; sigmaQ2(1,1) and sigmaQ2(2,2) will be positiv and thus increasing covM
	// without the fabs() covM will decrease in some situations, although this should never happen
	if ( d > 0.0 || fabs(phiK1 - phiK) > 0.0 )
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
	// if theres was no motion the above calculations will not be done - thus set the old valuse
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
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Robot::updatePosition( CommBasicObjects::CommBasePositionUpdate update )
{
	bool isCovUpdated = false;

	CommBasicObjects::CommBasePose upd_oldPos = update.get_old_position();
	CommBasicObjects::CommBasePose upd_correctedPos = update.get_corrected_position();
	CommBasicObjects::CommBasePose newCorrectedPos;

	// robot motion between current position and the position the laserscan (selfloc-scan) was taken
	double deltaX = this->getBasePosition().get_x() - upd_oldPos.get_x();
	double deltaY = this->getBasePosition().get_y() - upd_oldPos.get_y();
	double deltaA = piToPiRad(this->getBasePosition().get_base_azimuth()) - piToPiRad(
			upd_oldPos.get_base_azimuth());
	deltaA = piToPiRad(deltaA);

	double dAlpha = piToPiRad(upd_correctedPos.get_base_azimuth()) - piToPiRad(
			upd_oldPos.get_base_azimuth());

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
	newCorrectedPos.set_base_azimuth(piToPiRad(upd_correctedPos.get_base_azimuth() + (a7 * deltaX + a8
			* deltaY + a9 * deltaA)));

	// calculate covMatrix
	isCovUpdated = updateCovMatrix(upd_correctedPos, newCorrectedPos);
	// now in newCorrectedPos exists the new covMatrix

	// robot motion once more. this is because of the motion while calculating covM
	//<hochdorfer,lutz>
	deltaX = this->getBasePosition().get_x() - upd_oldPos.get_x();
	deltaY = this->getBasePosition().get_y() - upd_oldPos.get_y();
	deltaA = piToPiRad(this->getBasePosition().get_base_azimuth()) - piToPiRad(
			upd_oldPos.get_base_azimuth());
	deltaA = piToPiRad(deltaA);

//	std::cout << "new: " << newCorrectedPos.get_x() << " " << newCorrectedPos.get_y() << " "
//			<< newCorrectedPos.get_base_azimuth() << std::endl;
//	std::cout << "old: " << upd_correctedPos.get_x() + deltaX << " " << upd_correctedPos.get_y()
//			+ deltaY << " " << piToPiRad(piToPiRad(upd_correctedPos.get_base_azimuth()) + deltaA)
//			<< std::endl;

	//</hochdorfer,lutz>
	newCorrectedPos.set_cov_invalid(upd_correctedPos.get_cov_invalid());

	// update of the robot Position
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	{
	this->robotPos = newCorrectedPos;
	this->oldPos = this->robotPos;
	}
	posGuard.release();

//	printf("this Pos        (cnt=%lu) ( %9.4f %9.4f %6.2f deg)\n",
//			upd_correctedPos.get_update_count(), this->getBasePosition().get_x(),
//			this->getBasePosition().get_y(), this->getBasePosition().get_base_azimuth() / M_PI
//					* 180.0);
//
//	printf("corrected covM  (0,0)(1,1)(2,2): %8.1f; %8.1f; %8.1f \n",
//			upd_correctedPos.get_cov(0, 0), upd_correctedPos.get_cov(1, 1),
//			upd_correctedPos.get_cov(2, 2));
//
//	printf("this covM       (0,0)(1,1)(2,2): %8.1f; %8.1f; %8.1f \n",
//			this->getBasePosition().get_cov(0, 0), this->getBasePosition().get_cov(1, 1),
//			this->getBasePosition().get_cov(2, 2));
//	std::cout << "Set Cov invalid: " << this->getBasePosition().get_cov_invalid() << std::endl;

	return 0;
}

void Robot::initVariables()
{
	this->robotVel.setVX(0);
	this->robotVel.setVY(0);
	this->robotVel.set_WZ_base(0);

	this->robotPos.set_x(0.0);
	this->robotPos.set_y(0.0);
	this->robotPos.set_base_azimuth(0.0);

	this->rawPos.set_x(0.0);
	this->rawPos.set_y(0.0);
	this->rawPos.set_base_azimuth(0.0);


	this->oldPos.set_x(0.0);
	this->oldPos.set_y(0.0);
	this->oldPos.set_base_azimuth(0.0);

	oldalpha = 0;
	oldxpos = 0;
	oldypos = 0;
	totalDistance = 0;
	totalRotationLeft = 0;
	totalRotationRight = 0;
	
	// initialize covariance matrix
	this->oldPos.set_cov(0, 0, 50* 50 );
	this->oldPos.set_cov(1,1, 50*50);
	this->oldPos.set_cov(2,2, 5*5/180.0*M_PI);
	this->robotPos.set_cov(0,0, oldPos.get_cov(0,0) );
	this->robotPos.set_cov(1,1, oldPos.get_cov(1,1) );
	this->robotPos.set_cov(2,2, oldPos.get_cov(2,2) );
}

int Robot::resetPosition()
{
	//std::cout << "Reset Base Position!" << std::endl;

	// reset of the robot Position
	{
		SmartACE::SmartGuard posGuard(mutexRobotPos);
			
		this->initVariables();
		this->_ignoreOdometryEvent = true;

		posGuard.release();		
	}
	
	while( false == robotinoOdom.set(0.0, 0.0, 0.0, 100) )
	{
		std::cout << "Setting the Odometry timed out!!! --> Retry..." << std::endl;
	}
	std::cout << "Odometry set." << std::endl;
	
	{
		SmartACE::SmartGuard posGuard(mutexRobotPos);
		
		this->_ignoreOdometryEvent = false;

		posGuard.release();
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -PI <= a <= PI
double Robot::piToPiRad( double a )
{
	a += M_PI;
	bool was_neg = a < 0;
	a = fmod(a, 2*M_PI);
	if ( was_neg )
		a += 2*M_PI;
	a -= M_PI;
	return a;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CommBasicObjects::CommBasePose Robot::getBasePosition()
{
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	return robotPos;
}

CommBasicObjects::CommBaseVelocity Robot::getBaseVelocity()
{
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	return robotVel;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CommBasicObjects::CommBasePose Robot::getBaseRawPosition()
{
	SmartACE::SmartGuard posGuard(mutexRobotPos);
	return rawPos;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Robot::getBatteryVoltage()
{
	return power.voltage();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Robot::getBatteryCurrent()
{
	return power.current();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Robot::getExternalPower()
{
	return power.ext_power();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//double Robot::getVx()
//{
//	robotinoDriveModel.unproject(&actualVx, &actualVy, &actualOmega, robotinoM1.actualVelocity(),
//			robotinoM2.actualVelocity(), robotinoM3.actualVelocity());
//	return (double) actualVx;
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//double Robot::getVy()
//{
//	robotinoDriveModel.unproject(&actualVx, &actualVy, &actualOmega, robotinoM1.actualVelocity(),
//			robotinoM2.actualVelocity(), robotinoM3.actualVelocity());
//	return (double) actualVy;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::setVxVyOmega( double vX, double vY, double omega )
{
	if(robotinoBumper.getState() == false){

	if ( vX > this->maxVelX )
		vX = this->maxVelX;
	else if ( vX < -(this->maxVelX) )
		vX = -this->maxVelX;
	newVx = vX;
	if ( vY > this->maxVelY )
		vY = this->maxVelY;
	else if ( vY < -(this->maxVelY) )
		vY = -this->maxVelY;
	newVy = vY;

	if ( omega > this->maxRotVel )
		omega = this->maxRotVel;
	else if ( omega < -(this->maxRotVel) )
		omega = -this->maxRotVel;
	newOmega = omega;

	robotinoDrive.setVelocity(newVx, newVy, newOmega);
	} else {
		robotinoDrive.setVelocity(0, 0, 0);
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//double Robot::getOmegaRad()
//{
//	robotinoDriveModel.unproject(&actualVx, &actualVy, &actualOmega, robotinoM1.actualVelocity(),
//			robotinoM2.actualVelocity(), robotinoM3.actualVelocity());
//	return actualOmega;
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//double Robot::getOmegaDeg()
//{
//	robotinoDriveModel.unproject(&actualVx, &actualVy, &actualOmega, robotinoM1.actualVelocity(),
//			robotinoM2.actualVelocity(), robotinoM3.actualVelocity());
//	return actualOmega * 180.0 / M_PI;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Robot::getDistance()
{
	return totalDistance;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Robot::getTotalRotationLeft()
{
	return totalRotationLeft;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Robot::getTotalRotationRight()
{
	return totalRotationRight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::setParameters( double maxVelX, double maxVelY, double maxRotVel )
//, int maxVelAcc, int maxVelDecel, int maxRotVelAcc, int maxRotVelDecel)
{
	this->maxVelX = maxVelX;
	this->maxVelY = maxVelY;
	this->maxRotVel = maxRotVel;

	//printf("set parameters: maxVelX %f, maxVelY %f, maxRotVel %f \n", maxVelX, maxVelY, maxRotVel);
}

std::vector<bool> Robot::getDigitalInputArray( ) const{
	SmartACE::SmartGuard posGuard(lockIO);
	std::vector<bool> out;
	for(unsigned int i=0;i<8;++i){
		out.push_back(digitalInputs[i]);
	}
	return out;
}

std::vector<float> Robot::getAnalogInputArray() const{
	SmartACE::SmartGuard posGuard(lockIO);
	std::vector<float> out;
	for(unsigned int i=0;i<8;++i){
		out.push_back(analogInputs[i]);
	}
	return out;
}

void Robot::setDigitalOutput(unsigned int outputNumber, bool outputValue){
	digitalOutput.setOutputNumber(outputNumber);
	digitalOutput.setValue(outputValue);

}

void Robot::setPowerOutput(float value){
	powerOutput.setValue(value);
}

void Robot::setRelay(unsigned int relayNumber, bool state){
	SmartACE::SmartGuard relayGuard(lockRelay);
	std::cout<<"Robot::setRelay: "<<relayNumber<<" state : "<<state<<std::endl;
	try {
		relay.setRelayNumber(relayNumber);
		relay.setValue(state);

	} catch (rec::robotino::api2::RobotinoException e) {
		std::cerr<<"Error setting relay: "<<e.what()<<std::endl;
	}

}

void Robot::setAnalogOutput(unsigned int outputNumber, double outputValue){
	std::cout<<"WARNING: Analog Output is not supported!"<<std::endl;
}

bool Robot::getBumperState(){
	return robotinoBumper.getState();
}




//void Robot::timerExpired(const ACE_Time_Value & absolute_time,const void * arg){
void Robot::timerExpired(const Smart::TimePoint &abs_time, const void * arg){

	std::cout<<"[Robot:laserSafetyFieldTimerExpired] LaserSafetyField blocked timeout!"<<std::endl;

	COMP->getComponentImpl()->getTimerManager()->cancelTimer(laserSafetyFieldTimerId);
	laserSafetyFieldTimerId = -1;
	CommBasicObjects::CommLaserSafetyEventState state;
	state.setProtectiveState(CommBasicObjects::SafetyFieldState::BLOCKED);
	COMP->laserSafetyEventServiceOut->put(state);

}

void Robot::timerCancelled(){};
void Robot::timerDeleted(const void * arg){};
