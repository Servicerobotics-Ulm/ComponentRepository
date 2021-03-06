// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#include "WebotsAPITask.hh"
#include "ComponentWebotsMobileRobot.hh"

#include <iostream>
#include <cmath>
#include <random>
#include <math.h>

#include <webots/Node.hpp>
#include <webots/Pen.hpp>

using namespace webots;

WebotsAPITask::WebotsAPITask(SmartACE::SmartComponent *comp) :
		WebotsAPITaskCore(comp) {
}

WebotsAPITask::~WebotsAPITask() {
	std::cout << "destructor WebotsAPITask\n";
}

int WebotsAPITask::on_entry() {
	ParameterStateStructCore::WebotsType p = COMP->getParameters().getWebots();
	webotsRobotName = p.getRobotName();
	char environment[256] = "WEBOTS_ROBOT_NAME=";
	putenv(strcat(environment, webotsRobotName.c_str()));

	webotsRobot = new Supervisor();
	if (!webotsRobot) {
		std::cerr << "in webots is no robot with name '" << webotsRobotName
				<< "'" << std::endl;
		return 1;
	}
	webotsKeyboard = webotsRobot->getKeyboard();
	webotsKeyboard->enable(webotsRobot->getBasicTimeStep());

	int i = 0;
	for (auto const it : p.getMotorName()) {
		Motor *motor = webotsRobot->getMotor(it);
		if (!motor) {
			std::cerr << "no webots motor with name '" << it
					<< "' in a robot with name '" << webotsRobotName << "'"
					<< std::endl;
			return 1;
		}
		motor->setPosition(INFINITY);
		motor->setVelocity(0);
		motor->setAcceleration(-1.0); // overrides the acceleration from webots, use maxAcceleration instead
		webotsMotors.push_back(webotsMotor());
		webotsMotors[i++].motor = motor;
	}
	i = 0;
	for (auto const it : p.getMaxAcceleration())
		maxAcceleration[i++] = it;
	i = 0;
	for (auto const it : p.getRadius())
		webotsMotors[i++].radius = it;
	i = 0;
	for (auto const it : p.getDistanceToRobotCentre())
		webotsMotors[i++].distanceToRobotCentre = it;
	i = 0;
	for (auto const it : p.getHeading())
		webotsMotors[i++].heading = it;
	return 0;
}

// BUG: robotino 3 will not move correct if world's coordinate system is set to ENU (maybe asymmetric friction is not correct calculated by physics engine)
// BUG: in documentation of all functions is mm instead of m
// set robot speed to vx m/s forward, vy m/s left, and rotate omega radians/s counterclockwise
void WebotsAPITask::setVxVyOmega(double vX, double vY, double omega) {
	std::lock_guard<std::mutex> guard(mutex_targetSpeed);
	targetSpeed[0] = vX;
	targetSpeed[1] = vY;
	targetSpeed[2] = omega;
}

// transforms webots global coordinates from NUE to ENU if needed
// webots (x, y, z) -> x=forward y=up z=right -> smartsoft (x, -z, y)
// see also convert_nue_to_enu.py script from webots or the controller based on it
Coord WebotsAPITask::webots2smart(const double *d) {
	if (isNUE) {
		Coord c = { d[0], -d[2], d[1] };  // script does the same
		return c;
	} else {
		Coord c = { d[0], d[1], d[2] };
		return c;
	}
}
// todo: smart2webots() smartsoft (x, y, z) -> x=forward y=left z=up -> webots (x, z, -y)

// to change to another robot:
// * edit System/smartsoft/src/ComponentWebotsRobot_data/configuration.json (as in Robotino.json or Pioneer3DX.json or Tiago.json)
// * edit System/model/*.systemParam scanner to
//     LMS100: x=190 min_range=10  max_range=20000 opening_angle=210 resolution=0.5  frequency=50
//     LMS200: x= 70 min_range=10  max_range=80000 opening_angle=180 resolution=1    frequency=25
//     Hokuyo: x=202 min_range=200 max_range= 5600 opening_angle=240 resolution=0.36 frequency=25
// * edit System/src/ComponentWebotsLaserLMS1xx_data/configuration.json to
//     "name": "LidarPioneer" or "LidarRobotino3" or "LidarTiago"
// * the controller in world.wbt should be set to "<extern>" or "" as needed

std::string space2underscore(std::string text) {
	for (int i = 0; i < text.length(); i++) {
		if (text[i] == ' ')
			text[i] = '_';
	}
	return text;
}

// this task is used as an webots controller, see webotsRobot->step(timeStep) below.
// webots controller is synchronized by webots, so SmartSoft should NOT set a timer of frequency how often this function is called.
int WebotsAPITask::on_execute() {

	isNUE = webotsRobot->getRoot()->getField("children")->getMFNode(0)->
			getField("coordinateSystem")->getSFString() == "NUE";
	double timeStep = webotsRobot->getBasicTimeStep();
	int nrProgram = 0;
	int programStep = 0;
	int subProgram;
	const Pose2D zeroPose = { 0, 0, 0 };
	Pose2D sum = zeroPose;
	Pose2D sumSq = zeroPose;
	int nrSummands = 0;

	// used for acceleration, see also targetSpeed[3] from .hh
	// {x, y, angular}
	double actualSpeed[3] = { 0, 0, 0 };

	// the function 'step' blocks until webots is ready for the next timestep
	// see https://cyberbotics.com/doc/guide/controller-programming#synchronization-of-simulation-and-controller-steps
	while (webotsRobot->step(timeStep) != -1) {
		Pose2D newRealPose, newOdomPose;

		// how odometry is done of an simulated robot:
		// * get previous and current (exact) position of robot
		// * calculate exact relative movement and rotation of robot
		// * add some noise (random error) to it
		// * use it to update the odometry position/heading
		// an real robot does the same:
		// * measure the real movement/rotation with sensors (wheel tick etc.)
		// * it already had some error
		// * use it to calculate the change in robot's position/heading

		Node *supervisorNode = webotsRobot->getSelf();

//      webots2smart(supervisorNode->getField("translation")->getSFVec3f());

		// robotCoordinateSystem is used to measure the position/heading of the robot:
		//   a transformation node inside of the robot in webots
		//   should be named by DEF keyword with "CoordinateSystem" + space2underscore(webotsRobotName)
		//   x=front, y=left, z=up (relative to robot)
		//   z=0 is at floor level
		//   x=y=0 is at the turning point of the robot (centre between wheels)
		Node *robotCoordinateSystem = webotsRobot->getFromDef(
				"CoordinateSystem" + space2underscore(webotsRobotName));
		if (!robotCoordinateSystem)
			robotCoordinateSystem = supervisorNode;

		Coord translation = webots2smart(robotCoordinateSystem->getPosition());

		const double *xyzAxis = robotCoordinateSystem->getOrientation();
		const double _xAxis[3] = { xyzAxis[0], xyzAxis[3], xyzAxis[6] };
		Coord xAxis = webots2smart(_xAxis);
		double heading = atan2(xAxis.y, xAxis.x);

		const double *_velocities = supervisorNode->getVelocity();
		Coord velocities = webots2smart(_velocities);
		const double _velocities2[3] = { _velocities[3], _velocities[4],
				_velocities[5] };
		Coord velocities2 = webots2smart(_velocities2);

		newRealPose.x = translation.x;
		newRealPose.y = translation.y;
		newRealPose.heading = heading;
		if (sequence == 0) {
			oldRealPose = newRealPose;
			oldOdomPose = zeroPose;
			for (int i = 3; i--;)
				targetSpeed[i] = 0.0;
		}

		double vx = velocities.x;
		double vy = velocities.y;
		double omega = velocities2.z;

		double delta_x = newRealPose.x - oldRealPose.x;
		double delta_y = newRealPose.y - oldRealPose.y;
		double delta_heading = newRealPose.heading - oldRealPose.heading;
		double d = sqrt(delta_x * delta_x + delta_y * delta_y); // distance (newPose to oldPose)
		double delta = 0; // global direction of movement
		if (d > 0)
			delta = atan2(delta_y, delta_x);

		std::random_device rd { };
		std::mt19937 gen { rd() };

		// from Robot.cc, updateCovMatrix() calculates errors too
		// variance of error(distance traveled)  = |distance| * constant
		// => constant = variance / |distance| = standardDeviation(distance)^2 / distance
		// normDist() needs standard deviation = sqrt(variance) = sqrt(|distance|*constant)
		// (note that variance is linear congruent to distance, but the standard deviation is not linear congruent)
		// (note that the constant must have the same measurement unit than the distance or standard deviation)
		// error of distance should have a standard deviation of 50m after 1000m traveled (unit m)
		// lamdaSigmaD = 50*50/1000.0;
		// => standardDeviation(1m) = sqrt(1m*50m*50m/1000m) = 1.58m (really big, probably unit mm was meant instead)
		// error of heading should have a standard deviation of 5 degree after 1 rotation (unit converted to radians)
		// lamdaSigmaDeltaAlpha = (5*5/360.0) /180.0 * M_PI;
		// error of heading should have a standard deviation of 2 degree after 1000 m traveled (unit converted to radians)
		// lamdaSigmaDeltaBeta = (2*2/1000.0) /180.0 * M_PI;
		// (really small, probably unit mm was meant instead)
		// wrong assumption in calculation of covariance matrix: robot is only moving forward
		//   if robot moves backwards, this will be calculated as the robot is turning backwards and forwards again (alpha1 and alpha2 will be large)
		//   each time step (covariance matrix update), so moving backwards generates an much larger error than moving forwards

		/*
		 InternalParameter OdometryRandomError{
		 //@doc"e.g. 0.05m * 0.05m / 1m = 0.0025 m (after traveling 1m, distance error has standard deviation of 0.05m)"
		 varianceOfDistancePerMeter: Double = 0.0025
		 //@doc"e.g. (5°*5°)/360° /180°*pi = 0.001212 (after rotating 360 degrees, heading error has standard deviation of 5 degrees)"
		 varianceOfHeadingPerRadians: Double = 0.001212
		 //@doc"e.g. (2°/180°*pi)^2/1m  = 0.001218 (after traveling 1m, heading error has standard deviation of 2 degrees)"
		 varianceOfHeadingPerMeter: Double = 0.001218
		 }
		 */
		const double varianceD =
				COMP->getParameters().getOdometryRandomError().getVarianceOfDistancePerMeter();
		const double varianceAlpha =
				COMP->getParameters().getOdometryRandomError().getVarianceOfHeadingPerRadians();
		const double varianceBeta =
				COMP->getParameters().getOdometryRandomError().getVarianceOfHeadingPerMeter();

		std::normal_distribution<> normDistD { 0, std::sqrt(
				std::fabs(d * varianceD)) };
		d += normDistD(gen);

		// error to rotation:
		//std::normal_distribution<> normDistAlpha{0, std::sqrt(std::fabs(delta_heading)*lamdaeltaAlpha)};
		std::normal_distribution<> normDistAlpha { 0, std::sqrt(
				std::fabs(delta_heading) * varianceAlpha) };
		delta_heading += normDistAlpha(gen);

		// error to heading because of movement:
		std::normal_distribution<> normDistBeta { 0, std::sqrt(
				std::fabs(d) * varianceBeta) };
		delta_heading += normDistBeta(gen);

		newOdomPose.x = oldOdomPose.x
				+ d * cos(delta + (oldOdomPose.heading - oldRealPose.heading));
		newOdomPose.y = oldOdomPose.y
				+ d * sin(delta + (oldOdomPose.heading - oldRealPose.heading));
		newOdomPose.heading = oldOdomPose.heading + delta_heading;

		// todo: do all the other update call in other components, see RobotinoAPITask processEvent()
		// todo: there is no error in vx vy omega
		COMP->robot->update(newOdomPose.x, newOdomPose.y, newOdomPose.heading,
				vx, vy, omega, sequence);

//		std::cout << "realPose x:" << newRealPose.x << " y:" << newRealPose.y
//				<< " heading:" << newRealPose.heading << std::endl;
//        std::cout << "oldOdomPose x:" << oldOdomPose.x << " y:" << oldOdomPose.y << " heading:" << oldOdomPose.heading << std::endl;
//		std::cout << "newOdomPose x:" << newOdomPose.x << " y:" << newOdomPose.y
//				<< " heading:" << newOdomPose.heading << std::endl;

		oldRealPose = newRealPose;
		oldOdomPose = newOdomPose;
		++sequence;

		int kx = 0, ky = 0, komega = 0, kspace = 0;
		int key;
		while ((key = webotsKeyboard->getKey()) != -1) {
			if (key == 'W' || key == webotsKeyboard->UP)
				kx = 1;
			if (key == 'S' || key == webotsKeyboard->DOWN)
				kx = -1;
			if (key == 'A' || key == webotsKeyboard->LEFT)
				komega = 1;
			if (key == 'D' || key == webotsKeyboard->RIGHT)
				komega = -1;
			if (key == 'Q')
				ky = 1;
			if (key == 'E')
				ky = -1;
			if (key == ' ')
				kspace = 1;
		}

		// start lock
		{
			std::lock_guard<std::mutex> guard(mutex_targetSpeed);
			if(COMP->getParameters().getWebots().getKeyboardControl() && (kx || ky || komega || kspace) ) {
				targetSpeed[0] = kx;
				targetSpeed[1] = ky;
				targetSpeed[2] = komega * 1.2;
			}
			double maxSteps = 0;
			for (int i = 3; i--;) {
				double stepsNeeded = abs(targetSpeed[i] - actualSpeed[i])
						/ (maxAcceleration[i] * (timeStep / 1000.0));
				if (stepsNeeded > maxSteps)
					maxSteps = stepsNeeded;
			}
			if (maxSteps < 1)
				maxSteps = 1;
			for (int i = 3; i--;)
				actualSpeed[i] += (targetSpeed[i] - actualSpeed[i]) / maxSteps;
		} // end lock
		for (auto &i : webotsMotors)
			i.motor->setVelocity(
					actualSpeed[0] * cos(i.heading / 180 * M_PI) / i.radius
							+ actualSpeed[1] * sin(i.heading / 180 * M_PI)
									/ i.radius
							+ actualSpeed[2] * i.distanceToRobotCentre
									/ i.radius);

		/////////////////////////
		// baseStateServiceOut //
		/////////////////////////

		// this was done in OdemTask.cc previously
		// same is done in BaseStateQueryServiceAnswHandler::handleQuery(...)

		CommBasicObjects::CommTimeStamp time_stamp;
		CommBasicObjects::CommBasePose base_position;
		CommBasicObjects::CommBaseVelocity base_velocity;
		CommBasicObjects::CommBaseState base_state;

		time_stamp.set_now(); // Set the timestamp to the current time
		base_velocity = COMP->robot->getBaseVelocity();
		base_state.set_time_stamp(time_stamp);
		base_state.set_base_position(COMP->robot->getBasePosition());
		base_state.set_base_raw_position(COMP->robot->getBaseRawPosition());
		base_state.set_base_velocity(base_velocity);
		if (COMP->getGlobalState().getGeneral().getVerbose() == true) {
			std::cout << "Base Pose: "
					<< base_state.getBasePose().get_base_pose3d().get_position()
					<< std::endl;
			std::cout << "Base Odom: "
					<< base_state.get_base_raw_position().get_base_pose3d().get_position()
					<< std::endl;
		}

		const Smart::StatusCode status = COMP->baseStateServiceOut->put(
				base_state);
		if (status != Smart::SMART_OK) {
			std::cerr << "ERROR: failed to push base state ("
					<< Smart::StatusCodeConversion(status) << ")" << std::endl;
		}
	}
	return 1;  // if webots simulation stops, quit this task too
}

void WebotsAPITask::resetOdomPose() {
	COMP->robot->update(0, 0, 0, 0, 0, 0, 0);
	sequence = 0;
}

int WebotsAPITask::on_exit() {
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
