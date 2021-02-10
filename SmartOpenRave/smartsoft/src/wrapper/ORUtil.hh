//--------------------------------------------------------------------------
//  Copyright (C) 2012, 2017 Timo Hegele, Timo Blender
//
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
#ifndef ORUTIL_H_
#define ORUTIL_H_

#include "python2.7/Python.h"
#include <vector>
#include "pyhelper.hh"
#include "util/MessageHandler.hh"
#include <EulerTransformationMatrices.hh>

#define ORUTIL_SAMPLE_TIME 0.25/8
#define ORUTIL_MAX_ITER 1500
#define ORUTIL_MAX_TRIES 4

#include <sstream>

//#include "smartSoft.hh"
#include "aceSmartSoft.hh"
#include <exception>

namespace SpecificManipulator {
	class Manipulator;
}

class ORUtil {
private:
	PyObject* pClass;								//python pointer to the python-openrave-class instance
	bool initialized;
	SpecificManipulator::Manipulator* manipulator;
	//TODO actually decrease all references. --> dictionary of iterate parameters
	vector<PyObject*> garbageReferences;			//collection that holds python object pointers that need to be deleted in the desctructor

SmartACE::SmartMutex mutex;

public:
	ORUtil();
	virtual ~ORUtil();
	enum IterationStrategy { SIMPLE, CENTERED };

	struct TrajectoryPoint {
		vector<double> jointValues;					//the joint values/angles for the TrajectoryPoint
		vector<double> gripperJointValues;			//same for the gripper
		double time;								//the duration of the movement for reaching the specified joint values
	};


	struct PlanningParameter {						//parameters needed for invoking iteration and pathplanning
		int verbose;								//additional outputs to view the iteration process
		int maxiter;								//maxiter specifies how much iterations the random planning process should perform
		int maxtries;								//maxtries specifies how many times the planning process should be restarted if a solution could not be found within the spcifies maxiter
		std::vector<double> targetLoc3D;			//Target Position (3D)
		double elevation;							//Target Angles -> angles to iterate about. For example rollAngle = 90 with rollAngleFrom = -20...
		double roll;								//...and rollAngleTo = 20 will lead to iteration borders of 70 and 110 degrees
		double azimuth;
		double depthFrom;							//delta where to start the depth iteration
		double depthTo;								//...
		double sideFrom;
		double sideTo;
		double heightFrom;
		double heightTo;
		double elevationAngleFrom;
		double elevationAngleTo;
		double rollAngleFrom;
		double rollAngleTo;
		double azimuthAngleFrom;
		double azimuthAngleTo;
		double depthStepSize;						//stepsize for iteration process
		double heightStepSize;
		double elevationAngleStepSize;
		double rollAngleStepSize;
		double azimuthAngleStepSize;

		PlanningParameter()
		{
			verbose = 0;
			maxiter = ORUTIL_MAX_ITER;
			maxtries = ORUTIL_MAX_TRIES;

			targetLoc3D.push_back(0);
			targetLoc3D.push_back(0);
			targetLoc3D.push_back(0);

			depthFrom = depthTo = depthStepSize = 0;
			heightFrom = heightTo = heightStepSize = 0;
			sideFrom = sideTo = 0;
			azimuthAngleFrom = azimuthAngleTo = azimuth = azimuthAngleStepSize = 0;
			elevationAngleFrom = elevationAngleTo = elevation = elevationAngleStepSize = 0;
			rollAngleFrom = rollAngleTo = roll = rollAngleStepSize = 0;
		}

		std::string toString() const {
			std::stringstream ss;

			ss << "maxiter: " << maxiter << "\n";
			ss << "maxtries: " << maxtries << "\n";

			ss << "targetLoc3D: ";
			for (size_t i = 0; i < targetLoc3D.size(); ++i){
				ss << targetLoc3D[i] << ", ";
			}
			ss << "\n";

			ss << "depth: " << depthFrom << ", " << depthTo << ", step: " << depthStepSize << "\n";
			ss << "height: " << heightFrom << ", " << heightTo << ", step: " << heightStepSize << "\n";
			ss << "side: " << sideFrom << ", " << sideTo << "\n";

			ss << "azimuth: " << azimuth << "\n";
			ss << "elevation: " << elevation << "\n";
			ss << "roll: " << roll << "\n";

			ss << "azimuth bounds: " << azimuthAngleFrom << ", " << azimuthAngleTo << ", step: " << azimuthAngleStepSize << "\n";
			ss << "elevation bounds: " << elevationAngleFrom << ", " << elevationAngleTo << ", step: " << elevationAngleStepSize << "\n";
			ss << "roll bounds: " << rollAngleFrom << ", " << rollAngleTo << ", step: " << rollAngleStepSize << "\n";

			return ss.str();
		}
	};


	bool init(const string& environmentFile,const string& openRavePythonSrc, bool isViewerEnabled, SpecificManipulator::Manipulator& robot, const string& planner, double speedMulti, bool waitForAnimation = true);

	void drawTCPAxes();
	void resetAll();
	void addTriMesh(const string& name, std::vector< std::vector<uint32_t> > &vertices, std::vector< std::vector<double> > &points);
	void deleteAllKinbodies(bool withManipulator, bool onlyManipulator);
	bool createKinbody(const string& data);
	bool deleteKinbody(const string& name);
	bool moveKinbody(const string& name, vector<vector<double> >& transformation);
	bool renameKinbody(const string& nameOld, const string& nameNew);
	int setJoints(const std::vector<double>& jointAngles);
	int moveJoints(const std::vector<double>& targetJointAngles, vector<TrajectoryPoint>& resultTraj, double sampleTime = ORUTIL_SAMPLE_TIME, unsigned int maxiter = ORUTIL_MAX_ITER, unsigned int maxtries = ORUTIL_MAX_TRIES);
	int moveJointsConstrained(const std::vector<double>& targetJointAngles, PlanningParameter& params, vector<TrajectoryPoint>& resultTraj, double sampleTime = ORUTIL_SAMPLE_TIME);
	PyObject* getKinBody(const string& objectName);
	int getKinBodyTransform(const string& objectName, vector<vector<double> >& dVec);
	int getEndEffectorTransform(vector<vector<double> >& pose6D);
	int getKinBodyPos(const string& objectName, vector<double>& dVec);
	int searchIKSolution(PlanningParameter& params, vector<double>& ikSolution);
	bool doDoorManipulation(vector<TrajectoryPoint>& resultTrajToHandle, vector<TrajectoryPoint>& resultTrajOpenDoor, const string& furniture, const string& door, double openAmount, double sampleTime=ORUTIL_SAMPLE_TIME);
	int iterForIKSolutionsCOMPS(PlanningParameter& params, vector<double>& ikSolution, vector<vector<double> >& pose6D);
	int iterForIKSolutionsCentered(PlanningParameter& params, vector<double>& ikSolution, vector<vector<double> >& pose6D);
	bool loadFile(const string& environmentFile);
    bool saveEnvironment(const string& filename);
    bool openGripper(vector<TrajectoryPoint>& resultTraj, double sampleTime = ORUTIL_SAMPLE_TIME);
    bool closeGripper(vector<TrajectoryPoint>& resultTraj, double sampleTime = ORUTIL_SAMPLE_TIME);
    int grabObject(const string& objectID, vector<TrajectoryPoint>& resultTraj, double sampleTime = ORUTIL_SAMPLE_TIME);
    int releaseGrabbedObject(vector<TrajectoryPoint>& resultTraj, double sampleTime = ORUTIL_SAMPLE_TIME);
    bool setActiveDOFs(const vector<int>& mask);
    int getCurrentORJoints(vector<double>& joints);
    SpecificManipulator::Manipulator* getManipulator();

    int planPathCOMPS(vector<TrajectoryPoint>& resultTrajs);

};

#endif /* ORUTIL_H_ */
