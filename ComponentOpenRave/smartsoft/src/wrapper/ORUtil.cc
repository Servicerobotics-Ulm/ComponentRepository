//--------------------------------------------------------------------------
//  Copyright (C) 2012 2017 Timo Hegele, Timo Blender
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
#include "ORUtil.hh"
#include <iostream>
#include "specificManipulator/ManipulatorFactory.hh"
#include "pyhelper.hh"
#include "specificManipulator/Manipulator.hh"

using namespace std;

ORUtil::ORUtil()
{
}

bool ORUtil::init(const string& environmentFile, const string& openRavePythonSrc, bool isViewerEnabled,
		SpecificManipulator::Manipulator& robot, const string& planner, double speedMulti, bool waitForAnimation)
{
	int result = 0;
	this->manipulator = &robot;
	vector<int> maskWithGripper;
	vector<int> maskWithoutGripper;
	maskWithGripper = this->manipulator->getActiveDofs();
	maskWithoutGripper = this->manipulator->getActiveArmDofs();
	SmartACE::SmartGuard guard(this->mutex);

	//prepare the arguments for the instance of the python object
	if (!isViewerEnabled)
	{
		waitForAnimation = false;
	}
	//init Python interpreter and add the location of the OpenRave-Python-class to the search path of the Python interpreter
	Py_Initialize();
	PyObject* args = PyTuple_New(9);
	PyTuple_SetItem(args, 0, PyString_FromString((char*) environmentFile.c_str())); //environmentFile
	PyTuple_SetItem(args, 1, ((isViewerEnabled) ? Py_True : Py_False)); //isViewerEnabled
	PyTuple_SetItem(args, 2, PyString_FromString((char*) this->manipulator->getRobotURI().c_str())); //robotURI
	PyTuple_SetItem(args, 3, PyString_FromString((char*) this->manipulator->getManipulatorName().c_str())); //manipulatorName e.g. "arm"
	PyTuple_SetItem(args, 4, PyString_FromString((char*) planner.c_str())); //planner
	PyTuple_SetItem(args, 5, createPyList(maskWithGripper)); //maskWithGripper
	PyTuple_SetItem(args, 6, createPyList(maskWithoutGripper)); //maskWithoutGripper
	PyTuple_SetItem(args, 7, PyFloat_FromDouble(speedMulti)); //speedMulti
	PyTuple_SetItem(args, 8, ((waitForAnimation) ? Py_True : Py_False)); //waitForAnimationFlag

	string sympyPath = "sys.path[:0] = [\"/usr/lib/python2.7/dist-packages\"]";

	string srcPath = "sys.path.append(\"";
	srcPath.append(openRavePythonSrc);
	srcPath.append("\")");


	PyRun_SimpleString("import sys");
	PyRun_SimpleString("import time");
	PyRun_SimpleString(sympyPath.c_str());//append path to openRave Implementation to the search paths of the Python interpreter
	PyRun_SimpleString(srcPath.c_str());//append path to openRave Implementation to the search paths of the Python interpreter
	PyRun_SimpleString("print sys.path");
	PyRun_SimpleString("import sympy");
	PyRun_SimpleString("print sympy.__version__");

	PyObject* fileName = PyString_FromString("ORUtil_class");//new reference (filename of the Python Class)
	std::cout<<"Path to python implementation file"<<srcPath.c_str()<<std::endl;

	PyObject* pythonModule = PyImport_Import(fileName);//new reference		 (create a Python module from the file)

	Py_DECREF(fileName);
	stringstream errStr;
	if (pythonModule != NULL)
	{
		PyObject* pDict = PyModule_GetDict(pythonModule);//borrowed reference	 (get a dictionary for the module)
		if(pDict != NULL)
		{
			PyObject* classImplementation = PyDict_GetItemString(pDict, "OpenRaveManipulation");//borrowed reference	(get the implementation of the class from the dict of the module)
			if (classImplementation)
			{
				this->pClass = PyObject_CallObject(classImplementation, args); //new reference	(call the constructor of the class and receive an instance)
				if(this->pClass)
				{
					result = 0;
				}
				else
				{
					result = -1;
					errStr << "unable to initialize python class instance" << endl;
					errStr << "constructor parameters were:" << endl;
					errStr << "environment file --> " << environmentFile << endl;
					errStr << "openRavePythonSrc path --> " << openRavePythonSrc << endl;
					errStr << "isViewerEnabled --> " << isViewerEnabled << endl;
					errStr << "manipulator --> " << this->manipulator->getManipulatorType() << endl;
					errStr << "manipulator file --> " << this->manipulator->getRobotURI() << endl;
					errStr << "planner --> " << planner << endl;
					errStr << "speedMulti --> " << speedMulti << endl;
					errStr << "waitForAnimation --> " << waitForAnimation;
				}
			}
			else
			{
				result = -2;
				errStr << "unable to get the implementation of the python class" << endl;
				errStr << "openRavePythonSrc path --> " << openRavePythonSrc << endl;
			}
		}
		else
		{
			result = -3;
			errStr << "unable to generate a dictionary from the modul" << endl;
			errStr << "openRavePythonSrc path --> " << openRavePythonSrc << endl;
		}
	}else
	{
		result = -4;
		errStr << "unable to get the module at the specified directory" << endl;
		errStr << "openRavePythonSrc path --> " << openRavePythonSrc << endl;
		errStr << "HINT: it is also possible that openravepy could not be imported properly" << endl;
		errStr << "check for the openravepy sym-link to the desired OR-Version in /usr/lib/python2.6/dist-packages/openravepy" << endl;
	}

	if (result != 0)
	{
	std::cerr << "-----------------------------------------------------" << endl;
	std::cerr << "ERROR initializing CPP-Part of OpenRave" << endl;
	std::cerr << "path: " << openRavePythonSrc << endl;
	std::cerr << "Additional Informations (if any): " << endl;
	std::cerr << errStr.str() << endl;
	std::cerr << "-----------------------------------------------------" << endl;
	}else
	{
		guard.release();
		// Blender: Depends on gripper (vacuum = getActiveArmDofs)
		this->setActiveDOFs(this->manipulator->getActiveDofs());
		this->initialized = true;
	}

	Py_DECREF(pythonModule);

	return (result == 0);
}

ORUtil::~ORUtil()
{
	SmartACE::SmartGuard guard(this->mutex);
	if(this->initialized)
	{
		Py_DECREF(this->pClass);
		for (size_t i = 0; i < this->garbageReferences.size(); ++i)
		{
			Py_DECREF(this->garbageReferences[i]);
		}
	}
	Py_Finalize();
}

/**
 * Deletes all objects from the environment, except the manipulator and
 * reloads the standard environment that was first loaded during initiation
 */
void ORUtil::resetAll()
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "resetAll", (NULL));
	Py_DECREF(tmp);
}

void ORUtil::drawTCPAxes()
{
	SmartACE::SmartGuard guard(this->mutex);

	std::string name = "arm";
	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "drawTCPAxes", ((char*) "s"), (char*) name.c_str());
	Py_DECREF(tmp);
}

/**
 * Adds a Trimesh structure to the Environment
 */
void ORUtil::addTriMesh(const string& name, std::vector<std::vector<uint32_t> > &vertices, std::vector<std::vector<double> > &points)
{	/*
	std::cout << "vertices size:" << vertices.size() << std::endl;
	std::cout << "points size: " << points.size() << std::endl;
	std::cout << "vertices: " << std::endl;
	for (int i = 0; i < vertices.size(); i++){
		for (int j = 0; j < vertices.at(i).size(); j++){
			//std::cout << i << "/" << j << ": " << vertices.at(i).at(j) << std::endl;
	}}
	std::cout << "points:" << std::endl;
	for (int i = 0; i < points.size(); i++){
		std::cout << "size at " << i << ": " << points.at(i).size() << std::endl;
		for (int j = 0; j < points.at(i).size(); j++){
			std::cout << i << "/" << j << ": " << points.at(i).at(j) << std::endl;
	}}*/
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* pyPoints = PyFloatList2DFromCPPDoubleVec2D(points);
	PyObject* pyVertices = PyIntList2DFromCPPIntVec2D(vertices);
	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "addTriMesh", ((char*) "sOO"), (char*) name.c_str(), pyVertices, pyPoints);
	Py_DECREF(tmp);
}

/**
 * deletes kinbodies from the current Environment
 * @param withManipulator whether the manipulator should also be removed
 * @param onlyManipulator whether the manipulator is the only object that should be removed
 *
 * example:
 * <code>
 * deleteAllKinbodies(False,False)	//removes all objects except the manipulator
 * deleteAllKinbodies(True,True) 	//removes the manipulator only
 * deleteAllKinbodies(True,False) 	//removes the everything
 * </code>
 */
void ORUtil::deleteAllKinbodies(bool withManipulator, bool onlyManipulator)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* pBoolWithManip = PyBool_FromLong(withManipulator);
	PyObject* pBoolOnlyManip = PyBool_FromLong(onlyManipulator);
	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "deleteAllKinbodies", ((char*) ("OO")), pBoolWithManip, pBoolOnlyManip);
	Py_DECREF(pBoolWithManip);
	Py_DECREF(pBoolOnlyManip);
	Py_DECREF(tmp);
}

/**
 * creates and adds a new Kinbody to the current environment
 * @param data data string in the shape of the OpenRAVE Custom XML Format
 * @return true if creating and adding of the kinbody succeeded
 */
bool ORUtil::createKinbody(const string& data)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "createKinbody", ((char*) ("s")), (char*) data.c_str());
	bool result = (Py_True == tmp);
	Py_DECREF(tmp);
	return result;
}

/**
 * deletes a KinBody from the current environment
 * @param name the name of the kinbody withing the OpenRave Environment
 * @return true if deleting of the kinbody succeeded
 */
bool ORUtil::deleteKinbody(const string& name)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "deleteKinbody", ((char*) ("s")), (char*) name.c_str());
	bool result = (Py_True == tmp);
	Py_DECREF(tmp);
	return result;
}

/**
 * moves a KinBody in the current environment
 * @param name the name of the kinbody withing the OpenRave Environment
 * @param transformation 6D Transformation matrix that specifies the new position
 * @return true if moving succeeded
 */
bool ORUtil::moveKinbody(const string& name, vector<vector<double> >& transformation)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* transMatrix = PyFloatList2DFromCPPDoubleVec2D(transformation);
	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "moveKinbody", ((char*) ("sO")), (char*) name.c_str(), transMatrix);
	bool result = (Py_True == tmp);
	Py_DECREF(transMatrix);
	Py_DECREF(tmp);
	return result;
}

/**
 * rename a KinBody in the current environment
 * @param old name the name of the kinbody withing the OpenRave Environment
 * @param new name of the kinbody
 * @return true if moving succeeded
 */
bool ORUtil::renameKinbody(const string& nameOld, const string& nameNew)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "renameKinbody", ((char*) ("ss")), (char*) nameOld.c_str(), nameNew.c_str());
	bool result = (Py_True == tmp);
	Py_DECREF(tmp);
	return result;
}

/**
 * sets the joint angles for the manipulator within the OpenRave environment (without planning)
 * @param jointAngles real-world angles
 * @return	0 if success
 * 			-1 if the given number of joints didn't match the number of required joint values for the manipulator
 * 			-2 if another error occured
 *
 */
int ORUtil::setJoints(const std::vector<double>& jointAngles)
{
	int result = 0;
	std::vector<double> openRaveAngles;
	openRaveAngles.resize(jointAngles.size());
	this->manipulator->convertRealAnglesToOpenRaveAngles(jointAngles, openRaveAngles);

	{
		SmartACE::SmartGuard guard(this->mutex);

		PyObject* joints = createPyList(openRaveAngles);
		PyObject* resValue = PyObject_CallMethod(this->pClass, (char*) "setJoints", ((char*) ("O")), joints);
		Py_DECREF(joints);
		result = PyInt_AsLong(resValue);
		Py_DECREF(resValue);
	}
	return result;
}

/**
 * moves the manipulator by planning to the specified joint angle values
 * @param jointAngles target joint angles for the planning process
 * @param resultTraj resulting trajectory of the path
 * @param maxiter specifies how much iterations the random planning process should perform
 * @param maxtries specifies how many times the planning process should be restarted if a solution could not be found within the spcifies maxiter
 * @return 	the number of trajectory points
 * 			-1 if the planning failed
 */
int ORUtil::moveJoints(const std::vector<double>& targetJointAngles, vector<TrajectoryPoint>& resultTraj, double sampleTime,
		unsigned int maxiter, unsigned int maxtries)
{
	std::cout << "TIMO !!!!!!!!!!!! moveJoints, targetJointAngles.size: " << targetJointAngles.size() << std::endl;
	for (int i = 0; i < targetJointAngles.size(); i++) {
		std::cout << "TIMO !!!!!!!!!!!! moveJoints, targetJointAngles: " << targetJointAngles[i] << std::endl;
	}


	SmartACE::SmartGuard guard(this->mutex);

	int result;
	std::vector<double> openRaveAngles;
	openRaveAngles.resize(targetJointAngles.size());
	this->manipulator->convertRealAnglesToOpenRaveAngles(targetJointAngles, openRaveAngles);
	PyObject* joints = createPyList(openRaveAngles);
	//std::cout << ">>>> " << __FILE__ << " >>>>> " << __LINE__ << "\n";
	PyObject* pyResTraj = PyObject_CallMethod(this->pClass, (char*) "moveJoints", ((char*) "Odii"), joints, sampleTime, maxiter, maxtries);
	//std::cout << ">>>> " << __FILE__ << " >>>>> " << __LINE__ << "\n";
	if (pyResTraj != Py_None)
	{
		result = this->manipulator->parseTrajectory(pyResTraj, resultTraj);
	} else
	{
		result = -1;
	}
	Py_DECREF(joints);
	Py_DECREF(pyResTraj);
	return result;
}

/**
 * moves the manipulator by planning to the specified joint angle values
 * @param jointAngles target joint angles for the planning process
 * @param resultTraj resulting trajectory of the path
 * @param maxiter specifies how much iterations the random planning process should perform
 * @param maxtries specifies how many times the planning process should be restarted if a solution could not be found within the specifies maxiter
 * @return 	the number of trajectory points
 * 			-1 if the planning failed
 */
int ORUtil::moveJointsConstrained(const std::vector<double>& targetJointAngles, PlanningParameter& params, vector<TrajectoryPoint>& resultTraj, double sampleTime)
{
	PyObject* pParams = PyDict_New();
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthFrom"), PyFloat_FromDouble(params.depthFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "sideFrom"), PyFloat_FromDouble(params.sideFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "sideTo"), PyFloat_FromDouble(params.sideTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthTo"), PyFloat_FromDouble(params.depthTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightFrom"), PyFloat_FromDouble(params.heightFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightTo"), PyFloat_FromDouble(params.heightTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleFrom"), PyFloat_FromDouble(params.elevationAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleTo"), PyFloat_FromDouble(params.elevationAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleFrom"), PyFloat_FromDouble(params.rollAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleTo"), PyFloat_FromDouble(params.rollAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleFrom"), PyFloat_FromDouble(params.azimuthAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleTo"), PyFloat_FromDouble(params.azimuthAngleTo));
	SmartACE::SmartGuard guard(this->mutex);

	int result;
	std::vector<double> openRaveAngles;
	openRaveAngles.resize(targetJointAngles.size());
	this->manipulator->convertRealAnglesToOpenRaveAngles(targetJointAngles, openRaveAngles);
	PyObject* joints = createPyList(openRaveAngles);
	PyObject* pyResTraj = PyObject_CallMethod(this->pClass, (char*) "moveJointsConstrained", ((char*) "OOd"), joints, pParams, sampleTime);
	if (pyResTraj != Py_None)
	{
		result = this->manipulator->parseTrajectory(pyResTraj, resultTraj);
	} else
	{
		result = -1;
	}
	Py_DECREF(joints);
	Py_DECREF(pyResTraj);
	return result;
}

/**
 * gets a kinbody pointer of the specified object within the current environment
 * @param objectName the name of the object (kinbody) within the current environment
 * @return pointer to the kinbody-python object !!WARNING!! object will be Py_None if object could not be get
 */
PyObject* ORUtil::getKinBody(const string& objectName)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* result = PyObject_CallMethod(this->pClass, (char*) "getKinBody", ((char*) "s"), (char*) objectName.c_str());
	this->garbageReferences.push_back(result); //memorize object for later destruction !!HINT!! could be omitted/removed if the caller handles the new reference itself
	return result;
}

/**
 * gets the 6D pose of the specified object within the current environment
 * @param objectName the name of the object (kinbody) within the current environment
 * @param dVec 2D vector that holds the transformation (4x4)
 * @return	0 if success
 * 			-1 if the object could not be get
 */
int ORUtil::getKinBodyTransform(const string& objectName, vector<vector<double> >& dVec)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "getKinBodyTransform", ((char*) "s"), (char*) objectName.c_str());
	int a = CPPDoubleVec2DFromPyFloatList2D(tmp, dVec);
	Py_DECREF(tmp);
	return a;
}

/**
 * gets the 6D pose of the manipulators' endeffector
 * @param dVec 2D vector that holds the transformation (4x4)
 * @return	0 if success
 * 			-1 if the object could not be get
 */
int ORUtil::getEndEffectorTransform(vector<vector<double> >& pose6D)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* tmp = PyObject_CallMethod(this->pClass, (char*) "getEndEffectorTransform", (NULL));
	int a = CPPDoubleVec2DFromPyFloatList2D(tmp, pose6D);
	Py_DECREF(tmp);
	return a;
}

/**
 * gets the 3D position of the specified object within the current environment
 * @param vector that holds the position (x,y,z)
 * @return	0 if success
 * 			-1 if the object could not be get
 */
int ORUtil::getKinBodyPos(const string& objectName, std::vector<double>& dVec)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* pos3D = PyObject_CallMethod(this->pClass, (char*) "getKinBodyPos", ((char*) "s"), (char*) objectName.c_str());
	int a = CPPDoubleVecFromPyFloatSequence(pos3D, dVec);
	Py_DECREF(pos3D);
	return a;
}

/**
 * searches for an IKSolution at the given 6D Pose.
 * This will only work if the manipulator supports 6D IK solutions (which is not the case for the katana)
 * @param ikSolution
 * @param pose6D
 * @return 0 if an IK solution was found
 * 		  -1 if the manipulator doesn't support 6D IK solutions
 * 		  -2 if some other error occured during searching for IK solution
 *        -3 if no IK solution was found
 */
int ORUtil::searchIKSolution(PlanningParameter& params, vector<double>& ikSolution)
{
//	this->manipulator->parseIterParameters(params, *this);  //TODO

	arma::mat matrix(4, 4);
	std::vector<std::vector<double> > pose6D;
	EulerTransformationMatrices::create_zyx_matrix(params.targetLoc3D[0], params.targetLoc3D[1], params.targetLoc3D[2], params.azimuth, params.elevation, params.roll, matrix);


	pose6D.resize(4);
	for (int i = 0; i < 4; i++)
	{
	pose6D[i].resize(4);
		for (int j = 0; j < 4; j++)
		{
			pose6D[i][j] = matrix.at(i, j);
		}
	}


	SmartACE::SmartGuard guard(this->mutex);

	int result = 0;
	PyObject* pPose6D = PyFloatList2DFromCPPDoubleVec2D(pose6D);
	PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "searchIKSolution", ((char*) "O"), pPose6D);
	Py_DECREF(pPose6D);
	if (!PyInt_Check(pResult))//if the return type is no integer (= successful execution)
	{
		CPPDoubleVecFromPyFloatSequence(pResult, ikSolution);
		this->manipulator->convertOpenRaveAnglesToRealAngles(ikSolution, ikSolution);
//		CPPDoubleVec2DFromPyFloatList2D(pPose6D, pose6D);
		result = 0;
	} else
	{
		result = PyInt_AsLong(pResult);
	}
	return result;
}

/**
 * performes door manipulation.
 * This will only work if the manipulator supports 6D IK solutions (which is not the case for the katana)
 * NOTE: the boundaries must be valid in order to ensure so that the goal bounds can be reached, based on the start-bounds and handleBounds
 * NOTE: if no boundaries are specified, certain default boundaries are taken
 *
 * @param resultTrajToHandle traj to get to the handle traj (output param)
 * @param resultTrajOpenDoor traj to open the door			(output param)
 * @param furniture name of the furniture where the desired door is mounted on
 * @param door name of the door that should be opened
 * @param doorHandleRealTransform the pose where the door handle of the desired door was detected
 * @param openAmount how far the door should be opened. Interpreted as rad if the door is a revoluting door, or as distance in meters if the door is prismatic
 * @param sampleTime timestep size to sample the trajectory in (optional)
 */

bool ORUtil::doDoorManipulation(vector<TrajectoryPoint>& resultTrajToHandle, vector<TrajectoryPoint>& resultTrajOpenDoor, const string& furniture, const string& door, double openAmount, double sampleTime)
{
	vector<double> jointsStart;
	this->getCurrentORJoints(jointsStart);
	SmartACE::SmartGuard guard(this->mutex);
	PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "doDoorManipulation", ((char*) "ssdd"),	(char*) furniture.c_str(),
																											(char*) door.c_str(),
																											openAmount,
																							sampleTime);
	//if there is a result at all
	if (pResult != NULL)
	{
		//if there are actually 2 trajectories present, which is needed (one that leads to the handle and one that opens the door)
		if (PyList_Size(pResult) > 1)
		{
			PyObject* pTraj1 = PyList_GetItem(pResult, 0);
			PyObject* pTraj2 = PyList_GetItem(pResult, 1);

			cout << "starting parsing traj" << endl;
			this->manipulator->parseTrajectory(pTraj1, resultTrajToHandle);
			this->manipulator->parseTrajectory(pTraj2, resultTrajOpenDoor);
//			cout << "finished parsing traj" << endl;
//			cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
//			for(int i = 0; i < jointsStart.size(); ++i)
//			{
//				cout << jointsStart[i] << " " << endl;
//			}
			Py_DECREF(pTraj1);
			Py_DECREF(pTraj2);
			Py_DECREF(pResult);
			return true;
		}else
		{
			Py_DECREF(pResult);
			return false;
		}
	}else
	{
		return false;
	}
}

/**
 *	this method invokes a sampling in order to find a valid IK solution at the given pose within the given boundaries
 *	this
 *	@param params @see ORUtil::PlanningParameter
 * 	@param outputValue: ikSolution the first IK that could be found within the iteration process
 * 	@param outputValue: pose6D the resulting 6D pose which was determined by the iteration process
 *	@return	 0 if success
 * 			-1 if no IK-Solution was found
 */
int ORUtil::iterForIKSolutionsCOMPS(PlanningParameter& params, vector<double>& ikSolution, vector<vector<double> >& pose6D)
{
	int result = 0;
	this->manipulator->parseIterParameters(params, *this);
	PyObject* pParams = PyDict_New();
	PyObject* targetLoc = createPyList(params.targetLoc3D);
	PyDict_SetItem(pParams, PyString_FromString((char*) "targetLoc3D"), targetLoc);
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevation"), PyFloat_FromDouble(params.elevation));
	PyDict_SetItem(pParams, PyString_FromString((char*) "roll"), PyFloat_FromDouble(params.roll));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuth"), PyFloat_FromDouble(params.azimuth));
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthFrom"), PyFloat_FromDouble(params.depthFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthTo"), PyFloat_FromDouble(params.depthTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "sideFrom"), PyFloat_FromDouble(params.sideFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "sideTo"), PyFloat_FromDouble(params.sideTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightFrom"), PyFloat_FromDouble(params.heightFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightTo"), PyFloat_FromDouble(params.heightTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleFrom"), PyFloat_FromDouble(params.elevationAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleTo"), PyFloat_FromDouble(params.elevationAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleFrom"), PyFloat_FromDouble(params.rollAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleTo"), PyFloat_FromDouble(params.rollAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleFrom"), PyFloat_FromDouble(params.azimuthAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleTo"), PyFloat_FromDouble(params.azimuthAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthStepSize"), PyFloat_FromDouble(params.depthStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightStepSize"), PyFloat_FromDouble(params.heightStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleStepSize"), PyFloat_FromDouble(params.elevationAngleStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleStepSize"), PyFloat_FromDouble(params.rollAngleStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleStepSize"), PyFloat_FromDouble(params.azimuthAngleStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "verbose"), PyBool_FromLong(params.verbose));
	SmartACE::SmartGuard guard(this->mutex);

	//now call the python method method...
	PyObject* pResult = NULL;
	pResult = PyObject_CallMethod(this->pClass, (char*) "iterForIKSolutionsCOMPS", ((char*) "O"), pParams);


	PyObject* pIKSolution = PyTuple_GetItem(pResult, 0);
	PyObject* pPose6D = PyTuple_GetItem(pResult, 1);

	//pResult
	if (pIKSolution != Py_None)
	{
		CPPDoubleVecFromPyFloatSequence(pIKSolution, ikSolution);
		this->manipulator->convertOpenRaveAnglesToRealAngles(ikSolution, ikSolution);
		CPPDoubleVec2DFromPyFloatList2D(pPose6D, pose6D);
		result = 0;
	} else
	{
		result = -1;
	}
	Py_DECREF(pResult);
	Py_DECREF(targetLoc);
	Py_DECREF(pParams);
	return result;
}

/**
 * iterates for a IK solution at the specified position with the specified iteration parameters
 * this iteration variant iterates from the mid of the point and the angle values
 * explanation for iteration behavior based of corresponding upper and lower bound values:
 * 		xxxFrom = -0.05, xxxTo = 0.05 ==> will iterate in the specified step sizes from the mid to the outer boundaries (0, 0.5, -0.5 ...)
 * 		xxxFrom = -0.05, xxxTo = 0.08 ==> will iterate as the above starting from 0. After one boundary is reached, it will be iterated only in the other boundaries' direction
 *      xxxFrom =  0.05, xxxTo = 0.08 ==> if both boundaries have the same algebraic sign, it will be symmetrical iterated from the middle to both boundaries.
 *		Please note the possible loss of steps due to fractions from the determination of the middle value (which will be rounded to 3 decimals = 1 mm)
 *		xxxFrom =  0.08, xxxTo = 0.05 ==> does the same as the above. It is not necessary to specify the xxxFrom boundary with a lower value than the xxxTo boundary
 * iteration hierarchy:
 * 						1. azimuth
 * 						2. roll
 * 						3. elevation angle
 *						4. height
 *						5. depth
 *	Meaning for example all azimuth angles are tried before the next roll angle is tried
 *
 * @param params @see ORUtil::PlanningParameter
 * @param ikSolution the first IK that could be found within the iteration process
 * @param pose6D the resulting 6D pose which was determined by the iteration process
 * @return	 0 if success
 * 			-1 if no IK-Solution was found
 */
int ORUtil::iterForIKSolutionsCentered(PlanningParameter& params, vector<double>& ikSolution, vector<vector<double> >& pose6D)
{
	int result = 0;
	this->manipulator->parseIterParameters(params, *this);
	PyObject* pParams = PyDict_New();
	PyObject* targetLoc = createPyList(params.targetLoc3D);
	PyDict_SetItem(pParams, PyString_FromString((char*) "targetLoc3D"), targetLoc);
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevation"), PyFloat_FromDouble(params.elevation));
	PyDict_SetItem(pParams, PyString_FromString((char*) "roll"), PyFloat_FromDouble(params.roll));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuth"), PyFloat_FromDouble(params.azimuth));
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthFrom"), PyFloat_FromDouble(params.depthFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthTo"), PyFloat_FromDouble(params.depthTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightFrom"), PyFloat_FromDouble(params.heightFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightTo"), PyFloat_FromDouble(params.heightTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleFrom"), PyFloat_FromDouble(params.elevationAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleTo"), PyFloat_FromDouble(params.elevationAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleFrom"), PyFloat_FromDouble(params.rollAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleTo"), PyFloat_FromDouble(params.rollAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleFrom"), PyFloat_FromDouble(params.azimuthAngleFrom));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleTo"), PyFloat_FromDouble(params.azimuthAngleTo));
	PyDict_SetItem(pParams, PyString_FromString((char*) "depthStepSize"), PyFloat_FromDouble(params.depthStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "heightStepSize"), PyFloat_FromDouble(params.heightStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "elevationAngleStepSize"), PyFloat_FromDouble(params.elevationAngleStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "rollAngleStepSize"), PyFloat_FromDouble(params.rollAngleStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "azimuthAngleStepSize"), PyFloat_FromDouble(params.azimuthAngleStepSize));
	PyDict_SetItem(pParams, PyString_FromString((char*) "verbose"), PyBool_FromLong(params.verbose));
	SmartACE::SmartGuard guard(this->mutex);

	//now call the python method method...
	PyObject* pResult = NULL;
	pResult = PyObject_CallMethod(this->pClass, (char*) "iterForIKSolutionsCentered", ((char*) "O"), pParams);


	PyObject* pIKSolution = PyTuple_GetItem(pResult, 0);
	PyObject* pPose6D = PyTuple_GetItem(pResult, 1);

	//pResult
	if (pIKSolution != Py_None)
	{
		CPPDoubleVecFromPyFloatSequence(pIKSolution, ikSolution);
		this->manipulator->convertOpenRaveAnglesToRealAngles(ikSolution, ikSolution);
		CPPDoubleVec2DFromPyFloatList2D(pPose6D, pose6D);
		result = 0;
	} else
	{
		result = -1;
	}
	Py_DECREF(pResult);
	Py_DECREF(targetLoc);
	Py_DECREF(pParams);
	return result;
}

/**
 * loads a complete environment (or a single kinbody) into the current environment
 * @param url to the file
 * @return True if success
 */
bool ORUtil::loadFile(const string& environmentFile)
{
	SmartACE::SmartGuard guard(this->mutex);

	bool result = false;
	try
	{
		PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "loadEnvironment", ((char*) ("s")), (char*) environmentFile.c_str());
		result = (pResult == Py_True);
		Py_DECREF(pResult);
	} catch (exception& e)
	{
		std::cerr << e.what();
	}
	return result;
}

/**
 * saves the complete current environment to a file.
 * Will be a dae file
 * @return True if success
 */
bool ORUtil::saveEnvironment(const string& filename)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "saveEnvironment", ((char*) "s"), (char*) filename.c_str());
	bool result = (pResult == Py_True);
	Py_DECREF(pResult);
	return result;
}

/**
 * opens the gripper of the manipulator
 * @return True if success
 */
bool ORUtil::openGripper(vector<TrajectoryPoint>& resultTraj, double sampleTime)
{
	SmartACE::SmartGuard guard(this->mutex);

	int checkResultTmp;
	PyObject* pyResTraj = PyObject_CallMethod(this->pClass, (char*) "openGripper", ((char*) "d"), sampleTime);
	if (pyResTraj != Py_None)
	{
		checkResultTmp = this->manipulator->parseTrajectory(pyResTraj, resultTraj);
	} else
	{
		checkResultTmp = -1;
	}
	Py_DECREF(pyResTraj);
	return checkResultTmp >= 0;
}

/**
 * closes the gripper of the manipulator
 * @return True if success
 */
bool ORUtil::closeGripper(vector<TrajectoryPoint>& resultTraj, double sampleTime)
{
	SmartACE::SmartGuard guard(this->mutex);

	int checkResultTmp;
	PyObject* pyResTraj = PyObject_CallMethod(this->pClass, (char*) "closeGripper", ((char*) "d"), sampleTime);
	if (pyResTraj != Py_None)
	{
		checkResultTmp = this->manipulator->parseTrajectory(pyResTraj, resultTraj);
	} else
	{
		checkResultTmp = -1;
	}
	Py_DECREF(pyResTraj);
	return checkResultTmp >= 0;
}

/**
 * binds the specified object (kinbody) to the gripper
 * so it can be considered for collision free path planning
 * @return	>= 0  if success
 * 			-1 if the trajectory could not be created
 * 			-2 if object could not be found in the environment
 * 			-3 if some other error emerges from python code
 */
int ORUtil::grabObject(const string& objectID, vector<TrajectoryPoint>& resultTraj, double sampleTime)
{
	SmartACE::SmartGuard guard(this->mutex);

	int result = 0;
	// Use getObject when using the vacuum gripper, otherwise use grabObject
	PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "getObject", ((char*) "s"), (char*) objectID.c_str());
	if (!PyInt_Check(pResult))
	{
		result = this->manipulator->parseTrajectory(pResult, resultTraj);
	} else
	{
		result = PyInt_AsLong(pResult);
	}
	Py_DECREF(pResult);
	return result;
}

/**
 * unbinds the specified object (kinbody) from the gripper
 * @return	>= 0  if success
 * 			-1 if the trajectory could not be created
 * 			-2 if there is no object in the gripper
 * 			-3 if some other error emerges from python code
 */
int ORUtil::releaseGrabbedObject(vector<TrajectoryPoint>& resultTraj, double sampleTime)
{
	SmartACE::SmartGuard guard(this->mutex);

	int result = 0;
	//PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "releaseGrabbedObject", ((char*) "d"), sampleTime);
	PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "releaseObject", (NULL));
	if (!PyInt_Check(pResult))
	{
		result = this->manipulator->parseTrajectory(pResult, resultTraj);
	} else
	{
		result = PyInt_AsLong(pResult);
	}
	Py_DECREF(pResult);
	return result;
}

/**
 * sets the active joints of the manipulator
 * the active joints will _only_ be considered for _setting_ the joint values and _not_ path planning
 * @param mask sets which joints should be active
 * example: the mask [0,1,2,3,4] will set the joints 0-4 active and all others inactive
 * @return True is success
 */
bool ORUtil::setActiveDOFs(const vector<int>& mask)
{
	SmartACE::SmartGuard guard(this->mutex);

	PyObject* pMask = createPyList(mask);
	PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "setActiveDOFs", ((char*) "O"), pMask);
	bool result = (pResult == Py_True);
	Py_DECREF(pResult);
	Py_DECREF(pMask);
	return result;
}

/**
 * gets the active joints of the manipulator in the virtual environment
 * @param vector that will hold the resulting joints
 * @return 0 on success, -1 else
 */
int ORUtil::getCurrentORJoints(vector<double>& joints)
{
	SmartACE::SmartGuard guard(this->mutex);
	vector<double> resultJoints;
	PyObject* pResult = PyObject_CallMethod(this->pClass, (char*) "getCurrentORJoints", (NULL));
	int result = CPPDoubleVecFromPyFloatSequence(pResult, resultJoints);
	this->manipulator->convertOpenRaveAnglesToRealAngles(resultJoints, joints);
	Py_DECREF(pResult);
	return result;
}

/**
 * @return the specific manipulator of this class instance
 */
SpecificManipulator::Manipulator* ORUtil::getManipulator()
{
	return this->manipulator;
}

