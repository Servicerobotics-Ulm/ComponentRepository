//--------------------------------------------------------------------------
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
//--------------------------------------------------------------------------
//
//  Copyright (C)  2021 Thomas Feldmeier
//
//        schlegel@hs-ulm.de
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

#include "PoseUpdateActivity.hh"
#include "ComponentWebotsURServer.hh"

#include <iostream>

#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Connector.hpp>
#include <webots/Keyboard.hpp>

using namespace webots;

PoseUpdateActivity::PoseUpdateActivity(SmartACE::SmartComponent *comp) :
    PoseUpdateActivityCore(comp) {
    std::cout << "constructor PoseUpdateActivity\n";
}
PoseUpdateActivity::~PoseUpdateActivity() {
    std::cout << "destructor PoseUpdateActivity\n";
}

void PoseUpdateActivity::setVacuumGripper(bool isVacuum) {
    isVacuumOn = isVacuum;
}

void PoseUpdateActivity::handleEnterState(const std::string &substate) {
    if (substate == "trajectory")
        newProgram = prTrajectory;
    else if (substate == "neutral")
        newProgram = prNeutral;
}

void PoseUpdateActivity::on_trajectorySendServer(const CommManipulatorObjects::CommManipulatorTrajectory &input) {
    if (input.get_joint_count() != nrJoints) {
        std::cerr << "wrong number joints" << std::endl;
        return;
    }
    if (input.getFlag() != CommManipulatorObjects::ManipulatorTrajectoryFlag::JOINT_ANGLES) {
        std::cerr << "ERROR: only joint_angles allowed, not TCP" << std::endl;
        return;
    }
    std::unique_lock<std::mutex> lock(jointMutex);
    for (int i = 0; i < input.get_trajectory_size(); i++) {
        Trajectory t;
        auto x = input.getJoint_anglesElemAtPos(i);
        t.time = x.getTime();
        for (int j = 0; j < nrJoints; j++)
            t.jointTargetPosition[j] = x.getValuesElemAtPos(j);
        trajectory.push_back(t);
    }
}

void PoseUpdateActivity::on_baseStateServiceIn(const CommBasicObjects::CommBaseState &input) {
}

void PoseUpdateActivity::getMobileManipulatorState(CommManipulatorObjects::CommMobileManipulatorState &mobileState) {
    ParameterStateStruct params = COMP->getParameters();
    CommManipulatorObjects::CommManipulatorState state;
    {
        std::unique_lock<std::mutex> lock(jointMutex);
        state.set_id(stateIdCounter++);
        state.set_pose_manipulator(params.getManipulator().getX(), params.getManipulator().getY(),
            params.getManipulator().getZ(), params.getManipulator().getAzimuth(),
            params.getManipulator().getElevation(), params.getManipulator().getRoll());
        state.set_joint_count(nrJoints);
        for (int i = 0; i < nrJoints; ++i) {
            state.set_joint_angle(i, jointPosition[i]);
        }
    }
    // todo:
    // state.set_pose_TCP_manipulator(x, y, z, yaw, pitch, roll, 1.0);
    state.set_valid(true);
    mobileState.set_manipulator_state(state);
    CommBasicObjects::CommBaseState baseState;
    if (params.getBase().getOn_base()) {
        if (baseStateServiceInGetUpdate(baseState) == Smart::SMART_OK) {
            mobileState.set_base_state(baseState);
        } else {
            mobileState.set_base_state(default_baseState);
            mobileState.set_valid(false);
        }
    } else {
        mobileState.set_base_state(default_baseState);
    }
}

int PoseUpdateActivity::on_entry() {
    return 0;
}

void PoseUpdateActivity::setGoalReached(bool reached) {
    if (goalReached == reached)
        return;
    goalReached = reached;
    CommManipulatorObjects::CommManipulatorEventState state;
    state.setEvent(
        reached ?
            CommManipulatorObjects::ManipulatorEvent::GOAL_REACHED :
            CommManipulatorObjects::ManipulatorEvent::GOAL_NOT_REACHED);
    COMP->manipulatorEventServiceOut->put(state);
    std::cout << (reached ? "GOAL_REACHED" : "GOAL_NOT_REACHED") << std::endl;
}

int PoseUpdateActivity::on_execute() {
    ParameterStateStruct params = COMP->getParameters();
    CommBasicObjects::CommBasePose default_base_position;
    default_base_position.set_x(params.getBase().getX());
    default_base_position.set_y(params.getBase().getY());
    default_base_position.set_z(params.getBase().getZ());
    CommBasicObjects::CommBaseVelocity zero_velocity;
    default_baseState.setBasePose(default_base_position);
    default_baseState.set_base_velocity(zero_velocity);

// ********** robot ***********
    std::string name = params.getWebots().getRobotName();
    char environment[256] = "WEBOTS_ROBOT_NAME=";
    putenv(strcat(environment, name.c_str()));
    std::cout << " \033[0;32mConnect to webots robot with name '" << name << "' ...\033[0m" << std::endl;
    Supervisor *robot = new Supervisor();
    if (!robot) {
        std::cerr << "Webots Robot '" << name << "' not found" << std::endl;
        return -1;
    }
    std::cout << "Robot '" << name << "'" << std::endl;

// *********** VacuumGripper *********
    DistanceSensor* distanceSensor = robot->getDistanceSensor("VacuumGripperDistanceSensor");
    distanceSensor->enable(robot->getBasicTimeStep());

    Connector* connector = robot->getConnector("VacuumGripperConnectorName");
    connector->enablePresence(robot->getBasicTimeStep());
    Keyboard* keyboard = robot->getKeyboard();
    keyboard->enable(robot->getBasicTimeStep());

// *********** joints *************
    std::string jointNames[nrJoints] = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
        "wrist_2_joint", "wrist_3_joint" };
    Motor *jointMotor[nrJoints];
    PositionSensor *jointSensor[nrJoints];

    Trajectory firstPosition;
    firstPosition.time = 0.0;
    for (int i = 0; i < nrJoints; i++) {
        jointMotor[i] = robot->getMotor(jointNames[i]);
        jointSensor[i] = robot->getPositionSensor(jointNames[i] + "_sensor");
        if (!jointMotor[i] || !jointSensor[i]) {
            std::cerr << "Webots Motor or PositionSensor " << i << " not found";
            return -1;
        }
        jointSensor[i]->enable(robot->getBasicTimeStep());
    }
    // do one timeStep to get data from sensors
    // todo: use supervisor to read position without time delay
    robot->step(robot->getBasicTimeStep());
    for (int i = 0; i < nrJoints; i++) {
        firstPosition.jointTargetPosition[i] = jointSensor[i]->getValue();
        std::cout << " first" << i << ":" << firstPosition.jointTargetPosition[i];
    }
    {
        std::unique_lock<std::mutex> lock(jointMutex);
        trajectory.insert(trajectory.begin(), firstPosition);
    }
    double jointTargetPosition[nrJoints];
    double timeSpend = 0.0; // the time already spend on actual trajectory point
    while (robot->step(robot->getBasicTimeStep()) != -1) {
        // TODO: remove (debug only)
        int key = keyboard->getKey();
        if(key=='0' || key=='1') {
          std::cout << "key=" << key << " presence=" << connector->getPresence() << "distance=" << distanceSensor->getValue() << std::endl;
          if(key=='0')
            connector->unlock();
          else
            connector->lock();
        }

        Program program = newProgram;
        if (program == prNeutral)
            continue;
        if(isVacuumOn)
            connector->unlock();
        else if(distanceSensor->getValue())
        { // start mutex
            std::unique_lock<std::mutex> lock(jointMutex);
            for (int i = 0; i < nrJoints; i++)
                jointPosition[i] = jointSensor[i]->getValue();
            timeSpend += robot->getBasicTimeStep() / 1000.0;
            if (trajectory.size() > 1)
                setGoalReached(false);
            while (trajectory.size() > 1 && timeSpend > trajectory[1].time) {
                std::cout << "trajectory done: time=" << trajectory[0].time;
                for (int i = 0; i < nrJoints; i++) {
                    std::cout << " " << i << ":" << trajectory[0].jointTargetPosition[i];
                }
                std::cout << std::endl;
                timeSpend -= trajectory[1].time;
                trajectory.erase(trajectory.begin());
            }
            if (trajectory.size() == 1) {
                for (int i = 0; i < nrJoints; i++)
                    jointTargetPosition[i] = trajectory[0].jointTargetPosition[i];
                timeSpend = 0.0;
                bool isReached = true;
                for (int i = 0; i < nrJoints; i++)
                    if (std::abs(jointTargetPosition[i] - jointPosition[i]) > 0.002)
                        isReached = false;
                if (isReached)
                    setGoalReached(true);
            } else {
                double timeQuotient = timeSpend / trajectory[1].time;
                for (int i = 0; i < nrJoints; i++)
                    jointTargetPosition[i] = trajectory[0].jointTargetPosition[i] * (1.0 - timeQuotient)
                        + trajectory[1].jointTargetPosition[i] * timeQuotient;
            }
        } // end mutex
        for (int i = 0; i < nrJoints; i++)
            jointMotor[i]->setPosition(jointTargetPosition[i]);
        /*
         if (params.getManipulator().getVerbose()) {
         std::cout << "program=" << program;
         for (int i = 0; i < nrJoints; i++) {
         std::cout << " " << i << ":" << jointPosition[i];
         }
         std::cout << std::endl;
         }
         */
        CommManipulatorObjects::CommMobileManipulatorState mobileState;
        getMobileManipulatorState(mobileState);
        this->posePushServerPut(mobileState);
    }
    delete robot;
    return 1; // webots world has ended, stop thread
}

int PoseUpdateActivity::on_exit() {
    return 0;
}