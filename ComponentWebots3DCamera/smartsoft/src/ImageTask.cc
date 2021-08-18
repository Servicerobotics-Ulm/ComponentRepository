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
//  Copyright (C) 2011, 2017 Matthias Lutz, Dennis Stampfer, Matthias Rollenhagen, Nayabrasul Shaik
//                2021 Thomas Feldmeier
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

#include "ImageTask.hh"
#include "ComponentWebots3DCamera.hh"
#include <iostream>
#include <webots/Supervisor.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include "EulerTransformationMatrices.hh"

using namespace std;
using namespace webots;

ImageTask::ImageTask(SmartACE::SmartComponent *comp) :
    ImageTaskCore(comp) {
}
ImageTask::~ImageTask() {
}

// called from SmartStateChangeHandler.cc
void ImageTask::handleEnterState(const std::string &substate) {
    if (substate == "pushimage")
        newProgram = prPushImage;
    else if (substate == "queryonly") {
        newProgram = prQueryOnly;
        COMP->NewestImageMutex.acquire();
        COMP->newestImage = NULL;
        COMP->NewestImageMutex.release();
    }
    else if (substate == "neutral")
        newProgram = prNeutral;
}

// called from SmartStateChangeHandler.cc
void ImageTask::handleQuitState(const std::string &substate) {
    if (substate == "nonneutral")
        newProgram = prNeutral;
}

int ImageTask::on_entry() {
    return 0;
}
int ImageTask::on_exit() {
    return 0;
}
int ImageTask::on_execute() {
    ParameterStateStruct params = COMP->getParameters();

    // ********** robot ***********
    string name = params.getWebots().getRobotName();
    char environment[256] = "WEBOTS_ROBOT_NAME=";
    putenv(strcat(environment, name.c_str()));
    std::cout << "\033[0;32mConnect to webots robot with name '" << name << "' ...\033[0m" << std::endl;
    Supervisor *robot = new Supervisor();
    if (!robot) {
        cerr << "Webots Robot '" << name << "' not found" << endl;
        return -1;
    }
    std::cout << "done" << std::endl;

    // *********** camera *************
    name = params.getWebots().getCameraName();
    Camera *camera = robot->getCamera(name);
    if (!camera) {
        cerr << "Webots Camera '" << name << "' not found" << endl;
        return -1;
    }
    unsigned int width = camera->getWidth();
    unsigned int height = camera->getHeight();
    double fieldOfView = camera->getFov();
    cout << "Camera '" << name << "' width " << width << " height " << height << " fieldOfView " << fieldOfView << endl;
    // *colorData = new unsigned char[...]
    unsigned char colorData[width * height * 3];
    DomainVision::CommVideoImage colorImage;
    arma::mat intrinsic = arma::zeros(4, 4);
    // tan(fieldOfView/2) = width/2 / fx  =>  fx = ...
    intrinsic(0, 0) = width / 2.0 / tan(fieldOfView / 2.0);
    intrinsic(1, 1) = width / 2.0 / tan(fieldOfView / 2.0);
    intrinsic(0, 2) = width / 2.0;
    intrinsic(1, 2) = height / 2.0;
    intrinsic(2, 2) = 1;
    colorImage.set_parameters(width, height, DomainVision::FormatType::RGB24);
    colorImage.set_intrinsic(intrinsic);
    colorImage.setIs_valid(true);
    colorImage.set_data(colorData);
    colorImage.setDistortion_model(DomainVision::ImageDistortionModel::NONE);

    // ************ rangeFinder ************
    name = params.getWebots().getRangeFinderName();
    RangeFinder *rangeFinder = robot->getRangeFinder(name);
    if (!rangeFinder) {
        cerr << "Webots RangeFinder '" << name << "' not found" << endl;
        return -1;
    }
    int width2 = rangeFinder->getWidth();
    int height2 = rangeFinder->getHeight();
    double fieldOfView2 = rangeFinder->getFov();
    double minRange = rangeFinder->getMinRange();
    double maxRange = rangeFinder->getMaxRange();
    cout << "RangeFinder '" << name << "' width " << width2 << " height " << height2 << " fieldOfView " << fieldOfView2
        << " minRange " << minRange << " maxRange " << maxRange << endl;
    arma::mat intrinsic2 = arma::zeros(4, 4);
    intrinsic2(0, 0) = width2 / 2.0 / tan(fieldOfView2 / 2.0);
    intrinsic2(1, 1) = width2 / 2.0 / tan(fieldOfView2 / 2.0);
    intrinsic2(0, 2) = width2 / 2.0;
    intrinsic2(1, 2) = height2 / 2.0;
    intrinsic2(2, 2) = 1;
    arma::mat extrinsics = arma::zeros(1, 12);
    extrinsics(0, 0) = 1;
    extrinsics(0, 4) = 1;
    extrinsics(0, 8) = 1;
    DomainVision::CommDepthImage depthImage;
    depthImage.set_intrinsic(intrinsic2);
    depthImage.set_extrinsic(extrinsics);
    depthImage.setWidth(width2).setHeight(height2).setFormat(DomainVision::DepthFormatType::FLOAT).setPixel_size(32).setIs_valid(
        true).setDistortion_model(DomainVision::ImageDistortionModel::NONE);

    DomainVision::CommRGBDImage *image = new DomainVision::CommRGBDImage;
    image->setColor_image(colorImage);
    image->setDepth_image(depthImage);
    int seqCount = 0;
    bool isCameraOn = false;
    Program program = Program::prNeutral;
    // todo: parameter for time between two pushed images
    //       (using robot->getBasicTimeStep() can be very slow)
    while (robot->step(robot->getBasicTimeStep()) != -1) {
        Program program = newProgram;
        if (program == Program::prPushImage || (program == Program::prQueryOnly && isQueryImage)) {
            if (!isCameraOn) {
                isCameraOn = true;
                std::cout << "Enabling Camera" << std::endl;
                camera->enable(robot->getBasicTimeStep());
                rangeFinder->enable(robot->getBasicTimeStep());
                //wait one timeStep to get the valid image
                if (robot->step(robot->getBasicTimeStep()) == -1)
                    break;
            }
            // generate newestImage
            const unsigned char *bgraData = camera->getImage();
            for (int i = 0; i < width * height; i++) {
                colorData[i * 3] = bgraData[i * 4 + 2];
                colorData[i * 3 + 1] = bgraData[i * 4 + 1];
                colorData[i * 3 + 2] = bgraData[i * 4 + 0];
            }
            colorImage.set_data(colorData);
            colorImage.setSeq_count(seqCount);
            depthImage.setSeq_count(seqCount);
            image->setSeq_count(seqCount);
            seqCount++;
            const float *depthData = rangeFinder->getRangeImage();
            depthImage.setMin_distcance(params.getHardware_properties().getMin_distance());
            depthImage.setMax_distcance(params.getHardware_properties().getMax_distance());
            depthImage.set_distances(depthData, width2, height2);
            CommBasicObjects::CommBaseState base_state;
            arma::mat sensorMat;
            EulerTransformationMatrices::create_zyx_matrix(params.getSensor_pose().getX(),
                params.getSensor_pose().getY(), params.getSensor_pose().getZ(), params.getSensor_pose().getAzimuth(),
                params.getSensor_pose().getElevation(), params.getSensor_pose().getRoll(), sensorMat);
            image->setIs_valid(true);
            if (params.getBase().getOn_ptu()) {
                CommBasicObjects::CommDevicePoseState devicePoseState;
                if (COMP->ptuPosePushNewestClient->getUpdate(devicePoseState) != Smart::SMART_OK) {
                    std::cerr << "[Image Task] WARNING: failed to get current ptu device state" << std::endl;
                    image->setIs_valid(false);
                }
                sensorMat = devicePoseState.get_device_pose3d_robot().getHomogeneousMatrix() * sensorMat;

            } else if (params.getBase().getOn_ur()) {
                CommManipulatorObjects::CommMobileManipulatorState mobileManipulatorState;
                if (COMP->urPosePushTimedClient->getUpdate(mobileManipulatorState) != Smart::SMART_OK) {
                    std::cerr << "[Image Task] WARNING: failed to get current UR device state" << std::endl;
                    image->setIs_valid(false);
                }
                double x, y, z, azimuth, elevation, roll;
                mobileManipulatorState.getManipulator_state().get_pose_TCP_robot(x, y, z, azimuth, elevation, roll,
                    0.001);
                CommBasicObjects::CommPose3d tcpPose(x, y, z, azimuth, elevation, roll, 0.001);
                sensorMat = tcpPose.getHomogeneousMatrix() * sensorMat;
            }
            if (params.getBase().getOn_base()) {
                if (COMP->basePushTimedClient->getUpdate(base_state) != Smart::SMART_OK) {
                    std::cerr << "[Image Task] WARNING: failed to get current base state" << std::endl;
                    image->setIs_valid(false);
                }
            } else {
                CommBasicObjects::CommBasePose tmp;
                tmp.set_x(params.getBase().getX());
                tmp.set_y(params.getBase().getY());
                tmp.set_z(params.getBase().getZ());
                tmp.set_base_azimuth(params.getBase().getBase_a());
                base_state.set_base_position(tmp);
                CommBasicObjects::CommBaseVelocity zero_velocity;
                base_state.set_base_velocity(zero_velocity);
                base_state.set_time_stamp(CommBasicObjects::CommTimeStamp::now());
            }
            CommBasicObjects::CommPose3d sensorPose(sensorMat);
            image->setSensor_pose(sensorPose);
            image->setBase_state(base_state);
            colorImage.set_sensor_pose(sensorPose);
            colorImage.set_base_state(base_state);
            image->setColor_image(colorImage);
            image->setDepth_image(depthImage);
            COMP->NewestImageMutex.acquire();
            COMP->newestImage = image;
            COMP->NewestImageMutex.release();
            if (params.getSettings().getPushnewest_rgbd_image()) {
                COMP->rGBDImagePushServiceOut->put(*image);
            }
            if (params.getSettings().getPushnewest_color_image()) {
                COMP->rGBImagePushServiceOut->put(colorImage);
            }
            if (params.getSettings().getPushnewest_depth_image()) {
                COMP->depthPushNewestServer->put(depthImage);
            }
            if (params.getSettings().getDebug_info()) {
                std::cout << "[Image Task] Current RGBD frame number =" << image->getSeq_count() << " is "
                    << (image->getIs_valid() ? "valid" : "invalid") << ", program = " << program << "\n";
            }
        } else if (isCameraOn) {
            isCameraOn = false;
            std::cout << "Disabling Camera" << std::endl;
            camera->disable();
            rangeFinder->disable();
        }
    }
    delete robot;
    return 1; // webots world has ended, stop thread
}
