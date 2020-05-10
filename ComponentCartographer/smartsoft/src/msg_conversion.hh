
//--------------------------------------------------------------------------
//BSD 3-Clause License
//
//  Copyright (C) Servicerobotics Ulm
//  University of Applied Sciences Ulm
//  Prittwitzstr. 10
//  89075 Ulm
//  Germany
//  All rights reserved.
//
//  Author: Nayabrasul Shaik
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//* Neither the name of the copyright holder nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/*
 * msg_conversion.hh
 *
 *  Created on: May 21, 2019
 *      Author: shaikv3
 */

#ifndef SMARTSOFT_SRC_MSG_CONVERSION_HH_
#define SMARTSOFT_SRC_MSG_CONVERSION_HH_

#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/sensor/odometry_data.h"

#include "CommBasicObjects/CommMobileLaserScan.hh"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

using PointCloudWithIntensities = cartographer::sensor::PointCloudWithIntensities;
using Time = cartographer::common::Time;
using Rigid3d = cartographer::transform::Rigid3d;
using TranslationVector = Eigen::Vector3f;
using OdometryData = cartographer::sensor::OdometryData;

#define degree2radians(x) x * 0.01745329;
#define radians2degree(x) x * 57.29578;

std::tuple<PointCloudWithIntensities,Time> ToCartographer_PointCloudWithIntensities(const CommBasicObjects::CommMobileLaserScan& in_laser);
void ToCartographer_CommonTime(const CommBasicObjects::CommTimeStamp& ss_time, Time& carto_time);

Eigen::Quaterniond ToEigenQuaternion_rotation(const CommBasicObjects::CommPose3d& pose);
Eigen::Vector3d ToEigenVector3d_translation(const CommBasicObjects::CommPose3d& pose);
Rigid3d ToCartographer_Rigid3d(const CommBasicObjects::CommPose3d& pose);

Eigen::Vector3f ToTranslationVector(const CommBasicObjects::CommPose3d& pose);
OdometryData ToCartographer_OdometryData(const CommBasicObjects::CommBasePose& in_pose, const CommBasicObjects::CommTimeStamp& in_time);
double pi_to_pi(double angle);
void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
#endif /* SMARTSOFT_SRC_MSG_CONVERSION_HH_ */
