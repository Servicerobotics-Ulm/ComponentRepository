
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
 * msg_conversion.cc
 *
 *  Created on: May 21, 2019
 *      Author: shaikv3
 */

#include "msg_conversion.hh"
#include "cartographer/transform/transform.h"

std::tuple<PointCloudWithIntensities,Time> ToCartographer_PointCloudWithIntensities(const CommBasicObjects::CommMobileLaserScan& in_laser)
{

  PointCloudWithIntensities point_cloud;


  size_t scan_size = in_laser.get_scan_size();
  double angle_resolution_radians = in_laser.get_scan_resolution();
  uint8_t laser_frequency = 50; // s300 frequency 12.5Hz , hukoyu :10Hz, LMS1xx : 50
  float time_increment = (1.0/laser_frequency)/scan_size;


  //assert(angle_radians < angle_radians + (scan_size*angle_resolution_radians) && " start angle failure");
  //assert(angle_radians > 0.0f && " start angle >0.0f failure");
    int timing_index = 0;
  //for (size_t i = 0; i < scan_size; ++i) {
    for (int i = scan_size-1; i >=0 ; --i) {
      ++timing_index;
      //int timing_index = i;
	  float factor = 1.0;
	  const float first_echo = in_laser.get_scan_distance(i, 1);
	  float angle_radians    = factor * in_laser.get_scan_angle(i);

	  const Eigen::AngleAxisf rotation(angle_radians, Eigen::Vector3f::UnitZ());
	  const cartographer::sensor::TimedRangefinderPoint point{ rotation * (first_echo * Eigen::Vector3f::UnitX()),
				                                               factor * timing_index * time_increment
	                                                         };

	  point_cloud.points.push_back(point);
	  point_cloud.intensities.push_back(in_laser.get_scan_intensity(i));

	  //std::cout << " i = " << i << ", l = " << first_echo <<", angle = " << angle_radians << "\n";
  }
  //std::cout << std::endl;
  //std::cout <<"-------------------------------------------------" <<std::endl;

  cartographer::common::Time timestamp;
  CommBasicObjects::CommTimeStamp ts = CommBasicObjects::CommTimeStamp::now();
  //ToCartographer_CommonTime(in_laser.get_scan_time_stamp(), timestamp);
  ToCartographer_CommonTime(ts, timestamp);

  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }

  return std::make_tuple(point_cloud, timestamp);
}


void ToCartographer_CommonTime(const CommBasicObjects::CommTimeStamp& ss_time, Time& carto_time)
{

	carto_time =  cartographer::common::FromUniversal((ss_time.getSec() + cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll +
	                                                  (ss_time.getUsec()*1000 + 50) / 100);  // + 50 to get the rounding correct.

}

Eigen::Quaterniond ToEigenQuaternion_rotation(const CommBasicObjects::CommPose3d& pose){

	//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code_2
	double yaw = pose.get_azimuth();
	double pitch = pose.get_elevation();
	double roll = pose.get_roll();

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double w = cy * cp * cr + sy * sp * sr;
    double x = cy * cp * sr - sy * sp * cr;
    double y = sy * cp * sr + cy * sp * cr;
    double z = sy * cp * cr - cy * sp * sr;
    return Eigen::Quaterniond(w, x, y, z);
}

Eigen::Vector3d ToEigenVector3d_translation(const CommBasicObjects::CommPose3d& pose){

	return Eigen::Vector3d(pose.get_x(1), pose.get_y(1), pose.get_z(1));
}


Rigid3d ToCartographer_Rigid3d(const CommBasicObjects::CommPose3d& pose)
{
	return Rigid3d(ToEigenVector3d_translation(pose), ToEigenQuaternion_rotation(pose));
}

Eigen::Vector3f ToTranslationVector(const CommBasicObjects::CommPose3d& pose)
{
	return Eigen::Vector3f(pose.get_x(1), pose.get_y(1), pose.get_z(1));
}

OdometryData ToCartographer_OdometryData(const CommBasicObjects::CommBasePose& in_pose, const CommBasicObjects::CommTimeStamp& in_time)
{
	cartographer::common::Time timestamp;
	ToCartographer_CommonTime(in_time, timestamp);

	Rigid3d OdomPose = ToCartographer_Rigid3d(in_pose.getPose3D());

	return OdometryData {timestamp, OdomPose};
}

double pi_to_pi(double angle){

	   angle+=M_PI;
	   double ret_angle = fmod(angle,2*M_PI);

	   if(angle<0)
	     ret_angle+=2*M_PI;

	   ret_angle-=M_PI;

	   return ret_angle;
}

void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	yaw = atan2(siny_cosp, cosy_cosp);
}
