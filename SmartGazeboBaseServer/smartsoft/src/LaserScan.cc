//--------------------------------------------------------------------------
//
//  Copyright (C) 2017 Timo Blender
//
//      schlegel@hs-ulm.de
//
//      Service Robotics Ulm
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
//
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
//-------------------------------------------------------------------------

#include "LaserScan.hh"

#include <math.h>
#include "SmartGazeboBaseServer.hh"

LaserScan::LaserScan(Queue* queue) : mutex(), sem() {
	this->queue = queue;
}

LaserScan::~LaserScan() {}

void LaserScan::init() {

	this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	this->node->Init();

	this->subscriber = this->node->Subscribe(COMP->getGlobalState().getSettings().getLaserTopic(), &LaserScan::OnMsg, this);
	std::cout << "Subscribed to LaserScan topic !" << std::endl;
}

void LaserScan::OnMsg(ConstLaserScanStampedPtr &_msg) {

	// !!! Do not use cout or sleep within this callback function !!!
	
	CommBasicObjects::CommMobileLaserScan commMobileLaserScan;

	// Set timestamp
	timeval _receive_time;
	gettimeofday(&_receive_time, 0);
	commMobileLaserScan.set_scan_time_stamp(CommBasicObjects::CommTimeStamp(_receive_time));
	
	commMobileLaserScan.set_scanner_x(_msg->scan().world_pose().position().x(), 1);
	commMobileLaserScan.set_scanner_y(_msg->scan().world_pose().position().y(), 1);
	commMobileLaserScan.set_scanner_z(_msg->scan().world_pose().position().z(), 1);

	// See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	double ysqr = _msg->scan().world_pose().orientation().y() * _msg->scan().world_pose().orientation().y();
	double t3 = +2.0 * (_msg->scan().world_pose().orientation().w() * _msg->scan().world_pose().orientation().z() + _msg->scan().world_pose().orientation().x() * _msg->scan().world_pose().orientation().y());
	double t4 = +1.0 - 2.0 * (ysqr + _msg->scan().world_pose().orientation().z() * _msg->scan().world_pose().orientation().z());
	double yaw = std::atan2(t3, t4);

	commMobileLaserScan.set_scanner_azimuth(yaw);

    CommBasicObjects::CommPose3d SensorOffset(70.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    commMobileLaserScan.set_sensor_pose(SensorOffset);

	commMobileLaserScan.set_scan_length_unit(1);

	double openingAngle = (_msg->scan().angle_max() - _msg->scan().angle_min()) * 180.0 / M_PI;
	double resolution = _msg->scan().angle_step() * 180.0 / M_PI;

	// TODO: Check and adapt if necessary
	commMobileLaserScan.set_scan_double_field_of_view(_msg->scan().angle_min()*180.0/M_PI, resolution);

	commMobileLaserScan.set_max_scan_size(_msg->scan().count());

	commMobileLaserScan.set_min_distance(_msg->scan().range_min(), 1);
	commMobileLaserScan.set_max_distance(_msg->scan().range_max(), 1);

	if (_msg->scan().count() == 0) {
		commMobileLaserScan.set_scan_valid(false);
		commMobileLaserScan.set_scan_size(_msg->scan().count());
	}

	else {
		commMobileLaserScan.set_scan_valid(true);
		commMobileLaserScan.set_scan_size(_msg->scan().count());

		for (int i = 0; i < _msg->scan().count(); i++) {
			
			if (std::isinf(_msg->scan().ranges(i))) {
				commMobileLaserScan.set_scan_distance(i, _msg->scan().range_max(),1);
			}
			else {
				commMobileLaserScan.set_scan_distance(i, _msg->scan().ranges(i),1);
			}
			commMobileLaserScan.set_scan_index(i, i);
		}

		for (int i = 0; i < _msg->scan().count(); i++) {
			commMobileLaserScan.set_scan_intensity(i, _msg->scan().intensities(i));
		}
	}

	this->mutex.acquire();
	this->commMobileLaserScan = commMobileLaserScan;
	this->queue->addEntry(this->commMobileLaserScan);
	this->mutex.release();
}
