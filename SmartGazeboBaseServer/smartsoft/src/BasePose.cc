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

#include "BasePose.hh"
#include "SmartGazeboBaseServer.hh"


BasePose::BasePose() : mutex() {}

BasePose::~BasePose() {}

void BasePose::OnMsg(ConstPosePtr &_msg) {

	// !!! Do not use cout or sleep within this callback function !!!

	CommBasicObjects::CommBasePose commBasePose;

	commBasePose.set_x(_msg->position().x(), 1);
	commBasePose.set_y(_msg->position().y(), 1);
	commBasePose.set_z(_msg->position().z(), 1);

	// See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	double ysqr = _msg->orientation().y() * _msg->orientation().y();
	double t3 = +2.0 * (_msg->orientation().w() * _msg->orientation().z() + _msg->orientation().x() * _msg->orientation().y());
	double t4 = +1.0 - 2.0 * (ysqr + _msg->orientation().z() * _msg->orientation().z());
	double yaw = std::atan2(t3, t4);

	commBasePose.set_base_azimuth(yaw);
	
	// TODO: Convert to elevation / roll
	commBasePose.set_base_elevation(0);
	commBasePose.set_base_roll(0);

	this->mutex.acquire();
	this->commBasePose = commBasePose;
	this->mutex.release();
}

void BasePose::init() {

	this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	this->node->Init();

	this->subscriber = this->node->Subscribe(COMP->getGlobalState().getSettings().getBasePoseTopic(), &BasePose::OnMsg, this);
	std::cout << "Subscribed to BasePose topic !" << std::endl;
}

CommBasicObjects::CommBasePose BasePose::getBasePose() {
	this->mutex.acquire();
	CommBasicObjects::CommBasePose pose = this->commBasePose;
	this->mutex.release();
	return pose;
}
