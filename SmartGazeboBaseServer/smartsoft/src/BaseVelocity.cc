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

#include "BaseVelocity.hh"
#include "SmartGazeboBaseServer.hh"

BaseVelocity::BaseVelocity() : mutex() {}

BaseVelocity::~BaseVelocity() {}

void BaseVelocity::init() {

	this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	this->node->Init();

	this->subscriber = this->node->Subscribe(COMP->getGlobalState().getSettings().getBaseVelTopic(), &BaseVelocity::OnMsg, this);
	std::cout << "Subscribed to BaseVelocity topic !" << std::endl;
}

void BaseVelocity::OnMsg(ConstLinkDataPtr &_msg) {
	
	// !!! Do not use cout or sleep within this callback function !!!
	
	CommBasicObjects::CommBaseVelocity commBaseVelocity;

	// TODO: Apply the correct conversion here. 
	// This workaround is only valid for 2 DOFs like differential-drive (no omni-drive).
	double vel = sqrt(pow(_msg->linear_velocity().x(),2)+pow(_msg->linear_velocity().y(),2));

	commBaseVelocity.set_vX(vel, 1);
	
	// TODO: Set vy and vz
	commBaseVelocity.set_vY(0, 1);
	commBaseVelocity.set_vZ(0, 1);

	// TODO: Set wx and wy
	commBaseVelocity.set_WX_base(0);
	commBaseVelocity.set_WY_base(0);
	commBaseVelocity.set_WZ_base(_msg->angular_velocity().z());

	this->mutex.acquire();
	this->commBaseVelocity = commBaseVelocity;
	this->mutex.release();
}

CommBasicObjects::CommBaseVelocity BaseVelocity::getBaseVelocity() {
	this->mutex.acquire();
	CommBasicObjects::CommBaseVelocity velocity = this->commBaseVelocity;
	this->mutex.release();
	return velocity;
}
