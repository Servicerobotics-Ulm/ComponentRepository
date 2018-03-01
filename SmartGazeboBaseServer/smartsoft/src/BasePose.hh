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

#ifndef BASEPOSE_HH_
#define BASEPOSE_HH_

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <CommBasicObjects/CommBasePose.hh>

#include <aceSmartSoft.hh>

class BasePose {
private:
	CommBasicObjects::CommBasePose commBasePose;
	gazebo::transport::NodePtr node;
	gazebo::transport::SubscriberPtr subscriber;
	SmartACE::SmartMutex mutex;

public:
	BasePose();
	virtual ~BasePose();
	void init();
	void OnMsg(ConstPosePtr &_msg);
	CommBasicObjects::CommBasePose getBasePose();
};

#endif /* BASEPOSE_HH_ */
