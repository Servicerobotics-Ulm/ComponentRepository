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

#ifndef LASERSCAN_HH_
#define LASERSCAN_HH_

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommPose3d.hh>

#include "Queue.hh"

class LaserScan {
private:
	CommBasicObjects::CommMobileLaserScan commMobileLaserScan;
	gazebo::transport::NodePtr node;
	gazebo::transport::SubscriberPtr subscriber;
	SmartACE::SmartMutex mutex;
	SmartACE::SmartSemaphore sem;
    void OnMsg(ConstLaserScanStampedPtr &_msg);
    Queue* queue;

public:
	LaserScan(Queue* queue);
	virtual ~LaserScan();
	void init();
};

#endif /* LASERSCAN_HH_ */
