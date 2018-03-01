//--------------------------------------------------------------------------
//
//  Copyright (C)  2017 Timo Blender
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
#include "Queue.hh"

#include <iostream>

Queue::Queue()
: bufferMutex()
, bufferSema(0)
{
}
Queue::~Queue()
{
}

void Queue::addEntry (CommBasicObjects::CommMobileLaserScan commLaserScan){
	SmartACE::SmartGuard guard(bufferMutex);

	buffer.push_back(commLaserScan);

	bufferSema.release();
}

CommBasicObjects::CommMobileLaserScan Queue::removeEntry(){

	bufferSema.acquire();

	SmartACE::SmartGuard guard(bufferMutex);

	CommBasicObjects::CommMobileLaserScan obj = buffer.front();
	buffer.pop_front();

	return obj;
}
