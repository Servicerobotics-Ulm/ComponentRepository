//--------------------------------------------------------------------------
//
//  Copyright (C) 2012 Matthias Hörger
//
//        hoerger@hs-ulm.de
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



#include <RobotinoOdom.hh>
#include "aceSmartSoft.hh"
#include "ComponentRobotinoBaseServer.hh"

/**
	* Called when new odomtery data is available.
	* @param x Global x position of Robotino in m.
	* @param y Global y position of Robotino in m.
	* @param phi Global orientation of Robotino in rad.
	* @param vx x velocity of Robotino.
	* @param vy y velocity of Robotino.
	* @param omega Angular velocity of Robotino.
	* @param sequence The sequence number of the current readings. When starting your application the sequence number is 0. The sequence number
	*         is increased 1 each time the odometry is updated.
	* @remark This function is called from the thread in which Com::processEvents() is called.
	* @see Com::processEvents
	*/
void RobotinoOdom::readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence ){

//	std::cout<<"New Odom Reading: x: "<<x<<"y: "<<y<<" phi: "<<phi<<" vx: "<<vx<<" vy: "<<vy<<" omega: "<<omega<< "seg: "<<sequence<<std::endl;

//	std::cout<<"New vx: "<<vx<<" vy: "<<vy<<" omega: "<<omega<<std::endl;

	SmartACE::SmartGuard guard(COMP->CurrentOdomLock); //TODO
		COMP->robot->update(x,y,phi,vx,vy,omega,sequence);
	guard.release();
}
