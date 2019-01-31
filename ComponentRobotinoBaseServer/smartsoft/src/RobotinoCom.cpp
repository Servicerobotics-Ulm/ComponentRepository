//--------------------------------------------------------------------------
//
//  Copyright (C) 2012 Christian Schlegel, Matthias Hörger
//
//        schlegel@hs-ulm.de
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

#include "RobotinoCom.hh"
#include <iostream>

void RobotinoCom::errorEvent( const char* errorString )
{
	std::cerr << "Robotino Connection Error: " << errorString << std::endl;
}

void RobotinoCom::connectedEvent()
{
	std::cout << "Connected to Robotino." << std::endl;
}

void RobotinoCom::connectionClosedEvent()
{
	std::cout << "Connection to Robotino closed." << std::endl;
}

void RobotinoCom::logEvent( const char* message, int level )
{
	std::cout << message << std::endl;
}

//	void RobotinoCom::pingEvent(float timeMs)
//	{
//		std::cout << "Ping: " << timeMs << "ms" << std::endl;
//	}


