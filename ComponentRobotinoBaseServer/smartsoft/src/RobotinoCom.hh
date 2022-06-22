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

#ifndef ROBOTINOCOM_H_
#define ROBOTINOCOM_H_

#include "rec/robotino/api2/all.h"

class RobotinoCom: public rec::robotino::api2::Com
{
public:
	/**
	 * @brief Default constructor
	 */
	RobotinoCom() :
// api2/Com connectToServer(true) wirf Exception "The connection has been refused."
// Benutzen Sie den Konstruktor
//
// Com(const char* name, bool multiThreadedSerialization, bool localIPCEnabled)
//
// also
//
// Com("mycom",true,false)
//
// Dann werden nur TCP sockets benutzt, so dass auch der normale Nutzer Zugriff hat. Ansonsten werden local sockets (files) benutzt und das braucht root Rechte.

//		Com("SmartRobotinoBaseServer")
		Com("SmartRobotinoBaseServer", true, false)
	{
	}

	/**
	 * @brief Default destructor
	 */
	~RobotinoCom()
	{
	}
};

#endif /* ROBOTINOCOM_H_ */
