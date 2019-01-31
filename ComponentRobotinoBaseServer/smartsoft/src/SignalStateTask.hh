// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------

// --------------------------------------------------------------------------
//
//  Copyright (C) 2014 Matthias Lutz
//
//        lutz@hs-ulm.de
//
//        Servicerobotics Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This file is part of the "SmartSoft Communication Classes".
//  It provides basic standardized data types for communication between
//  different components in the mobile robotics context. These classes
//  are designed to be used in conjunction with the SmartSoft Communication
//  Library.
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
// --------------------------------------------------------------------------
#ifndef _SIGNALSTATETAST_HH
#define _SIGNALSTATETAST_HH

#include "CommBasicObjects/enumBaseTagType.hh"
#include "aceSmartSoft.hh"
#include "SignalStateTaskCore.hh"

class SignalStateTask  : public SignalStateTaskCore
{
private:
public:
	SignalStateTask(SmartACE::SmartComponent *comp);
	virtual ~SignalStateTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
	CommBasicObjects::BaseTagType signalState;
	CommBasicObjects::BaseTagType newSignalState;

	bool localizationState;

	bool blink;
	bool blinkLoc;
	mutable SmartACE::SmartMutex lockSignalState; // why ? Gautam


	void setSignalState(CommBasicObjects::BaseTagType state);
	CommBasicObjects::BaseTagType getSignalState() const;

	void setLocalizationState(bool state);
	bool getLocalizationState()const;

	//virtual int on_entry();
	//virtual int on_execute();
	//virtual int on_exit();
};

#endif
