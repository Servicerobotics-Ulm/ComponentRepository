//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
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

//------------------------------------------------------------------------
//
//  Copyright (C) 2017 Matthias Lutz
//
//        lutz@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartURServer component".
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
//--------------------------------------------------------------------------

#include "IoQueryServerHandler.hh"
#include "SmartURServerLegacy.hh"

#include "UniversalRobotic.hh"

IoQueryServerHandler::IoQueryServerHandler(Smart::IQueryServerPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues>* server)
:	IoQueryServerHandlerCore(server)
{
	
}

IoQueryServerHandler::~IoQueryServerHandler()
{
	
}


void IoQueryServerHandler::handleQuery(const Smart::QueryIdPtr &id, const CommBasicObjects::CommIOValues& request)
{
	std::cout << "IO request : " << request <<std::endl;
	std::cout << "(IoQueryServerHandler) request: Digital In: " <<request.getDigitalInputValuesSize()
			<<" Analog In: " << request.getAnalogInputValuesSize()<<std::endl;
	if(request.getDigitalOutputValuesSize()>0){
		for(unsigned int i=0;i<request.getDigitalOutputValuesSize();i++){
			CommBasicObjectsIDL::CommDigitalOutputRequest value = request.getDigitalOutputValuesElemAtPos(i);
			std::cout<<__FUNCTION__<<": Digital Output number: "<<value.outputNumber<<" value: "<<value.outputValue<<std::endl;
			UR->setDigitalOutputValue(value.outputNumber, (bool)value.outputValue);
		}
	}

	if(request.getAnalogOutputValuesSize()>0){
		for(unsigned int i=0;i<request.getAnalogOutputValuesSize();i++){
			CommBasicObjectsIDL::CommAnalogOutputRequest value = request.getAnalogOutputValuesElemAtPos(i);
			UR->setDigitalOutputValue(value.outputNumber, value.outputValue);
		}
	}


	CommBasicObjects::CommIOValues answer;

	std::vector<bool> digitalIn = UR->getDigitalInputValues();
	std::vector<float> analogIn = UR->getAnalogInputValues();

	answer.resizeDigitalInputValues(digitalIn.size());
	for(unsigned int i=0;i<digitalIn.size();i++){
		answer.setDigitalInputValuesElemAtPos(i,digitalIn[i]);
		std::cout << "Reading from UR : Digital In i = " << i << " value = " <<digitalIn[i] <<std::endl;
	}

	for(unsigned int i=0;i<analogIn.size();i++){
		answer.getAnalogInputValuesRef().push_back(analogIn[i]);
		std::cout << "Reading from UR : Analog In i = " << i << " value = " <<analogIn[i] <<std::endl;
	}

	//answer.set(result);	
	this->server->answer(id, answer);
}
