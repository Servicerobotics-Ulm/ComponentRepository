//--------------------------------------------------------------------------
//
//  Copyright (C) 2019 Oleksandr Shlapak
//
//      Servicerobotic Ulm
//      Ulm University of Applied Sciences
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
//--------------------------------------------------------------------------


#include "TrafficLightServiceInHandler.hh"

#include <wiringPi.h>
#include <iostream>

TrafficLightServiceInHandler::TrafficLightServiceInHandler(Smart::InputSubject<DomainHMI::CommTrafficLight> *subject, const int &prescaleFactor)
:	TrafficLightServiceInHandlerCore(subject, prescaleFactor)
{
	std::cout << "constructor TrafficLightServiceInHandler\n";
}
TrafficLightServiceInHandler::~TrafficLightServiceInHandler() 
{
	std::cout << "destructor TrafficLightServiceInHandler\n";
}

void TrafficLightServiceInHandler::on_TrafficLightServiceIn(const DomainHMI::CommTrafficLight &input)
{
	// implement business logic here
	// (do not use blocking calls here, otherwise this might block the InputPort TrafficLightServiceIn)
    if (input.getGreen()){
		std::cout << "Green! "  << std::endl;
		digitalWrite(1, HIGH);
    } else{
    	digitalWrite(1, LOW);
    }

    if(input.getYellow()){
		std::cout << "Yellow! "  << std::endl;
		digitalWrite(16, HIGH);
    } else {
    	digitalWrite(16, LOW);
    }

    if(input.getRed()){
       std::cout << "Red!" << std::endl;
	   digitalWrite(15, HIGH);
    } else {
    	digitalWrite(15, LOW);
    }
}
