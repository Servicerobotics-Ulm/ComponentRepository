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


#include "GenerateOut.hh"
#include "ComponentTrafficLightTest.hh"

#include <iostream>

GenerateOut::GenerateOut(SmartACE::SmartComponent *comp) 
:	GenerateOutCore(comp)
{
	std::cout << "constructor GenerateOut\n";
}
GenerateOut::~GenerateOut() 
{
	std::cout << "destructor GenerateOut\n";
}



int GenerateOut::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}




int GenerateOut::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel
	
	// to get the incoming data, use this methods:
	//Smart::StatusCode status;

	std::cout << "Hello from GenerateOut " << std::endl;



	  Smart::StatusCode status;
	  DomainHMI::CommTrafficLight tl;

	  std::cout << "Generate Red" << std::endl;
	  tl.setRed(true);
	  tl.setYellow(false);
	  tl.setGreen(false);
	  status = COMP->trafficLightServiceOut->send(tl);
	  if (status != Smart::SMART_OK) {
		  std::cout << "Error: " << status << std::endl;
	  }
	  sleep(1);

	  std::cout << "Generate Yellow" << std::endl;
	  tl.setRed(false);
	  tl.setYellow(true);
	  tl.setGreen(false);
	  status = COMP->trafficLightServiceOut->send(tl);
	  if (status != Smart::SMART_OK) {
		  std::cout << "Error: " << status << std::endl;
	  }
	  sleep(1);

	  std::cout << "Generate Green" << std::endl;
	  tl.setRed(false);
	  tl.setYellow(false);
	  tl.setGreen(true);
	  status = COMP->trafficLightServiceOut->send(tl);
	  if (status != Smart::SMART_OK) {
		  std::cout << "Error: " << status << std::endl;
	  }
	  sleep(1);

	  std::cout << "Generate Yellow" << std::endl;
	  tl.setRed(false);
	  tl.setYellow(true);
	  tl.setGreen(false);
	  status = COMP->trafficLightServiceOut->send(tl);
	  if (status != Smart::SMART_OK) {
		  std::cout << "Error: " << status << std::endl;
	  }
	  sleep(1);

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int GenerateOut::on_exit()
{

	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
