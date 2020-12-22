//--------------------------------------------------------------------------
//
//  Copyright (C) 	1997-2000 Christian Schlegel
// 					2009/2010 Andreas Steck
//
//      steck@hs-ulm.de
//		schlegel@hs-ulm.de
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
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
//--------------------------------------------------------------------------

#include "switchFetchEvents.hh"
#include "ComponentTCLSequencer.hh"

#include <stdio.h>
#include "lispSeparator.h"
#include "eventInterface.hh"


std::string switchFetchEvents(const std::string& moduleInst, const std::string& componentName, const std::string& componentInstanceName, const std::string& service, const std::string& inString)
{
//	static char outString[LISP_STRING];

//	std::cout<<"switchFetchEvents - compInstName: "<<componentInstanceName<<" inString: "<<inString<<" service: "<<service<<std::endl;

	std::ostringstream outString;
	outString << "(error (unknown error))";

	// get all event messages since last query
	if(service == "GET")
	{
		std::string eventList;
		int    out;

		out = COMP->eventInterface->get(eventList);
		if (out == 0)
		{
//			// everything is ok
//			if ((eventList.length()+strlen("(ok ())")) > LISP_STRING)
//			{
//				// too many events already in return list
//				outString.str("(error (too many event messages))");
//				std::cout<<"THIS SHOULD NEVER HAPPEN! Buffersize: "<<LISP_STRING<<" Reuqired size: "<<eventList.length()+strlen("(ok ())")<<std::endl;
//				std::cout<<"MESSAGE |"<<eventList<<"|END"<<std::endl;
//			}
//			else
//			{
//				std::cout << "switchFetchEvents: " << eventList << std::endl;
				outString.str("");
				outString<<"(ok ("<< eventList <<"))";
//			}
		}
		else
		{
			// error occured
			outString.str("(error (unknown error))");
		}

	} else {
		// default method
		outString.str("(error (unknown method number))");
	}
	return outString.str();
}
