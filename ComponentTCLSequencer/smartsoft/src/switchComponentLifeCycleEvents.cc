//--------------------------------------------------------------------------
//
//  Copyright (C) 	1997-2000 Christian Schlegel
// 					2009-2012 Andreas Steck
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

#include "switchComponentLifeCycleEvents.hh"
#include "ComponentTCLSequencer.hh"
#include <stdio.h>
#include "lispSeparator.h"
#include "eventInterface.hh"

// ---------------------------------------------------------
//
//
// interact with components own events (e.g. shutdown)
//
//
// ---------------------------------------------------------


std::string switchComponentLifeCycleEvents(const std::string& moduleInst, const std::string& compnameTypename, const std::string& compname, const std::string& service, const std::string& inString)
{
	std::ostringstream outString;
	outString << "(error (unknown error))";

	std::cout<<__FUNCTION__<<" service: "<<service<<std::endl;

	if(service == "SHUTDOWNEVENT-activate")
	{
		int evtId = 0;
		outString.str("");
		outString<< "(ok ("<<evtId<<"))";
	}
	else if (service == "SHUTDOWNEVENT-deactivate")
		//event deactivate
	{
		Smart::StatusCode status;
		char *input  = (char *)NULL;
		char *pointer = (char *)NULL;
		char *param1  = (char *)NULL;

		pointer = input = strdup(inString.c_str());
		do
		{
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));

		std::string str(param1);
		// remove " "
		str = str.substr(1, str.length()-2);
		int id = atoi( param1 );


		outString.str("");
		outString<< "(ok ("<<id<<"))";

	} // case 3: event deactivate
	else
	{
		outString.str("(error (unknown method number))");
	}

	return outString.str();
}
