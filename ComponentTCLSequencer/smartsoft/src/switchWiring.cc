//--------------------------------------------------------------------------
//
//  Copyright (C) 	2009-2011 Andreas Steck
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

#include "switchWiring.hh"
#include "ComponentTCLSequencer.hh"
#include <stdio.h>

std::string switchWiring(const std::string& moduleInst, const std::string& compnameTypename, const std::string& compname, const std::string& service, const std::string& inString)
{
	std::ostringstream outString;

	// wiring connect
	if(service == "connect")
	{
		Smart::StatusCode status;
		char *input  = (char *)NULL;
		char *pointer = (char *)NULL;
		char *param1  = (char *)NULL;

		pointer = input = strdup(inString.c_str());

		// slaveComp
		do
		{
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));
		std::string slaveComp(param1);

		// slavePort
		do
		{
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));
		std::string slavePort(param1);

		// serverComp
		do
		{
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));
		std::string serverComp(param1);

		// serverService
		do
		{
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));
		std::string serverService(param1);

		std::cout << "wiringMaster->connect(" << slaveComp << ", " << slavePort << ", " << serverComp << ", " << serverService << ")\n";
		status = COMP->wiringMaster->connect(slaveComp, slavePort, serverComp, serverService);
		switch (status)
		{
			case Smart::SMART_OK:
				outString<< "(ok ())";
				break;
			case Smart::SMART_CANCELLED:
				outString<< "(error (smart cancelled))";
				break;
			case Smart::SMART_UNKNOWNCOMPONENT:
				outString<< "(error (smart unknown component))";
				break;
			case Smart::SMART_UNKNOWNPORT:
				outString<< "(error (smart unknown port))";
				break;
			case Smart::SMART_SERVICEUNAVAILABLE:
				outString<< "(error (smart service unavailable))";
				break;
			case Smart::SMART_INCOMPATIBLESERVICE:
				outString<< "(error (smart incompatible service))";
				break;
			case Smart::SMART_ERROR_COMMUNICATION:
				outString<< "(error (smart error communication))";
				break;
			case Smart::SMART_ERROR:
				outString<< "(error (smart error))";
				break;
			default:
				outString<< "(error (unknown error))";
				break;
		}
	}
	// wiring disconnect
	else if(service == "disconnect")
	{
		Smart::StatusCode status;
		char *input  = (char *)NULL;
		char *pointer = (char *)NULL;
		char *param1  = (char *)NULL;

		pointer = input = strdup(inString.c_str());

		// slaveComp
		do
		{
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));
		std::string slaveComp(param1);

		// slavePort
		do
		{
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));
		std::string slavePort(param1);

		std::cout << "wiringMaster->disconnect(" << slaveComp << ", " << slavePort << ")\n";
		status = COMP->wiringMaster->disconnect(slaveComp, slavePort);
		switch (status)
		{
			case Smart::SMART_OK:
				outString<< "(ok ())";
				break;
			case Smart::SMART_CANCELLED:
				outString<< "(error (smart cancelled))";
				break;
			case Smart::SMART_UNKNOWNCOMPONENT:
				outString<< "(error (smart unknown component))";
				break;
			case Smart::SMART_UNKNOWNPORT:
				outString<< "(error (smart unknown port))";
				break;
			case Smart::SMART_ERROR_COMMUNICATION:
				outString<< "(error (smart error communication))";
				break;
			case Smart::SMART_ERROR:
				outString<< "(error (smart error))";
				break;
			default:
				outString<< "(error (unknown error))";
				break;
		}
	} //  wiring disconnect


	// --------------------------------------------------------
	// default method
	// --------------------------------------------------------
	else
	{
		return "(error (unknown method number))";
	} // default

	// return
	return outString.str();
}
