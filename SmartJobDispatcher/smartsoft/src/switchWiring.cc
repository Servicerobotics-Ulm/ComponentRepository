//--------------------------------------------------------------------------
//  BSD 3-Clause License
//
//  Copyright (C) Servicerobotics Ulm
//  University of Applied Sciences Ulm
//  Prittwitzstr. 10
//  89075 Ulm
//  Germany
//  All rights reserved.
//
//  Author: Matthias Lutz
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//* Neither the name of the copyright holder nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//--------------------------------------------------------------------------


#include "switchWiring.hh"
#include "SmartJobDispatcher.hh"
#include "lispSeparator.h"
#include <stdio.h>

//#include "CommSymbolicPlannerObjects/mystrsep.hh"

std::string switchWiring(char* inString, unsigned function)
{
	std::string outString;
//	static char outString[LISP_STRING];

	switch (function)
	{
	// wiring connect
	case 1:
	{
		Smart::StatusCode status;
		char *input  = (char *)NULL;
		char *pointer = (char *)NULL;
		char *param1  = (char *)NULL;

		pointer = input = strdup(inString);

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
				outString = "(ok ())";
				break;
			case Smart::SMART_CANCELLED:
				outString = "(error (smart cancelled))";
				break;
			case Smart::SMART_UNKNOWNCOMPONENT:
				outString = "(error (smart unknown component))";
				break;
			case Smart::SMART_UNKNOWNPORT:
				outString = "(error (smart unknown port))";
				break;
			case Smart::SMART_SERVICEUNAVAILABLE:
				outString = "(error (smart service unavailable))";
				break;
			case Smart::SMART_INCOMPATIBLESERVICE:
				outString = "(error (smart incompatible service))";
				break;
			case Smart::SMART_ERROR_COMMUNICATION:
				outString = "(error (smart error communication))";
				break;
			case Smart::SMART_ERROR:
				outString = "(error (smart error))";
				break;
			default:
				outString = "(error (unknown error))";
				break;
		}
		break;
	} // case 1: wiring connect


	// wiring disconnect
	case 2:
	{
		Smart::StatusCode status;
		char *input  = (char *)NULL;
		char *pointer = (char *)NULL;
		char *param1  = (char *)NULL;

		pointer = input = strdup(inString);

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
				outString = "(ok ())";
				break;
			case Smart::SMART_CANCELLED:
				outString = "(error (smart cancelled))";
				break;
			case Smart::SMART_UNKNOWNCOMPONENT:
				outString = "(error (smart unknown component))";
				break;
			case Smart::SMART_UNKNOWNPORT:
				outString = "(error (smart unknown port))";
				break;
			case Smart::SMART_ERROR_COMMUNICATION:
				outString = "(error (smart error))";
				break;
			case Smart::SMART_ERROR:
				outString = "(error (smart error))";
				break;
			default:
			 	outString = "(error (unknown error))";
				break;
		}
		break;
	} // case 2: wiring disconnect


	// --------------------------------------------------------
	// default method
	// --------------------------------------------------------
	default:
	{
		outString = "(error (unknown method number))";
		return outString;
		break;
	} // default
	} // switch

	// return
	return outString;
}
