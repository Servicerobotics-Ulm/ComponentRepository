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


#include "switchHighLevelCommand.hh"
#include "SmartJobDispatcher.hh"
#include "lispSeparator.h"

#include <stdio.h>

std::string switchHighLevelCommand(char* inString, unsigned function)
{
	  std::string outString;
	  outString = "(error (unknown error))";

	switch(function)
	{

		case 1:
		{
			outString = COMP->queryParam(COMP->paramMaster,COMP->connections.jobSourceEventClient.serverName, inString);
			break;
		} // case 1: symbol param
//
//		// tcl event activate
//		case 2:
//		{
//			CHS::StatusCode status;
//			CommBasicObjects::CommTCLMessage tclParam;
//			CHS::EventId tclId;
//			char *input  = (char *)NULL;
//			char *pointer = (char *)NULL;
//			char *param1  = (char *)NULL;
//			char *eventParam  = (char *)NULL;
//
//			pointer = input = strdup(inString);
//			do
//			{
//				param1 = strsep(&input,LISP_SEPARATOR);
//			} while ((param1 != NULL) && (strlen(param1)==0));
//
//			do
//			{
//				eventParam = strsep(&input,LISP_SEPARATOR);
//			} while ((eventParam != NULL) && (strlen(eventParam)==0));
//			tclParam.setLisp(eventParam);
//			std::cout<<"Register EventParam: "<<eventParam<<std::endl;
//
//			// CONTINOUS
//			if( strcasecmp(param1, "CONTINUOUS") == 0 )
//			{
//				status = COMP->highLevelCommandEventClient->activate(CHS::continuous, tclParam, tclId);
//				switch(status)
//				{
//					case CHS::SMART_OK:
//						sprintf(outString,"(ok (%d))", (int)tclId);
//						break;
//					case CHS::SMART_DISCONNECTED:
//						sprintf(outString,"(error (smart disconnected))");
//						break;
//					case CHS::SMART_ERROR_COMMUNICATION:
//						sprintf(outString,"(error (smart communication error))");
//						break;
//					case CHS::SMART_ERROR:
//			            sprintf(outString,"(error (unknown error))");
//			            break;
//					default:
//			            sprintf(outString,"(error (unknown error))");
//			            break;
//				} // switch
//			} // CONTINOUS
//
//			// SINGLE
//			else if( strcasecmp(param1, "SINGLE") == 0 )
//			{
//				status = COMP->highLevelCommandEventClient->activate(CHS::single, tclParam, tclId);
//				switch(status)
//				{
//					case CHS::SMART_OK:
//						sprintf(outString,"(ok (%d))", (int)tclId);
//						break;
//					case CHS::SMART_DISCONNECTED:
//						sprintf(outString,"(error (smart disconnected))");
//						break;
//					case CHS::SMART_ERROR_COMMUNICATION:
//						sprintf(outString,"(error (smart communication error))");
//						break;
//					case CHS::SMART_ERROR:
//			            sprintf(outString,"(error (unknown error))");
//			            break;
//					default:
//			            sprintf(outString,"(error (unknown error))");
//			            break;
//				} // switch
//			}
//			break;
//		} // case 3: event activate
//
//		// tcl event deactivate
//		case 3:
//		{
//			CHS::StatusCode status;
//			char *input  = (char *)NULL;
//			char *pointer = (char *)NULL;
//			char *param1  = (char *)NULL;
//
//			pointer = input = strdup(inString);
//			do
//			{
//				param1 = strsep(&input,LISP_SEPARATOR);
//			} while ((param1 != NULL) && (strlen(param1)==0));
//
//			std::string str(param1);
//			// remove " "
//			str = str.substr(1, str.length()-2);
//			int id = atoi( param1 );
//
//			status = COMP->highLevelCommandEventClient->deactivate(id);
//			switch(status)
//			{
//			case CHS::SMART_OK:
//				sprintf(outString,"(ok (%d))", (int)id);
//				break;
//			case CHS::SMART_WRONGID:
//				sprintf(outString,"(error (smart wrongid))");
//				break;
//			case CHS::SMART_ERROR_COMMUNICATION:
//				sprintf(outString,"(error (smart communication error))");
//				break;
//			case CHS::SMART_ERROR:
//				sprintf(outString,"(error (unknown error))");
//				break;
//			default:
//				sprintf(outString,"(error (unknown error))");
//				break;
//			} // switch
//
//			break;
//		} // case 3: event deactivate

		case 4 :
		{
			CommBasicObjects::CommTaskMessage cp;

			std::string input(inString);
			Smart::StatusCode status;
			cp.setMsg(input);

			// everything is ok
			status = COMP->taskResultOut->put(cp);
			switch (status)
			{
					case Smart::SMART_OK:
						outString = "(ok ())";
							break;
					case Smart::SMART_DISCONNECTED:
						outString = "(error (smart disconnected))";
							break;
					case Smart::SMART_ERROR_COMMUNICATION:
						outString = "(error (smart communication error))";
							break;
					case Smart::SMART_ERROR:
						outString = "(error (smart error))";
							break;
					default:
						outString = "(error (unknown error))";
							break;
			}

			break;
		}

	  case 2:
	  {
		  // base state
		  outString = COMP->setState(COMP->stateMaster,COMP->connections.jobSourceEventClient.serverName, inString);
		  break;
	  }

		case 8:
		{
			// waitForLifeCycleState

			outString = COMP->waitForLifeCycleState(COMP->stateMaster,COMP->connections.jobSourceEventClient.serverName, inString);
			break;
		}



	    default:
	    {
	    	 outString = "(error (unknown method number))";
	    } // default

	} // switch
	return outString;
}
