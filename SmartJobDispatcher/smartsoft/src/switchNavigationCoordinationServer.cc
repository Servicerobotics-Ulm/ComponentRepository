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


#include "switchNavigationCoordinationServer.hh"
#include "SmartJobDispatcher.hh"
#include "lispSeparator.h"

#include <stdio.h>


std::string switchNavigationCoordinationServer(char* inString, unsigned function)
{
	std::string outString;
	outString = "(error (unknown error))";

	switch(function)
	{
	//  param
	case 1:
	{
		outString = COMP->queryParam(COMP->paramMaster,COMP->getGlobalState().getModuleConnection().getNavigationCoordinationServer(), inString);
		break;
	} // case 1:  param

	case 2:
	{
		// state
		outString = COMP->setState(COMP->stateMaster,COMP->getGlobalState().getModuleConnection().getNavigationCoordinationServer(), inString);
		break;
	}

	case 3:
	{
		// waitForLifeCycleState
		outString = COMP->waitForLifeCycleState(COMP->stateMaster,COMP->getGlobalState().getModuleConnection().getNavigationCoordinationServer(), inString);
		break;
	}

	//topologymap
	case 4:
	{
		  CommBasicObjects::CommVoid request;
		  CommNavigationObjects::CommNavigationTopologyMap answer;

		  Smart::StatusCode status;

	          std::cout << "vor status = COMP->navigationTopologyMapQueryClient->query(request,answer);\n";
		  status = COMP->navigationTopologyMapQueryReq->query(request,answer);
	          std::cout << "nach status = COMP->navigationTopologyMapQueryClient->query(request,answer);\n";
		  switch (status)
		  {
		  case Smart::SMART_OK:
		  {
			  unsigned int numerParts = answer.getVerticesSize();
			  std::stringstream streamParts;
			  streamParts<<"(";
			  for(unsigned int i=0;i<numerParts; i++){
				  CommNavigationObjects::CommNavigationTopologyPart part = answer.getVerticesElemAtPos(i);

				  streamParts<< "(" << part.getId() << " " << part.getName() << " " << part.getType().to_string();
				  streamParts<< "(";
				  unsigned int contained_locations_size = part.getContained_location_idsSize();
				  for(unsigned int j=0;j<contained_locations_size; j++){
					  streamParts<< part.getContained_location_idsElemAtPos(i) << " ";
				  }
				  streamParts<< "))";


			  }
			  streamParts<<")";

			  unsigned int numerConnections = answer.getEdgesSize();
			  std::stringstream streamConnections;
			  streamConnections<<"(";
			  for(unsigned int i=0;i<numerConnections; i++){
				  CommNavigationObjects::CommNavigationTopologyConnection connection = answer.getEdgesElemAtPos(i);
				  streamConnections<< "(" << connection.getType().to_string() << " " << connection.getFromPart() << " " << connection.getToPart();
				  streamConnections<< " " << connection.getLocationId()<< ")";
			  }
			  streamConnections<<")";

			  std::stringstream ss;
			  ss <<"(ok ("<< streamParts.str() << streamConnections.str() <<"))";
			  outString = ss.str();
			  break;
		  }
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
		  } // switch(status)
		  break;
	}

	default:
	{
		outString = "(error (unknown method number))";
	} // default

	} // switch
	return outString;
}
