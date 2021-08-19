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

#include "switchKnowledgeBase.hh"
#include "SmartJobDispatcher.hh"

#include <CommBasicObjects/CommTCLMessage.hh>

#include "lispSeparator.h"
#include <stdio.h>


// ---------------------------------------------------------
//
//
// interact with base module
//
//
// ---------------------------------------------------------

std::string switchKnowledgeBase(char* inString, unsigned function)
{
  std::string outString;

  outString = "(error (unknown error))";

  switch(function)
  {
  case 1:
  {
	  CommBasicObjects::CommKBRequest request;
	  CommBasicObjects::CommKBResponse answer;

	  //std::string input(inString);
	  //std::string help;
	  Smart::StatusCode status;
	  request.setRequest(inString);

//      std::cout<<"switchKnowledgeBase request: "<<request.getLisp()<<std::endl;
	  status = COMP->commKBQueryReq->query(request, answer);
//	  std::cout<<"switchKnowledgeBase answer: "<<answer.getLisp()<<std::endl;


	  switch (status)
	  {
	  case Smart::SMART_OK:
	  {

		  outString  = "(ok " + answer.getResponse() + ")";
//		  sprintf(outString,"(ok %s)", answer.getLisp().c_str());
		  break;
	  }
	  case Smart::SMART_DISCONNECTED:
		  outString = ("(error (smart disconnected))");
		  break;
	  case Smart::SMART_ERROR_COMMUNICATION:
		  outString = ( "(error (smart communication error))");
		  break;
	  case Smart::SMART_ERROR:
		  outString = ( "(error (smart error))");
		  break;
	  default:
		  outString = ( "(error (unknown error))");
		  break;
	  } // switch(status)

	  break;
  } // case 1

  case 2:
  {
	  // base state
	  outString = COMP->setState(COMP->stateMaster,COMP->connections.commKBQueryReq.serverName, inString);
	  break;
  }

  case 8:
  {
	// waitForLifeCycleState

	  outString = COMP->waitForLifeCycleState(COMP->stateMaster,COMP->connections.commKBQueryReq.serverName, inString);
	  std::cout<<__FILE__<<" waitForLifeCycleState res: "<<outString<<std::endl;
	  break;
  }

  default:
  {
	  outString = "(error (unknown method number))";
	  std::cout << "lisp interface (switch base default) out:" << outString << ":\n";
	  break;
	}
  }

  return outString;
}
