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


#include "switchSymbolicPlanner.hh"
#include "SmartJobDispatcher.hh"
#include "lispSeparator.h"
#include <stdio.h>


#include <DomainSymbolicPlanner/CommSymbolicPlannerPlan.hh>
#include <DomainSymbolicPlanner/CommSymbolicPlannerRequest.hh>


// ---------------------------------------------------------
//
//
// interact with symbolic planner module
//
//
// ---------------------------------------------------------

// (ok ())
// (ok (<id>))      with event id
// (<answer>)       with answer from command with status
//
// (error (unknown error))
// (error (wrong parameter))
// (error (unknown event mode))
// (error (wrong or no event id))
// (error (unknown method number))
// (error (smart fatal error))
// (error (smart communication error))
// (error (smart communication canceled))
//
// and all the values returned from symbolic planner interface

std::string switchSymbolicPlanner(char* inString, unsigned function)
{
	std::string outString;
	outString = "(error (unknown error))";


  switch(function)
  {
  case 1:
  {
	  DomainSymbolicPlanner::CommSymbolicPlannerRequest request;
	  DomainSymbolicPlanner::CommSymbolicPlannerPlan answer;

	  std::string input(inString);
	  std::string help;
	  Smart::StatusCode status;

	  if(request.set(input) == 0)
	  {
		  //
		  // everything is ok
		  //
		  status = COMP->symbolicPannerServiceReq->query(request, answer);
		  switch (status)
		  {
			case Smart::SMART_OK:
				answer.get_plan(help);
				outString = "(ok (" + help + "))";
//				sprintf(outString,"(ok (%s))", help.c_str());
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
		  } // switch(status)
	  }
	  else
	  {
		  outString = "(error (wrong parameter))";
	  }
	  break;
  } // case 1
  case 2:
  {
	  // base state
	  outString = COMP->setState(COMP->stateMaster,COMP->connections.symbolicPannerServiceReq.serverName, inString);
	  break;
  }


  case 8:
	{
		// waitForLifeCycleState

		outString = COMP->waitForLifeCycleState(COMP->stateMaster,COMP->connections.symbolicPannerServiceReq.serverName, inString);
		break;
	}


  default:
  {
	  outString = "(error (unknown method number))";
	  std::cout << "lisp interface (switch symbolic planner default) out:" << outString << ":\n";
	  break;
	}
  }

  return outString;
}
