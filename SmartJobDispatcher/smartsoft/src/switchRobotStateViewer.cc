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


#include "switchRobotStateViewer.hh"
#include "SmartJobDispatcher.hh"

#include "lispSeparator.h"
#include <stdio.h>


std::string switchRobotStateViewer(char* inString, unsigned function)
{
	std::string outString;
	outString = "(error (unknown error))";

	switch(function)
	{
	// param
	case 1:
	{
		outString = COMP->queryParam(COMP->paramMaster,COMP->getGlobalState().getModuleConnection().getRobotStateViewer(), inString);
		break;
	} // case 1: param

	case 2:
	{
		// base state
		outString = COMP->setState(COMP->stateMaster,COMP->getGlobalState().getModuleConnection().getRobotStateViewer(), inString);
		break;
	}

	case 3:
	{
		// waitForLifeCycleState
		outString = COMP->waitForLifeCycleState(COMP->stateMaster,COMP->getGlobalState().getModuleConnection().getRobotStateViewer(), inString);
		break;
	}


	default:
	{
		outString = "(error (unknown method number))";
	} // default

	} // switch
	return outString;
}
