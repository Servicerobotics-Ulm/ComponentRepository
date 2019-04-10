//--------------------------------------------------------------------------
//
//  Copyright (C) 	2011 Andreas Steck
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

#include "switchTimer.hh"
#include "ComponentTCLSequencer.hh"
#include <stdio.h>
#include <time.h>
//#include "CommBasicObjects/CommTimeStamp.hh"

std::string switchTimer(const std::string& moduleInst, const std::string& compnameTypename, const std::string& compname, const std::string& service, const std::string& inString)
{

	std::ostringstream outString;
	outString << "(error (unknown error))";

	// relative timer event activate
	if(service == "relative-activate")
	{
		int id;
		char *running;
		char *token;
		char *mode;

		// copy string
		token = running = strdupa (inString.c_str());

		// extract mode single || continuous
		do
		{
			mode = strsep (&running, LISP_SEPARATOR);
		} while ((mode != NULL) && (strlen(mode)==0));

		int sec, msec;

		// extract sec
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		sec = atoi(token);

		// extract msec
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		msec = atoi(token);

		std::cout << "[c++] relative time: " << sec << " : " << msec << std::endl;
		id = COMP->handlerTimerService.scheduleTimer("relative", ACE_Time_Value( sec, msec*1000));
		outString.str("");
		outString << "(ok ("<<id<<"))";
	} // case 1: relative timer event activate

	// absolute timer event activate
	else if(service == "absolute-activate")
	{
		int id;
		char *running;
		char *token;
		char *mode;

		// copy string
		token = running = strdupa (inString.c_str());

		// extract mode single || continuous
		do
		{
			mode = strsep (&running, LISP_SEPARATOR);
		} while ((mode != NULL) && (strlen(mode)==0));

		int sec, msec;
		time_t t;
		//CommBasicObjects::CommTimeStamp currentTime;
		//struct timeval tv;
		struct tm *timeinfo;
		timeinfo = localtime ( &t );
		timeinfo->tm_isdst = 1;

		// extract year
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		timeinfo->tm_year = atoi(token) - 1900;

		// extract month
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		timeinfo->tm_mon = atoi(token) - 1;

		// extract day
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		timeinfo->tm_mday = atoi(token);

		// extract hour
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		timeinfo->tm_hour = atoi(token);

		// extract min
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		timeinfo->tm_min = atoi(token);

		// extract sec
		do
		{
			token = strsep (&running, LISP_SEPARATOR);
		} while ((token != NULL) && (strlen(token)==0));
		timeinfo->tm_sec = atoi(token);

		t = mktime ( timeinfo );
		  ACE_Time_Value t_currentTime = ACE_OS::gettimeofday();
		//currentTime.set_now();

		sec = t - t_currentTime.sec();
		msec = 0;

		std::cout << "[c++] absolute time: " << sec << " : " << msec << std::endl;
		id = COMP->handlerTimerService.scheduleTimer("absolute", ACE_Time_Value( sec, msec*1000));

		outString.str("");
		outString << "(ok ("<<id<<"))";

	} // case 2: absolute timer event activate

	// timer event deactivate
	else if(service == "relative-deactivate" || service == "absolute-deactivate")
	{
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
		if(COMP->handlerTimerService.cancelTimer(id) == 0)
		{
			outString << "(ok ("<< id <<"))";
		}
		else
		{
			outString << "(error (smart wrongid))";
		}

	} // case 3/4: relative timer event deactivate
	else
	{
		outString.str("(error (unknown method number))");
	} // default

	return outString.str();
}
