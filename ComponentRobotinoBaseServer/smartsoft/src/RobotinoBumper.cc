//--------------------------------------------------------------------------
//
//  Copyright (C) 2015 Matthias Lutz
//
//        lutz@hs-ulm.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
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


#include "RobotinoBumper.hh"
#include "CommBasicObjects/CommBumperEventState.hh"
#include "ComponentRobotinoBaseServer.hh"


void RobotinoBumper::setTimoutConfiguration(int sec, int msec){

	SmartACE::SmartGuard guard(lock);
	timeoutSec = sec;
	timeoutMsec = msec;
}

bool RobotinoBumper::getState(){
	SmartACE::SmartGuard guard(lock);
	return lastState;
}
void RobotinoBumper::bumperEvent( bool hasContact )
{

	SmartACE::SmartGuard guard(lock);

	    //state changed
		if(hasContact != lastState){

			if(hasContact){
				//we need some timeout here. If station is invisible for more than x seconds, we abort this task
				//COMP->ini.laser.noStationVisibleTimeout
				if(bumperTimerId == -1){
					std::cout << "[RobotinoBumper::bumperEvent()]  scheduleTimer relative time: " << timeoutSec << " : " << timeoutMsec << std::endl;
					std::chrono::seconds sec(timeoutSec);
					std::chrono::milliseconds msec(timeoutMsec);
					bumperTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,NULL, sec+msec);
				} else {
					std::cout<<__FUNCTION__<<":"<<__LINE__<<"ERROR: this should never had happened!"<<std::endl;
				}

			} else {
				//abort timer
				if(bumperTimerId != -1)
				{
					std::cout << "[RobotinoBumper::bumperEvent()]  cancelTimer!"<< std::endl;
					COMP->getComponentImpl()->getTimerManager()->cancelTimer(bumperTimerId);
					bumperTimerId = -1;
				} else {
					std::cout<<__FUNCTION__<<":"<<__LINE__<<" ERROR: this should never had happened!"<<std::endl;
				}

				//send free state immediately
				CommBasicObjects::CommBumperEventState state;
				state.setNewState(CommBasicObjects::BumperEventType::BUMPER_NOT_PRESSED);
				COMP->bumperEventServiceOut->put(state);
			}

			lastState = hasContact;
		} else {
            //state not changed this is only used for output!
			if (hasContact){
				std::cout << "Bumper has contact" << std::endl;
				if(bumperTimerId == -1){
					std::cout << "[RobotinoBumper::bumperEvent()]  scheduleTimer relative time: " << timeoutSec << " : " << timeoutMsec << std::endl;
					std::chrono::seconds sec(timeoutSec);
					std::chrono::milliseconds msec(timeoutMsec);
					bumperTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,NULL, sec+msec);
				}
			}
		}

}

//void RobotinoBumper::timerExpired(const ACE_Time_Value & absolute_time, const void * arg)
void RobotinoBumper::timerExpired(const Smart::TimePoint &abs_time, const void * arg)
{
			CommBasicObjects::CommBumperEventState state;
			{
			SmartACE::SmartGuard guard(lock);
			std::cout<<"[RobotinoBumper:timerExpired] Bumper contact timeout!"<<std::endl;

			COMP->getComponentImpl()->getTimerManager()->cancelTimer(bumperTimerId);
			bumperTimerId = -1;

			state.setNewState(CommBasicObjects::BumperEventType::BUMPER_PRESSED);
			}
			COMP->bumperEventServiceOut->put(state);
}

void RobotinoBumper::timerCancelled(){};
void RobotinoBumper::timerDeleted(const void * arg){};




