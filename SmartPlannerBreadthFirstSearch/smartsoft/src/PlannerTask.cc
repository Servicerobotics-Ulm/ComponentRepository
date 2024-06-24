//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs.
// If you want the toolchain to re-generate this file, please
// delete it before running the code generator.
//--------------------------------------------------------------------------

// --------------------------------------------------------------------------
//
//  Copyright (C) 2009 Christian Schlegel, Andreas Steck, Matthias Lutz
//
//        schlegel@hs-ulm.de
//        steck@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This file is part of the "SmartSoft smartPlannerBreadthFirstSearch component".
//  It provides planning services based on grid maps.
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
// --------------------------------------------------------------------------

#include "PlannerTask.hh"
#include "SmartPlannerBreadthFirstSearch.hh"

#include <iostream>

PlannerTask::PlannerTask(SmartACE::SmartComponent *comp)
:	PlannerTaskCore(comp)
{
	std::cout << "constructor PlannerTask\n";
}
PlannerTask::~PlannerTask()
{
	std::cout << "destructor PlannerTask\n";
}


//void PlannerTask::on_CurMapClient(const CommNavigationObjects::CommGridMap &input)
//{
//	// upcall triggered from InputPort CurMapClient
//	// - use a local mutex here, because this upcal is called asynchroneously from outside of this task
//	// - do not use longer blocking calls here since this upcall blocks the InputPort CurMapClient
//	// - if you need to implement a long-running procedure, do so within the on_execute() method and in
//	//   there, use the method curMapClientGetUpdate(input) to get a copy of the input object
//}
//void PlannerTask::on_BaseStateClient(const CommBasicObjects::CommBaseState &input)
//{
//	// upcall triggered from InputPort BaseStateClient
//	// - use a local mutex here, because this upcal is called asynchroneously from outside of this task
//	// - do not use longer blocking calls here since this upcall blocks the InputPort BaseStateClient
//	// - if you need to implement a long-running procedure, do so within the on_execute() method and in
//	//   there, use the method baseStateClientGetUpdate(input) to get a copy of the input object
//}

int PlannerTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further

	plannerMap = new Smart::PlannerMapClass(0, // axsizecells,
			0, // aysizecells,
			0, // acellsizemm,
			0, // axoffsetmm,
			0); // ayoffsetmm);

	return 0;
}
int PlannerTask::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel

	int robotX,robotY,robotZ,robotA;    // current position of the robot

	double x1g,y1g,x2g,y2g;
	double xNextGoal, yNextGoal;        // intermediate goal point for the next step
	double xGoal, yGoal;                // final goal point which has to be reached
	int    type;
	Smart::StatusCode status;
	int status2;
	double timeDiff;


	bool           aIsValid;
	int            aXOffsetCells,aYOffsetCells,aXOffsetMM,aYOffsetMM;
	unsigned int   aXSizeCells,aYSizeCells,aXSizeMM,aYSizeMM,aCellSizeMM;
	unsigned int   aMapId;
	struct timeval aTime;

//	// wait for the next timed trigger
//	COMP->PlannerTriggerLock.acquire();
//	COMP->PlannerTrigger.wait();
//	COMP->PlannerTriggerLock.release();

	// ------------------------------------------------------------------
	// access the global configuration information
	// ------------------------------------------------------------------
//		COMP->PlannerGlobalLock.acquire();
//		COMP->localState = COMP->globalState;
//		COMP->PlannerGlobalLock.release();
	ParameterStateStruct localState = COMP->getGlobalState();


	// ------------------------------------------------------------------
	// make sure the fifo for breadth first search is empty
	// ------------------------------------------------------------------
	status2        	= fifoFree();
	generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_NO_ERROR;

	// ------------------------------------------------------------------
	// get next current map and put it into the planner map class
	// ------------------------------------------------------------------
	//std::cout << "before acquiring curMapClient - line " << __LINE__ << std::endl;
	//do {
	   status = COMP->curMapClient->getUpdate(currentGridMap);
	  if(status != Smart::SMART_OK) {
		 std::cout << "no map received with getUpdate(currentGridMap); return code: " << status << std::endl;
		 usleep(50000);
			 return 0;
		  }
	  //	  		std::cout << "no map received with getUpdate(currentGridMap); return code: " << status << std::endl;
	  //		  	usleep(50000);
	  //}
	//} while(status != Smart::SMART_OK);
	//std::cout << "after acquiring curMapClient - line " << __LINE__ << std::endl;

	currentGridMap.get_parameter(aMapId,aIsValid,aTime,aXOffsetMM,aYOffsetMM,aXOffsetCells,aYOffsetCells,aCellSizeMM,aXSizeMM,aYSizeMM,aXSizeCells,aYSizeCells);


	//if(aIsValid!=true){
	//  generalstatus = PLANNER_INVALID_MAP;
	//}
	//else
	if(localState.getCommNavigationObjects().getPlannerParams().getID().getId() != aMapId)
	{
		// map id not vaild
		generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_WRONG_MAPID;
		std::cout << "PLANNER: map id not correct, do nothing (planner/map :" << localState.getCommNavigationObjects().getPlannerParams().getID().getId()
			<< " " << aMapId << "\n";
	}
	else
	{
		status2 = plannerMap->setMapperMap(currentGridMap);
		if (status2 != 0)
		{
			std::cout << "PLANNER: fatal error, can't allocate required map memory" << std::endl;
			exit(-1);
		}

		// ----------------------------------------------------------------
		// convert from mapper to planner cell values
		// ----------------------------------------------------------------
		status2 = plannerMap->convert();
		// ----------------------------------------------------------------
		// mark goal region in the map
		// ----------------------------------------------------------------
		COMP->PlannerGoalLock.acquire();
		goalFifoPtr = COMP->goalFifoHead;
		status2 = goalFifoNext(COMP->goalFifoHead, COMP->goalFifoTail, &goalFifoPtr,&type,&x1g,&y1g,&x2g,&y2g);
		COMP->PlannerGoalLock.release();

		if (status2 != 0)
		{
			// --------------------------------------------------------------
			// there are no goal lines available
			// --------------------------------------------------------------
			generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_NO_GOAL_AVAILABLE;
		}
		else
		{
			generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_GOAL_NOT_MARKED;
			while (status2==0)
			{
				switch (type)
				{
				case PLANNER_LINE:
				{
					status2 = plannerMap->bresenham(x1g,y1g,x2g,y2g,MODE_GOAL);
					break;
				}

				case PLANNER_CIRCLE:
				{
					status2 = plannerMap->circle(x1g,y1g,x2g,MODE_GOAL);
					break;
				}

				default:
				{
					status2 = 1;
					break;
				}
				}

				if (status2 == 0)
				{
					generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_GOAL_OK;
				}
				COMP->PlannerGoalLock.acquire();
				status2 = goalFifoNext( COMP->goalFifoHead, COMP->goalFifoTail, &goalFifoPtr,
						&type, &x1g, &y1g, &x2g, &y2g);
				COMP->PlannerGoalLock.release();
			}//while(status==0)
			if (generalstatus != CommNavigationObjects::PlannerEventType::PLANNER_GOAL_OK)
			{
				// ------------------------------------------------------------
				// had some problems to mark the goal region
				// generalstatus is PLANNER_GOAL_NOT_MARKED
				// ------------------------------------------------------------
				std::cout << "PLANNER: fatal error, problems marking the goal cells" << std::endl;
			}
			else
			{
				// ------------------------------------------------------------
				// successfully marked the goal cells in the map
				// ------------------------------------------------------------

				// ------------------------------------------------------------
				// get roboter position from base server
				// ------------------------------------------------------------
				status = this->baseStateClientGetUpdate(base_state);
				if(status!=Smart::SMART_OK)
				{
					std::cerr << "WARNING: failed to get current base state (" << status << "), pushing invalid scan" << std::endl;
				}

				robotX = base_state.get_base_position().get_x();
				robotY = base_state.get_base_position().get_y();
				robotA = base_state.get_base_position().get_base_azimuth();


				// ROBOTERPOSITION
				status2 = plannerMap->markCurrentPosition(robotX, robotY);
				if (status2 != 0)
				{
					switch (status2)
					{
					case 1:
					{
						generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_START_OCCUPIED_OBSTACLE;
						std::cout << "PLANNER: fatal error, current robot position already occupied by an obstacle" << std::endl;
						break;
					}

					case 2:
					{
						generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_START_OCCUPIED_GOAL;
						std::cout << "PLANNER: fatal error, current robot position already marked as goal" << std::endl;
						break;
					}

					default:
					{
						generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_UNKNOWN_ERROR;
						std::cout << "PLANNER: fatal error, unknown error code" << std::endl;
						break;
					}
					}
				}
				else
				{
					// ------------------------------------------------------------
					// no error occured during making the current position
					// ------------------------------------------------------------
					status2 = plannerMap->waveFrontFlood();
					if (status2 != 0)
					{
						// ------------------------------------------------------------
						// was not able to find a path, no valid goal point
						// ------------------------------------------------------------
						generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_NO_PATH;
						std::cout << "PLANNER: fatal error, no path found since no valid goal point" << std::endl;

						plannerGoal.set_goal(0.0,0.0,0.0,0.0,0.0,0.0,localState.getCommNavigationObjects().getPlannerParams().getID().getId(),1);
						COMP->plannerGoalServer->put(plannerGoal);
					}
					else
					{
						// ------------------------------------------------------------
						// found path, calculate next way point
						// ------------------------------------------------------------
						generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_PATH_FOUND;
						status2 = plannerMap->waveFrontOptimizeFirstSegment(robotX,robotY,xNextGoal,yNextGoal);
						status2 = plannerMap->waveFrontFindGoal(robotX,robotY,xGoal,yGoal);
						std::cout << "PLANNER Robot position pos ("<<robotX << ","<<robotY<<")"<<std::endl;
						std::cout << "PLANNER  next goal (" << xNextGoal << "," << yNextGoal << ") goal (" << xGoal << "," << yGoal << ")" << std::endl;

						plannerGoal.set_goal(xNextGoal,yNextGoal,0.0,xGoal,yGoal,0.0,localState.getCommNavigationObjects().getPlannerParams().getID().getId(),0);
						COMP->plannerGoalServer->put(plannerGoal);

						generalstatus = CommNavigationObjects::PlannerEventType::PLANNER_NO_ERROR;
					}
				} //else if (status2 != 0)
			} //else if (generalstatus != PLANNER_GOAL_OK)
		} //else if (status2 != 0)
	} // else  if(localState.id != aMapId)

   // ------------------------------------------------------------
   // report on status
   // ------------------------------------------------------------
   switch (generalstatus)
   {
   case CommNavigationObjects::PlannerEventType::PLANNER_NO_ERROR:
   {
	   std::cout << "PLANNER: ok" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_UNKNOWN_ERROR:
   {
	   std::cout << "PLANNER: unknown error" << std::endl;
	   break;
   }
   case CommNavigationObjects::PlannerEventType::PLANNER_NO_GOAL_AVAILABLE:
   {
	   std::cout << "PLANNER: no goal available" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_GOAL_OK:
   {
	   std::cout << "PLANNER: goal ok" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_GOAL_NOT_MARKED:
   {
	   std::cout << "PLANNER: goal not marked" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_START_OCCUPIED_OBSTACLE:
   {
	   std::cout << "PLANNER: start cell occupied by an obstacle" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_START_OCCUPIED_GOAL:
   {
	   std::cout << "PLANNER: start cell occupied by goal" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_NO_PATH:
   {
	   std::cout << "PLANNER: no path" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_PATH_FOUND:
   {
	   std::cout << "PLANNER: path found" << std::endl;
	   break;
   }

   case CommNavigationObjects::PlannerEventType::PLANNER_WRONG_MAPID:
   {
	   std::cout << "PLANNER: wrong map id" << std::endl;
	   break;
   }

   default:
   {
	   std::cout << "PLANNER: default (unknown error type)" << std::endl;
	   exit(-1);
	   break;
   }
   } //switch (generalstatus)

   // -------------------------------------------------------------
   // now check whether generalstatus should be reported via event
   // -------------------------------------------------------------

   if (generalstatus == CommNavigationObjects::PlannerEventType::PLANNER_NO_PATH)
   {
	   //
	   // special treatment for "no path event" since this fires only
	   // when it lasts for at least X seconds (value from ini file)
	   //
	   if (COMP->noPathFlag == 0)
	   {
		   // first occurence of noPath situation
		   COMP->noPathFlag = 1;
		   ::gettimeofday(&(COMP->beginNoPathTime),0);
	   }
	   else
	   {
		   // not the first occurence of noPath situation
		   ::gettimeofday(&(COMP->currentTime),0);
		   timeDiff  = (double)(COMP->currentTime.tv_usec - COMP->beginNoPathTime.tv_usec)/1000000.0;
		   timeDiff += (double)(COMP->currentTime.tv_sec  - COMP->beginNoPathTime.tv_sec);

		   if (timeDiff > COMP->getGlobalState().getSettings().getNo_path_event_timeout())
		   {
			   COMP->noPathFlag = 0;
			   // ---------------------------------------------------
			   // put event into object
			   // ---------------------------------------------------
			   CommNavigationObjects::PlannerEventState plannerEventState;
			   plannerEventState.set(CommNavigationObjects::PlannerEventType::PLANNER_NO_PATH);
			   COMP->plannerEventServer->put(plannerEventState);
		   }
	   }
   }
   else
   {
	   CommNavigationObjects::PlannerEventState plannerEventState;
	   plannerEventState.set(generalstatus);
	   COMP->plannerEventServer->put(plannerEventState);
   }


   #if DEBUGSAVE
	 sprintf(filename,"plamap%d.xpm",filecounter++);
	 map->saveXPM(filename);
   #endif

   #if DEBUGMODE
	 ::gettimeofday(&timeEnd,(struct timezone*)NULL);
	 subTime(timeEnd,timeStart);

	 cout << "Needed path planning time: " << timeEnd.tv_sec << " " << timeEnd.tv_usec << "\n";

	 flag = 0;

	 do {
	   cout << "One cycle in debug mode\n";
	   cout << "-1- continue\n";
	   cin >> input;
	   switch(input) {
		 case 1:
		   flag = 1;
		   break;
		 default:
		   cout << "Wrong input\n";
		   break;
	   }
	 } while(flag==0);

	 /////////////////////////////////////////////////////////
	 // Save planner map for debug purposes
	 /////////////////////////////////////////////////////////
	 char                 filename2[255];
	 sprintf(filename2,"test-planner-map-%d.xpm",2);
	 std::ofstream myFile2 ( filename2 , std::ios::out | std::ios::binary);
	 plannerMap->save_xpm( myFile2 );
	 myFile2.close();
	 ////////////////////////////////////////////////////////
   #endif
	 //publish map containing the wavefront for visualization purpose
	 COMP->currGridMapPushServiceOut->put(*plannerMap);
	return 0;
}
int PlannerTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
