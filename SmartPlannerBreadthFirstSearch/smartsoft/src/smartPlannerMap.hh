// --------------------------------------------------------------------------
//
//  Copyright (C) 2009 Christian Schlegel, Matthias Lutz, Andreas Steck
//
//        schlegel@hs-ulm.de
//        steck@hs-ulm.de
//        lutz@hs-ulm.de
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


#ifndef _SMARTPLANNERMAP_HH
#define _SMARTPLANNERMAP_HH

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>

#include "CommNavigationObjects/CommGridMap.hh"

#include "smartPlannerFifo.hh"
#include "smartPlannerGoalFifo.hh"
#include "smartPlannerTypes.hh"
#include "smartPlannerMapTypes.hh"
#include "CommNavigationObjects/CommPlannerEventParameter.hh"
#include "CommNavigationObjects/CommPlannerEventResult.hh"
#include "CommNavigationObjects/PlannerEventState.hh"

#define MAPPER_CELL_FREE          0
#define MAPPER_CELL_OBSTACLE      128
#define MAPPER_CELL_GROWING       129
#define MAPPER_CELL_UNDELETABLE   130


namespace Smart
{

class PlannerMapClass : public CommNavigationObjects::CommGridMap
{
protected:
  //PlannerMapType data;

  struct {
    int x;
    int y;
    int direction;
  } waveFront[8];

  int index(double world,double offset);
  // Converts world coordinates into indices relativ to map offset.

  int internalBresenham(int x1,int y1,int x2,int y2);
  // Bresenham algorithm to draw lines. This method is used internally to
  // test, whether all cells on the line are empty. In this case 0 is
  // returned, else 1. All coordinates are in map indices.

public:
  // = Initialization and termination

  PlannerMapClass();
  // Default constructor creates map of size (0,0) at position (0,0) and
  // cellsize 0.

  PlannerMapClass(int,int,int,int,int);
  // Initialize the actual map of size (x,y) [number of cells] with
  // cellsize [mm] at position (xo,yo) [mm,mm]

/* // use auto generated
   //   if there are really no problems - please remove the
  PlannerMapClass(const PlannerMapClass& map);
  // Copy constructor

  const PlannerMapClass& operator= (const PlannerMapClass&);
  // Assignment operator.
*/


  virtual ~PlannerMapClass();
  // Implicitly destroy the map class.

  // = User interface functions.
  // All methods return status information. If status is 0, everything
  // is OK, else an error occured.

  int setMapperMap(CommNavigationObjects::CommGridMap&);
  //int setMapperMap(MapperActClass&);
  // Function used to copy the received actual map into the planner's map
  // class. The size of the map is automatically adjusted.

  int markCurrentPosition(double x,double y);
  // Mark this position as the current robot position used for path planning.
  // Returns different error codes:
  // 0  everything is OK
  // 1  actual position already marked with an obstacle
  // 2  actual position already marked as goal cell
  // 3  unknown cell value at current position

  int convert();
  // Used to convert map from mapper to cell values used within the planner.

  int debugEmptyMap();
  // Clear map and draw border. Use planner cell values. This function is
  // used for debugging and test purposes.

  int saveAscii(const char *filename);
  // Save actual map (used for debugging purposes).

  void save_xpm( std::ostream &os ) const;
  // Save actual map (used for debugging purposes).

  // = Path planning functions.

  int bresenham(double x1,double y1,double x2,double y2,int write);
  // Bresenham algorithm to draw lines. All coordinates are in world
  // coordinates and are converted to map indices inside this function.
  //
  // Parameter: write  MODE_OBSTACLE occupy cell with obstacle without check
  //                                 if cell is free. Always return 0.
  //                   MODE_GOAL     if cell is free mark cell as goal cell
  //                                 and write cell coordinates on fifo. If
  //                                 at least one goal cell was empty return
  //                                 0 else return 1.
  //                   MODE_TEST     Return 0 if all cells on the line are empty,
  //                                 else return 1. Doesn't modify the grid.

  int circle(double x,double y,double r,int write);
  // Circle algorithm to draw goal circle. All coordinates are in world
  // coordinates and are converted to map indices inside this function.
  //
  // Parameter: write  MODE_OBSTACLE
  //                   MODE_GOAL
  //                   MODE_TEST
  // see bresenham algorithm

  int waveFrontFlood(void);
  // Apply the wave front expansion algorithm to find a path. If a path
  // could be found 0 is returned, else 1 is returned.

  int waveFrontOptimizeFirstSegment(double x,double y,double &xgoal,double &ygoal);
  // Optimize the beginning of the found path and select the next intermediate
  // way point. All coordinates are in world coordinates.

  int waveFrontFindGoal(double x,double y,double &xgoal,double &ygoal);
  // Detect the goal point out of a goal region which will be reached probably.
  // All coordinates are in world coordinates.

};

} //namespace Smart
#endif

