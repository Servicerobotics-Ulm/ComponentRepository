// --------------------------------------------------------------------------
//
//  Copyright (C) 2009 Christian Schlegel
//
//        schlegel@hs-ulm.de
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


#ifndef _PLANNERTYPES_H
#define _PLANNERTYPES_H

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------
// different modes of the bresenham algorithm
// ---------------------------------------------------------------------
#define MODE_OBSTACLE    0
#define MODE_GOAL        1
#define MODE_TEST        2

// ---------------------------------------------------------------------
// Different values the grid cells can be ocuupied with
// ---------------------------------------------------------------------
// don't change the following values, they are used as indices in arrays
// values 8-15 are used to mark path found by the path planner
#define PLANNER_NORTH     0
#define PLANNER_WEST      1
#define PLANNER_SOUTH     2
#define PLANNER_EAST      3     
#define PLANNER_FREE      16
#define PLANNER_START     17
#define PLANNER_GOAL      18
#define PLANNER_OBSTACLE  19
#define PLANNER_GROWING   20

// ---------------------------------------------------------------------
// different typedefs used by the planner algorithm
// ---------------------------------------------------------------------
#define PLANNER_LINE      1
#define PLANNER_CIRCLE    2

#ifdef __cplusplus
}
#endif

#endif

