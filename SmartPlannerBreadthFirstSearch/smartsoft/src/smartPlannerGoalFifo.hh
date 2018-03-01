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

#ifndef _PLANNERGOALFIFO_H
#define _PLANNERGOALFIFO_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct GoalFifoStruct *GoalFifoStructPtr;

typedef struct GoalFifoStruct **GoalFifoStructPtrPtr;

typedef struct GoalFifoStruct {
  GoalFifoStructPtr previous;
  GoalFifoStructPtr next;
  double            x1;
  double            y1;
  double            x2;
  double            y2;
  int               type;
} GoalFifoType;


int goalFifoNext(GoalFifoStructPtr head_dest,GoalFifoStructPtr tail_dest,
                 GoalFifoStructPtrPtr ptr,
                 int *entry,double *x1,double *y1,double *x2,double *y2);
int goalFifoRead(GoalFifoStructPtr head_dest,GoalFifoStructPtr tail_dest,
                 int *entry,double *x1,double *y1,double *x2,double *y2);
int goalFifoInit(GoalFifoStructPtrPtr head_dest,GoalFifoStructPtrPtr tail_dest);
int goalFifoWrite(GoalFifoStructPtr head_dest,GoalFifoStructPtr tail_dest,
                  int entry,double x1,double y1,double x2,double y2);
int goalFifoFree(GoalFifoStructPtr head_dest,GoalFifoStructPtr tail_dest);


#ifdef __cplusplus
}
#endif

#endif

