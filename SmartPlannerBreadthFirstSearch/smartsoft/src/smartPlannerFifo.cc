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



#include "smartPlannerFifo.hh"

#ifdef __cplusplus
extern "C" {
#endif

// global variables

static int fifoReadIndex = 0;
static int fifoWriteIndex = 0;
static int fifoNrOfEntries = 0;
static FifoArrayType fifoArray[PLANNER_FIFO_SIZE];


// functions to manage the fifo structure for path planning

int fifoInit(void)
{
  fifoReadIndex = 0;
  fifoWriteIndex = 0;
  fifoNrOfEntries = 0;

  return 0;
}

int fifoWrite(int x,int y)
{
//printf("in smartPlannerFifo.c fifoWrite -- line %d\n", __LINE__);
  if (fifoNrOfEntries >= PLANNER_FIFO_SIZE) return 1;
//printf("in smartPlannerFifo.c fifoWrite -- line %d\n", __LINE__);
  fifoNrOfEntries++;
//printf("in smartPlannerFifo.c fifoWrite -- line %d\n", __LINE__);
  fifoArray[fifoWriteIndex].x = x;
//printf("in smartPlannerFifo.c fifoWrite -- line %d\n", __LINE__);
  fifoArray[fifoWriteIndex].y = y;
//printf("in smartPlannerFifo.c fifoWrite -- line %d\n", __LINE__);
  fifoWriteIndex++;
//printf("in smartPlannerFifo.c fifoWrite -- line %d\n", __LINE__);
  if (fifoWriteIndex >= PLANNER_FIFO_SIZE) fifoWriteIndex=0;
//printf("in smartPlannerFifo.c fifoWrite -- line %d\n", __LINE__);

  return 0;
}

int fifoRead(int *x,int *y)
{
  if (fifoNrOfEntries <= 0) return 1;
  fifoNrOfEntries--;
  *x = fifoArray[fifoReadIndex].x;
  *y = fifoArray[fifoReadIndex].y;
  fifoReadIndex++;
  if (fifoReadIndex >= PLANNER_FIFO_SIZE) fifoReadIndex=0;

  return 0;
}

int fifoFree(void)
{
  fifoReadIndex = 0;
  fifoWriteIndex = 0;
  fifoNrOfEntries = 0;

  return 0;
}

#ifdef __cplusplus
}
#endif

