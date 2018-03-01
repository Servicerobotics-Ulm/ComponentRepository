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


#ifndef _SMART_PLANNERMAPTYPES_H
#define _SMART_PLANNERMAPTYPES_H

#include <sys/time.h>

typedef struct {
  int                size;
  unsigned char      *cell;
} PlannerMapArrayType;

typedef struct {
  int                 status;         // 0 everything is OK
  int                 xOffsetMM;      //
  int                 yOffsetMM;      //
  int                 cellSizeMM;     //
  int                 xSizeCells;     //
  int                 ySizeCells;     //
  struct timeval      time;           //
  PlannerMapArrayType map;
} PlannerMapType;

#endif

