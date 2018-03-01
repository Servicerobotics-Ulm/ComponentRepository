// --------------------------------------------------------------------------
//
//  Copyright (C) 1998 Christian Schlegel
//
//        schlegel@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This file is part of the "SmartSoft CDL component".
//  It provides navigation services based on the CDL
//  Curvature Distance Lookup approach.
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

#ifndef _SMARTCDLDEFINE_H
#define _SMARTCDLDEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------------------------------------------
//
// ------------------------------------------------------------------

#define CDL_FREE_TURN_LEFT   1
#define CDL_FREE_TURN_RIGHT  2
#define CDL_FREE_NO_TURN     3
#define CDL_FREE_BOTH        4

typedef enum {
  CDL_STRATEGY_1,            // cdl lookup strategy 1
  CDL_STRATEGY_2,            // cdl lookup strategy 2
  CDL_STRATEGY_3,            // cdl lookup strategy 3
  CDL_STRATEGY_4,            // cdl lookup strategy 4
  CDL_STRATEGY_5,            // cdl lookup strategy 5
  CDL_STRATEGY_6,            // cdl lookup strategy 6
  CDL_STRATEGY_7,            // cdl lookup strategy 7
  CDL_STRATEGY_10,           // cdl lookup strategy 10
  CDL_STRATEGY_11,           // cdl lookup strategy 11
  CDL_STRATEGY_12,           // cdl lookup strategy 12
  CDL_STRATEGY_13,           // cdl lookup strategy 13
  CDL_STRATEGY_14,			 // cdl lookup strategy for coverage planner motion
  CDL_STRATEGY_15,			 // cdl lookup strategy for path based navigation
  CDL_STRATEGY_16			 // cdl lookup strategy for path based navigation
} CdlStrategyType;

typedef enum {
  CDL_EVAL_STANDARD,
  CDL_EVAL_PASSING,
  CDL_EVAL_STOPPING,
  CDL_EVAL_APPROACH_EXACT,
  CDL_EVAL_APPROACH_OBJECT
} CdlEvalFunctionType;

#ifdef __cplusplus
}
#endif

#endif

