// --------------------------------------------------------------------------
//
//  Copyright (C) 2008 Christian Schlegel
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




// smartCdlLookup

#define DEBUG_VW_WINDOW       0   // dump vw window with parameters into file
#define DEBUG_ONLY_ONE_CYCLE  0   // stop after one cycle
#define DEBUG_INTERACTIVE     0   // ask for parameters before calculation

// smartCdlServer

#define DEBUG_SIMULATED_LASERSCAN 0
// This flag allows to use the specified laser scan instead of the
// real laserscan. The module is not connected to the smartLaserServer.

