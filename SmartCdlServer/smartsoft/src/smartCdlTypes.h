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

#ifndef _SMARTCDLTYPES_H
#define _SMARTCDLTYPES_H

#ifdef __cplusplus
extern "C" {
#endif


struct cdl_line_struct {
  double x1;
  double y1;
  double x2;
  double y2;
};

struct cdl_polygon_struct {
  //struct cdl_line_struct line[CDL_MAX_LINES];
  struct cdl_line_struct* line;
  int number_of_segments;
};

struct cdl_index_struct {
  double my;
  int    trans_dir;
  int    rot_dir;
};

#ifdef __cplusplus
}
#endif

#endif

