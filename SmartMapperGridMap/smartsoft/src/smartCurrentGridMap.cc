// --------------------------------------------------------------------------
//
//  Copyright (C) 2008 Christian Schlegel, Andreas Steck
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
//  This file is part of the "SmartSoft Mapper/GridMapper component".
//  It provides mapping services based on grid maps. The current map
//  represents the latest snapshot of the local surrounding based on
//  laserscans. The current map can be preoccupied by the longterm map.
//  The longterm map is a simple occupancy grid map.
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

#include "smartCurrentGridMap.hh"

#include <assert.h>

using namespace Smart;


SmartCurrentGridMap::SmartCurrentGridMap() : CommGridMap()
{
  // constructor
  initializeGrowing();
}


SmartCurrentGridMap::SmartCurrentGridMap(int x,int y,int size,int growing)
{
  // constructor
  growingType = growing;
  growingSize = 0;

  idl_CommGridMap.id = 0;
  idl_CommGridMap.is_valid = false;
  idl_CommGridMap.time.sec = 0;
  idl_CommGridMap.time.usec = 0;
  idl_CommGridMap.xOffsetMM = 0;
  idl_CommGridMap.yOffsetMM = 0;
  idl_CommGridMap.xOffsetCells = 0;
  idl_CommGridMap.yOffsetCells = 0;
  idl_CommGridMap.cellSizeMM = size;
  idl_CommGridMap.xSizeMM = x;
  idl_CommGridMap.ySizeMM = y;
  idl_CommGridMap.xSizeCells = (int)ceil((double)x/(double)size);
  idl_CommGridMap.ySizeCells = (int)ceil((double)y/(double)size);
  idl_CommGridMap.size = idl_CommGridMap.xSizeCells*idl_CommGridMap.ySizeCells;

  idl_CommGridMap.cell.resize(idl_CommGridMap.size);
  initializeGrowing();
}


SmartCurrentGridMap::SmartCurrentGridMap(int x,int y,int px,int py,int size,int growing,int id)
{
  // constructor
  growingType = growing;
  growingSize = 0;

  idl_CommGridMap.id = id;
  idl_CommGridMap.is_valid = false;
  idl_CommGridMap.time.sec = 0;
  idl_CommGridMap.time.usec = 0;
  idl_CommGridMap.xOffsetMM = px;
  idl_CommGridMap.yOffsetMM = py;
  idl_CommGridMap.xOffsetCells = (int)floor((double)px/(double)size);
  idl_CommGridMap.yOffsetCells = (int)floor((double)py/(double)size);
  idl_CommGridMap.cellSizeMM = size;
  idl_CommGridMap.xSizeMM = x;
  idl_CommGridMap.ySizeMM = y;
  idl_CommGridMap.xSizeCells = (int)ceil((double)x/(double)size);
  idl_CommGridMap.ySizeCells = (int)ceil((double)y/(double)size);
  idl_CommGridMap.size = idl_CommGridMap.xSizeCells*idl_CommGridMap.ySizeCells;

  idl_CommGridMap.cell.resize(idl_CommGridMap.size);
  initializeGrowing();

}

/* use auto generated - for more information check the header file

SmartCurrentGridMap const &SmartCurrentGridMap::operator=(SmartCurrentGridMap const &other)
{
  // assignment operator
  if (this != &other)
  {
    _gridmap = other._gridmap;
    _gridmap.cell.length(_gridmap.size);

    for ( unsigned int i=0; i < _gridmap.size; i++)
    {
      _gridmap.cell[i] = other._gridmap.cell[i];
    }

  }
  return (*this);
}


SmartCurrentGridMap::SmartCurrentGridMap(SmartCurrentGridMap const &other)
{
  // copy constructor
  _gridmap = other._gridmap;
  _gridmap.cell.length(_gridmap.size);

  for ( unsigned int i=0; i < _gridmap.size; i++)
  {
    _gridmap.cell[i] = other._gridmap.cell[i];
  }
}
*/

SmartCurrentGridMap::~SmartCurrentGridMap()
{
  // destructor
}


int SmartCurrentGridMap::clearMap()
{
  unsigned int x,y;

  for (x=0; x < idl_CommGridMap.xSizeCells; x++)
  {
    for (y=0; y < idl_CommGridMap.ySizeCells; y++)
    {
    	idl_CommGridMap.cell[x+y * idl_CommGridMap.xSizeCells] = MAPPER_FREE;
    }
  }

  return drawBorder();
}


int SmartCurrentGridMap::update( CommBasicObjects::CommMobileLaserScan const &scan )
{
  int xa,ya,xs,ys;
  //double a;
  double x, y, z; // x1, y1;
  int status;
  unsigned int i;
  //double laserPolarAngleRad; // = laserPolarAngle*M_PI/180.0;
  //double laserPolarDistance; // = 1000.0*lp[j];
  CommBasicObjects::CommBaseState base_state = scan.get_base_state();
  for (i=0; i < scan.get_scan_size(); i++)
  {
    //laserPolarAngleRad = i/2.0 * M_PI / 180.0;
    // a =  scan.get_base_state().get_base_position().get_base_alpha(); // angle of robot in real world
    //laserPolarDistance = scan.get_scan_distance(i);

    // draw line from scanner pos to measured laser point
    xa = (int)(floor(scan.get_scanner_x() /(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.xOffsetCells);
    ya = (int)(floor(scan.get_scanner_y() /(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.yOffsetCells);

    //scan.get_scan_cartesian_point_world( (unsigned int) i, x, y );
    //scan.get_scan_cartesian_point_scanner( (unsigned int) i, x1, y1 );
    scan.get_scan_cartesian_3dpoint_world(i, x, y, z );

    xs = (int)(floor(x /(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.xOffsetCells);
    ys = (int)(floor(y /(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.yOffsetCells);

    status = bresenham(xa,ya,xs,ys);  // draw line
    status = obstacleGrowing(xs,ys);  // do obstacle growing
  }
  return status;
}


int SmartCurrentGridMap::setLtmOccupation( int threshold, Smart::SmartLtmGridMap &ltm )
{
  //std::cout << "SmartCurrentGridMap::setLtmOccupation - not implemented yet !!\n";
  int status;

  status = ltm.getPartialMap(idl_CommGridMap.xOffsetMM,
		  idl_CommGridMap.yOffsetMM,
		  idl_CommGridMap.xSizeMM,
		  idl_CommGridMap.ySizeMM,
                             threshold,
                             idl_CommGridMap.cell);
  //return status;
  return drawBorder();
}




int SmartCurrentGridMap::convertObstacleGrowing()
{
  unsigned int counter;
  int status;

  for (counter=0; counter < (idl_CommGridMap.xSizeCells*idl_CommGridMap.ySizeCells); counter++)
  {
    if (idl_CommGridMap.cell[counter] == MAPPER_GROWING)
    {
    	idl_CommGridMap.cell[counter] = MAPPER_OBSTACLE;
    }
  }
  status = drawBorder();

  return status;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRIVATE MEMBERS

int SmartCurrentGridMap::drawBorder(void)
{
  unsigned int x,y;

  for ( x=0; x < idl_CommGridMap.xSizeCells; x++)
  {
	  idl_CommGridMap.cell[x + 0* idl_CommGridMap.xSizeCells] = MAPPER_UNDELETABLE;
	  idl_CommGridMap.cell[x + (idl_CommGridMap.ySizeCells-1) * idl_CommGridMap.xSizeCells] = MAPPER_UNDELETABLE;
  }

  for (y=0; y < idl_CommGridMap.ySizeCells; y++)
  {
	  idl_CommGridMap.cell[ 0 + y * idl_CommGridMap.xSizeCells] = MAPPER_UNDELETABLE;
	  idl_CommGridMap.cell[ (idl_CommGridMap.xSizeCells - 1) + y * idl_CommGridMap.xSizeCells] = MAPPER_UNDELETABLE;
  }

  return 0;
}


int SmartCurrentGridMap::initializeGrowing(void)
{
  int status;

  switch (growingType) {
    case MAPPER_GROWING_NO:
      growingSize   = 0;
      status        = 0;
      break;
    case MAPPER_GROWING_CIRCLE_16:
      growingSize   = 16;
      growing_x[0]  =  3;
      growing_x[1]  =  3;
      growing_x[2]  =  2;
      growing_x[3]  =  1;
      growing_x[4]  =  0;
      growing_x[5]  = -1;
      growing_x[6]  = -2;
      growing_x[7]  = -3;
      growing_x[8]  = -3;
      growing_x[9]  = -3;
      growing_x[10] = -2;
      growing_x[11] = -1;
      growing_x[12] =  0;
      growing_x[13] =  1;
      growing_x[14] =  2;
      growing_x[15] =  3;
      growing_y[0]  =  0;
      growing_y[1]  = -1;
      growing_y[2]  = -2;
      growing_y[3]  = -3;
      growing_y[4]  = -3;
      growing_y[5]  = -3;
      growing_y[6]  = -2;
      growing_y[7]  = -1;
      growing_y[8]  =  0;
      growing_y[9]  =  1;
      growing_y[10] =  2;
      growing_y[11] =  3;
      growing_y[12] =  3;
      growing_y[13] =  3;
      growing_y[14] =  2;
      growing_y[15] =  1;
      status        =  0;
      break;
    case MAPPER_GROWING_STAR_16:
      //growingSize   = 32;
      growingSize   = 16;
      growing_x[0]  =  4;
      growing_x[1]  = -4;
      growing_x[2]  = -4;
      growing_x[3]  =  4;
      growing_x[4]  =  3;
      growing_x[5]  = -3;
      growing_x[6]  = -3;
      growing_x[7]  =  3;
      growing_x[8]  =  2;
      growing_x[9]  = -2;
      growing_x[10] = -2;
      growing_x[11] =  2;
      growing_x[12] =  1;
      growing_x[13] = -1;
      growing_x[14] = -1;
      growing_x[15] =  1;
      growing_y[0]  =  4;
      growing_y[1]  =  4;
      growing_y[2]  = -4;
      growing_y[3]  = -4;
      growing_y[4]  =  3;
      growing_y[5]  =  3;
      growing_y[6]  = -3;
      growing_y[7]  = -3;
      growing_y[8]  =  2;
      growing_y[9]  =  2;
      growing_y[10] = -2;
      growing_y[11] = -2;
      growing_y[12] =  1;
      growing_y[13] =  1;
      growing_y[14] = -1;
      growing_y[15] = -1;

      ///////////////
/*
      growing_x[16]  =  8;
      growing_x[17]  = -8;
      growing_x[18]  = -8;
      growing_x[19]  =  8;
      growing_x[20]  =  7;
      growing_x[21]  = -7;
      growing_x[22]  = -7;
      growing_x[23]  =  7;
      growing_x[24]  =  6;
      growing_x[25]  = -6;
      growing_x[26] = -6;
      growing_x[27] =  6;
      growing_x[28] =  5;
      growing_x[29] = -5;
      growing_x[30] = -5;
      growing_x[31] =  5;

      growing_y[16]  =  8;
      growing_y[17]  =  8;
      growing_y[18]  = -8;
      growing_y[19]  = -8;
      growing_y[20]  =  7;
      growing_y[21]  =  7;
      growing_y[22]  = -7;
      growing_y[23]  = -7;
      growing_y[24]  =  6;
      growing_y[25]  =  6;
      growing_y[26] = -6;
      growing_y[27] = -6;
      growing_y[28] =  5;
      growing_y[29] =  5;
      growing_y[30] = -5;
      growing_y[31] = -5;
*/
      status        =  0;
      break;
    case MAPPER_GROWING_STAR_32:

/*
 yc = [85 -85 -450 -450 -175 -70  70 175 200 200 85]; 
 xc = [180 180 60 -320 -270 -270 -270 -230 -90 70 180];
 figure1 = figure();
 axes1 = axes('Parent',figure1,'XDir','reverse');
 box(axes1,'on');
 grid(axes1, 'on');
 hold(axes1,'all');
 plot(yc,xc)
 
 yo = [8 8 -8 -8 7 7 -7 -7 6 6 -6 -6 5 5 -5 -5 4 4 -4 -4 3 3 -3 -3 2 2 -2 -2 1 1 -1 -1];
 xo = [8 -8 -8 8 7 -7 -7 7 6 -6 -6 6 5 -5 -5 5 4 -4 -4 4 3 -3 -3 3 2 -2 -2 2 1 -1 -1 1];
 yo = yo.*50;
 xo = xo.*50;
 plot(yo,xo,'x')
*/

      growingSize   = 32;
      growing_x[0]  =  4;
      growing_x[1]  = -4;
      growing_x[2]  = -4;
      growing_x[3]  =  4;
      growing_x[4]  =  3;
      growing_x[5]  = -3;
      growing_x[6]  = -3;
      growing_x[7]  =  3;
      growing_x[8]  =  2;
      growing_x[9]  = -2;
      growing_x[10] = -2;
      growing_x[11] =  2;
      growing_x[12] =  1;
      growing_x[13] = -1;
      growing_x[14] = -1;
      growing_x[15] =  1;
      growing_y[0]  =  4;
      growing_y[1]  =  4;
      growing_y[2]  = -4;
      growing_y[3]  = -4;
      growing_y[4]  =  3;
      growing_y[5]  =  3;
      growing_y[6]  = -3;
      growing_y[7]  = -3;
      growing_y[8]  =  2;
      growing_y[9]  =  2;
      growing_y[10] = -2;
      growing_y[11] = -2;
      growing_y[12] =  1;
      growing_y[13] =  1;
      growing_y[14] = -1;
      growing_y[15] = -1;

      ///////////////

      growing_x[16]  =  6;
      growing_x[17]  = -6;
      growing_x[18]  = -6;
      growing_x[19]  =  6;
      growing_x[20]  =  6;
      growing_x[21]  = -6;
      growing_x[22]  = -6;
      growing_x[23]  =  6;
      growing_x[24]  =  6;
      growing_x[25]  = -6;
      growing_x[26] = -6;
      growing_x[27] =  6;
      growing_x[28] =  5;
      growing_x[29] = -5;
      growing_x[30] = -5;
      growing_x[31] =  5;

      growing_y[16]  =  6;
      growing_y[17]  =  6;
      growing_y[18]  = -6;
      growing_y[19]  = -6;
      growing_y[20]  =  6;
      growing_y[21]  =  6;
      growing_y[22]  = -6;
      growing_y[23]  = -6;
      growing_y[24]  =  6;
      growing_y[25]  =  6;
      growing_y[26] = -6;
      growing_y[27] = -6;
      growing_y[28] =  5;
      growing_y[29] =  5;
      growing_y[30] = -5;
      growing_y[31] = -5;

/////////


/*
      growingSize   = 12;
      growing_x[0]  =  3;
      growing_x[1]  = -3;
      growing_x[2]  = -3;
      growing_x[3]  =  3;
      growing_x[4]  =  2;
      growing_x[5]  = -2;
      growing_x[6]  = -2;
      growing_x[7]  =  2;
      growing_x[8]  =  1;
      growing_x[9]  = -1;
      growing_x[10] = -1;
      growing_x[11] =  1;
      growing_y[0]  =  3;
      growing_y[1]  =  3;
      growing_y[2]  = -3;
      growing_y[3]  = -3;
      growing_y[4]  =  2;
      growing_y[5]  =  2;
      growing_y[6]  = -2;
      growing_y[7]  = -2;
      growing_y[8]  =  1;
      growing_y[9]  =  1;
      growing_y[10] = -1;
      growing_y[11] = -1;
      status        =  0;
*/
      break;
    case MAPPER_GROWING_CIRCLE_8:
      growingSize   =  8;
      growing_x[0]  =  2;
      growing_x[1]  =  1;
      growing_x[2]  =  0;
      growing_x[3]  = -1;
      growing_x[4]  = -2;
      growing_x[5]  = -1;
      growing_x[6]  =  0;
      growing_x[7]  =  1;
      growing_y[0]  =  0;
      growing_y[1]  =  1;
      growing_y[2]  =  2;
      growing_y[3]  =  1;
      growing_y[4]  =  0;
      growing_y[5]  = -1;
      growing_y[6]  = -2;
      growing_y[7]  = -1;
      status        =  0;
      break;
    case MAPPER_GROWING_CIRCLE_20:
      growingSize   =  20;
	  growing_x[0]  =  4;
	  growing_x[1]  =  4;
	  growing_x[2]  =  3;
	  growing_x[3]  =  2;
	  growing_x[4]  =  1;
	  growing_x[5]  =  0;
	  growing_x[6]  = -1;
	  growing_x[7]  = -2;
	  growing_x[8]  = -3;
	  growing_x[9]  = -4;
	  growing_x[10] = -4;
	  growing_x[11] = -4;
	  growing_x[12] = -3;
	  growing_x[13] = -2;
	  growing_x[14] = -1;
	  growing_x[15] =  0;
	  growing_x[16] =  1;
	  growing_x[17] =  2;
	  growing_x[18] =  3;
	  growing_x[19] =  4;

	  growing_y[0]  =  0;
	  growing_y[1]  =  1;
	  growing_y[2]  =  2;
	  growing_y[3]  =  3;
	  growing_y[4]  =  4;
	  growing_y[5]  =  4;
	  growing_y[6]  =  4;
	  growing_y[7]  =  3;
	  growing_y[8]  =  2;
	  growing_y[9]  =  1;
	  growing_y[10] =  0;
	  growing_y[11] = -1;
	  growing_y[12] = -2;
	  growing_y[13] = -3;
	  growing_y[14] = -4;
	  growing_y[15] = -4;
	  growing_y[16] = -4;
	  growing_y[17] = -3;
	  growing_y[18] = -2;
	  growing_y[19] = -1;

	  status        =  0;
	  break;

    case MAPPER_GROWING_CIRCLE_32:
      growingSize   =  32;
  	  growing_x[0]  =  6;
  	  growing_x[1]  =  6;
  	  growing_x[2]  =  6;
  	  growing_x[3]  =  5;
  	  growing_x[4]  =  4;
  	  growing_x[5]  =  3;
  	  growing_x[6]  =  2;
  	  growing_x[7]  =  1;
  	  growing_x[8]  =  0;
  	  growing_x[9]  = -1;
  	  growing_x[10] = -2;
  	  growing_x[11] = -3;
  	  growing_x[12] = -4;
  	  growing_x[13] = -5;
  	  growing_x[14] = -6;
  	  growing_x[15] = -6;

  	  growing_x[16] = -6;
  	  growing_x[17] = -6;
  	  growing_x[18] = -6;
  	  growing_x[19] = -5;
  	  growing_x[20] = -4;
  	  growing_x[21] = -3;
  	  growing_x[22] = -2;
  	  growing_x[23] = -1;
  	  growing_x[24] =  0;
  	  growing_x[25] =  1;
  	  growing_x[26] =  2;
  	  growing_x[27] =  3;
  	  growing_x[28] =  4;
  	  growing_x[29] =  5;
  	  growing_x[30] =  6;
  	  growing_x[31] =  6;

  	  growing_y[0]  =  0;
  	  growing_y[1]  =  1;
  	  growing_y[2]  =  2;
  	  growing_y[3]  =  3;
  	  growing_y[4]  =  4;
  	  growing_y[5]  =  5;
  	  growing_y[6]  =  6;
  	  growing_y[7]  =  6;
  	  growing_y[8]  =  6;
  	  growing_y[9]  =  6;
  	  growing_y[10] =  6;
  	  growing_y[11] =  5;
  	  growing_y[12] =  4;
  	  growing_y[13] =  3;
  	  growing_y[14] =  2;
  	  growing_y[15] =  1;
  	  growing_y[16] =  0;

  	  growing_y[17] = -1;
  	  growing_y[18] = -2;
  	  growing_y[19] = -3;
  	  growing_y[20] = -4;
  	  growing_y[21] = -5;
  	  growing_y[22] = -6;
  	  growing_y[23] = -6;
  	  growing_y[24] = -6;
  	  growing_y[25] = -6;
  	  growing_y[26] = -6;
  	  growing_y[27] = -5;
  	  growing_y[28] = -4;
  	  growing_y[29] = -3;
  	  growing_y[30] = -2;
  	  growing_y[31] = -1;

  	  status = 0;
  	  break;

    case MAPPER_GROWING_CIRCLE_40:
      growingSize   =  40;
	  growing_x[0]  =  7;
	  growing_x[1]  =  7;
	  growing_x[2]  =  7;
	  growing_x[3]  =  7;
	  growing_x[4]  =  6;
	  growing_x[5]  =  5;
	  growing_x[6]  =  4;
	  growing_x[7]  =  3;
	  growing_x[8]  =  2;
	  growing_x[9]  =  1;
	  growing_x[10] =  0;
	  growing_x[11] = -1;
	  growing_x[12] = -2;
	  growing_x[13] = -3;
	  growing_x[14] = -4;
	  growing_x[15] = -5;
	  growing_x[16] = -6;
	  growing_x[17] = -7;
	  growing_x[18] = -7;
	  growing_x[19] = -7;
	  growing_x[20] = -7;

	  growing_x[21] = -7;
	  growing_x[22] = -7;
	  growing_x[23] = -7;
	  growing_x[24] = -6;
	  growing_x[25] = -5;
	  growing_x[26] = -4;
	  growing_x[27] = -3;
	  growing_x[28] = -2;
	  growing_x[29] = -1;
	  growing_x[30] =  0;
	  growing_x[31] =  1;
	  growing_x[32] =  2;
	  growing_x[33] =  3;
	  growing_x[34] =  4;
	  growing_x[35] =  5;
	  growing_x[36] =  6;
	  growing_x[37] =  7;
	  growing_x[38] =  7;
	  growing_x[39] =  7;




	  growing_y[0]  =  0;
	  growing_y[1]  =  1;
	  growing_y[2]  =  2;
	  growing_y[3]  =  3;
	  growing_y[4]  =  4;
	  growing_y[5]  =  5;
	  growing_y[6]  =  6;
	  growing_y[7]  =  7;
	  growing_y[8]  =  7;
	  growing_y[9]  =  7;
	  growing_y[10] =  7;
	  growing_y[11] =  7;
	  growing_y[12] =  7;
	  growing_y[13] =  7;
	  growing_y[14] =  6;
	  growing_y[15] =  5;
	  growing_y[16] =  4;
	  growing_y[17] =  3;
	  growing_y[18] =  2;
	  growing_y[19] =  1;
	  growing_y[20] =  0;


	  growing_y[21] = -1;
	  growing_y[22] = -2;
	  growing_y[23] = -3;
	  growing_y[24] = -4;
	  growing_y[25] = -5;
	  growing_y[26] = -6;
	  growing_y[27] = -7;
	  growing_y[28] = -7;
	  growing_y[29] = -7;
	  growing_y[30] = -7;
	  growing_y[31] = -7;
	  growing_y[32] = -7;
	  growing_y[33] = -7;
	  growing_y[34] = -6;
	  growing_y[35] = -5;
	  growing_y[36] = -4;
	  growing_y[37] = -3;
	  growing_y[38] = -2;
	  growing_y[39] = -1;

	  status        =  0;
	  break;



    default:
      status = 1;
      break;
  }
  return status;
}


int SmartCurrentGridMap::bresenham(int xa,int ya,int xs,int ys)
{
  int x,y,z,dx,dy,dz,i1,i2,index;
  unsigned char update1,update2;

  dx = abs(xa-xs);
  dy = abs(ya-ys);

  if (xa<xs)
  {
    x = xa;
    y = ya;
    update1 = MAPPER_FREE;
    update2 = MAPPER_OBSTACLE;
    if (ya>ys) z=-1; else z=1;
  }
  else
  {
    x = xs;
    y = ys;
    update1 = MAPPER_OBSTACLE;
    update2 = MAPPER_FREE;
    if (ys>ya) z=-1; else z=1;
  }

  if (dx>dy) i2=dx; else i2=dy;
  dz = i2/2;

  // the map border must not be destroyed during update
  if ((0<x) && (x < (int)idl_CommGridMap.xSizeCells-1) && (0<y) && (y < (int)idl_CommGridMap.ySizeCells-1)) {
      //hochdorfer lutz 2011-01-25 added MAPPER_UNDELETABLE
      //ignore cells with value MAPPER_UNDELETABLE
      if ( (idl_CommGridMap.cell[x+y * idl_CommGridMap.xSizeCells] != MAPPER_UNDELETABLE))
      {
    	  idl_CommGridMap.cell[x+y * idl_CommGridMap.xSizeCells] = update1;
      }
  }

  for (i1=1; i1 <=i2; i1++)
  {
    if (dz<dx)
    {
      dz = dz+dy;
      x = x+1;
    }
    if (dz>=dx)
    {
      dz = dz-dx;
      y = y+z;
    }

    index = x+y*idl_CommGridMap.xSizeCells;

    // the map border must not be destroyed during update
    if ((0<x) && (x < (int)idl_CommGridMap.xSizeCells-1) && (0<y) && (y < (int)idl_CommGridMap.ySizeCells-1))
    {
      //hochdorfer lutz 2011-01-25 added MAPPER_UNDELETABLE
      //ignore cells with value MAPPER_UNDELETABLE
      if ( (idl_CommGridMap.cell[index] != MAPPER_UNDELETABLE))
      {
        if (i1!=i2)
        {
          if ((idl_CommGridMap.cell[index] != MAPPER_GROWING))
          {
        	  idl_CommGridMap.cell[index] = MAPPER_FREE;
          }
        }
        if (i1==i2) idl_CommGridMap.cell[index] = update2;
      }
    }
  }
  return 0;
}


int SmartCurrentGridMap::obstacleGrowing(int x,int y)
{
  int x1,y1;

  for (int i=0; i < growingSize; i++) {
    x1 = x + growing_x[i];
    y1 = y + growing_y[i];

    if ((0<=x1) && (x1 < (int)idl_CommGridMap.xSizeCells) && (0<=y1) && (y1 < (int)idl_CommGridMap.ySizeCells))
    {
      assert(idl_CommGridMap.cell.size() > x1+y1*idl_CommGridMap.xSizeCells);
      if (idl_CommGridMap.cell[x1+y1*idl_CommGridMap.xSizeCells] != MAPPER_UNDELETABLE)
      {
    	  idl_CommGridMap.cell[x1+y1*idl_CommGridMap.xSizeCells] = MAPPER_GROWING;
      }
    }
  }
  return 0;
}

