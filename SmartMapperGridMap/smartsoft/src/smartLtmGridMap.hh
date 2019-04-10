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

#ifndef SMART_LTM_GRID_MAP_HH
#define SMART_LTM_GRID_MAP_HH

#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>

#include "CommBasicObjects/CommMobileLaserScan.hh"
#include "CommNavigationObjects/CommGridMap.hh"


namespace Smart
{

class SmartLtmGridMap : public CommNavigationObjects::CommGridMap
{

public:

  // Default constructor creates map of size (0,0) at position (0,0) and
  // cellsize 0.
  SmartLtmGridMap();

  // Initialize the longterm map of size (x,y) at position (0,0) and
  // cellsize (cell). All cells are set to 0. All values in [mm].
  SmartLtmGridMap(int x,int y,int size);

  // Initialize the longterm map of size (x,y) at position (px,py) and
  // cellsize (cell) with (mapId). All cells are set to 0. All values
  // in [mm].
  SmartLtmGridMap(int x,int y,int px,int py,int size,int id);

/* // use auto generated
   //   if there are really no problems - please remove them

  // Assignment operator for longterm map. The size of the goal map is
  // adjusted to the size of the source map
  SmartLtmGridMap const &operator=(SmartLtmGridMap const &other);

  // Copy constructor
  SmartLtmGridMap(SmartLtmGridMap const &other);
*/


  // Implicitly destroy the map class
  virtual ~SmartLtmGridMap();

  // Set all cells on that line to the specified value. This method is
  // used to generate a ltm preoccupation from world model files within
  // the world model server.
  // start x,y  end x,y  value to be set
  int occupyLine(int x1,int y1,int x2,int y2,int n);

  // Update the longtermmap with new laser scan data.
  // Parameter specifies kalman filter weighting [0,255].
  int update( CommBasicObjects::CommMobileLaserScan const &scan, double n );

  // All cells will be initialized to the given value [0,255]. 0 denotes
  // free and 255 denotes occupied.
  int clearMap(int);

  // Copy the specified part of the LTM into the destination grid. The
  // destination grid must be big enough, there are no more tests
  // inside this method. Values equal and above the threshold are
  // considered as occupied. If the destination cell is labeled as
  // GROWING then it is not overwritten ! The values concerning map
  // size are specified in [mm].
  int getPartialMap( int partXoffset,int partYoffset,
                     int partXsize,int partYsize,
                     int threshold,
                     CommNavigationObjectsIDL::CommGridMap_cell_type &destCell);

  // write map context to yaml a file and the map data in a pgm image file.
  void save_yaml_pgm(std::ostream &os_yaml, std::string pgmFileName);


  // write map context to yaml a file and the map data in a ppm image file.
  void save_yaml_ppm(std::ostream &os_yaml, std::string ppmFileName);

  // load a stored map from a yaml file. The yaml file contains the file name
  // of the image file which contains the map data. The method can handle pgm
  // and ppm images as input data.
  int load_yaml( std::string fname);

private:

  // Initialize border cells with obstacle
  int drawBorder();

  // Used for map updates. Last argument specifies Kalman filter
  // adaptation rate.
  int bresenham(int,int,int,int,double);

  // Used for saving the map context in yaml format to the ostream os_yaml.
  // The string mapFileName contains the file name of the corresponding map file.
  void write_yaml(std::ostream &os_yaml, std::string mapFileName);

  void write_ascii(std::string filename);

}; // class SmartLtmGridMap

} // namespace Smart

#endif

