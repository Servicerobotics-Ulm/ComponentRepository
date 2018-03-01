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

#ifndef SMART_CURRENT_GRID_MAP_HH
#define SMART_CURRENT_GRID_MAP_HH

#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>

#include "CommBasicObjects/CommMobileLaserScan.hh"
#include "CommNavigationObjects/CommGridMap.hh"


#include "smartLtmGridMap.hh"

#define MAPPER_MAX_GROWING_SIZE	40
//#define MAPPER_MAX_GROWING_SIZE	16

namespace Smart
{

class SmartCurrentGridMap : public CommNavigationObjects::CommGridMap
{

public:

  // Default constructor creates map of size (0,0) at position (0,0) and
  // cellsize 0 without obstacleGrowing.
  SmartCurrentGridMap();

  // Initialize the current map of size (x,y) at position (0,0) and
  // cellsize (cell) and no obstacle growing. All cells are set to
  // free. All values in [mm].
  SmartCurrentGridMap(int x,int y,int size,int growing);

  // Initialize the actual map of size (x,y) at position (px,py) and
  // cellsize (cell) and obstacle growing type (growing type). The last
  // parameter specifies the mapId. All cells are set to free. All
  // values in [mm].
  SmartCurrentGridMap(int x,int y,int px,int py,int size,int growing,int id);

/* // use auto generated
   //   if there are really no problems - please remove them

  // Assignment operator
  SmartCurrentGridMap const &operator=(SmartCurrentGridMap const &other);

  // Copy constructor
  SmartCurrentGridMap(SmartCurrentGridMap const &other);
*/

  // Implicitly destroy the map class
  virtual ~SmartCurrentGridMap();

  // All cells will be marked as free, except the border cells, which are
  // marked as obstacles.
  int clearMap();

  // Update the current map with new laser scan data.
  int update( CommBasicObjects::CommMobileLaserScan const &scan );

  // Preoccupy the current map with values provided in the longterm map. The
  // longterm map provides a specific method to get access to the internal
  // longterm map representation so please have a look there for further
  // details.
  int setLtmOccupation( int threshold, Smart::SmartLtmGridMap &gm );

  // Converts obstacle growing cells into normal obstacles
  int convertObstacleGrowing();

private:

  // Type of obstacle growing, {circle16,circle8,star16,star32}
  int growingType;

  // Number of cells for obstacle growing.
  // 0 :  no obstacle growing at all
  //
  // 8 :     #
  //        # #
  //       # O #
  //        # #
  //         #
  //
  // 12:  #     #
  //       #   #
  //        # #
  //         O
  //        # #
  //       #   #
  //      #     #
  //
  // 16: #       #
  //      #     #
  //       #   #
  //        # #
  //         O
  //        # #
  //       #   #
  //      #     #
  //     #       #
  //
  // 32:
  //   #           #
  //    #         #
  //     #       #
  //      #     #
  //       #   #
  //        # #
  //         O
  //        # #
  //       #   #
  //      #     #
  //     #       #
  //    #         #
  //   #           #
  //
  // 16:        ###
  //           #   #
  //          #     #
  //          #  O  #
  //          #     #
  //           #   #
  //            ###
  //
  // 20:
  //
  //            ###
  //           #   #
  //          #     #
  //         #       #
  //         #   O   #
  //         #       #
  //          #     #
  //           #   #
  //            ###
  //


  // 32:
  //
  //         #####
  //        #     #
  //       #       #
  //      #         #
  //     #           #
  //     #           #
  //     #     O     #
  //     #           #
  //     #           #
  //      #         #
  //       #       #
  //        #     #
  //         #####
  //




// 40:
//
//           #####
//          #     #
//         #       #
//        #         #
//       #           #
//      #             #
//     #               #
//     #               #
//     #       O       #
//     #               #
//     #               #
//      #             #
//       #           #
//        #         #
//         #       #
//          #     #
//           #####
//


 int growingSize;

  int growing_x[MAPPER_MAX_GROWING_SIZE];
  int growing_y[MAPPER_MAX_GROWING_SIZE];


  // Used in the constructor.
  int initializeGrowing(void);

  // Initialize border cells with obstacle
  int drawBorder();

  // Draw line from index xa/ya to index xs/ys, where all intermediate
  // cells are labeled as free and the last cell is marked as obstacle.
  // During cell update the map border must not be destroyed. All coordinates
  // are cell index values.
  int bresenham(int,int, int, int);

  // Obstacle growing at index x,y in current map.
  int obstacleGrowing(int, int);


}; // class SmartCurrentGridMap

} // namespace Smart

#endif

