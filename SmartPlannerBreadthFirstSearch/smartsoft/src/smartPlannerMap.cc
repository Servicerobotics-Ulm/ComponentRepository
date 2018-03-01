// --------------------------------------------------------------------------
//
//  Copyright (C) 2009 Christian Schlegel, Matthias Lutz, Andreas Steck
//
//        schlegel@hs-ulm.de
//        steck@hs-ulm.de
//        lutz@hs-ulm.de
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


#include "smartPlannerMap.hh"

using namespace Smart;

PlannerMapClass::PlannerMapClass() : CommGridMap()
{

  waveFront[0].x=0;
  waveFront[4].x=0;
  waveFront[0].y=1;
  waveFront[4].y=1;
  waveFront[0].direction=PLANNER_NORTH;
  waveFront[4].direction=PLANNER_NORTH;
  waveFront[1].x=-1;
  waveFront[5].x=-1;
  waveFront[1].y=0;
  waveFront[5].y=0;
  waveFront[1].direction=PLANNER_WEST;
  waveFront[5].direction=PLANNER_WEST;
  waveFront[2].x=0;
  waveFront[6].x=0;
  waveFront[2].y=-1;
  waveFront[6].y=-1;
  waveFront[2].direction=PLANNER_SOUTH;
  waveFront[6].direction=PLANNER_SOUTH;
  waveFront[3].x=1;
  waveFront[7].x=1;
  waveFront[3].y=0;
  waveFront[7].y=0;
  waveFront[3].direction=PLANNER_EAST;
  waveFront[7].direction=PLANNER_EAST;
}

PlannerMapClass::PlannerMapClass(int x,int y,int size,int xo,int yo)
{
  // constructor

  idl_CommGridMap.id = 0;
  idl_CommGridMap.is_valid = false;
  idl_CommGridMap.time.sec = 0;
  idl_CommGridMap.time.usec = 0;
  idl_CommGridMap.xOffsetMM = xo;
  idl_CommGridMap.yOffsetMM = yo;
  idl_CommGridMap.xOffsetCells = (int)floor((double)xo/(double)size);
  idl_CommGridMap.yOffsetCells = (int)floor((double)yo/(double)size);
  idl_CommGridMap.cellSizeMM = size;
  idl_CommGridMap.xSizeMM = x*size;
  idl_CommGridMap.ySizeMM = y*size;
  idl_CommGridMap.xSizeCells = x;
  idl_CommGridMap.ySizeCells = y;
  idl_CommGridMap.size = idl_CommGridMap.xSizeCells*idl_CommGridMap.ySizeCells;

  idl_CommGridMap.cell.resize(idl_CommGridMap.size);

  waveFront[0].x=0;
  waveFront[4].x=0;
  waveFront[0].y=1;
  waveFront[4].y=1;
  waveFront[0].direction=PLANNER_NORTH;
  waveFront[4].direction=PLANNER_NORTH;
  waveFront[1].x=-1;
  waveFront[5].x=-1;
  waveFront[1].y=0;
  waveFront[5].y=0;
  waveFront[1].direction=PLANNER_WEST;
  waveFront[5].direction=PLANNER_WEST;
  waveFront[2].x=0;
  waveFront[6].x=0;
  waveFront[2].y=-1;
  waveFront[6].y=-1;
  waveFront[2].direction=PLANNER_SOUTH;
  waveFront[6].direction=PLANNER_SOUTH;
  waveFront[3].x=1;
  waveFront[7].x=1;
  waveFront[3].y=0;
  waveFront[7].y=0;
  waveFront[3].direction=PLANNER_EAST;
  waveFront[7].direction=PLANNER_EAST;
}

PlannerMapClass::~PlannerMapClass()
{
  // destructor
};


/* use auto generated - for more information check the header file
PlannerMapClass::PlannerMapClass(const PlannerMapClass& other)
{
  // copy constructor
}

PlannerMapClass const &PlannerMapClass::operator=(const PlannerMapClass& other)
{
  // assignment operator
}
*/

int PlannerMapClass::index(double w,double offset)
{
  return (int)(floor((double)(w-offset)/(double)idl_CommGridMap.cellSizeMM));
}

int PlannerMapClass::setMapperMap(CommNavigationObjects::CommGridMap& a)
{
  bool           aStatus;
  int            aXOffsetCells,aYOffsetCells,aXOffsetMM,aYOffsetMM;
  unsigned int   aXSizeCells,aYSizeCells,aXSizeMM,aYSizeMM,aCellSizeMM;
  unsigned int   aMapId;
  struct timeval aTime;
  int            index = 0;
  int            status;


  status = a.get_parameter(aMapId,aStatus,aTime,aXOffsetMM,aYOffsetMM,aXOffsetCells,
                          aYOffsetCells,aCellSizeMM,aXSizeMM,aYSizeMM,aXSizeCells,aYSizeCells);


  if (idl_CommGridMap.cell.size() != aXSizeCells*aYSizeCells) {
    // adjust map size
	  idl_CommGridMap.cell.resize(aXSizeCells*aYSizeCells);
  }

  idl_CommGridMap.id = aMapId;
  idl_CommGridMap.xOffsetCells = aXOffsetCells;
  idl_CommGridMap.yOffsetCells = aYOffsetCells;
  idl_CommGridMap.xOffsetMM  = aXOffsetMM;
  idl_CommGridMap.yOffsetMM  = aYOffsetMM;
  idl_CommGridMap.cellSizeMM = aCellSizeMM;
  idl_CommGridMap.xSizeMM = aXSizeMM;
  idl_CommGridMap.ySizeMM = aYSizeMM;
  idl_CommGridMap.xSizeCells = aXSizeCells;
  idl_CommGridMap.ySizeCells = aYSizeCells;
  idl_CommGridMap.is_valid     = aStatus;
  idl_CommGridMap.size = aYSizeCells*aXSizeCells;


  for (unsigned int y=0;y<aYSizeCells;y++) {
     for (unsigned int x=0;x<aXSizeCells;x++) {
    	 idl_CommGridMap.cell[index++] = a.get_cells(x,y);
     }
   }

  return 0;
}

int PlannerMapClass::convert(void)
{
  int content;

  for (unsigned int i=0;i<idl_CommGridMap.size;i++) {
    switch(idl_CommGridMap.cell[i]) {
      case MAPPER_CELL_FREE:
        content = PLANNER_FREE;
        break;
      case MAPPER_CELL_OBSTACLE:
        content = PLANNER_OBSTACLE;
        break;
      case MAPPER_CELL_GROWING:
        content = PLANNER_GROWING;
        break;
      default:
        content = PLANNER_OBSTACLE;
        break;
    }
    idl_CommGridMap.cell[i]=content;
  }
  return 0;
}

int PlannerMapClass::debugEmptyMap(void)
{
  unsigned int x,y;

  for (x=0;x<idl_CommGridMap.xSizeCells;x++) {
    for (y=0;y<idl_CommGridMap.ySizeCells;y++) {
    	idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells]=PLANNER_FREE;
      // [x][y]
    }
  }
  for (x=0;x<idl_CommGridMap.xSizeCells;x++) {
	  idl_CommGridMap.cell[x+0*idl_CommGridMap.xSizeCells]=PLANNER_OBSTACLE;
    // [x][0]
	  idl_CommGridMap.cell[x+(idl_CommGridMap.ySizeCells-1)*idl_CommGridMap.xSizeCells]=PLANNER_OBSTACLE;
    // [x][ysize-1]
  }
  for (y=0;y<idl_CommGridMap.ySizeCells;y++) {
	  idl_CommGridMap.cell[0+y*idl_CommGridMap.xSizeCells]=PLANNER_OBSTACLE;
    // [0][y]
	  idl_CommGridMap.cell[(idl_CommGridMap.xSizeCells-1)+y*idl_CommGridMap.xSizeCells]=PLANNER_OBSTACLE;
    // [xsize-1][y]
  }
  return 0;
}

int PlannerMapClass::markCurrentPosition(double x,double y)
{
  int xi,yi;
  int content;

  xi = index(x,idl_CommGridMap.xOffsetMM);
  yi = index(y,idl_CommGridMap.yOffsetMM);

  content = idl_CommGridMap.cell[xi+yi*idl_CommGridMap.xSizeCells];
  switch(content) {
    case PLANNER_FREE:
    	idl_CommGridMap.cell[xi+yi*idl_CommGridMap.xSizeCells]=PLANNER_START;
      return 0;
      break;
    case PLANNER_OBSTACLE:
      return 1;
      break;
    case PLANNER_GROWING:
      return 1;
      break;
    case PLANNER_GOAL:
      return 2;
      break;
    default:
      return 3;
      break;
  }
}

int PlannerMapClass::internalBresenham(int x1,int y1,int x2,int y2)
{
  int x,y,z;
  int dx,dy,dz;
  int i1,i2;
  int flag;
  int content;

  flag=1;
  dx=abs(x1-x2);
  dy=abs(y1-y2);
  if (x1 < x2) {
    x=x1;
    y=y1;
    if (y1 > y2) z=-1; else z=1;
  } else {
    x=x2;
    y=y2;
    if (y2 > y1) z=-1; else z=1;
  }
  if (dx > dy) i2=dx; else i2=dy;
  dz=i2/2;
  content = idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells];
  // [x][y]
  if ((content==PLANNER_OBSTACLE) || (content==PLANNER_GROWING)) return 1;
  for (i1=1;i1<=i2;i1++) {
    if (dz < dx) {
      dz=dz+dy;
      x=x+1;
    }
    if (dz >= dx) {
      dz=dz-dx;
      y=y+z;
    }
    content = idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells];
    if ((content==PLANNER_OBSTACLE) || (content==PLANNER_GROWING)) return 1;
  }
  return 0;
}

int PlannerMapClass::saveAscii(const char *filename)
{
  FILE *file;

  file = fopen(filename,"w");
  if (file==NULL) {
    std::cerr << "Unable to open data file " << filename << " for actual map\n";
    return 1;
  } else {
    fprintf(file,"Format : SFB527 ()\n");
    fprintf(file,"Content : Planner Actual Map\n");
    fprintf(file,"XSize [cells] : %d\n",idl_CommGridMap.xSizeCells);
    fprintf(file,"YSize [cells] : %d\n",idl_CommGridMap.ySizeCells);
    fprintf(file,"XOffset [mm] : %d\n",idl_CommGridMap.xOffsetMM);
    fprintf(file,"YOffset [mm] : %d\n",idl_CommGridMap.yOffsetMM);
    fprintf(file,"Cellsize [mm] : %d\n",idl_CommGridMap.cellSizeMM);
    fprintf(file,"Time [sec,usec] : %d %d\n",idl_CommGridMap.time.sec,idl_CommGridMap.time.usec);
    fprintf(file,"cell obstacle : %d\n",PLANNER_OBSTACLE);
    fprintf(file,"cell growing  : %d\n",PLANNER_GROWING);
    fprintf(file,"cell goal     : %d\n",PLANNER_GOAL);
    fprintf(file,"cell start    : %d\n",PLANNER_START);
    fprintf(file,"cell free     : %d\n",PLANNER_FREE);
    fprintf(file,"cell north    : %d\n",PLANNER_NORTH);
    fprintf(file,"cell west     : %d\n",PLANNER_WEST);
    fprintf(file,"cell south    : %d\n",PLANNER_SOUTH);
    fprintf(file,"cell east     : %d\n",PLANNER_EAST);
    fprintf(file,"DATA AREA ASCII\n");

    for (unsigned int y=0;y<idl_CommGridMap.ySizeCells;y++) {
      for (unsigned int x=0;x<idl_CommGridMap.xSizeCells;x++) {
        switch(idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells]) {
          case PLANNER_OBSTACLE:
            fprintf(file,"#");
            break;
          case PLANNER_GROWING:
            fprintf(file,"*");
            break;
          case PLANNER_GOAL:
            fprintf(file,"G");
            break;
          case PLANNER_START:
            fprintf(file,"S");
            break;
          case PLANNER_FREE:
            fprintf(file,".");
            break;
          case PLANNER_NORTH:
            fprintf(file,"n");
            break;
          case PLANNER_WEST:
            fprintf(file,"w");
            break;
          case PLANNER_SOUTH:
            fprintf(file,"s");
            break;
          case PLANNER_EAST:
            fprintf(file,"e");
            break;
          default:
            fprintf(file,"!");
            break;
        }
      }
      fprintf(file,"\n");
    }
    fclose(file);
  }
  return 0;
}

void PlannerMapClass::save_xpm( std::ostream &os ) const
{
  int   h,r,g,b,a;

  os << "/* XPM */\n";
  os << "static char *noname[] = {\n";
  os << "/* width height ncolors chars_per_pixel */\n";
  os << "\"" << idl_CommGridMap.xSizeCells << " " << idl_CommGridMap.ySizeCells << " 10 2\",\n";
  os << "/* colors */\n";


  os << std::setfill('0');

    // special value: planner obstacle
    h = PLANNER_OBSTACLE;
    r = 0;
    g = 0;
    b = 0;

    os << std::hex << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";

    // special value: planner growing
    h = PLANNER_GROWING;
    r = 64;
    g = 64;
    b = 64;


    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";

    // special value: planner goal
    h = PLANNER_GOAL;
    r = 0;
    g = 0;
    b = 255;


    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";

    // special value: planner start
    h = PLANNER_START;
    r = 0;
    g = 255;
    b = 0;

    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";

    // special value: planner free
    h = PLANNER_FREE;
    r = 255;
    g = 255;
    b = 255;

    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";

    // special value: planner north
    h = PLANNER_NORTH;
    r = 128;
    g = 64;
    b = 64;

    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";

    // special value: planner west
    h = PLANNER_WEST;
    r = 128;
    g = 96;
    b = 96;

    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";


    // special value: planner south
    h = PLANNER_SOUTH;
    r = 128;
    g = 160;
    b = 160;

    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";


    // special value: planner east
    h = PLANNER_EAST;
    r = 128;
    g = 192;
    b = 192;

    os << "\"" << std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";


    // special value: planner default
    h = 99;
    r = 64;
    g = 128;
    b = 128;

    os << "\"" <<std::setw(2) << h << " c "
       << "#" << std::setw(2) << r
              << std::setw(2) << g
              << std::setw(2) << b << "\",\n";

    os << "/* pixels */\n";

    for (int y=(idl_CommGridMap.ySizeCells-1);y>=0;y--) {
      os << "\"";
      for (unsigned int x=0;x<idl_CommGridMap.xSizeCells;x++) {
        a = (int)(idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells]);
         switch(a) {
          case PLANNER_OBSTACLE:
          case PLANNER_GROWING:
          case PLANNER_START:
          case PLANNER_GOAL:
          case PLANNER_FREE:
          case PLANNER_NORTH:
          case PLANNER_WEST:
          case PLANNER_SOUTH:
          case PLANNER_EAST:
            // these are valid entries so don't change them
            break;
          default:
            // not a valid entry
            a = 99;
            break;
        }
        os << std::setw(2) <<  a;
      }
      if (y==0) {
        os << "\"\n";
      } else {
        os << "\",\n";
      }
    }
    os << "};\n";
}

// -------------------------------------------------------------------------
//
// Methods which implement the path planning algorithm
//
// -------------------------------------------------------------------------

// --------------------------------------------------------------------------
// circle algorithm to draw goal circle
//
// Parameter: write  MODE_OBSTACLE occupy cell with obstacle without check
//                                 whether cell is free. Always return 0.
//                   MODE_GOAL     if cell is free mark cell as goal cell
//                                 and write cell coordinates on fifo. If
//                                 at least one goal cell was empty return
//                                 0 else return 1.
//                   MODE_TEST     Return 0 if all cells on the line are empty,
//                                 else return 1. Doesn't modify the grid.
// --------------------------------------------------------------------------
int PlannerMapClass::circle(double xw,double yw,double r,int write)
{
  int    status;
  int    flag;
  double dx,dy;

  dx = r*cos(M_PI/4.0);
  dy = r*sin(M_PI/4.0);

  // now draw circle in grid
  // approximate via 8 lines
  //
  //         C
  //     D       B
  //
  //   E           A
  //
  //     F       H
  //         G
  //

  status = 0;
  status += bresenham(xw+r,yw,xw+dx,yw+dy,write);  // A B
  status += bresenham(xw+dx,yw+dy,xw,yw+r,write);  // B C
  status += bresenham(xw,yw+r,xw-dx,yw+dy,write);  // C D
  status += bresenham(xw-dx,yw+dy,xw-r,yw,write);  // D E
  status += bresenham(xw-r,yw,xw-dx,yw-dy,write);  // E F
  status += bresenham(xw-dx,yw-dy,xw,yw-r,write);  // F G
  status += bresenham(xw,yw-r,xw+dx,yw-dy,write);  // G H
  status += bresenham(xw+dx,yw-dy,xw+r,yw,write);  // H A

  switch(write) {
    case MODE_OBSTACLE:
      flag = 0;
      break;
    case MODE_GOAL:
      if (status == 8) {
        // no goal cell was empty
        flag = 1;
      } else {
        // at least one goal cell was empty
        flag = 0;
      }
      break;
    case MODE_TEST:
      if (status == 0) {
        // all cells are empty
        flag = 0;
      } else {
        // not all cells are empty
        flag = 1;
      }
      break;
    default:
      // error
      flag = 2;
      break;
  }
  return flag;
}



// --------------------------------------------------------------------------
// Bresenham algorithm to draw lines
//
// Parameter: write  MODE_OBSTACLE occupy cell with obstacle without check
//                                 if cell is free. Always return 0.
//                   MODE_GOAL     if cell is free mark cell as goal cell
//                                 and write cell coordinates on fifo. If
//                                 at least one goal cell was empty return
//                                 0 else return 1.
//                   MODE_TEST     Return 0 if all cells on the line are empty,
//                                 else return 1. Doesn't modify the grid.
// --------------------------------------------------------------------------
int PlannerMapClass::bresenham(double x1w,double y1w,double x2w,double y2w,int write)
{
  int x1,y1,x2,y2;
  int x,y,z;
  int dx,dy,dz;
  int i1,i2;
  int flag;
  int mapindex;

  x1 = index(x1w,idl_CommGridMap.xOffsetMM);
  y1 = index(y1w,idl_CommGridMap.yOffsetMM);
  x2 = index(x2w,idl_CommGridMap.xOffsetMM);
  y2 = index(y2w,idl_CommGridMap.yOffsetMM);

  flag=1;
  dx=abs(x1-x2);
  dy=abs(y1-y2);
  if (x1 < x2) {
    x=x1;
    y=y1;
    if (y1 > y2) z=-1; else z=1;
  } else {
    x=x2;
    y=y2;
    if (y2 > y1) z=-1; else z=1;
  }
  if (dx > dy) i2=dx; else i2=dy;
  dz=i2/2;
  mapindex = x+y*idl_CommGridMap.xSizeCells;
  switch(write) {
    case MODE_OBSTACLE:
    	idl_CommGridMap.cell[mapindex]=PLANNER_OBSTACLE;
      break;
    case MODE_GOAL:
      if (idl_CommGridMap.cell[mapindex] == PLANNER_FREE) {
        fifoWrite(x,y);
        idl_CommGridMap.cell[mapindex]=PLANNER_GOAL;
        flag=0;
      }
      break;
    case MODE_TEST:
      if ((idl_CommGridMap.cell[mapindex] == PLANNER_OBSTACLE) ||
          (idl_CommGridMap.cell[mapindex] == PLANNER_GROWING)) return 1;
      break;
    default:
      // wrong mode of operation
      break;
  }
  for (i1=1;i1<=i2;i1++) {
    if (dz < dx) {
      dz=dz+dy;
      x=x+1;
    }
    if (dz >= dx) {
      dz=dz-dx;
      y=y+z;
    }
    mapindex = x+y*idl_CommGridMap.xSizeCells;
    switch(write) {
      case MODE_OBSTACLE:
    	  idl_CommGridMap.cell[mapindex]=PLANNER_OBSTACLE;
        break;
      case MODE_GOAL:
        if (idl_CommGridMap.cell[mapindex] == PLANNER_FREE) {
          fifoWrite(x,y);
          idl_CommGridMap.cell[mapindex]=PLANNER_GOAL;
          flag=0;
        }
        break;
      case MODE_TEST:
        if ((idl_CommGridMap.cell[mapindex] == PLANNER_OBSTACLE) ||
            (idl_CommGridMap.cell[mapindex] == PLANNER_GROWING)) return 1;
        break;
      default:
        // wrong mode of operation
        break;
    }
  }
  if (write == MODE_GOAL) return flag; else return 0;
}

int PlannerMapClass::waveFrontFlood(void)
{
  int zentrum;
  int zelle;
  int i;
  int xneu;
  int yneu;
  int x;
  int y;
  int status;
  int count;

  while (fifoRead(&x,&y) == 0) {
    zentrum = idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells];
    if (zentrum == PLANNER_GOAL)
      zentrum=PLANNER_NORTH;
    count = 4;
    for (i=zentrum;i<zentrum+count;i++) {
      xneu  = x+waveFront[i].x;
      yneu  = y+waveFront[i].y;
      zelle = idl_CommGridMap.cell[xneu+yneu*idl_CommGridMap.xSizeCells];
      switch (zelle) {
        case PLANNER_START:
        	idl_CommGridMap.cell[xneu+yneu*idl_CommGridMap.xSizeCells]=waveFront[i].direction+8;
          return 0;
          break;
        case PLANNER_FREE:
        	idl_CommGridMap.cell[xneu+yneu*idl_CommGridMap.xSizeCells]=waveFront[i].direction;
          status=fifoWrite(xneu,yneu);
          break;
      }
    }
  }
  return 1;
}

int PlannerMapClass::waveFrontOptimizeFirstSegment(double xworld,double yworld,
                                                   double &xgoal,double &ygoal)
{
  int help2;
  int wert;
  int xneu,yneu;
  int start_x;
  int start_y;
  int x;
  int y;

  x = index(xworld,idl_CommGridMap.xOffsetMM);
  y = index(yworld,idl_CommGridMap.yOffsetMM);

  start_x = x;
  start_y = y;
  xneu    = x;
  yneu    = y;
  help2   = internalBresenham(start_x,start_y,xneu,yneu);

  while ((idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells] != PLANNER_GOAL) && (help2 == 0)) {
    x=xneu;
    y=yneu;
    wert=idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells];
    if (wert != PLANNER_GOAL) {
      wert = wert % 8;
      idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells]=wert + 8;
      xneu=x-waveFront[wert].x;
      yneu=y-waveFront[wert].y;
    } else {
      /* --------------------------------------------------------------- */
      /* wird mit x/y, welche den Wegepunkt angeben, der gerade noch per */
      /* Luftlinie erreichbar ist, die Zielzelle erreicht, so beinhalten */
      /* xneu/yneu, welche auf dem Weg stets eine Zelle voraus sind,     */
      /* keine sinnvollen Werte mehr. In diesem Fall xneu/yneu nicht     */
      /* weiter verwenden.                                               */
      /* --------------------------------------------------------------- */
      xneu=0;
      yneu=0;
    }
    /* -------------------------------------------------------------- */
    /* wenn x/y auf die Zielzelle zeigen, enthaelt xneu/yneu keinen   */
    /* sinnvollen Zellindex, so dass algorithmus_bresenham nicht      */
    /* aufgerufen werden darf. In diesem Falle ist die Belegung von   */
    /* help2 mit OK oder NOK egal, da die while-Schleife sowieso      */
    /* aufgrund des Wertes von help1 abgebrochen wird.                */
    /* -------------------------------------------------------------- */
    if (idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells] != PLANNER_GOAL)
      help2=internalBresenham(start_x,start_y,xneu,yneu);
    else
      help2=0;
  }
  xgoal = x*(int)idl_CommGridMap.cellSizeMM+idl_CommGridMap.xOffsetMM;
  ygoal = y*(int)idl_CommGridMap.cellSizeMM+idl_CommGridMap.yOffsetMM;

  return 0;
}

int PlannerMapClass::waveFrontFindGoal(double xstart,double ystart,
                                       double &xgoal,double &ygoal)
{
  int wert;
  int xneu,yneu;
  int x,y;

  x = index(xstart,idl_CommGridMap.xOffsetMM);
  y = index(ystart,idl_CommGridMap.yOffsetMM);

  while (idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells] != PLANNER_GOAL) {
    wert=idl_CommGridMap.cell[x+y*idl_CommGridMap.xSizeCells] % 8;
    xneu=(x)-waveFront[wert].x;
    yneu=(y)-waveFront[wert].y;
    x=xneu;
    y=yneu;
  }
  xgoal = x*(int)idl_CommGridMap.cellSizeMM+idl_CommGridMap.xOffsetMM;
  ygoal = y*(int)idl_CommGridMap.cellSizeMM+idl_CommGridMap.yOffsetMM;

  return 0;
}

