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

#include "smartLtmGridMap.hh"


#include <yaml.h>
#ifdef WITH_OPENCV_YAML
#include <cxcore.h>
#include <highgui.h>
#endif
#include <iostream>
#if defined (__GNUC__) && defined(__unix__)
#include <libgen.h>
#elif defined (WIN32) || defined (WIN64)
#include <stdio.h>
#endif

using namespace std;

using namespace Smart;


SmartLtmGridMap::SmartLtmGridMap() : CommGridMap()
{
  // constructor
}


SmartLtmGridMap::SmartLtmGridMap(int x,int y,int size)
{
  // constructor
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
}


SmartLtmGridMap::SmartLtmGridMap(int x,int y,int px,int py,int size,int id)
{
  // constructor
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
}


/* use auto generated - for more information check the header file

SmartLtmGridMap const &SmartLtmGridMap::operator=(SmartLtmGridMap const &other)
{
  // assignment operator
  if (this != &other)
  {
    idl_CommGridMap = other.idl_CommGridMap;
    idl_CommGridMap.cell.length(idl_CommGridMap.size);

    for ( unsigned int i=0; i < idl_CommGridMap.size; i++)
    {
      idl_CommGridMap.cell[i] = other.idl_CommGridMap.cell[i];
    }

  }
  return (*this);
}


SmartLtmGridMap::SmartLtmGridMap(SmartLtmGridMap const &other)
{
  // copy constructor
  idl_CommGridMap = other.idl_CommGridMap;
  idl_CommGridMap.cell.length(idl_CommGridMap.size);

  for ( unsigned int i=0; i < idl_CommGridMap.size; i++)
  {
    idl_CommGridMap.cell[i] = other.idl_CommGridMap.cell[i];
  }
}
*/

SmartLtmGridMap::~SmartLtmGridMap()
{
  // destructor
}


int SmartLtmGridMap::occupyLine(int x1,int y1,int x2,int y2,int n)
{
  int     x,y,z,dx,dy,dz,i1,i2;
  int     xa,ya,xs,ys;
  int     index;

  xa = (int)(floor(x1/(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.xOffsetCells);
  ya = (int)(floor(y1/(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.yOffsetCells);
  xs = (int)(floor(x2/(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.xOffsetCells);
  ys = (int)(floor(y2/(double)idl_CommGridMap.cellSizeMM)-(double)idl_CommGridMap.yOffsetCells);

  dx = abs(xa-xs);
  dy = abs(ya-ys);

  if (xa<xs)
  {
    x = xa;
    y = ya;
    if (ya>ys) z=-1; else z=1;
  }
  else
  {
    x = xs;
    y = ys;
    if (ys>ya) z=-1; else z=1;
  }

  if (dx>dy) i2=dx; else i2=dy;
  dz=i2/2;

  for (i1=0;i1<=i2;i1++)
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

    if ((0<=x) && (x < (int)idl_CommGridMap.xSizeCells) && (0<=y) && (y < (int)idl_CommGridMap.ySizeCells))
    {
      index      = x + y * idl_CommGridMap.xSizeCells;
      idl_CommGridMap.cell[index] = (unsigned char)n;
    }
  }
  return 0;
}


int SmartLtmGridMap::update( CommBasicObjects::CommMobileLaserScan const &scan, double n )
{
  int xa,ya,xs,ys;
  //double a;
  double x, y, z; // x1, y1;
  unsigned int i;
  int status = -1;
  //double laserPolarAngleRad; // = laserPolarAngle*M_PI/180.0;
  //double laserPolarDistance; // = 1000.0*lp[j];

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

    status = bresenham( xa, ya, xs, ys, n);
  }
  return status;
}


int SmartLtmGridMap::clearMap(int value)
{
  unsigned int x,y;

  for (x=0; x < idl_CommGridMap.xSizeCells; x++)
  {
    for (y=0; y < idl_CommGridMap.ySizeCells; y++)
    {
      idl_CommGridMap.cell[x+y * idl_CommGridMap.xSizeCells] = value;
    }
  }

  return drawBorder();
}


int SmartLtmGridMap::getPartialMap( int partXoffset,int partYoffset,
                                    int partXsize,int partYsize,
                                    int threshold,
                     CommNavigationObjectsIDL::CommGridMap_cell_type &destCell)
{

  int xCells,yCells;           // number of cells to be copied
  int xbCells,ybCells;         // begin of cells to be copied in LTM
  int sourceIndex,destIndex;
  int sourceValue,destValue;

  xCells  = (int)ceil(    (double)partXsize  / (double)idl_CommGridMap.cellSizeMM);
  yCells  = (int)ceil(    (double)partYsize  / (double)idl_CommGridMap.cellSizeMM);
  xbCells = (int)floor( ( (double)partXoffset/ (double)idl_CommGridMap.cellSizeMM) - (double)idl_CommGridMap.xOffsetCells);
  ybCells = (int)floor( ( (double)partYoffset/ (double)idl_CommGridMap.cellSizeMM) - (double)idl_CommGridMap.yOffsetCells);

  for (int y=0; y < yCells; y++)
  {
    for (int x=0; x < xCells; x++)
    {
      sourceIndex = (x+xbCells) + (y+ybCells) * idl_CommGridMap.xSizeCells;
      destIndex   = (x+y*xCells);
      sourceValue = (int)idl_CommGridMap.cell[sourceIndex];
      destValue   = (int)destCell[destIndex];


      if (destValue == MAPPER_GROWING)
      {
        // destination cell will not be overwritten !
      }
      else if (sourceValue == MAPPER_UNDELETABLE)
      {
    	  destCell[destIndex] = MAPPER_UNDELETABLE;
      }
      else if (sourceValue == MAPPER_UNKNOWN)
      {
    	  destCell[destIndex] = MAPPER_UNKNOWN;
      }
      else
      {
        if (sourceValue < threshold)
        {
          // mark destination cell as free
          destCell[destIndex] = MAPPER_FREE;
        }
        else
        {
          // mark destination cell as occupied
          destCell[destIndex] = MAPPER_OBSTACLE;
        }
      }

    }


  }

  return 0;
}



void SmartLtmGridMap::save_yaml_pgm(std::ostream &os_yaml, std::string pgmFileName)
{
#ifdef WITH_OPENCV_YAML
	// 1. write map context information to yaml file
	write_yaml(os_yaml, pgmFileName);

	// 2. write map data to pgm file
	const unsigned int map_size_x = idl_CommGridMap.xSizeCells;
	const unsigned int map_size_y = idl_CommGridMap.ySizeCells;

	// create a single channel image with the same size and double depth as the map
	//	IplImage* image = cvCreateImage(cvSize(map_size_x, map_size_y), IPL_DEPTH_64F, 1);
	IplImage* image = cvCreateImage(cvSize(map_size_x, map_size_y), IPL_DEPTH_8U, 1);

	// copy map into a image
	// We consider and whiter pixels free and blacker pixles as occupied. Therefore we set:
	const char maxPixelValue = 255;
	const char occupied = 0;
	const char freeSpace = maxPixelValue;
	int scaleFactor = 2; // in smartsoft we use occupancy values from 0 to 127. for storing the values in a image we have to scale them from 0 to 255 --> scaleFactor 2

	int value = 0;
	for (unsigned int y = 0; y < map_size_y; y++)
	{
		for (unsigned int x = 0; x < map_size_x; x++)
		{
			char* ptr = image->imageData + (y * image->widthStep) + x;
			value = this->get_cells(x, y);

			if(value >= MAPPER_FREE && value <= 127) // TODO: we should define the maximum value for occupancy by a #define
			{
				*ptr = (char)(maxPixelValue - (value * scaleFactor));
			}
			else if(value == MAPPER_UNDELETABLE) // special map value MAPPER_UNDELETABLE
			{
				*ptr = occupied;
			}
			else // we export all other special map values as free space
			{
				*ptr = freeSpace;
			}

		}
	}
	cvFlip(image, NULL, 0); // 0 == horizontal flip

	// save image to file pgmFileName
	if(!cvSaveImage(pgmFileName.c_str(), image))
	{
		cout << "Could not save: " << pgmFileName << endl;
	}

	// delete temporary image
	cvReleaseImage(&image);

#else
	std::cout << "!!! For use with Robotino the parts (save_yaml_pgm) using OpenCV are deactivated! In order to use them define WITH_OPENCV_YAML in the makefile. !!!" << std::endl;
#endif
}


void SmartLtmGridMap::save_yaml_ppm(std::ostream &os_yaml, std::string ppmFileName)
{
#ifdef WITH_OPENCV_YAML
	// 1. write map context information to yaml file
	write_yaml(os_yaml, ppmFileName);

	// 2. write map data to ppm file
	const unsigned int map_size_x = idl_CommGridMap.xSizeCells;
	const unsigned int map_size_y = idl_CommGridMap.ySizeCells;

	// create a three channel image with the same size and double depth as the map
	//	IplImage* image = cvCreateImage(cvSize(map_size_x, map_size_y), IPL_DEPTH_64F, 3);
	IplImage* image = cvCreateImage(cvSize(map_size_x, map_size_y), IPL_DEPTH_8U, 3);

	// copy map into a image
	// We consider and whiter pixels free and blacker pixles as occupied. Therefore we set:
	const char maxPixelValue = 255;
	int scaleFactor = 2; // in smartsoft we use occupancy values from 0 to 127. for storing the values in a image we have to scale them from 0 to 255 --> scaleFactor 2

	unsigned char* pixels = NULL;
	unsigned char* p = NULL;
	int rowstride, n_channels = 0;

	// Get values that we'll need to iterate through the pixels
	rowstride = image->widthStep;
	n_channels = image->nChannels;
	pixels = (unsigned char*) (image->imageData);

	// define colors for special values. Warning the channel order is BGR and not RGB
	int red[3];
	red[0] = 0;
	red[1] = 0;
	red[2] = 255;

	int freeSpace[3];
	freeSpace[0] = 0;
	freeSpace[1] = 0;
	freeSpace[2] = 0;

	int value = 0;
	char channelValue = 0;
	for (unsigned int y = 0; y < map_size_y; y++)
	{
		for (unsigned int x = 0; x < map_size_x; x++)
		{
			// get cell value
			value = this->get_cells(x, y);
			// get pixel
			p = pixels + y * rowstride + x * n_channels;

			if(value >= MAPPER_FREE && value <= 127) // TODO: we should define the maximum value for occupancy by a #define
			{
				channelValue = (char)(maxPixelValue - (value * scaleFactor));
				p[0] = channelValue;
				p[1] = channelValue;
				p[2] = channelValue;
			}
			else if(value == MAPPER_UNDELETABLE) // special map value MAPPER_UNDELETABLE
			{
				p[0] = red[0];
				p[1] = red[1];
				p[2] = red[2];
			}
			else // we export all other special map values as free space
			{
				p[0] = freeSpace[0];
				p[1] = freeSpace[1];
				p[2] = freeSpace[2];
			}
		}
	}
	cvFlip(image, NULL, 0); // 0 == horizontal flip

	// save image to file pgmFileName
	if(!cvSaveImage(ppmFileName.c_str(), image))
	{
		cout << "Could not save: " << ppmFileName << endl;
	}

	// delete temporary image
	cvReleaseImage(&image);
#else
	std::cout << "!!! For use with Robotino the parts (save_yaml_ppm) using OpenCV are deactivated! In order to use them define WITH_OPENCV_YAML in the makefile. !!!" << std::endl;
#endif
}

int SmartLtmGridMap::load_yaml( std::string fname)
{
#ifdef WITH_OPENCV_YAML
	// Overall load process
	// 1. read yaml file
	// 2. read the map data from image file

        std::ifstream is(fname.c_str());
        if (is.fail()) {
                std::cerr << "[MAPPER] Could not open " << fname << "\n";
                return -1;
        }


	// 1. read yaml file
	YAML::Parser parser(is);
	YAML::Node doc;
	parser.GetNextDocument(doc);

        is.close();

	double res;
	int negate;
	double free_th;
	std::string mapfname = "";
	double origin[3];

	try {
		doc["resolution"] >> res;
	} catch (YAML::InvalidScalar) {
		std::cerr << "[MAPPER] The map does not contain a resolution tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["negate"] >> negate;
	} catch (YAML::InvalidScalar) {
		std::cerr << "[MAPPER] The map does not contain a negate tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["free_thresh"] >> free_th;
	} catch (YAML::InvalidScalar) {
		std::cerr << "[MAPPER] The map does not contain a free_thresh tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["origin"][0] >> origin[0];
		doc["origin"][1] >> origin[1];
		doc["origin"][2] >> origin[2];
	} catch (YAML::InvalidScalar) {
		std::cerr << "[MAPPER] The map does not contain an origin tag or it is invalid.\n";
		return -1;
	}
	try {
		doc["image"] >> mapfname;
		// TODO: make this path-handling more robust
		if (mapfname.size() == 0) {
			std::cerr << "[MAPPER] The image tag cannot be an empty string.\n";
			return -1;
		}
                if (mapfname[0] != '/') {
                        // dirname can modify what you pass it
                        char* fname_copy = strdup(fname.c_str());

#if defined (__GNUC__) && defined(__unix__)
						mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
#elif defined (WIN32) || defined (WIN64)
						char dir[_MAX_DIR];
						_splitpath(fname_copy, NULL, dir, NULL, NULL);
						mapfname = std::string(dir) + '/' + mapfname;
#endif
                        free(fname_copy);
                }
	} catch (YAML::InvalidScalar) {
		std::cerr << "[MAPPER] The map does not contain an image tag or it is invalid.\n";
		return -1;
	}


	// 2. read the map data from image file
	unsigned char* pixels;
	unsigned char* p;
	int rowstride, n_channels;
	int i, j;
	int k;
	int color_sum;
	int cellValue;
	double color_avg;

	IplImage* img = NULL;

        std::ifstream isMap_tmp(mapfname.c_str());
        if (isMap_tmp.fail()) {
                std::cerr << "[MAPPER] Could not open " << mapfname << "\n";
                return -1;
        } else {
		isMap_tmp.close();
	}

	// Load the image using OpenCV.  If we get NULL back, the image load failed.
	if (!(img = cvLoadImage(mapfname.c_str(), CV_LOAD_IMAGE_UNCHANGED)))
	{
		std::string errmsg = std::string("failed to open image file \"") + mapfname + std::string("\"");
		throw std::runtime_error(errmsg);
	}

	long x = img->width;
	long y = img->height;
	const double scaleFactorMMtoM = 1000; // scale factor for converting from mm to m

	// store context information
	idl_CommGridMap.id = 0;
	idl_CommGridMap.is_valid = true;
	idl_CommGridMap.time.sec = 0;
	idl_CommGridMap.time.usec = 0;
	idl_CommGridMap.xOffsetMM = (int)floor(origin[0] * scaleFactorMMtoM );	// UNTESTED
	idl_CommGridMap.yOffsetMM = (int)floor(origin[1] * scaleFactorMMtoM );	// UNTESTED
	idl_CommGridMap.xOffsetCells = (int)floor(origin[0] / res);	// UNTESTED
	idl_CommGridMap.yOffsetCells = (int)floor(origin[1] / res);	// UNTESTED
	idl_CommGridMap.cellSizeMM = (int)floor(res * scaleFactorMMtoM ); 		// UNTESTED
	idl_CommGridMap.xSizeMM = (int)floor(x * res * scaleFactorMMtoM );
	idl_CommGridMap.ySizeMM = (int)floor(y * res * scaleFactorMMtoM );
	idl_CommGridMap.xSizeCells = (int)ceil(x);
	idl_CommGridMap.ySizeCells = (int)ceil(y);
	idl_CommGridMap.size = idl_CommGridMap.xSizeCells*idl_CommGridMap.ySizeCells;

	idl_CommGridMap.cell.resize(idl_CommGridMap.size);

	// Get values that we'll need to iterate through the pixels
	rowstride = img->widthStep;
	n_channels = img->nChannels;

	// Copy pixel data into the map structure
	pixels = (unsigned char*) (img->imageData);

	int r = 0;
	int g = 0;
	int b = 0;

	if(n_channels == 3) // is it a RGB image?
	{
		for (j = 0; j < img->height; j++) {
			for (i = 0; i < img->width; i++) {

				// get pixel
				p = pixels + j * rowstride + i * n_channels;

				// in OpenCV the channel order is BGR and not RGB
				r = p[2];
				g = p[1];
				b = p[0];

				// check if the pixel has a special value
				if(r == 255 && g == 0 && b == 0) // R=255, G=0, B = 0 --> red (undeletable)
				{
					cellValue = MAPPER_UNDELETABLE;
				}
//				Place to handle additional special vaulues
				else if(r == 205 && g == 205 && b == 205) // Unknown
				{
					cellValue = MAPPER_UNKNOWN;
				}
//				else if(r == 0 && g == 0 && b == 255) // R=0, G=0, B=255 --> ???
//				{
//
//				}
				else // no special values
				{
					// Compute mean of RGB for this pixel
					color_sum = 0;
					for (k = 0; k < n_channels; k++)
					{
						color_sum += *(p + k);
					}
					color_avg = color_sum / (double) n_channels;

					// If negate is true, we consider blacker pixels free, and whiter
					// pixels free.  Otherwise, it's vice versa.
					// smartLtmGridMap use only cell values from 0 to 127 therefore we have
					// to downscale by factor 2.
					if (negate)
					{
						cellValue = floor(color_avg / 2);
					}
					else
					{
						cellValue = floor((255 - color_avg) / 2);
					}
				}
				// store the cell value in the map. The y-axis point in image coordinates into the opposite direction.
				// Therfore we have to count j down from img->height-1 to 0.
				idl_CommGridMap.cell[i + ((img->height-1) - j)* idl_CommGridMap.xSizeCells] = cellValue;
			}
		}
	}
	else if(n_channels == 1) // is it a greyscale image?
	{
		for (j = 0; j < img->height; j++) {
			for (i = 0; i < img->width; i++) {

				// get pixel
				p = pixels + j * rowstride + i * n_channels;
				color_sum = *p;

				if(color_sum == MAPPER_UNKNOWN){
					cellValue = MAPPER_UNKNOWN;
				} else {

					// If negate is true, we consider blacker pixels free, and whiter
					// pixels free.  Otherwise, it's vice versa.
					// smartLtmGridMap use only cell values from 0 to 127 therefore we have
					// to downscale by factor 2.
					if (negate)
					{
						cellValue = floor(color_sum / 2);
					}
					else
					{
						cellValue = floor((255 - color_sum) / 2);
					}
				}


				// store the cell value in the map. The y-axis point in image coordinates into the opposite direction.
				// Therfore we have to count j down from img->height-1 to 0.
				idl_CommGridMap.cell[i + ((img->height-1) - j) * idl_CommGridMap.xSizeCells] = cellValue;
			}
		}
	}

	// clean up the temporary image
	cvReleaseImage(&img);
	return 0;
#else
	std::cout << "!!! For use without OpenCV the yaml based functions are deactivated! In order to use them define WITH_OPENCV_YAML in the makefile. !!!" << std::endl;
#endif

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRIVATE MEMBERS


void SmartLtmGridMap::write_ascii(std::string filename){

	  ofstream mapfile;
	  mapfile.open (filename.c_str());


	unsigned int x,y;

	for (x=0; x < idl_CommGridMap.xSizeCells; x++)
	{
		for (y=0; y < idl_CommGridMap.ySizeCells; y++)
		{

			int value = (int)idl_CommGridMap.cell[x+y * idl_CommGridMap.xSizeCells];
			mapfile << " "<< value;
			if(value == MAPPER_UNKNOWN){
				mapfile << "u";
			}

		}
		mapfile <<"\n";
	}

	mapfile.close();

}


int SmartLtmGridMap::drawBorder(void)
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

int SmartLtmGridMap::bresenham(int xa,int ya,int xs,int ys,double n)
{
  int     x,y,z,dx,dy,dz,i1,i2;
  int     index;
  double  oldvalue;
  double  update1,update2;
  double  free,occupied;

  free     = MAPPER_FREE;
  occupied = MAPPER_OBSTACLE;

  dx = abs(xa-xs);
  dy = abs(ya-ys);

  if (xa < xs)
  {
    x = xa;
    y = ya;
    update1 = free;
    update2 = occupied;
    if (ya>ys) z=-1; else z=1;
  }
  else
  {
    x = xs;
    y = ys;
    update1 = occupied;
    update2 = free;
    if (ys>ya) z=-1; else z=1;
  }

  if (dx > dy) i2=dx;
  else i2=dy;

  dz = i2/2;
  index      = x + y * idl_CommGridMap.xSizeCells;

  if ( (0<=x) && (x < (int)idl_CommGridMap.xSizeCells) && (0<=y) && (y < (int)idl_CommGridMap.ySizeCells) )
  {
     //hochdorfer lutz 2011-01-25 added MAPPER_UNDELETABLE
     //ignore cells with value MAPPER_UNDELETABLE
     if ( (idl_CommGridMap.cell[index] != MAPPER_UNDELETABLE))
     {
    	oldvalue   = (double)idl_CommGridMap.cell[index];
    	idl_CommGridMap.cell[index] = (unsigned char)(oldvalue + floor(1/n*(update1-oldvalue)));
     }
  }

  for ( i1=1; i1 <= i2; i1++)
  {
    if (dz < dx)
    {
      dz = dz+dy;
      x = x+1;
    }

    if (dz >= dx)
    {
      dz = dz-dx;
      y = y+z;
    }

    index      = x + y * idl_CommGridMap.xSizeCells;

    if ( (0<=x) && (x < (int)idl_CommGridMap.xSizeCells) && (0<=y) && (y < (int)idl_CommGridMap.ySizeCells) )
    {
      //hochdorfer lutz 2011-01-25 added MAPPER_UNDELETABLE
      //ignore cells with value MAPPER_UNDELETABLE
      if ( (idl_CommGridMap.cell[index] != MAPPER_UNDELETABLE))
      {
         if (i1!=i2)
         {
           oldvalue   = (double)idl_CommGridMap.cell[index];
           idl_CommGridMap.cell[index] = (unsigned char)(oldvalue+floor(1/n*(free-oldvalue)));
	 }

        if (i1==i2)
        {
          oldvalue   = (double)idl_CommGridMap.cell[index];
          idl_CommGridMap.cell[index] = (unsigned char)(oldvalue+floor(1/n*(update2-oldvalue)));
        }
      }
    }
  } // for

  return 0;
}

void SmartLtmGridMap::write_yaml(std::ostream &os_yaml, std::string mapFileName)
{
	const double scaleFactorMMtoM = 1000; // scale factor for converting from mm to m

	const double res = (double)(idl_CommGridMap.cellSizeMM / scaleFactorMMtoM); // convert from mm to m
	const int negate = 0;		// in smartsoft we consider wither pixels free
	const double occ_th = 0.65;	// WARNING: in smartsoft we dont use for occupied and free different thersholds. Therefore we set this default values.
	const double free_th = 0.196;	// WARNING: in smartsoft we dont use for occupied and free different thersholds. Therefore we set this default values.
	double origin[3];
	origin[0] = idl_CommGridMap.xOffsetMM / scaleFactorMMtoM;
	origin[1] = idl_CommGridMap.yOffsetMM / scaleFactorMMtoM;
	origin[2] = 0.0;
	const std::string mapfname = mapFileName;

	try
	{
/*
		YAML::Writer w(os_yaml);
		w.Scalar("resolution", res);
		w.Scalar("negate", negate);
		w.Scalar("occupied_thresh", occ_th);
		w.Scalar("free_thresh", free_th);
		w.Seq("origin", origin, origin + sizeof(origin)/sizeof(origin[0]));
		w.Scalar("image", mapFileName);
*/

/*
		YAML::Emitter out;
		out << YAML::BeginMap;
		out << YAML::Key << "resolution";  // should we instead have a one-off method for creating a key/value pair?
		out << YAML::Value << res;

		out << YAML::Key << "negate";
		out << YAML::Value << negate;

		out << YAML::Key << "occupied_thresh";
		out << YAML::Value << occ_th;

		out << YAML::Key << "free_thresh";
		out << YAML::Value << free_th;

		out << YAML::Key << "origin";
		out << YAML::Flow;
		out << YAML::BeginSeq << origin[0] << origin[1] << origin[2] << YAML::EndSeq;

		out << YAML::Key << "image";
		out << YAML::Value << mapFileName;

		out << YAML::EndMap;

		std::string output = out.c_str();
		std::cout << "map content" << output << endl;
		os_yaml << output;
*/
		// write yaml file using a hack quite similar to the hack in ROS  http://www.ros.org/doc/api/map_server/html/map__saver_8cpp_source.html
		// fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n", mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);
		os_yaml << "resolution: " << res << endl;
		os_yaml << "negate: " << negate << endl;
		os_yaml << "occupied_thresh: " << occ_th << endl;
		os_yaml << "free_thresh: " << free_th << endl;
		os_yaml << "origin: [" << origin[0] << ", " << origin[1] << ", " << origin[2] << "]" << endl;
		os_yaml << "image: " << mapFileName << endl;

	}
	catch(...)
	{
		std::cerr << "ERROR: writing yaml file"<< endl;
		exit(-1);
	}

}
