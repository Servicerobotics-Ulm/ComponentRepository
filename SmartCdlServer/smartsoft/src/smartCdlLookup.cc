// --------------------------------------------------------------------------
//
//  Copyright (C) 1998/2008 Christian Schlegel, Andreas Steck
//		  2013 Matthias Lutz, Dennis Stampfer
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

#include "smartCdlLookup.hh"
#include <iostream>
#include <iomanip>
#include <set>
#include "EulerTransformationMatrices.hh"
#include <iterator>
#include "SmartCdlServer.hh"
#include <zlib.h>


#define dbg(x) std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << " " << (#x) << " = " << x << std::endl;
#define dbgline() std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << std::endl;
#define dbg_pause(msg) {std::cout << "### PAUSE@" << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << ": " << msg; std::cin.ignore(); }

CommBasicObjects::CommBasePose basePosition_old;
int imgCntTMP = 0;


// global variables to calculate the sum of x/y movements. obstacles not seen 
// by the laser will be projected with these sums once they are larger than the cellsize.
// -> local history for CDL
double t_x_sum = 0;
double t_y_sum = 0;
double t_alpha_sum = 0;


#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	//functions to show some point:
	#define show_px(x, y) { cvCircle(img, cvPoint((-cdl_c_y_min + -y) * cdl_to_pixel_scale, (cdl_c_x_max + -x) * cdl_to_pixel_scale), dotsize, CV_RGB(0, 255, 0), fill); }
	#define show_px_color(x, y, color) { cvCircle(img, cvPoint((-cdl_c_y_min + -y) * cdl_to_pixel_scale, (cdl_c_x_max + -x) * cdl_to_pixel_scale), dotsize, color, fill); }

#endif


void transformWorldPointToRobot_LookUP(double wx, double wy, double wa, double px, double py, double &lx, double &ly) {
					double cos_a = cos(wa);
					double sin_a = sin(wa);
					lx = (px - wx) * cos_a + (py - wy) * sin_a;
					ly =-(px - wx) * sin_a + (py - wy) * cos_a;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -PI <= a <= PI
double piToPiRad( double a )
{
  a+=M_PI;
  bool was_neg = a<0;
  a = fmod(a, 2*M_PI );
  if (was_neg) a+=2*M_PI;
  a-=M_PI;
  return a;
}


void CdlLookupClass::initLookup(double pathNavRecover_max_dist, double pathNavPredictedGoalPose_minDist,
		            std::pair<double,double> PathNavPredictedGoalPose_controll1,
		            std::pair<double,double> PathNavPredictedGoalPose_controll2,
		            std::pair<double,double> PathNavPredictedGoalPose_controll3){

#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	  cdl_to_pixel_scale = 0.2;
	  if(cdl_c_res <=25)
	  {
	    dotsize =2;
	    fill = -1;
	  }else if(cdl_c_res>=50)
          {
	    dotsize = 2;
	    fill = 2;
	  }

	  pixelX = (double)(-cdl_c_y_min + cdl_c_y_max) * cdl_to_pixel_scale;
	  pixelY = (double)(-cdl_c_x_min + cdl_c_x_max) * cdl_to_pixel_scale;
	  gridsize = (double)cdl_c_res * cdl_to_pixel_scale;
#endif

	  // this array contains the remaining distance information
	  distLookup = new int** [cdl_max_x_cells];
	  for(unsigned int i = 0; i<cdl_max_x_cells; i++)
	  {
		  distLookup[i] = new int* [cdl_max_y_cells];
		  for(unsigned int j = 0; j<cdl_max_y_cells; j++)
		  {
			  distLookup[i][j] = new int [cdl_max_curvature];
		  }
	  }


	  // this array contains the remaining angle information in rad*1000.0
	  alphaLookup = new int** [cdl_max_x_cells];
	  for(unsigned int i = 0; i<cdl_max_x_cells; i++){
		  alphaLookup[i] = new int* [cdl_max_y_cells];
		  for(unsigned int j = 0; j<cdl_max_y_cells; j++)
		  {
			  alphaLookup[i][j] = new int [cdl_max_curvature];
		  }

	  }

	  // this array contains the maximum allowed rotational acceleration
	  rotAccLookup = new double [cdl_max_curvature];

	  // this array contains the maximum allowed translational acceleration
	  transAccLookup = new double[cdl_max_curvature];

	  // contains curvature index corresponding to v,w
	  indexVW = new int*[cdl_max_tra_cells];
	  for(unsigned int i = 0; i<cdl_max_tra_cells; i++)
	  {
		  indexVW[i] = new int[cdl_max_rot_cells];
	  }

	  // contains calculated costs of this curvature
	  evaluation = new evaluationStruct*[cdl_max_tra_cells];
	  for(unsigned int i = 0; i<cdl_max_tra_cells; i++)
	  {
		  evaluation[i] = new evaluationStruct[cdl_max_rot_cells];
	  }

	  // indicates whether this index has already been inserted into indexList
	  indexFlag = new int[cdl_max_curvature];

	  // for test purposes only (CDL_EVAL_APPROACH_EXACT) !!!!
	  // 0 this curvature is not wanted
	  // 1 this curvature is ok
	  // this array is used to indicate, whether this curvature should be used
	  // or not
	  indexWanted = new int[cdl_max_curvature];


	  indexWantedVWCheck = new int[cdl_max_curvature];

	  // list of indizes in the current velocity window
	  indexList = new int [cdl_max_curvature];

	  // the remaining distances with respect to the curvature index
	  distanceList = new int [cdl_max_curvature];

	  // the remaining rotation angle with respect to the curvature index
	  // in [rad * 1000]
	  alphaList = new int [cdl_max_curvature];

	  _distlookControl.push_back( PathNavPredictedGoalPose_controll1 );
	  _distlookControl.push_back( PathNavPredictedGoalPose_controll2 );
	  _distlookControl.push_back( PathNavPredictedGoalPose_controll3 );
	  this->pathNavPredictedGoalPose_minDist = pathNavPredictedGoalPose_minDist;
	  this->pathNavRecover_max_dist = pathNavRecover_max_dist;

}


// -----------------------------------------------------------------
// Calculates intersection of two lines if available
//
// input lines are assumed as ax+by+c=0
// input    a1,b1,c1         Parameter line 1
// input    a2,b2,c2         Parameter line 2
// output   x,y              line intersection point
// output   status  CDL_OK   found intersection
//                  CDL_INF  lines are equal, infinity intersections
//                  CDL_NO   lines are parallel, no intersection
// -----------------------------------------------------------------
int CdlLookupClass::cdl_line_intersection(double a1,double b1,double c1,
                          double a2,double b2,double c2,
                          double &x,double &y)
{
  int status;

  if ((a1*b2-a2*b1) == 0.0) {
    // parallel lines
    status = -1;
  } else if ((a2==0.0) && (b2==0.0) && (c2==0.0)) {
    // equal lines
    status = 1;
  } else if ((a1==0.0) && (b1==0.0) && (c1==0.0)) {
    // equal lines
    status = 1;
  } else if ((a1/a2==b1/b2) && (a1/a2==c1/c2)) {
    // equal lines
    status = 1;
  } else {
    // lines not parallel
    x = (b1*c2-b2*c1)/(a1*b2-a2*b1);
    y = (c1*a2-c2*a1)/(a1*b2-a2*b1);
    status = 0;
  }
  return status;
}


double CdlLookupClass::linearinterpolation(const std::vector<std::pair<double, double> >& vec, const double x )
{
	std::vector<std::pair<double, double> >::const_iterator iter, end;
	iter = vec.begin();
	end = vec.end();

	double y = 0.0;

	if( iter != end )
	{
		if( x < (*iter).first )
		{
			y = (*iter).second;
		}
		else
		{
			while( end != iter )
			{
				const std::pair<double, double>& p = (*iter);
				++iter;

				if( x >= p.first )
				{
					if( end != iter )
					{
						if( x < (*iter).first )
						{
							const std::pair<double, double>& p2 = (*iter);
							double dx = p2.first - p.first;
							double dy = p2.second - p.second;

							if( 0.0 == dx )
							{
								y = p.second;
							}
							else
							{
								double a = dy / dx;
								y = a * ( x - p.first ) + p.second;
							}
						}
					}
					else
					{
						y = p.second;
					}
				}
			}
		}
	}

	return y;
}




// -----------------------------------------------------------------
// Transformation Line AB => ax+by=c
// Gradient          : b/a
// y-axis segment    : c
// return value      : OK/NOK
// -----------------------------------------------------------------
int CdlLookupClass::cdl_ab_axbyc(double pax,double pay,double pbx,double pby,
                 double &a,double &b,double &c)
{
  double      deltax,deltay;

  deltax = pbx - pax;
  deltay = pby - pay;
  if (fabs(deltax) < cdl_accuracy) {
    // parallel to y
    a = 1.0;
    b = 0.0;
    c = pax;
  } else if (fabs(deltay) < cdl_accuracy) {
    // parallel to x
    a = 0.0;
    b = 1.0;
    c = pay;
  } else if ((fabs(deltay) < cdl_accuracy) && (fabs(deltax) < cdl_accuracy)) {
    // distance A <-> B too small
    return -1;
  } else {
    a = -deltay/deltax;
    b = 1.0;
    c = a * pax + b * pay;
  }
  return 0;
}

// --------------------------------------------------------------
// check wether point p is inside polygon
//
// return: OK/NOK point is inside/outside -> true/false
// --------------------------------------------------------------
bool CdlLookupClass::cdl_check_point_inside_polygon(double x_p,double y_p)
{
  int     counter;
  double  x_help,y_help;
  double  t;
  double  x;
  int     i;

  counter=0;
  for (i=0;i<polygon.number_of_segments;i++) {
    if (polygon.line[i].y1 != polygon.line[i].y2) {
      if (polygon.line[i].y1 > polygon.line[i].y2) {
        x_help=polygon.line[i].x1;
        polygon.line[i].x1=polygon.line[i].x2;
        polygon.line[i].x2=x_help;
        y_help=polygon.line[i].y1;
        polygon.line[i].y1=polygon.line[i].y2;
        polygon.line[i].y2=y_help;
      }
      t=(y_p-polygon.line[i].y1)/(polygon.line[i].y2-polygon.line[i].y1);
      x=polygon.line[i].x1+t*(polygon.line[i].x2-polygon.line[i].x1);
      if ((0<=t) && (t<1.0) && (x<=x_p)) counter++;
    }
  }
  if ((counter%2)==0) return false; else return true;
}


// taken from CdlCalculate
bool CdlLookupClass::cdl_load_contour_ascii(char *filename)
{
  FILE   *file;
  int    num,i,format;

  file=fopen(filename,"r");
  if (file!=NULL) {
    // read first line
    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
    if (num!=1) {
      fprintf(stderr,"Error in line 1: unknown file format\n");
      fclose(file);
      return false;
    }

    // read second line
    num=fscanf(file,"Content : CDL CONTOUR\n");

    // read third line
    num=fscanf(file,"Comment : ---\n");

    num=fscanf(file,"CDL_OK : %d\n",&cdl_ok);
    num=fscanf(file,"CDL_INF : %d\n",&cdl_inf);
    num=fscanf(file,"CDL_NO : %d\n",&cdl_no);
    num=fscanf(file,"CDL_NOK : %d\n",&cdl_nok);
    num=fscanf(file,"CDL_ACCURACY : %f\n",&cdl_accuracy);
    //
    num=fscanf(file,"CDL_V_TRA_MAX : %f\n", &cdl_v_tra_max);
    num=fscanf(file,"CDL_V_TRA_MIN : %f\n", &cdl_v_tra_min);
    num=fscanf(file,"CDL_V_ROT_MAX : %f\n", &cdl_v_rot_max);
    num=fscanf(file,"CDL_V_ROT_MIN : %f\n", &cdl_v_rot_min);

    num=fscanf(file,"CDL_V_TRA_STEP : %f\n", &cdl_v_tra_step);
    num=fscanf(file,"CDL_V_ROT_STEP : %f\n", &cdl_v_rot_step);

    num=fscanf(file,"CDL_A_ROT_MAX : %f\n", &cdl_a_rot_max);
    num=fscanf(file,"CDL_A_TRA_MAX : %f\n", &cdl_a_tra_max);

    num=fscanf(file,"CDL_TRA_CELLS : %f\n", &cdl_tra_cells);
    num=fscanf(file,"CDL_ROT_CELLS : %f\n", &cdl_rot_cells);

    num=fscanf(file,"CDL_C_X_MIN : %f\n", &cdl_c_x_min);
    num=fscanf(file,"CDL_C_X_MAX : %f\n", &cdl_c_x_max);
    num=fscanf(file,"CDL_C_Y_MIN : %f\n", &cdl_c_y_min);
    num=fscanf(file,"CDL_C_Y_MAX : %f\n", &cdl_c_y_max);
    num=fscanf(file,"CDL_C_RES : %f\n", &cdl_c_res);

    num=fscanf(file,"CDL_MAX_DISTANCE : %f\n", &cdl_max_distance);
    num=fscanf(file,"CDL_CAPTURE_DISTANCE : %f\n", &cdl_capture_distance);

    num=fscanf(file,"CDL_C_X_CELLS : %f\n", &cdl_c_x_cells);
    num=fscanf(file,"CDL_C_Y_CELLS : %f\n", &cdl_c_y_cells);

    num=fscanf(file,"CDL_CURVATURE_INDICES : %f\n", &cdl_curvature_indices);
    num=fscanf(file,"CDL_ANGLE_STEP : %f\n", &cdl_angle_step);

    num=fscanf(file,"CDL_MAX_LINES : %d\n", &cdl_max_lines);
    num=fscanf(file,"CDL_MAX_TRA_CELLS : %d\n", &cdl_max_tra_cells);
    num=fscanf(file,"CDL_MAX_ROT_CELLS : %d\n", &cdl_max_rot_cells);
    num=fscanf(file,"CDL_MAX_X_CELLS : %d\n", &cdl_max_x_cells);
    num=fscanf(file,"CDL_MAX_Y_CELLS : %d\n", &cdl_max_y_cells);
    num=fscanf(file,"CDL_MAX_CURVATURE : %d\n", &cdl_max_curvature);

    std::cout<<"FILEs "<<
	 cdl_ok << " "<<
	 cdl_inf << " "<<
	 cdl_no << " "<<
	 cdl_nok << " "<<
	 cdl_accuracy << " "<<

	 cdl_v_tra_max << " "<<
	 cdl_v_tra_min << " "<<
	 cdl_v_rot_max << " "<<
	 cdl_v_rot_min << " "<<

	 cdl_v_tra_step << " "<<
	 cdl_v_rot_step << " "<<

	 cdl_a_rot_max << " "<<
	 cdl_a_tra_max << " "<<

	 cdl_tra_cells << " "<<
	 cdl_rot_cells << " "<<

	 cdl_c_x_min << " "<<
	 cdl_c_x_max << " "<<
	 cdl_c_y_min << " "<<
	 cdl_c_y_max << " "<<
	 cdl_c_res << " "<<

	 cdl_max_distance << " "<<
	 cdl_capture_distance << " "<<

	 cdl_c_x_cells << " "<<
	 cdl_c_y_cells << " "<<

	 cdl_curvature_indices << " "<<
	 cdl_angle_step << " "<<

	 cdl_max_lines << " "<<
	 cdl_max_tra_cells << " "<<
	 cdl_max_rot_cells << " "<<
	 cdl_max_x_cells << " "<<
	 cdl_max_y_cells << " "<<
	 cdl_max_curvature << " "<<std::endl;


     // read fourth line
	 num=fscanf(file,"NumberOfSegments : %d\n",&polygon.number_of_segments);
	 if (num!=1) {
	  fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
	  fclose(file);
	  return false;
	 }

	 polygon.line = new cdl_line_struct [cdl_max_lines];
	 //struct cdl_line_struct line[CDL_MAX_LINES];

	 // read data area
	 num=0;
	 for (i=0;i<polygon.number_of_segments;i++) {
	  num+=fscanf(file,"%lf %lf %lf %lf\n",&(polygon.line[i].x1),
 										   &(polygon.line[i].y1),
										   &(polygon.line[i].x2),
										   &(polygon.line[i].y2));
	 }
	 if (num!=(4*polygon.number_of_segments)) {
	  fclose(file);
	  return false;
	 }

    fclose(file);
    return true;
  }
  return false;
}





#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
// debug function to visualize CDL contour
void CdlLookupClass::drawCdlContour(IplImage *img, int cdl_x_max, int cdl_y_min, double cdl_to_pixel_scale, CvScalar color) {
	for(int i=0; i < polygon.number_of_segments; i++) {
		cvLine(img, 
			cvPoint((-cdl_y_min + -polygon.line[i].y1) * cdl_to_pixel_scale, (cdl_x_max + -polygon.line[i].x1) * cdl_to_pixel_scale), 
			cvPoint((-cdl_y_min + -polygon.line[i].y2) * cdl_to_pixel_scale, (cdl_x_max + -polygon.line[i].x2) * cdl_to_pixel_scale), 
			color,
			2);
	}
}
#endif



// does semantically the same as MRPT's inverseComposeFrom().
void inverseComposeFrom(arma::mat b_t, arma::mat b_r, arma::mat a_t, arma::mat a_r, arma::mat& out_t, arma::mat& out_r)
{

  out_r.fill(0);
  out_t.fill(0);

  arma::mat R_b_inv(3,3);
  arma::mat t_b_inv(3,1);

  const double tx = -b_t(0,0);
  const double ty = -b_t(1,0);
  const double tz = -b_t(2,0);

  t_b_inv(0,0) = tx*b_r(0,0)+ty*b_r(1,0)+tz*b_r(2,0);
  t_b_inv(1,0) = tx*b_r(0,1)+ty*b_r(1,1)+tz*b_r(2,1);
  t_b_inv(2,0) = tx*b_r(0,2)+ty*b_r(1,2)+tz*b_r(2,2);

  // 3x3 rotation part: transpose
  //out_R = b_r.adjoint();
  R_b_inv = arma::trans(b_r);

  arma::mat m_coords(3,1);
  arma::mat m_ROT(3,3);

  for (int i=0;i<3;i++)
    out_t(i,0) = t_b_inv(i,0) + R_b_inv(i,0)*a_t(0,0)+ R_b_inv(i,1)*a_t(1,0)+ R_b_inv(i,2)*a_t(2,0);

  // Rot part:
  out_r = R_b_inv * a_r;
}

// does semantically the same as MRPT's composeFrom().
void composeFrom(arma::mat b_t, arma::mat b_r, arma::mat a_t, arma::mat a_r, arma::mat& out_t, arma::mat& out_r)
{
  out_r.fill(0);
  out_t.fill(0);

  out_r = a_r *b_r;
  for (int i=0;i<3;i++){
	out_t(i,0) =  a_t(i,0)+ a_r(i,0)*b_t(0,0)+a_r(i,1)*b_t(1,0)+a_r(i,2)*b_t(2,0);
  }
}



// helper class to save laser hits
class XYCoord {
public:
	XYCoord(int x, int y) {
		this->x = x;
		this->y = y;
	}

	int x;
	int y;

	bool operator<(const XYCoord &c) const {
		if(this->x < c.x) 
			return true;
		else if(this->x == c.x && this->y < c.y)
			return true;
		else
			return false;
	}
};

std::set<XYCoord> grids_set;

// <asteck: date="21.07.2008">
#define EVAL_FUNC_1 1
//#define EVAL_FUNC_3 1
//#define EVAL_FUNC_4 1
// </asteck>

CdlLookupClass::CdlLookupClass()
{
	usePathBorders = false;
	usePathNav = false;
// constructor
};

CdlLookupClass::~CdlLookupClass()
{
// destructor
};

double CdlLookupClass::min00(double a,double b)
{
  if (a<b) return a;
  else return b;
}

double CdlLookupClass::max00(double a,double b)
{
  if (a<b) return b;
  else return a;
}

double CdlLookupClass::rad00(double a)
{
  return (a*M_PI/180.0);
}

double CdlLookupClass::deg00(double a)
{
  return (a*180.0/M_PI);
}

double CdlLookupClass::abs00(double a)
{
  if (a<0) return (-a);
  else return a;
}

double CdlLookupClass::angle00(double a)
{
  if (a < -M_PI || a >= M_PI)
    {
      a = a - 2*M_PI * floor(a / (2*M_PI));
      while (a < -M_PI) a += 2*M_PI;
      while (a >= M_PI) a -= 2*M_PI;
    }

  return(a);
}

int CdlLookupClass::intersectionsCircleLine
(double xm,double ym,double r,
 double pax,double pay,double pbx,double pby,
 double& x1,double& y1,double& x2,double& y2,
 int& ns)
{
  double deltax,deltay;
  double a,b,c;
  double ab2,dis,cc,wu;

  // convert line from 2 point description to ax+by=c
  deltax = pbx - pax;
  deltay = pby - pay;
  if (abs00(deltax) < cdl_accuracy)
    {
      // parallel to y
      a = 1.0;
      b = 0.0;
      c = pax;
    }
  else if (abs00(deltay) < cdl_accuracy)
    {
      // parallel to x
      a = 0.0;
      b = 1.0;
      c = pay;
    }
  else if ((abs00(deltay)<cdl_accuracy) && (abs00(deltax)<cdl_accuracy))
    {
      // distance A <-> B too small
      return 1;
    }
  else
    {
      a = -deltay/deltax;
      b = 1.0;
      c = a * pax + b * pay;
    }

  // calculate nr of intersections of line with circle
  ns  = 0;
  ab2 = a*a + b*b;
  if (ab2 != 0.0)
    {
      cc = c - a*xm - b*ym;
      dis = r*r*ab2 - cc*cc;
      if (dis >= 0.0)
        {
          ns = 2;
          wu = sqrt(dis);
          if (wu == 0.0) ns=1;
          if (b >= 0.0)
            {
              x1 = xm + (a*cc-b*wu)/ab2;
              y1 = ym + (b*cc+a*wu)/ab2;
              x2 = xm + (a*cc+b*wu)/ab2;
              y2 = ym + (b*cc-a*wu)/ab2;
            }
          else
            {
              x2 = xm + (a*cc-b*wu)/ab2;
              y2 = ym + (b*cc+a*wu)/ab2;
              x1 = xm + (a*cc+b*wu)/ab2;
              y1 = ym + (b*cc-a*wu)/ab2;
            }
        }
    }
  return 0;
}


int CdlLookupClass::freeBehavior(double securityDist,int &turnDirection)
{
  //<asteck: date="17.07.2008" type of variable changed to unsigned>
  unsigned int    index;
  // </asteck>
  int    occupiedRight;
  int    occupiedLeft;
  double distance;

  occupiedRight = 0;
  occupiedLeft  = 0;

  //<asteck: date="17.07.2008" variables added>
  double x,y,z;
  //</asteck>

  // examine left and right side of the robot
  for (index=0;index<scan.get_scan_size();index++)
    {
      // we need a robot coordinate system (independent of where the scanner is mounted on)
      scan.get_scan_cartesian_3dpoint_robot( index, x, y, z );
      distance = sqrt((x * x)
                      +(y * y));
      if ((y < 0.0) && (distance < securityDist))
        {
          occupiedRight = 1;
        }
      if ((y > 0.0) && (distance < securityDist))
        {
          occupiedLeft = 1;
        }
      //</asteck>
    }

  // now decide what to do next
  if ((occupiedLeft == 0) && (occupiedRight == 1))
    {
      // left free, right occupied ==> turn left
      turnDirection = CDL_FREE_TURN_LEFT;
    }
  else if ((occupiedLeft == 1) && (occupiedRight == 0))
    {
      // left occupied, right free ==> turn right
      turnDirection = CDL_FREE_TURN_RIGHT;
    }
  else if ((occupiedLeft == 1) && (occupiedRight == 1))
    {
      // both sides occupied ==> no turn, too dangerous
      turnDirection = CDL_FREE_NO_TURN;
    }
  else
    {
      // both sides free, this is strange, but then turn left
      turnDirection = CDL_FREE_BOTH;
    }
  return 0;
}


void CdlLookupClass::insertIRScanAsObstacle(const CommBasicObjects::CommMobileIRScan& scan, int indexCurvature){
	  unsigned int scanindex;
	  double x, y, z;
	  int ix,iy;
	  int px = 0;
	  int py = 0;

	  int vi,wi,ci,li;

	  for (scanindex=0;scanindex<irScan.get_scan_size();scanindex++){
	   // we need a robot coordinate system (independent of where the scanner is mounted on)
		  irScan.get_scan_cartesian_3dpoint_robot( scanindex, x, y, z );
	       if (x < 0.0)
	         {
	           ix = (int)(ceil((x-cdl_c_x_min)/cdl_c_res));
	 	  px = (ix * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
	         }
	       else
	         {
	           ix = (int)(floor((x-cdl_c_x_min)/cdl_c_res));
	 	  px = (ix * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
	         }
	       if (y < 0.0)
	         {
	           iy = (int)(ceil((y-cdl_c_y_min)/cdl_c_res));
	 	  py = (iy * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
	         }
	       else
	         {
	           iy = (int)(floor((y-cdl_c_y_min)/cdl_c_res));
	 	  py = (iy * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
	         }
	       //</asteck>
	 #if DEBUG_VW_WINDOW
	       if (file!=NULL)
	         {
	           fprintf(file,"scanindex %d ix iy %d %d scan.x scan.y %f %f\n",
	                   scanindex,ix,iy,x,y);
	         }
	 #endif

	       if ((ix>=0) && (ix<cdl_c_x_cells) && (iy>=0) && (iy<cdl_c_y_cells))
	         {
	     // NOT USED FOR IRSCAN
	 	 // if(use_obstacle_history) {
	 	 // 	grids_set.insert(XYCoord(px, py)); // insert new (discrete) laser points in (old)/global gridset
	 	 // }
	 #ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	           show_px_color(px, py, CV_RGB(0, 255, 0));
	 #endif

	           for (li=0;li<indexCurvature;li++)
	             {
	               ci=indexList[li];
	               if (distLookup[ix][iy][ci]<distanceList[ci])
	                 {
	                   distanceList[ci] = distLookup[ix][iy][ci];
	                 }
	               if (alphaLookup[ix][iy][ci]<alphaList[ci])
	                 {
	                   alphaList[ci]    = alphaLookup[ix][iy][ci];
	                 }
	             }
	         }
	      }
}

void CdlLookupClass::setPathNavPathList(std::vector< std::pair<double, double> > pathList, std::vector< double > pathWidth){
	pathNavList = pathList;
	pathWithList = pathWidth;

	//std::cout<<"Points list size: "<<pathNavList.size()<<std::endl;
}

void CdlLookupClass::insertLaserScanAsObstacle(const CommBasicObjects::CommMobileLaserScan& scan, int indexCurvature){

	  unsigned int scanindex;
	  double x, y, z;
	  int ix,iy;
	  int px = 0;
	  int py = 0;

	  int vi,wi,ci,li;


	  for (scanindex=0;scanindex<scan.get_scan_size();scanindex++){
	  // we need a robot coordinate system (independent of where the scanner is mounted on)
	      scan.get_scan_cartesian_3dpoint_robot( scanindex, x, y, z );
	      if (x < 0.0)
	        {
	          ix = (int)(ceil((x-cdl_c_x_min)/cdl_c_res));
		  px = (ix * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
	        }
	      else
	        {
	          ix = (int)(floor((x-cdl_c_x_min)/cdl_c_res));
		  px = (ix * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
	        }
	      if (y < 0.0)
	        {
	          iy = (int)(ceil((y-cdl_c_y_min)/cdl_c_res));
		  py = (iy * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
	        }
	      else
	        {
	          iy = (int)(floor((y-cdl_c_y_min)/cdl_c_res));
		  py = (iy * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
	        }
	      //</asteck>
	#if DEBUG_VW_WINDOW
	      if (file!=NULL)
	        {
	          fprintf(file,"scanindex %d ix iy %d %d scan.x scan.y %f %f\n",
	                  scanindex,ix,iy,x,y);
	        }
	#endif

	      if ((ix>=0) && (ix<cdl_c_x_cells) && (iy>=0) && (iy<cdl_c_y_cells))
	        {
		  if(use_obstacle_history) {
		  	grids_set.insert(XYCoord(px, py)); // insert new (discrete) laser points in (old)/global gridset
		  }
	#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	          show_px_color(px, py, CV_RGB(0, 255, 0));
	#endif

	          for (li=0;li<indexCurvature;li++)
	            {
	              ci=indexList[li];
	              if (distLookup[ix][iy][ci]<distanceList[ci])
	                {
	                  distanceList[ci] = distLookup[ix][iy][ci];
	                }
	              if (alphaLookup[ix][iy][ci]<alphaList[ci])
	                {
	                  alphaList[ci]    = alphaLookup[ix][iy][ci];
	                }
	            }
	        }
	     }
}


//void CdlLookupClass::bresenhamProjectPath(int x0,int y0,int x1,int y1,int width, double alpha, std::list< std::pair<double,double> >& points_list)
//{
//
//	int xs = x0;
//	int ys = y0;
//	int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
//	int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
//	int err = dx+dy, e2; /* error value e_xy */
//	arma::mat w_t(3,1);
//	w_t(0,0) = width;//x
//	w_t(1,0) = 0;//y
//
//	arma::mat zero_t(3,1);
//	zero_t.fill(0);
//	arma::mat zero_r(3,3);
//	zero_r.fill(0);
//	arma::mat w1_r(3,3);
//	arma::mat w2_r(3,3);
//
//	EulerTransformationMatrices::create_zyx_matrix(M_PI_2,0,0,w1_r);
//	EulerTransformationMatrices::create_zyx_matrix(-M_PI_2,0,0,w2_r);
//
//
//	for(;;){  /* loop */
//		//setPixel(x0,y0);
//		int pxg,pyg;
//		if (x0 < 0.0)
//		{
//			pxg = (x0 * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
//		}
//		else
//		{
//			pxg = (x0 * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
//		}
//		if (y0 < 0.0)
//		{
//			pyg = (y0 * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
//		}
//		else
//		{
//			pyg = (y0 * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
//		}
//
//
//		arma::mat p1(4,4);
//
//		arma::mat r1(4,4);
//		arma::mat r2(4,4);
//
//		//std::cout<<"xs/ys: "<<xs<<"/"<<ys<<" px/py: "<<pxg<<"/"<<pyg<<" Alpha: "<<alpha<<std::endl;
//
//		arma::mat p1_t(3,1);
//		p1_t(0,0) = pxg;//x
//		p1_t(1,0) = pyg;//y
//		arma::mat p1_r(3,3);
//		EulerTransformationMatrices::create_zyx_matrix(alpha,0,0,p1_r);
//
//		arma::mat res_t(3,1);
//		arma::mat res_r(3,3);
//
//		arma::mat res_t2(3,1);
//		arma::mat res_r2(3,3);
//		composeFrom(zero_t,w1_r, p1_t, p1_r, res_t,res_r);
//		composeFrom(w_t,zero_r,res_t,res_r,res_t,res_r);
//
//		composeFrom(zero_t,w2_r, p1_t, p1_r, res_t2,res_r2);
//		composeFrom(w_t,zero_r,res_t2,res_r2,res_t2,res_r2);
//
//
//		////////////////////////////////////////////////////////////
//
//		points_list.push_back(std::make_pair(res_t(0,0),res_t(1,0)));
//		points_list.push_back(std::make_pair(res_t2(0,0),res_t2(1,0)));
//
//		////////////////////////////////////////////////////////////
//#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
//		show_px_color(pxg, pyg, CV_RGB(0, 0, 0)); //middel line
//		show_px_color(res_t(0,0), res_t(1,0), CV_RGB(255, 125, 0)); // left
//		show_px_color(res_t2(0,0), res_t2(1,0), CV_RGB(0, 125, 255)); // rigth
//#endif
//
//		if (x0==x1 && y0==y1) break;
//		e2 = 2*err;
//		if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
//		if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
//	}
//
//}

void CdlLookupClass::bresenham(double x0,double y0,double x1,double y1, std::vector< std::pair<double,double> >& points_list)
{

	int ix0,iy0,ix1,iy1,px,py;
	mapPointToCDLGrid(x0,y0,ix0,iy0,px,py);
	mapPointToCDLGrid(x1,y1,ix1,iy1,px,py);


	int xs = ix0;
	int ys = iy0;
	int dx =  abs(ix1-ix0), sx = ix0<ix1 ? 1 : -1;
	int dy = -abs(iy1-iy0), sy = iy0<iy1 ? 1 : -1;
	int err = dx+dy, e2; /* error value e_xy */

	for(;;){  /* loop */
		//setPixel(x0,y0);
		int pxg,pyg;
		if (ix0 < 0.0)
		{
			pxg = (ix0 * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
		}
		else
		{
			pxg = (ix0 * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
		}
		if (y0 < 0.0)
		{
			pyg = (iy0 * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
		}
		else
		{
			pyg = (iy0 * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
		}


//		std::cout<<"xs/ys: "<<xs<<"/"<<ys<<" px/py: "<<pxg<<"/"<<pyg<<std::endl;

		////////////////////////////////////////////////////////////

		points_list.push_back(std::make_pair(pxg,pyg));

		////////////////////////////////////////////////////////////
#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
		show_px_color(pxg, pyg, CV_RGB(0, 0, 0)); //middel line
#endif

		if (ix0==ix1 && iy0==iy1) break;
		e2 = 2*err;
		if (e2 > dy) { err += dy; ix0 += sx; } /* e_xy+e_x > 0 */
		if (e2 < dx) { err += dx; iy0 += sy; } /* e_xy+e_y < 0 */
	}

}

void CdlLookupClass::mapPointToCDLGrid(double x, double y, int & ix,int& iy, int& px, int& py){

	if (x < 0.0)
	{
		ix = (int)(ceil((x-cdl_c_x_min)/cdl_c_res));
		px = (ix * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
	}
	else
	{
		ix = (int)(floor((x-cdl_c_x_min)/cdl_c_res));
		px = (ix * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
	}
	if (y < 0.0)
	{
		iy = (int)(ceil((y-cdl_c_y_min)/cdl_c_res));
		py = (iy * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
	}
	else
	{
		iy = (int)(floor((y-cdl_c_y_min)/cdl_c_res));
		py = (iy * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
	}

}

void CdlLookupClass::projectPointAngleDistance(double x,double y, double distance, double angle, double& outX, double& outY){

	arma::mat p_t(3,1);
	p_t(0,0) = x;//x
	p_t(1,0) = y;//y
	arma::mat p_r(3,3);
	EulerTransformationMatrices::create_zyx_matrix(angle,0,0,p_r);

	arma::mat zero_r(3,3);
	zero_r.fill(0);

	arma::mat w_t(3,1);
	w_t(0,0) = distance;//x
	w_t(1,0) = 0;//y
	arma::mat res_t(3,1);
	arma::mat res_r(3,3);
	composeFrom(w_t,zero_r, p_t, p_r, res_t,res_r);

	outX = res_t(0,0);
	outY = res_t(1,0);

}

void CdlLookupClass::projectPoseAngleDistance(double x,double y,double yaw, double distance, double angle, double& outX, double& outY){


	arma::mat zero_t(3,1);
	zero_t.fill(0);
	arma::mat zero_r(3,3);
	zero_r.fill(0);

	arma::mat p_t(3,1);
	p_t(0,0) = x;//x
	p_t(1,0) = y;//y
	arma::mat p_r(3,3);
	arma::mat w_r(3,3);
	EulerTransformationMatrices::create_zyx_matrix(yaw,0,0,p_r);
	EulerTransformationMatrices::create_zyx_matrix(angle,0,0,w_r);

	arma::mat w_t(3,1);
	w_t(0,0) = distance;//x
	w_t(1,0) = 0;//y
	arma::mat res_t(3,1);
	arma::mat res_r(3,3);
	composeFrom(zero_t,w_r, p_t, p_r, res_t,res_r);
	composeFrom(w_t,zero_r, res_t, res_r, res_t,res_r);

	outX = res_t(0,0);
	outY = res_t(1,0);

}

void CdlLookupClass::calculatePathBorderPoints(std::vector< std::pair<double,double> >& points_list){

	std::vector< std::pair<double,double> > leftOuterPointsList,rightOuterPointsList;
		std::vector< std::pair<double,double> > points_list_center_line;
		//std::vector< std::pair<double,double> > points_list;

		//	std::cout<<"PathNavListSize: "<<pathNavList.size()<<std::endl;

			//HACK to make pathWithList equal the size of pathNavList, to prevent the iterator from overshooting the end (last element will never be accessed)
			if(pathWithList.size() != pathNavList.size() ){
				pathWithList.push_back(-1);
			}
			std::vector <double>::iterator widthIt = pathWithList.begin();
			for (std::vector< std::pair<double,double> >::iterator it=pathNavList.begin();it != pathNavList.end();it++,widthIt++){

				// F I R S T   P O I N T
				std::vector< std::pair<double,double> >::iterator it_next = it;
				std::pair<double, double> firstPoint = *it;
				it_next++;
				if(it_next != pathNavList.end()){


					// S E C O N D   P O I N T
					std::pair<double, double> secondPoint = *it_next;
					//DRAW CENTER LINE - this is only for visualization
					bresenham(firstPoint.first, firstPoint.second, secondPoint.first, secondPoint.second, points_list_center_line);


					double alpha1 = atan2(firstPoint.second - secondPoint.second, firstPoint.first - secondPoint.first);

					double xout3,yout3,xout4,yout4;

					if(it != pathNavList.begin()){

						xout3 = rightOuterPointsList.back().first;
						yout3 = rightOuterPointsList.back().second;
						xout4 = leftOuterPointsList.back().first;
						yout4 = leftOuterPointsList.back().second;
						//projectPoseAngleDistance(firstPoint.first,firstPoint.second,alpha1, width_angle, theta, xout3, yout3);
						//projectPoseAngleDistance(firstPoint.first,firstPoint.second,alpha1, width_angle, -theta-M_PI, xout4, yout4);
					} else {
						//first element
						projectPoseAngleDistance(firstPoint.first,firstPoint.second,alpha1, (*widthIt*1000)/2, M_PI_2, xout3, yout3);
						projectPoseAngleDistance(firstPoint.first,firstPoint.second,alpha1, (*widthIt*1000)/2, -M_PI_2, xout4, yout4);
						rightOuterPointsList.push_back(std::make_pair(xout3,yout3));
						leftOuterPointsList.push_back(std::make_pair(xout4,yout4));
					}

	#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
						show_px_color(xout3, yout3, CV_RGB(255, 0, 0)); // Right
						show_px_color(xout4, yout4, CV_RGB(0, 255, 0)); // left
	#endif

					it_next++;
					if(it_next != pathNavList.end()){

						// T H I R D  P O I N T
						std::pair<double, double> thirdPoint = *it_next;

						//std::cout<<"x1: "<<firstPoint.first<<" y1: "<<firstPoint.second<<" x2: "<<secondPoint.first<<" y2: "<<secondPoint.second<<" x3: "<<thirdPoint.first<<" y3: "<<thirdPoint.second<<std::endl;

						//determine the angle between the two lines (three points)
						double angle1 = atan2(firstPoint.second-secondPoint.second, firstPoint.first-secondPoint.first);
						double angle2 = atan2(secondPoint.second-thirdPoint.second, secondPoint.first-thirdPoint.first);
						double theta = M_PI-(angle2-angle1);
	//					if (theta<0) {
	//						theta+= (2 * M_PI);
	//					}

						theta = theta/2.0;
						//std::cout<<"theta: "<<theta<<std::endl;

						double  width_angle;

						if(theta != 0){
							width_angle = ((*widthIt*1000)/2) / sin(theta);
						} else {
							width_angle = (*widthIt*1000)/2;
						}

						//std::cout<<"Angel width: "<<width_angle<<" width: "<<width<<std::endl;

						double xout1,yout1,xout2,yout2;

						projectPoseAngleDistance(secondPoint.first,secondPoint.second,alpha1, width_angle, -theta, xout1, yout1);
						projectPoseAngleDistance(secondPoint.first,secondPoint.second,alpha1, width_angle, -theta-M_PI, xout2, yout2);

						rightOuterPointsList.push_back(std::make_pair(xout2,yout2));
						leftOuterPointsList.push_back(std::make_pair(xout1,yout1));

						#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
						show_px_color(xout1, yout1, CV_RGB(0, 125, 255)); // left  //hellblau
						show_px_color(xout2, yout2, CV_RGB(125, 0, 255)); // right //lila
						#endif



						bresenham(xout1,yout1,xout4,yout4,points_list);
						bresenham(xout2,yout2,xout3,yout3,points_list);



					} else {

						//std::cout<<std::endl<<"Last Point case!"<<std::endl;
						double xout1,yout1,xout2,yout2;

						projectPoseAngleDistance(secondPoint.first,secondPoint.second,alpha1, (*widthIt*1000)/2, M_PI_2, xout2, yout2);
						projectPoseAngleDistance(secondPoint.first,secondPoint.second,alpha1, (*widthIt*1000)/2, -M_PI_2, xout1, yout1);
						rightOuterPointsList.push_back(std::make_pair(xout2,yout2));
						leftOuterPointsList.push_back(std::make_pair(xout1,yout1));

						bresenham(xout1,yout1,xout4,yout4,points_list);
						bresenham(xout2,yout2,xout3,yout3,points_list);


					}
				}


			}


}

void CdlLookupClass::insertPathBordersAsObstacleNew(int xs, int ys, int xg, int yg,  int indexCurvature){

	std::vector< std::pair<double,double> > points_list;

	calculatePathBorderPoints(points_list);

		//until here the points are collected, now perform cdl lookup


		int vi,wi,ci,li;

			double x,y;
			int ix,iy;
			int px = 0;
			int py = 0;

	//		std::cout<<"Points list size: "<<points_list.size()<<std::endl;

			for (std::vector< std::pair<double,double> >::iterator it=points_list.begin();it != points_list.end(); it++){

				x = it->first; y = it->second;
				if (x < 0.0)
				{
					ix = (int)(ceil((x-cdl_c_x_min)/cdl_c_res));
					px = (ix * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
				}
				else
				{
					ix = (int)(floor((x-cdl_c_x_min)/cdl_c_res));
					px = (ix * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
				}
				if (y < 0.0)
				{
					iy = (int)(ceil((y-cdl_c_y_min)/cdl_c_res));
					py = (iy * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
				}
				else
				{
					iy = (int)(floor((y-cdl_c_y_min)/cdl_c_res));
					py = (iy * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
				}
				//std::cout<<"ix: "<<ix<<" iy: "<<iy<<" cdl_c_x_cells: "<<cdl_c_x_cells << " cdl_c_y_cells "<<cdl_c_y_cells<<std::endl;
				if ((ix>=0) && (ix<cdl_c_x_cells) && (iy>=0) && (iy<cdl_c_y_cells))
				{
					//		  if(use_obstacle_history) {
						//		  	grids_set.insert(XYCoord(px, py)); // insert new (discrete) laser points in (old)/global gridset
					//		  }
		#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
						show_px_color(px, py, CV_RGB(0, 255, 0));
		#endif

					for (li=0;li<indexCurvature;li++)
					{
						ci=indexList[li];
						if (distLookup[ix][iy][ci]<distanceList[ci])
						{
							distanceList[ci] = distLookup[ix][iy][ci];
						}
						if (alphaLookup[ix][iy][ci]<alphaList[ci])
						{
							alphaList[ci]    = alphaLookup[ix][iy][ci];
						}
					}
				}
			}

	//		std::cout<<"DONE"<<std::endl;





}

//void CdlLookupClass::insertPathBordersAsObstacle(int xs, int ys, int xg, int yg,  int width, int indexCurvature){
//
//
//	std::list< std::pair<double,double> > points_list;
//
////	std::cout<<"PathNavListSize: "<<pathNavList.size()<<std::endl;
//
//	for (std::list< std::pair<double,double> >::iterator it=pathNavList.begin();it != pathNavList.end();){
//		double x = it->first;
//		double y = it->second;
//		int ix,iy,px,py;
//		mapPointToCDLGrid(x,y,ix,iy,px,py);
//
//		it++;
//		if(it != pathNavList.end()){
//
//			double x2 = it->first;
//			double y2 = it->second;
//			int ix2,iy2,px2,py2;
//			mapPointToCDLGrid(x2,y2,ix2,iy2,px2,py2);
//
//
//			double alpha = atan2(y - y2, x - x2);
//
//			bresenhamProjectPath(ix,iy,ix2,iy2,width,alpha,points_list);
//		}
//
//	}
//
//	int vi,wi,ci,li;
//
//	double x,y;
//	int ix,iy;
//	int px = 0;
//	int py = 0;
//
////	std::cout<<"Points list size: "<<points_list.size()<<std::endl;
//
//	for (std::list< std::pair<double,double> >::iterator it=points_list.begin();it != points_list.end(); it++){
//
//		x = it->first; y = it->second;
//		if (x < 0.0)
//		{
//			ix = (int)(ceil((x-cdl_c_x_min)/cdl_c_res));
//			px = (ix * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
//		}
//		else
//		{
//			ix = (int)(floor((x-cdl_c_x_min)/cdl_c_res));
//			px = (ix * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
//		}
//		if (y < 0.0)
//		{
//			iy = (int)(ceil((y-cdl_c_y_min)/cdl_c_res));
//			py = (iy * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
//		}
//		else
//		{
//			iy = (int)(floor((y-cdl_c_y_min)/cdl_c_res));
//			py = (iy * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
//		}
//		//std::cout<<"ix: "<<ix<<" iy: "<<iy<<" cdl_c_x_cells: "<<cdl_c_x_cells << " cdl_c_y_cells "<<cdl_c_y_cells<<std::endl;
//		if ((ix>=0) && (ix<cdl_c_x_cells) && (iy>=0) && (iy<cdl_c_y_cells))
//		{
//			//		  if(use_obstacle_history) {
//				//		  	grids_set.insert(XYCoord(px, py)); // insert new (discrete) laser points in (old)/global gridset
//			//		  }
//#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
//				show_px_color(px, py, CV_RGB(0, 255, 0));
//#endif
//
//			for (li=0;li<indexCurvature;li++)
//			{
//				ci=indexList[li];
//				if (distLookup[ix][iy][ci]<distanceList[ci])
//				{
//					distanceList[ci] = distLookup[ix][iy][ci];
//				}
//				if (alphaLookup[ix][iy][ci]<alphaList[ci])
//				{
//					alphaList[ci]    = alphaLookup[ix][iy][ci];
//				}
//			}
//		}
//	}
//
////	std::cout<<"DONE"<<std::endl;
//
//}


void CdlLookupClass::calculateClosestPointFromPointToLineSegment(
                const double &  px,
                const double &  py,
                const double &  x1,
                const double &  y1,
                const double &  x2,
                const double &  y2,
                double  &outx,
                double  &outy)
{
        double  delta_angle, dx, dy;

        //if line is a point then the result is the point itself
        if (x1==x2 && y1==y2)
        {
                outx = x1;
                outy = y1;
        }
        else
        {
                dx    = x2 - x1;
                dy    = y2 - y1;
                delta_angle = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
                if (delta_angle<0)
                {
                 outx = x1;
                 outy = y1;
                }
                else
                {
                        if (delta_angle > 1)
                        {
                                outx = x2;
                                outy = y2;
                        }
                        else
                        {
                                outx = x1 + (delta_angle * dx);
                                outy = y1 + (delta_angle * dy);
                        }
                }
        }
}

int CdlLookupClass::calculateSpeedValues(
  double v,double w,
  double posX,double posY,double posA,
  double vminIni,double vmaxIni,
  double wminIni,double wmaxIni,
  CdlStrategyType strategy,
  CdlEvalFunctionType evalFunc,
  double &vResult,double &wResult,
  double &vAccResult,double &wAccResult)
{

#if DEBUG_CDL_LOOKUP_FILE_OUT
	if(COMP->cdlLookupdebug.is_open()){
		COMP->cdlLookupdebug <<" =CALL CALCULATE====================================="<<std::endl;
	}
#endif


  double ciResult;

  double local_cdl_v_rot_min;
  double local_cdl_v_rot_max;
  double local_cdl_v_rot_step;
  double vmin,vmax;
  double wmin,wmax;
  int    ivmin,ivmax;
  int    iwmin,iwmax;

  int indexCurvature;
  int vi,wi,ci,li;

  int ix,iy;

//<asteck: date="17.07.2008">

  // initialize
  double costPassing = 0.0;
  double costValue   = 0.0;
//</asteck>

  double costHit;
  double costSpeed,costDist,costHeading,costResult;
  double vtrans,vrot,dist,angle;
  double vTransAllowed,vRotAllowed;

  double vRotStepHelp,vTransStepHelp;
  double posXm,posYm;
  double posXstop,posYstop,posAstop,deltaX,deltaY;

  double vAccMax, wAccMax;
  double brakeDist, brakeAngle;
  double radius;
  double passingDistance,hitDistance,hitAngle,goalDistance;
  double x1i,y1i,x2i,y2i;
  int    nrIntersections;

  double alpha;

  int i,j;
  int status;

#if DEBUG_INTERACTIVE
  cout << "CDL debug interactive\n\n";
  cout << "current v, w [mm/s, deg/s]\n";
  cin >> v;
  cin >> w;
  w = rad00(w);
  cout << "current pos x, y, a [mm,mm,deg]\n";
  cin >> posX;
  cin >> posY;
  cin >> posA;
  posA = rad00(posA);
  cout << "we set the following parameters:\n\n";
  cout << "vminIni vmaxIni " << vminIni << " " << vmaxIni << "\n";
  cout << "wminIni wmaxIni " << deg00(wminIni) << " " << deg00(wmaxIni) << "\n";
  cout << "strategy " << strategy << "\n\n";
#endif

#if DEBUG_VW_WINDOW
  FILE *file;
  FILE *eval;
  char filenameVW[100];
  char filenameScan[100];
  char filenameEval[100];
  static int fileNumber=0;

  sprintf(filenameVW,"vwwindow%d.dbg",fileNumber);
  sprintf(filenameScan,"vwscan%d.dbg",fileNumber);
  sprintf(filenameEval,"vweval%d.dbg",fileNumber);
  fileNumber++;
#endif

  // preparations for the following computations
  local_cdl_v_rot_min  = cdl_v_rot_min*M_PI/180.0;
  local_cdl_v_rot_max  = cdl_v_rot_max*M_PI/180.0;
  local_cdl_v_rot_step = cdl_v_rot_step*M_PI/180.0;

//
//
//
//
//
// Initialisierung der maxdistance gemaess dem radius vornehmen !!!!
// diesen Wert im Prinzip aus distlookup entnehmen !!!!
//
//
//
//
//
  for (i=0;i<cdl_curvature_indices;i++)
    {
      indexFlag[i]    = 0;
      distanceList[i] = (int)cdl_max_distance;
      alphaList[i]    = (int)(2*M_PI*1000.0);
    }

  for (i=0;i<cdl_max_tra_cells;i++)
    {
      for (j=0;j<cdl_max_rot_cells;j++)
        {
          evaluation[i][j].costHit      = 0.0;
          evaluation[i][j].costPassing  = 0.0;
          evaluation[i][j].costSpeed    = 0.0;
          evaluation[i][j].costDistance = 0.0;
          evaluation[i][j].costHeading  = 0.0;
          evaluation[i][j].costValue    = 0.0;
        }
    }

  // calculate current velocity window
  vmin = v - deltat * vacc;
  vmax = v + deltat * vacc;
  wmin = w - deltat * wacc;
  wmax = w + deltat * wacc;

  // consider physical constraints of the robot
  if (vmin < cdl_v_tra_min) vmin=cdl_v_tra_min;
  if (vmax > cdl_v_tra_max) vmax=cdl_v_tra_max;
  if (wmin < local_cdl_v_rot_min) wmin=local_cdl_v_rot_min;
  if (wmax > local_cdl_v_rot_max) wmax=local_cdl_v_rot_max;

  // consider constraints of allowed velocity window
  if (vmin < vminIni) vmin=vminIni;
  if (vmax > vmaxIni) vmax=vmaxIni;
  if (wmin < wminIni) wmin=wminIni;
  if (wmax > wmaxIni) wmax=wmaxIni;

  ivmin = (int)(floor((vmin-cdl_v_tra_min)/cdl_v_tra_step+0.5));
  ivmax = (int)(floor((vmax-cdl_v_tra_min)/cdl_v_tra_step+0.5));
  iwmin = (int)(floor((wmin-local_cdl_v_rot_min)/local_cdl_v_rot_step+0.5));
  iwmax = (int)(floor((wmax-local_cdl_v_rot_min)/local_cdl_v_rot_step+0.5));

#if DEBUG_VW_WINDOW
  currentScan.save(filenameScan);
  file=fopen(filenameVW,"w");
  if (file!=NULL)
    {
      fprintf(file,"vmin vmax wmin wmax %f %f %f %f\n",
              vmin,vmax,deg00(wmin),deg00(wmax));
      fprintf(file,"ivmin ivmax iwmin iwmax %d %d %d %d\n",
              ivmin,ivmax,iwmin,iwmax);
      fprintf(file,"goalX goalY %f %f\n",goalX,goalY);
    }
#endif

  // extract the relevant curvature indices
  indexCurvature=0;
  for (vi=ivmin;vi<=ivmax;vi++)
    {
      for (wi=iwmin;wi<=iwmax;wi++)
        {
          ci = indexVW[vi][wi];
          if (indexFlag[ci]==0)
            {
              indexFlag[ci]=1;
              indexList[indexCurvature++]=ci;
            }
        }
    }

#if DEBUG_VW_WINDOW
  if (file!=NULL)
    {
      for (vi=ivmin;vi<=ivmax;vi++)
        {
          for (wi=iwmin;wi<=iwmax;wi++)
            {
              fprintf(file,"vi wi v w ci %d %d %f %f %d\n",
                      vi,
                      wi,
                      CDL_V_TRA_MIN+vi*CDL_V_TRA_STEP,
                      deg00(local_cdl_v_rot_min+wi*local_cdl_v_rot_step),
                      indexVW[vi][wi]);
            }
        }
    }
#endif

//  for(int i=0;i<CDL_MAX_CURVATURE;i++)
// {
//	std::cout<<"before distanceList["<<i<<"]"<<distanceList[i]<<std::endl;
// }

//////////////////////////////////////////////////////
#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	img = cvCreateImage( cvSize(pixelX, pixelY), IPL_DEPTH_8U, 3 );
	//TODO color of image
	cvSet(img, cvScalar(255, 255, 255));


	if(strategy == CDL_STRATEGY_15){
		drawCdlContour(img, cdl_c_x_max, cdl_c_y_min, cdl_to_pixel_scale, CV_RGB(0, 255, 0));
	} else {
		drawCdlContour(img, cdl_c_x_max, cdl_c_y_min, cdl_to_pixel_scale, CV_RGB(255, 0, 0));
	}

	// show robot 0|0
	cvCircle(img, 
		cvPoint(
			-cdl_c_y_min * cdl_to_pixel_scale,
			cdl_c_x_max * cdl_to_pixel_scale // y value
		       ),
		dotsize, 
		CV_RGB(255, 0, 0), 
		fill);

	for(int x = 0; x < pixelX; x += gridsize) {
		for(int y = 0; y < pixelY; y += gridsize) {
			// show grid:
			cvRectangle(img, cvPoint(x, y), cvPoint(x + gridsize, y + gridsize), CV_RGB(128, 128, 128), 1);
			//cvCircle(img, cvPoint(x, y), 1, CV_RGB(0, 255, 0), 1);
		}
	}
#endif


//////////////////////////////////////////////////////


  int px = 0;
  int py = 0;



  // now calculate the min distances considering the laser scan

  ///////////////////////////////////////////////////////////
  // 1. transform the global old obstacle points to new local frame
  ///////////////////////////////////////////////////////////

if(use_obstacle_history) { ////////////////////////////////////////////
  CommBasicObjects::CommBasePose basePosition = scan.get_base_state().get_base_position();

  arma::mat res_t(3,1);
  arma::mat res_r(3,3);

  arma::mat a_t(3,1);
  a_t(0,0) = basePosition.get_x();
  a_t(1,0) = basePosition.get_y();
  a_t(2,0) = 0;
  arma::mat a_r(3,3);
  EulerTransformationMatrices::create_zyx_matrix(basePosition.getPose3D().get_orientation().getAzimuth(),0,0,a_r);

  arma::mat b_t(3,1);
  b_t(0,0) = basePosition_old.get_x();
  b_t(1,0) = basePosition_old.get_y();
  b_t(2,0) = 0;
  arma::mat b_r(3,3);
  EulerTransformationMatrices::create_zyx_matrix(basePosition_old.getPose3D().get_orientation().getAzimuth(),0,0,b_r);

  inverseComposeFrom(b_t,b_r, a_t, a_r, res_t,res_r);

  // now sum up the differences.
  //t_x_sum += basePosition.get_x() - basePosition_old.get_x();
  //t_y_sum += basePosition.get_y() - basePosition_old.get_y();
  t_x_sum += res_t(0,0);
  t_y_sum += res_t(1,0);

  // do not sum t_alpha!?
  //double t_alpha = piToPiRad(basePosition.get_base_alpha() - basePosition_old.get_base_alpha());
  double yaw, pitch, roll;
  EulerTransformationMatrices::zyx_from_matrix(res_r, yaw, pitch, roll);
  double t_alpha = yaw;

  double t_x = 0;
  double t_y = 0;

  if(fabs(t_x_sum) >= 30.0) {
	t_x = t_x_sum;
 	t_x_sum = 0;
  }
  if(fabs(t_y_sum) >= 30.0) {
	t_y = t_y_sum;
 	t_y_sum = 0;
  }
  
	

  basePosition_old = basePosition; 

  arma::mat trans(3,3);
  trans(0,0) = cos(t_alpha);
  trans(0,1) = -sin(t_alpha);
  trans(0,2) = t_x;

  trans(1,0) = sin(t_alpha);
  trans(1,1) = cos(t_alpha);
  trans(1,2) = t_y;

  trans(2,0) = 0;
  trans(2,1) = 0;
  trans(2,2) = 1;

  arma::mat trans_i(3,3);
  trans_i = inv(trans);
  std::set<XYCoord> grids_set_new;


  // transformieren old points to new local coord frame
  for(std::set<XYCoord>::iterator it = grids_set.begin(); it != grids_set.end(); ++it) {
	arma::mat point(1,3);
	point(0,0) = it->x;
	point(0,1) = it->y;
	point(0,2) = 1;

        point = arma::trans(trans_i * (arma::trans(point)));
	
	grids_set_new.insert(XYCoord(point(0,0), point(0,1)));

	//show_px_color(it->x, it->y, CV_RGB(0, 0, 255)); // gerasterte alte untransformiert
	//show_px_color(point(0,0), point(0,1), CV_RGB(255, 0, 255)); // alte transformiert = ungerastert/kontinuierliche werte
  }

  grids_set.clear(); //old points are no longer needed



  ///////////////////////////////////////////////////////////
  // 2. use old transformed obstcales points to check for collisions
  //    only points near to the robot are used
  //    points in viewing area of the laser are NOT used
  ///////////////////////////////////////////////////////////


  CommBasicObjects::CommPose3d laserPose = scan.get_sensor_pose();
  double laser_x_offset = laserPose.get_x();
  double laser_y_offset = laserPose.get_y();
  double laser_opening = fabs(piToPiRad(scan.get_scan_start_angle())) * 2.0;
  double m = tan( ((laser_opening/2.0)-M_PI_2) );
  double laser2_x_offset = 0;
  double laser2_y_offset = 0;
  double laser2_opening = 0;
  double m2 = 0.0;

  bool has_second_laser = false;


if(scan2.get_scan_size() > 0) {
	has_second_laser = true;

	CommBasicObjects::CommPose3d laser2Pose = scan2.get_sensor_pose();

	//laser2_x_offset = -282;
	laser2_x_offset = laser2Pose.get_x();
	laser2_y_offset = laser2Pose.get_y();

	laser2_opening = fabs(piToPiRad(scan2.get_scan_start_angle())) * 2.0;
	m2 = tan( ((laser2_opening/2.0)-M_PI_2) );
}

#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
//  dbg(laser_opening);
//  dbg(laser2_opening);
  show_px_color(laser_x_offset, 0, CV_RGB(0, 0, 0)); // show laser
  show_px_color(laser2_x_offset, 0, CV_RGB(0, 0, 0)); // show laser
#endif

  bool consider_point = false;
  for(std::set<XYCoord>::iterator gridit = grids_set_new.begin(); gridit != grids_set_new.end(); ++gridit) 
    {
      if (gridit->x < 0.0)
        {
          ix = (int)(ceil((gridit->x-cdl_c_x_min)/cdl_c_res));
	  px = (ix * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
        }
      else
        {
          ix = (int)(floor((gridit->x-cdl_c_x_min)/cdl_c_res));
	  px = (ix * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
        }
      if (gridit->y < 0.0)
        {
          iy = (int)(ceil((gridit->y-cdl_c_y_min)/cdl_c_res));
	  py = (iy * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
        }
      else
        {
          iy = (int)(floor((gridit->y-cdl_c_y_min)/cdl_c_res));
	  py = (iy * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
        }

      consider_point = false;

      // use only points near to the robot and not in viewing range of the laser
/*
      if(
	px < 1000 && px > -1000 && py < 1000 && py > -1000
	&&
	(
	px <=  m * -fabs(py) + laser_x_offset
	^
	(px <=  m2 * +fabs(py) + laser2_x_offset && has_second_laser ) // has_second_laser activates/deactivates this part
	)
      ) {
	consider_point = true;
      }
*/
	if(laser_opening <M_PI){
		/////////////////////////////////////
		// laser 1 opening < 180 deg
		if(
			//TODO this should be moved to a INI file param --> values for robotino3 factory4
			px < 1000 && px > -100 && py < 1000 && py > -1000 // this is the size of the history window/map of invisible obstacles in the blind spot.
			//px < 1000 && px > -1000 && py < 1000 && py > -1000 // this is the size of the history window/map of invisible obstacles in the blind spot.
			&&
			( px <=  m * -py + laser_x_offset + (m * laser_y_offset)// following y=m*x+b
			||
			px <=  m * +py + laser_x_offset - (m * laser_y_offset) // following y=m*x+b
                        ) 
		) { // following y=m*x+b
				if(has_second_laser) {
					if(laser2_opening < M_PI) {
						/////////////////////////////////////
						// laser 2 opening < 180 deg
						if(
						px >=  m2 * -py + laser2_x_offset + (m2 * laser2_y_offset) 
						||
						px >=  m2 * +py + laser2_x_offset - (m2 * laser2_y_offset) 
						) {
							consider_point = true;
						} else {
							// do not consider this point
							consider_point = false;
						}
						/////////////////////////////////////

					} else {
						/////////////////////////////////////
						// laser 2 opening > 180 deg
						if(
						px >=  m2 * -py + laser2_x_offset + (m2 * laser2_y_offset) 
						&&
						px >=  m2 * +py + laser2_x_offset - (m2 * laser2_y_offset) 
						) {
							consider_point = true;
						} else {
							// do not consider this point
							consider_point = false;
						}
						/////////////////////////////////////
					}

				} else {
					consider_point = true;
				}
		}






		/////////////////////////////////////
	} else {

		/////////////////////////////////////
		// laser 1 opening > 180 deg
		if(
			px < 1000 && px > -1000 && py < 1000 && py > -1000 // this is the size of the history window/map of invisible obstacles in the blind spot.
			&&
			px <=  m * -py + laser_x_offset + (m * laser_y_offset)// following y=m*x+b
			&&
			px <=  m * +py + laser_x_offset - (m * laser_y_offset)// following y=m*x+b
		) { // following y=m*x+b
				if(has_second_laser) {
					if(laser2_opening < M_PI) {
						/////////////////////////////////////
						// laser 2 opening < 180 deg
						if(
						px >=  m2 * -py + laser2_x_offset + (m2 * laser2_y_offset) 
						||
						px >=  m2 * +py + laser2_x_offset - (m2 * laser2_y_offset) 
						) {
							consider_point = true;
						} else {
							// do not consider this point
							consider_point = false;
						}
						/////////////////////////////////////

					} else {
						/////////////////////////////////////
						// laser 2 opening > 180 deg
						if(
						px >=  m2 * -py + laser2_x_offset + (m2 * laser2_y_offset) 
						&&
						px >=  m2 * +py + laser2_x_offset - (m2 * laser2_y_offset) 
						) {
							consider_point = true;
						} else {
							// do not consider this point
							consider_point = false;
						}
						/////////////////////////////////////
					}

				} else {
					consider_point = true;
				}
		}
		/////////////////////////////////////
	}


	if(consider_point) {
		grids_set.insert(XYCoord(px, py)); // insert in (old)/global gridset

		if ((ix>=0) && (ix<cdl_c_x_cells) && (iy>=0) && (iy<cdl_c_y_cells)
			&& !cdl_check_point_inside_polygon(px, py))
		  {
		    //grids_set.insert(XYCoord(gridit->x, gridit->y)); 
		#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
			    show_px_color(px, py, CV_RGB(0, 0, 255)); // alte transformiert = ungerastert/kontinuierliche werte
		#endif
		    for (li=0;li<indexCurvature;li++)
		      {
			ci=indexList[li];
			if (distLookup[ix][iy][ci]<distanceList[ci])
			  {
			    distanceList[ci] = distLookup[ix][iy][ci];
			  }
			if (alphaLookup[ix][iy][ci]<alphaList[ci])
			  {
			    alphaList[ci]    = alphaLookup[ix][iy][ci];
			 }
		     }
		  }

		#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
			if(cdl_check_point_inside_polygon(px, py))
			    show_px_color(px, py, CV_RGB(255, 0, 0)); // alte transformiert = ungerastert/kontinuierliche werte
		#endif
	}

    }
} //if(use_obstacle_history)///////////////////////////////////////////////////////////



  ///////////////////////////////////////////////////////////
  // 3. use current laser scan to check for collisions
  //    all laser points are used
  ///////////////////////////////////////////////////////////

/*
  for (scanindex=0;scanindex<scan.get_scan_size();scanindex++){
  // we need a robot coordinate system (independent of where the scanner is mounted on)
      scan.get_scan_cartesian_3dpoint_robot( scanindex, x, y, z );
      grids_set_new.insert(XYCoord(x, y));
      show_px_color(x, y, CV_RGB(0, 255, 0)); // alte transformiert = ungerastert/kontinuierliche werte
  }
*/

  //FIRST LASER
  insertLaserScanAsObstacle(scan,indexCurvature);

  //SECOND LASER
  // now calculate the min distances considering the second laser scan
  insertLaserScanAsObstacle(scan2,indexCurvature);

  // NOW insert the points from the ir Scan (if any)
  insertIRScanAsObstacle(irScan,indexCurvature);


  if(usePathNav == true && usePathBorders ==true){
	  insertPathBordersAsObstacleNew(pathNavStartX,pathNavStartY,pathNavGoalX,pathNavGoalY,indexCurvature);
  }









std::stringstream ss;
ss << std::setw(3) << std::setfill('0') << "cdl-out-" << imgCntTMP++ << ".jpg";





//  for(int i=0;i<CDL_MAX_CURVATURE;i++)
// {
//	std::cout<<"past first distanceList["<<i<<"]"<<distanceList[i]<<std::endl;
// }



//  for(int i=0;i<CDL_MAX_CURVATURE;i++)
// {
//	std::cout<<"past second distanceList["<<i<<"]"<<distanceList[i]<<std::endl;
// }
  //////////////////////////////////////////////////////////////////////////////////

#if DEBUG_VW_WINDOW
  if (file!=NULL)
    {
      for (li=0;li<indexCurvature;li++)
        {
          fprintf(file,"index distance angle %d %d %f\n",
                  indexList[li],
                  distanceList[indexList[li]],
                  deg00((double)alphaList[indexList[li]]/1000));
        }
    }
#endif


  // ------------------------------------------------------
  // now we have to calculate the evaluation function
  // ------------------------------------------------------
  vResult        = 0.0;
  wResult        = 0.0;
  vAccResult     = 0.0;
  wAccResult     = 0.0;
  costResult     = 0.0;
  vRotStepHelp   = local_cdl_v_rot_step/2.0;
  vTransStepHelp = cdl_v_tra_step/2.0;



//
//
// BEGIN TEST AREA CDL_EVAL_APPROACH_EXACT
//
//

  for (i=0;i<cdl_max_curvature;i++)
	{
	  indexWanted[i] = 1;
	  indexWantedVWCheck[i] = 0;
	}

// for person following
  if(strategy == CDL_STRATEGY_7){
	  double x, y;
	  x = goalX;
	  y = goalY;
	  int ix,iy;
	  int px = 0;
	  int py = 0;

	  int vi,wi,ci,li;

	  if (x < 0.0)
	  {
		  ix = (int)(ceil((x-cdl_c_x_min)/cdl_c_res));
		  px = (ix * cdl_c_res) + cdl_c_x_min	- cdl_c_res/2.0;
	  }
	  else
	  {
		  ix = (int)(floor((x-cdl_c_x_min)/cdl_c_res));
		  px = (ix * cdl_c_res) + cdl_c_x_min	+ cdl_c_res/2.0;
	  }
	  if (y < 0.0)
	  {
		  iy = (int)(ceil((y-cdl_c_y_min)/cdl_c_res));
		  py = (iy * cdl_c_res) + cdl_c_y_min - cdl_c_res/2.0;
	  }
	  else
	  {
		  iy = (int)(floor((y-cdl_c_y_min)/cdl_c_res));
		  py = (iy * cdl_c_res) + cdl_c_y_min + cdl_c_res/2.0;
	  }

	  if ((ix>=0) && (ix<cdl_c_x_cells) && (iy>=0) && (iy<cdl_c_y_cells))
	  {
		  for (li=0;li<indexCurvature;li++)
		  {
			  ci=indexList[li];
			  int dist = distLookup[ix][iy][ci];
			  if(dist<cdl_max_distance && dist>0){
				  indexWantedVWCheck[ci] = 1;

				  //std::cout<<"Wanted CI: "<<ci<<" dist: "<<dist<<std::endl;
			  }

		  }
	  }
  }


  if(strategy == CDL_STRATEGY_1){
	  //check if the desired speed vels are within the bounds of the lookup tables!
	  if(desiredTranslationalSpeed<= cdl_v_tra_max && desiredTranslationalSpeed >=cdl_v_tra_min &&
			  desiredRotationalSpeed <= local_cdl_v_rot_max && desiredRotationalSpeed >= local_cdl_v_rot_min){

		  int  vi=(int)(floor((desiredTranslationalSpeed-cdl_v_tra_min)/cdl_v_tra_step+0.5));
		  int  wi=(int)(floor((desiredRotationalSpeed-local_cdl_v_rot_min)/local_cdl_v_rot_step+0.5));

		  int ci   = indexVW[vi][wi];
		  indexWantedVWCheck[ci] = 1;
		  for (i=-4;i<=4;i++)
		  {
			j=ci+i;
			if (j>cdl_max_curvature) j-=cdl_max_curvature;
			if (j<0) j+=cdl_max_curvature;
			indexWantedVWCheck[j]=1;
			std::cout<<"indexWantedVWCheck - wanted "<<j<<std::endl;
		  }

		  std::cout<<"indexWantedVWCheck: "<<ci<<" - vdes|vcal ("<< desiredTranslationalSpeed << "|"<< (cdl_v_tra_min + vi*cdl_v_tra_step)
				  << ") - wdes|wcal ("<< desiredRotationalSpeed<<"|" << (local_cdl_v_rot_min + wi*local_cdl_v_rot_step) <<")"<<std::endl;
	  } else {
		  std::cout<<"[smartCdlLookup] Warning: The desired vels (v/w) exceed the lookup tables! --> STOP ROBOT!"<<std::endl;
	  }

  }

  for (vi=ivmin;vi<=ivmax;vi++)
    {
      for (wi=iwmin;wi<=iwmax;wi++)
        {
          ci   = indexVW[vi][wi];
          dist = (double)(distanceList[ci]);
          if (dist < 500.0)
            {
              for (i=-40;i<=40;i++)
                {
                  j=ci+i;
                  if (j>cdl_max_curvature) j-=cdl_max_curvature;
                  if (j<0) j+=cdl_max_curvature;
                  indexWanted[j]=0;
                }
            }
        }
    }

#if DEBUG_VW_WINDOW
  if (file!=NULL)
    {
      for (i=0;i<CDL_MAX_CURVATURE;i++)
        {
          fprintf(file,"curvature %d: flag %d wanted %d\n",i,indexFlag[i],indexWanted[i]);
        }
    }
#endif

//
//
// END TEST AREA CDL_EVAL_APPROACH_EXACT
//
//

//  std::cout << "new ----------------------------------------------------\n\n\n";



  //calculate goal for path nav
  if(usePathNav == true)
  {

	  double closestPoint_x,closestPoint_y;
	  double vX,vY;
	  double ux,uy;
	  //double distlook = 500;
	  double distlook;

	  distlook = linearinterpolation(_distlookControl, abs00(v));
	  if(distlook < pathNavPredictedGoalPose_minDist){
		  distlook = pathNavPredictedGoalPose_minDist;
	  }


#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	  show_px_color(pathNavStartX, pathNavStartY, CV_RGB(0, 0, 255)); // alte transformiert = ungerastert/kontinuierliche werte
	  show_px_color(pathNavGoalX, pathNavGoalY, CV_RGB(0, 0, 255)); // alte transformiert = ungerastert/kontinuierliche werte
#endif

	  calculateClosestPointFromPointToLineSegment(0,0,pathNavStartX,pathNavStartY,pathNavGoalX,pathNavGoalY,closestPoint_x,closestPoint_y);

#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	  show_px_color(closestPoint_x, closestPoint_y, CV_RGB(255, 0, 255)); // alte transformiert = ungerastert/kontinuierliche werte
#endif

	  //project goalpoint on line towards goal
	  vX = pathNavGoalX - closestPoint_x;
	  vY = pathNavGoalY - closestPoint_y;

	  ux = vX/sqrt((vX*vX + vY*vY));
	  uy = vY/sqrt((vX*vX + vY*vY));
	  if(!std::isfinite(ux)){
		  ux = 0;
		  std::cout<<"closest point == pathNavGoal --> ux = 0"<<std::endl;
	  }
	  if(!std::isfinite(uy)){
		  uy = 0;
		  std::cout<<"closest point == pathNavGoal --> uy = 0"<<std::endl;
	  }

	  pathNavGoalPoint_x = closestPoint_x + distlook * ux;
	  pathNavGoalPoint_y = closestPoint_y + distlook * uy;


	  //limit path Nav Goal Point to the pathNavGoal, to prevent the projected goal from overshooting the overall goal!
	  double distGoalPoint = std::sqrt( square(pathNavGoalX)+square(pathNavGoalY));
	  double distPathNavGoal = std::sqrt( square(pathNavGoalPoint_x)+square(pathNavGoalPoint_y));
	  if(distGoalPoint < distPathNavGoal){
		  std::cout<<"Limit PathNav Goal Point"<<std::endl;
		  pathNavGoalPoint_x = pathNavGoalX;
		  pathNavGoalPoint_y = pathNavGoalY;
	  }

	  /////////////////////////////////////////////////////////
	  // PREVENT ROBOT FROM OVERSHOOTING IN CORNERS
	  /////////////////////////////////////////////////////////

	  std::vector<std::pair<double, double> > _desiredVxControl;

	  double tmpSpeed = desiredTranslationalSpeed;
	  _desiredVxControl.push_back( std::make_pair( 100.0, tmpSpeed ) );
	  _desiredVxControl.push_back( std::make_pair( 200.0, tmpSpeed/2 ) );
	  _desiredVxControl.push_back( std::make_pair( 400.0, 100 ) );

	  desiredTranslationalSpeed = linearinterpolation(_desiredVxControl, abs00(pathNavGoalPoint_y));
	  if( tmpSpeed < 0 )
	  {
		  desiredTranslationalSpeed = -desiredTranslationalSpeed;
	  }

	  std::cout<<__FUNCTION__<<"desiredTranslationalSpeed: "<<desiredTranslationalSpeed<<std::endl;
	  /////////////////////////////////////////////////////////

	  {
//	  double ke = 0.6;
//	  double ko = (sqrt(4*ke))*-1;
//	  double e = pathNavGoalPoint_y/1000.0;

	  std::vector<std::pair<double, double> > _desiredVrotControl;

	  double tmpSpeed = desiredTranslationalSpeed;
	  _desiredVrotControl.push_back( std::make_pair( 0, 0 ) );
	  _desiredVrotControl.push_back( std::make_pair( 0.17, 0.1745 ) );
	  _desiredVrotControl.push_back( std::make_pair( 3.14, 3.14 ) );

	  std::cout<<"MATTHIAS"<<std::endl;
	  std::cout<<"pathNavGoalPoint_x: "<<pathNavGoalPoint_x<<" pathNavGoalPoint_y: "<<pathNavGoalPoint_y<<std::endl;
	  double heading_error = atan2(pathNavGoalPoint_y,pathNavGoalPoint_x);
	  std::cout<<"heading_error:"<<heading_error<<std::endl;
	  desiredRotationalSpeed = linearinterpolation(_desiredVrotControl, abs00(heading_error));
	  if(heading_error<0){
		  desiredRotationalSpeed = desiredRotationalSpeed*-1.0;
	  }


	  std::cout<<"desiredRotationalSpeed: "<<desiredRotationalSpeed<<std::endl;

//	  double v_tmp = desiredTranslationalSpeed/1000.0;
//
//	  // as sigi does
//	  double w = min00(v_tmp*v_tmp,1/max00(0.01,v_tmp))*(-1*(ko*v_tmp*O)  - (ke*e));
//
//	  std::cout<<"MATTHIAS: angular error: "<<O<<" resulting w: "<<w<<std::endl;
	  }


#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	  show_px_color(pathNavGoalPoint_x, pathNavGoalPoint_y, CV_RGB(255, 0, 0)); // alte transformiert = ungerastert/kontinuierliche werte
#endif

#if DEBUG_CDL_LOOKUP_FILE_OUT
	if(COMP->cdlLookupdebug.is_open()){
		COMP->cdlLookupdebug << "pathNavStartX: "<<pathNavStartX<<" pathNavStartY "<<pathNavStartY<< " pathNavGoalX "<<pathNavGoalX<<" pathNavGoalY " <<pathNavGoalY<<std::endl;
		COMP->cdlLookupdebug << "closestPoint_x: "<<closestPoint_x<<" closestPoint_y "<<closestPoint_y<<std::endl;
		COMP->cdlLookupdebug << "pathNavGoalPoint_x: "<<pathNavGoalPoint_x<<" pathNavGoalPoint_y "<<pathNavGoalPoint_y<<std::endl;
	}
#endif

  }

  double angle_lookup, dist_lookup;


  for (vi=ivmin;vi<=ivmax;vi++)
    {
      for (wi=iwmin;wi<=iwmax;wi++)
        {


          vtrans  = cdl_v_tra_min + vi*cdl_v_tra_step;
          vrot    = local_cdl_v_rot_min + wi*local_cdl_v_rot_step;
          ci      = indexVW[vi][wi];
          dist    = (double)(distanceList[ci]);
          angle   = (double)alphaList[ci]/1000.0;
          vAccMax = transAccLookup[ci];
          wAccMax = rotAccLookup[ci];

          angle_lookup = angle;
          dist_lookup = dist;

          // the following is necessary to prevent rounding errors
          if (abs00(vtrans) < vTransStepHelp) vtrans = 0.0;
          if (abs00(vrot)   < vRotStepHelp)   vrot   = 0.0;

          // We consider the distance and angle changes within the
          // current interval used for calculation of the next
          // motion command. To simplify the calculation and to be
          // on the safe side, we always substract that from the
          // remaining distance and angle.

          dist   = dist -abs00(v)*deltat;
          if (dist  < 0.0) dist =0.0;
          angle  = angle-abs00(w)*deltat;
          if (angle < 0.0) angle=0.0;

          // Sicherheitsabstand !!!! ??????
          //dist  -= 200.0;
         
          dist  -= safetyClearance;

//          angle -= rad00(5.0);

          angle -= rad00(2.0);
          if (dist  < 0.0) dist  = 0.0;
          if (angle < 0.0) angle = 0.0;


          // first we have to check whether the robot can stop on the
          // remaining distance and angle with the current speed

          vTransAllowed = sqrt(2.0 * dist  * vAccMax);
          vRotAllowed   = sqrt(2.0 * angle * wAccMax);

#if DEBUG_VW_WINDOW
          if (file!=NULL)
            {
              fprintf(file,"=====\n");
              fprintf(file,"vtrans vrot             %f %f\n",vtrans,deg00(vrot));
              fprintf(file,"ci corrected dist angle %d %f %f\n",ci,dist,deg00(angle));
              fprintf(file,"vAccMax wAccMax         %f %f\n",vAccMax,deg00(wAccMax));
              fprintf(file,"vtrans vTransAllowed    %f %f\n",vtrans,vTransAllowed);
              fprintf(file,"vrot   vRotAllowed      %f %f\n",deg00(vrot),
                      deg00(vRotAllowed));
            }
#endif

          ///////////////////////////////////////////////////////////////////////
	if(strategy == CDL_STRATEGY_1){
          if ((abs00(vtrans) <= vTransAllowed) && (abs00(vrot) <= vRotAllowed))
          {

          } else {
        	  if(indexWantedVWCheck[ci]==1){
#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
        		  std::cout<<"vw: ("<< vtrans <<"|"<< vrot <<") is not allowed - distO|dist: ("<<dist_lookup<<"|"<<dist<<") angleO|angle: ("<<angle_lookup<<"|"<<angle<<") vTransAllowed: "<<vTransAllowed<<" vRotAllowed: "<<vRotAllowed<<std::endl;
#endif
        	  }

          }
	}
          ///////////////////////////////////////////////////////////////////////


          if ((abs00(vtrans) <= vTransAllowed) && (abs00(vrot) <= vRotAllowed))
          {

          //printf("STRAT v ; w ; dist ; vTransAllowed; vAccMax  -- %5.2f ; %5.2f ; %6.2f ; %6.2f; %6.2f \n", vtrans , vrot, dist, vTransAllowed, vAccMax);
 //         printf(" - - - OK\n");
              // these velocities are allowed

// factors
#define BRAKE_SECURITY_FACTOR  1.0



              //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
              // now switch depending on the selected strategy
              //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        	  //TODO marker for eclipse editor
        	  switch (strategy)
        	  {

        	  case CDL_STRATEGY_1:
			  {

				  if(indexWantedVWCheck[ci]==1){
					  costHeading=1.0-abs00(angle00(desiredRotationalSpeed-vrot))/wmaxIni;
					  costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;
					  alpha1 = 1;
					  alpha2 = 1;
					  alpha3 = 1;
					  costValue = alpha1*costSpeed + alpha3*costHeading;
#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
					  std::cout<<ci<<" - v|desv: "<<vtrans<<"|"<<desiredTranslationalSpeed<<" w|desw: "<<vrot<<"|"<<desiredRotationalSpeed<<" cost: "<<costValue<<"              v:"<<v<<" w:"<<w<<std::endl;
#endif
				  } else {
//					  std::cout<<ci<<" not wanted - v|desv: "<<vtrans<<"|"<<desiredTranslationalSpeed<<" w|desw: "<<vrot<<"|"<<desiredRotationalSpeed<<std::endl;
					  costValue = 0;
				  }

				  break;
			  }

        	  case CDL_STRATEGY_16: {
#if DEBUG_CDL_LOOKUP_FILE_OUT
        			  if(COMP->cdlLookupdebug.is_open()){
        				  COMP->cdlLookupdebug <<" V: "<<vtrans<<" W: "<<vrot;
        				  COMP->cdlLookupdebug <<" desV: "<<desiredTranslationalSpeed<<" desW: "<<desiredRotationalSpeed;
        			  }
#endif

    			  if(vtrans > desiredTranslationalSpeed){
    				  costValue = 0;
#if DEBUG_CDL_LOOKUP_FILE_OUT
        		  if(COMP->cdlLookupdebug.is_open()){
        			  COMP->cdlLookupdebug << " ZERO Cost: "<<costValue<<std::endl;
        		  }
#endif
    			  } else {


//    				  //calculate if a speed combination brings us closer to the goal
//    				  double current_distGoalPoint = std::sqrt( square(pathNavGoalX)+square(pathNavGoalY));
//
//    				  // prevent the robot from moving away from the goal!
//					  double res_x,res_y;
//					  if(vrot != 0){
//						  double r = (vtrans/1000.0) / vrot;
//						  r = r*1000.0;
//						  double icc_x = 0; double icc_y = r;
//						  if(icc_y<0){
//							r = abs00(r);
//						  }
//						  //calculate point on line
//						  double s, alpha, h,ys, r_m;
//						  r_m = r/1000.0;
//						  s = vtrans * deltat;
//						  alpha = s/r;
//						  h = sin(alpha) * r_m;  ys = cos(alpha) * r_m;
//
//						  res_x = h*1000;
//						  if(icc_y<0){
//							  res_y = icc_y + (ys*1000.0);
//						  } else {
//							  res_y = icc_y - (ys*1000.0);
//						  }
//					  } else {
//
//						  double s;
//						  s = vtrans * deltat;
//						  res_x = s;
//						  res_y = 0;
//					  }
//
//    				  //std::cout<<"res_x: "<<res_x<<" res_y: "<<res_y<<std::endl;
//
//    				  double predicted_pose_goaldist = std::sqrt( square(res_x-pathNavGoalX)+square(res_y-pathNavGoalY) );
//    				  double diff_dist = current_distGoalPoint - predicted_pose_goaldist;
//    				  if(diff_dist>0){
//    					  costDist = 1.0-diff_dist/current_distGoalPoint;
//    				  } else {
//    					  costDist = 0;
//    				  }

    				  costDist = 0;

    				  //costSpeed=abs00(desiredTranslationalSpeed-vtrans);
					  costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;
					  alpha1=0.25;
					  alpha2=1.0;
					  alpha3=0.5;
					  double costRot = 1.0-abs00(vrot - desiredRotationalSpeed)/wmaxIni;
					  costValue=alpha1*costSpeed+alpha2*costRot+alpha3*costDist;
#if DEBUG_CDL_LOOKUP_FILE_OUT
        		  if(COMP->cdlLookupdebug.is_open()){
    				  COMP->cdlLookupdebug <<" costSpeed: "<<costSpeed<<" costRot: "<<costRot<<" costDist: "<<costDist;
        			  COMP->cdlLookupdebug << " Cost: "<<costValue<<std::endl;
        		  }
#endif

    			  }



        		  break;
        	  }

        	  //<startegy added by mlutz; 2015-05-05>; used for path based navigation
        	  //PATH NAV
        	  case CDL_STRATEGY_15: {
        		  double costGoalDist = 0.0;

#if DEBUG_CDL_LOOKUP_FILE_OUT
        			  if(COMP->cdlLookupdebug.is_open()){
        				  COMP->cdlLookupdebug <<" V: "<<vtrans<<" W: "<<vrot;
        			  }
#endif


        		  if(vrot == 0){
        			  double a,b,c,xig,yig;
        			  int goalLineEquationRes = cdl_ab_axbyc(pathNavStartX,pathNavStartY,pathNavGoalX,pathNavGoalY,a,b,c);
        			  if(goalLineEquationRes != 0){
        				  costValue = 0;
        				  std::cout<<"ERROR: was not able to calculate goal line equation!"<<std::endl;
        			  } else {
        				  int res = cdl_line_intersection(a,b,-c,0.0,1.0,0.0,xig,yig);
        				  if(res == 0 ){
        					  double distP1 = std::sqrt( square(xig-pathNavGoalPoint_x)+square(yig-pathNavGoalPoint_y) );

#if DEBUG_CDL_LOOKUP_FILE_OUT
        					  if(COMP->cdlLookupdebug.is_open()){
        						  COMP->cdlLookupdebug <<" Line Inter: "<<xig<<" "<<yig << " Dist: "<<distP1;
        					  }
#endif

        					  costGoalDist = min00(1/distP1,1.0);

        				  } else if (res == 1){
        					  //equal line --> goal is directly on line!
#if DEBUG_CDL_LOOKUP_FILE_OUT
        					  if(COMP->cdlLookupdebug.is_open()){
        						  COMP->cdlLookupdebug <<" GOAL ON LINE!";
        					  }
#endif

        					  costValue = 1000;
        				  } else {
        					  //parallel line
#if DEBUG_CDL_LOOKUP_FILE_OUT
        					  if(COMP->cdlLookupdebug.is_open()){
        						  COMP->cdlLookupdebug <<" GOAL ON parallel LINE!";
        					  }
#endif

        					  costValue = 0;
        				  }
        			  }
        		  } else {


        			  double r = vtrans/1000.0 / vrot;
        			  r = r*1000.0;

        			  double icc_x = 0;
        			  double icc_y = r;

        			  double x1i,y1i,x2i,y2i;
        			  int nrIntersections;

        			  intersectionsCircleLine(icc_x, icc_y,r,
        					  pathNavGoalPoint_x,pathNavGoalPoint_y,pathNavGoalX,pathNavGoalY,
        					  x1i,y1i,x2i,y2i,
        					  nrIntersections);


        			  if(nrIntersections>0){
        				  if(nrIntersections == 1){


        					  double distP1 = std::sqrt( square(x1i-pathNavGoalPoint_x)+square(y1i-pathNavGoalPoint_y) );
#if DEBUG_CDL_LOOKUP_FILE_OUT
        					  if(COMP->cdlLookupdebug.is_open()){
        						  COMP->cdlLookupdebug <<" Inter 1: "<<x1i<<" "<<y1i << " Dist: "<<distP1;
        					  }
#endif

        					  costGoalDist = min00(1/distP1,1.0);


        				  } else {
        					  double distP1 = std::sqrt( square(x1i-pathNavGoalPoint_x)+square(y1i-pathNavGoalPoint_y) );
        					  double distP2 = std::sqrt( square(x2i-pathNavGoalPoint_x)+square(y2i-pathNavGoalPoint_y) );
#if DEBUG_CDL_LOOKUP_FILE_OUT
        					  if(COMP->cdlLookupdebug.is_open()){
        						  COMP->cdlLookupdebug <<" Inter 1: "<<x1i<<" "<<y1i << " Dist: "<<distP1 << " Inter 2: "<<x2i<<" "<<y2i << " Dist: "<<distP2;
        					  }
#endif
        					  if(distP1<distP2){
        						  costGoalDist = min00(1/distP1,1.0);

        					  } else {
        						  costGoalDist = min00(1/distP2,1.0);
        					  }
        				  }

        			  } else {
        				  costGoalDist = 0;
#if DEBUG_CDL_LOOKUP_FILE_OUT
        				  if(COMP->cdlLookupdebug.is_open()){
        					  COMP->cdlLookupdebug << " No intersection!";
        				  }
#endif
        			  }
        		  }


        		  // prevent the robot from moving away from the goal!
        		  double res_x,res_y;

        		  if(vrot != 0){

        			  double r = (vtrans/1000.0) / vrot;
        			  r = r*1000.0;
        			  double icc_x = 0;
        			  double icc_y = r;
 //       			  COMP->cdlLookupdebug <<" icc_y: "<<icc_y;
				  if(icc_y<0){
					r = abs00(r);
				  }

        			  //calculate point on line
        			  double s, alpha, h,ys, r_m;
        			  r_m = r/1000.0;
        			  s = vtrans * deltat;
        			  alpha = s/r;
        			  h = sin(alpha) * r_m;
        			  ys = cos(alpha) * r_m;

   //     			  COMP->cdlLookupdebug <<" deltat: "<<deltat<<" s: "<<s<<" alpha: "<<alpha<<" h: "<<h<<" ys: "<<ys;

        			  res_x = h*1000;
        			  if(icc_y<0){
        				  res_y = icc_y + (ys*1000.0);
        			  } else {
        				  res_y = icc_y - (ys*1000.0);
        			  }

        		  } else {

        			  double s;
        			  s = vtrans * deltat;
        			  res_x = s;
        			  res_y = 0;
        		  }

        		  //std::cout<<"res_x: "<<res_x<<" res_y: "<<res_y<<std::endl;

        		  double predicted_pose_goaldist = std::sqrt( square(res_x-pathNavGoalPoint_x)+square(res_y-pathNavGoalPoint_y) );
        		  double current_pose_goaldist = std::sqrt( square(pathNavGoalPoint_x)+square(pathNavGoalPoint_y) );

#if DEBUG_CDL_LOOKUP_FILE_OUT
        		  if(COMP->cdlLookupdebug.is_open()){
//				COMP->cdlLookupdebug << " pathNavGoalPoint: "<<pathNavGoalPoint_x<<"|"<<pathNavGoalPoint_x;
//				COMP->cdlLookupdebug << " predPose: "<<res_x<<"|"<<res_y;
				COMP->cdlLookupdebug << " curPosDist: "<<current_pose_goaldist<<" predPosDist: "<<predicted_pose_goaldist;
        			  if(current_pose_goaldist>predicted_pose_goaldist){
        				  COMP->cdlLookupdebug << "DIR OK! ";
        			  } else {
        				  COMP->cdlLookupdebug << "DIR FALSE! ";
        			  }
        		  }
#endif
        		  if(current_pose_goaldist>predicted_pose_goaldist){

        			  if(vtrans > desiredTranslationalSpeed){
        				  costValue = 0;
        			  } else {

						  costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;

						  alpha1=0.25;
						  alpha2=1.0;
						  costValue=alpha1*costSpeed+alpha2*costGoalDist;
        			  }

        		  } else {

        			  costValue = 0;


        		  }

#if DEBUG_CDL_LOOKUP_FILE_OUT
        		  if(COMP->cdlLookupdebug.is_open()){
    				  COMP->cdlLookupdebug << " costGoalDist: "<<costGoalDist<< " costSpeed "<<costSpeed;
        			  COMP->cdlLookupdebug << " Cost: "<<costValue<<std::endl;
        		  }
#endif





        		  break;
        	  }







                case CDL_STRATEGY_10:
                  // ------------------------------------------------
                  // - assumes absolute current position
                  // - assumes absolute goal position
                  //
                  // - calculate position when assuming maximum
                  //   allowed decceleration on selected curvature
                  //   for determining all the relevant cost values
                  // ------------------------------------------------

                  //
                  // calculate stop position
                  //
                  if (abs00(vrot) <= vRotStepHelp)
                    {
                      //
                      //
                      // drive straight forward
                      //
                      //
                      if (abs00(vtrans) <= vTransStepHelp)
                        {
                          // no movement at all, but set brakeDist to an
                          // uncritical value to get the heading direction
                          // for determining the costGoal value
                          brakeDist  = 1.0;
                          brakeAngle = 0.0;
                        }
                      else
                        {
                          // robot moves straight forward
                          brakeDist  = BRAKE_SECURITY_FACTOR*(vtrans*vtrans)/(2*vAccMax);
                          brakeAngle = 0.0;
                        }
                      posXstop     = posX+brakeDist*cos(posA);
                      posYstop     = posY+brakeDist*sin(posA);
                      posAstop     = angle00(posA);

                      // intersection of circle around goal position
                      // and line through current position and goal position
                      status = intersectionsCircleLine(
                                 goalX,goalY,cdl_capture_distance,
                                 posX,posY,posXstop,posYstop,
                                 x1i,y1i,x2i,y2i,
                                 nrIntersections);
                      if ((status == 0) && (nrIntersections > 0))
                        {
                          // now check whether the intersections are in
                          // driving direction
                          double alpha1,alpha2,alpha;

                          alpha1 = atan2(posYstop-posY,posXstop-posX);
                          alpha2 = atan2(y1i-posY,x1i-posX);
                          alpha  = angle00(alpha2 - alpha1);

                          if ((alpha > -M_PI) && (alpha < M_PI))
                            {
                              // correct heading to hit the goal point therefore
                              // driving straight forward allows to hit the goal
                              costPassing = 1;
                            };
                        }
                      else
                        {
                          // goal point is in reverse direction to current heading or
                          // is too far away from driving direction
                          costPassing = 0;
                        }

                      // calculate whether goal point hits the goal position
                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }

#if DEBUG_VW_WINDOW
                      if (file!=NULL)
                        {
                          fprintf(file,"drive straight forward\n");
                          fprintf(file,"brakeDist brakeAngle %f %f\n",
                                  brakeDist,deg00(brakeAngle));
                          fprintf(file,"stopPosX stopPosY stopPosA %f %f %f\n",
                                  posXstop,posYstop,deg00(posAstop));
                          fprintf(file,"hitDistance hitAngle %f %f\n",
                                  hitDistance,deg00(hitAngle));
                          fprintf(file,"costHit costPassing %f %f\n",
                                  costHit,costPassing);
                        }
#endif

                    }
                  else if (abs00(vtrans) <= vTransStepHelp)
                    {
                      //
                      //
                      // turn in place
                      //
                      //
                      brakeDist       = 0.0;
                      brakeAngle      = BRAKE_SECURITY_FACTOR*(vrot*vrot)/(2*wAccMax);
                      if (vrot < 0.0) brakeAngle *= -1.0;
                      posXstop        = posX;
                      posYstop        = posY;
                      posAstop        = angle00(posA+brakeAngle);
                      deltaX          = goalX - posXstop;
                      deltaY          = goalY - posYstop;
                      passingDistance = sqrt(deltaX*deltaX+deltaY*deltaY);

                      if (passingDistance<cdl_capture_distance)
                        {
                          // turning in place allows to hit the goal since the
                          // robot is already within the circle around the goal
                          costPassing = 1;
                        }
                      else
                        {
                          costPassing = 0;
                        }

                      // calculate whether goal point hits the goal position
                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }

#if DEBUG_VW_WINDOW
                      if (file!=NULL)
                        {
                          fprintf(file,"turn in place\n");
                          fprintf(file,"brakeDist brakeAngle %f %f\n",
                                  brakeDist,deg00(brakeAngle));
                          fprintf(file,"costHit costPassing %f %f\n",
                                  costHit,costPassing);
                        }
#endif

                    }
                  else if (vrot > 0.0)
                    {
                      //
                      //
                      // drive on curvature, left
                      //
                      //
                      brakeAngle      = BRAKE_SECURITY_FACTOR*(vrot*vrot)/(2*wAccMax);
                      radius          = vtrans/vrot;
                      posXm           = posX  - radius * sin(posA);
                      posYm           = posY  + radius * cos(posA);
                      posXstop        = posXm + radius * sin(posA+brakeAngle);
                      posYstop        = posYm - radius * cos(posA+brakeAngle);
                      posAstop        = angle00(posA+brakeAngle);

                      deltaX          = posXm - goalX;
                      deltaY          = posYm - goalY;
                      passingDistance = abs00(sqrt(deltaX*deltaX+deltaY*deltaY)-radius);
                      if (passingDistance<cdl_capture_distance)
                        {
                          // the considered path goes through the specified
                          // goal point within a reasonable bound
                          costPassing = 1;
                        }
                      else
                        {
                          costPassing = 0;
                        }

                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }

#if DEBUG_VW_WINDOW
                      if (file!=NULL)
                        {
                          fprintf(file,"drive on curvature, left\n");
                          fprintf(file,"brakeAngle %f\n",
                                  deg00(brakeAngle));
                          fprintf(file,"stopPosX stopPosY stopPosA %f %f %f\n",
                                  posXstop,posYstop,deg00(posAstop));
                          fprintf(file,"posXm posYm deltx delty radius %f %f %f %f %f\n",
                                  posXm,posYm,deltaX,deltaY,radius);
                          fprintf(file,"hitDistance hitAngle %f %f\n",
                                  hitDistance,deg00(hitAngle));
                          fprintf(file,"costHit costPassing %f %f\n",
                                  costHit,costPassing);
                        }
#endif
                    }
                  else
                    {
                      //
                      //
                      // drive on curvature, right
                      //
                      //
                      brakeAngle      = -BRAKE_SECURITY_FACTOR*(vrot*vrot)/(2*wAccMax);
                      radius          = -vtrans/vrot;   // assume positive radius
                      posXm           = posX  + radius * sin(posA);
                      posYm           = posY  - radius * cos(posA);
                      posXstop        = posXm - radius * sin(posA+brakeAngle);
                      posYstop        = posYm + radius * cos(posA+brakeAngle);
                      posAstop        = angle00(posA+brakeAngle);
                      deltaX          = posXm - goalX;
                      deltaY          = posYm - goalY;
                      passingDistance = abs00(sqrt(deltaX*deltaX+deltaY*deltaY)-radius);

                      if (passingDistance<cdl_capture_distance)
                        {
                          // the considered path goes through the specified
                          // goal point within a reasonable bound
                          costPassing = 1;
                        }
                      else
                        {
                          costPassing = 0;
                        }

                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }

#if DEBUG_VW_WINDOW
                      if (file!=NULL)
                        {
                          fprintf(file,"drive on curvature, right\n");
                          fprintf(file,"brakeAngle %f\n",
                                  deg00(brakeAngle));
                          fprintf(file,"stopPosX stopPosY stopPosA %f %f %f\n",
                                  posXstop,posYstop,deg00(posAstop));
                          fprintf(file,"posXm posYm deltx delty radius %f %f %f %f %f\n",
                                  posXm,posYm,deltaX,deltaY,radius);
                          fprintf(file,"hitDistance hitAngle %f %f\n",
                                  hitDistance,deg00(hitAngle));
                          fprintf(file,"costHit costPassing %f %f\n",
                                  costHit,costPassing);
                        }
#endif
                    }

                  deltaX = posX - goalX;
                  deltaY = posY - goalY;
                  goalDistance = sqrt(deltaX * deltaX + deltaY * deltaY);

                  //
                  //
                  // evaluation functions
                  //
                  //
                  switch (evalFunc)
                    {
                    case CDL_EVAL_STANDARD:
                      //
                      //
                      //
                      break;
                    case CDL_EVAL_APPROACH_OBJECT:
                      //
                      //
                      //
                      break;
                    case CDL_EVAL_PASSING:
                      //
                      // std::cout << "CDL_EVAL_PASSING: posX = " << posX << "; goalX = " << goalX <<"; posY = "<<posY<< "; goalY = "<< goalY <<  "; goalDistance = " << goalDistance << std::endl;
                      //
                      if (goalDistance > rangeFarPassing)
                        {
                          desiredTranslationalSpeed = speedFarPassing;
                        }
                      else if (goalDistance > rangeMediumPassing)
                        {
                          desiredTranslationalSpeed = speedMediumPassing;
                        }
                      else
                        {
                          desiredTranslationalSpeed = speedNearPassing;
                        }
                      costHeading = 1.0-abs00(hitAngle)/M_PI;

                      costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                      if (costSpeed < 0.0) costSpeed = 0.0;

                      costDist=dist/cdl_max_distance;
                      if (costDist > 1.0) costDist = 1.0;

// if (indexWanted[ci]==0) {
//   // curvature is not wanted
//   costValue = 10000.0;
// } else {
//   // curvature is wanted
//   costValue = 20000.0;
// };
                      costValue = 0.0;
//                costValue += 1.0*costSpeed+1.0*costDist+2000.0*costHeading;
                      costValue += 1.0*costSpeed+1.0*costDist+200.0*costHeading;
                      costValue += 2*costPassing;

                      if (dist > bonusDistPassing)
                        {
                          costValue += 5000.0;
                        }

                      if ((vtrans > 5.0) && (vtrans < 100.0)) costValue = 0.0;
                      if (fabs(vtrans) < 5.0)
                        {
                          if (vrot >  rad00(10)) vrot =  rad00(10);
                          if (vrot < -rad00(10)) vrot = -rad00(10);
                        }

                      alpha  = angle00(posA - atan2(goalY-posY,goalX-posX));
                      if ((fabs(alpha) > rad00(45)) && (vtrans > 400.0)) costValue=0;


                      break;
                    case CDL_EVAL_STOPPING:
                      //
                      //
                      // std::cout << "CDL_EVAL_STOPPING: posX = " << posX << "; goalX = " << goalX <<"; posY = "<<posY<< "; goalY = "<< goalY <<  "; goalDistance = " << goalDistance << std::endl;
                      //
                      if (goalDistance > rangeFarStopping)
                        {
                          desiredTranslationalSpeed = speedFarStopping;
                        }
                      else if (goalDistance > rangeMediumStopping)
                        {
                          desiredTranslationalSpeed = speedMediumStopping;
                        }
                      else
                        {
                          desiredTranslationalSpeed = speedNearStopping;
                        }
                      costHeading = 1.0-abs00(hitAngle)/M_PI;

                      costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                      if (costSpeed < 0.0) costSpeed = 0.0;

                      costDist=dist/cdl_max_distance;
                      if (costDist > 1.0) costDist = 1.0;

// if (indexWanted[ci]==0) {
//   // curvature is not wanted
//   costValue = 10000.0;
// } else {
//   // curvature is wanted
//   costValue = 20000.0;
// };

                      costValue = 0.0;
//                costValue += 1.0*costSpeed+1.0*costDist+2000.0*costHeading;
//                costValue += 1.0*costSpeed+1.0*costDist+200.0*costHeading; //Lutz
                      costValue += 1.0*costSpeed+1.0*costDist+200.0*costHeading;
                      costValue += 2*costHit;

                      if (dist > bonusDistStopping)
                        {
                          costValue += 5000.0;
                        }

                      if ((vtrans > 5.0) && (vtrans < 50.0)) costValue = 0.0;
                      if (fabs(vtrans) < 5.0)
                        {
                          if (vrot >  rad00(10)) vrot =  rad00(10);
                          if (vrot < -rad00(10)) vrot = -rad00(10);
                        }
                      alpha  = angle00(posA - atan2(goalY-posY,goalX-posX));
                      if ((fabs(alpha) > rad00(45)) && (vtrans > 400.0)) costValue=0;

                      break;


                    case CDL_EVAL_APPROACH_EXACT:

                      if (indexWanted[ci]==0)
                        {
                          // curvature is not wanted
                          costValue = 10000.0;
                        }
                      else
                        {
                          // curvature is wanted
                          costValue = 20000.0;
                        };

                      costHeading = 1.0-abs00(hitAngle)/M_PI;

                      costDist=dist/cdl_max_distance;
                      if (costDist > 1.0) costDist = 1.0;

// if (goalDistance > 2500.0) {
//   desiredTranslationalSpeed = 100.0;
// } else if (goalDistance > 1000.0) {
//   desiredTranslationalSpeed = 100.0;
// } else {
//   desiredTranslationalSpeed = 50.0;
// }
// costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
// if (costSpeed < 0.0) costSpeed = 0.0;
// costValue += 2000.0*costHeading+1.0*costSpeed+1.0*costDist;

                      if (abs00(vrot) > rad00(30.0)) costValue = 0.0;
                      if (goalDistance > 1000.0)
                        {
                          if ((vtrans<95.0) || (vtrans>105.0)) costValue = 0.0;
                        }
                      else
                        {
                          if ((vtrans <45.0) || (vtrans>55.0)) costValue = 0.0;
                        }
                      costValue += 2000.0*costHeading;

                      break;
                    default:
                      costValue = 0.0;
                      break;
                    }


#if EVAL_FUNC_1
                  {
                    double free = 1000.0;
                    double angle = (60.0*M_PI/180.0);

                    if (hitAngle < -angle) hitAngle=-angle;
                    if (hitAngle >  angle) hitAngle= angle;
                    costHeading=1.0-abs00(angle00(hitAngle-vrot))/(2*angle);

                    if (hitDistance > free)
                      {
                        desiredTranslationalSpeed = 900.0;
                      }
                    else
                      {
                        desiredTranslationalSpeed = 200.0;
                      }
                    costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;
                    costDist=dist/5000.0;
                    if (costDist>1.0) costDist=1.0;

                    if ((dist > free) && (vtrans>100.0))
                      {
                        alpha1=1.0;
                        alpha2=1.0;
                        alpha3=2.0;
                        costValue=5+alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                      }
                    else
                      {
                        alpha1=1.0;
                        alpha2=1.0;
                        alpha3=1.0;
                        costValue=alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                      }
                  }
#endif

                  //
                  //
                  // evaluation function 3
                  //
                  //
#if EVAL_FUNC_3
                  costValue = 0;
                  if (goalDistance < 500.0)
                    {
                      desiredTranslationalSpeed = 50.0;
                    }
                  else if (goalDistance < 1000.0)
                    {
                      desiredTranslationalSpeed = 100.0;
//              costValue += costHit*10.0;
                    }
                  else
                    {
                      desiredTranslationalSpeed = 900.0;
//              if ((dist > 1000.0) || (angle > rad00(20.0))) costValue += 6;
//              if (abs00(deg00(hitAngle)) < 10.0) costValue+=3;
                      costValue += costPassing;
                    }

                  costHeading = 1.0-abs00(hitAngle)/M_PI;
                  costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                  if (costSpeed < 0.0) costSpeed = 0.0;
                  costDist=dist/CDL_MAX_DISTANCE;
                  if (costDist > 1.0) costDist = 1.0;

                  if ((dist > 1000.0) && (vtrans > 100.0))
                    {
                      alpha1 = 1.0;
                      alpha2 = 1.0;
                      alpha3 = 2.0;
                      costValue=5+alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                    }
                  else
                    {
                      alpha1 = 1.0;
                      alpha2 = 1.0;
                      alpha3 = 2.0;
                      costValue=alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                    }

                  evaluation[vi][wi].costHit      = costHit;
                  evaluation[vi][wi].costPassing  = costPassing;
                  evaluation[vi][wi].costSpeed    = costSpeed;
                  evaluation[vi][wi].costDistance = costDist;
                  evaluation[vi][wi].costHeading  = costHeading;
                  evaluation[vi][wi].costValue    = costValue;

#endif

                  //
                  //
                  // evaluation function 4
                  //
                  //
#if EVAL_FUNC_4
                  if (hitDistance > 2000.0)
                    {
                      desiredTranslationalSpeed = 500.0;
                    }
                  else
                    {
                      desiredTranslationalSpeed = 100.0;
                    }
                  if (dist > 1500.0)
                    {
                      costHeading = 1.0-abs00(hitAngle)/M_PI;
                      costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                      if (costSpeed < 0.0) costSpeed = 0.0;
                      costDist=dist/CDL_MAX_DISTANCE;
                      if (costDist > 1.0) costDist = 1.0;
                      costValue = 1.0*costSpeed+1.0*costDist+1000.0*costHeading;
                    }
                  else
                    {
                      costValue = 0.0;
                    }
#endif

#if DEBUG_VW_WINDOW
                  if (file!=NULL)
                    {
                      fprintf(file,"evaluation\n");
                      fprintf(file,"  hitAngle     %f\n",deg00(hitAngle));
                      fprintf(file,"  hitDistance  %f\n",hitDistance);
                      fprintf(file,"  costHeading  %f\n",costHeading);
                      fprintf(file,"  costSpeed    %f\n",costSpeed);
                      fprintf(file,"  costPassing  %f\n",costPassing);
                      fprintf(file,"  costValue    %f\n",costValue);
                    }
#endif

                  break;


                case CDL_STRATEGY_11:
                  //
                  // this strategy is used to turn in place
                  //
                  // the desired heading is specified by the current
                  // robot position and the goal position
                  //
                  // important hint:
                  // to prevent overshooting the goal orientation, hitAngles
                  // stopping before the goal orientation are better than
                  // hitAngles stopping after the desired orientation. Therefore,
                  // the evaluation function considers the turning direction
                  //
                {
                  double desiredHeading;   //     [-M_PI, M_PI]

                  desiredHeading = angle00(atan2(goalY-posY,goalX-posX));

                  double angleDelta = angle00(desiredHeading-posA);
                  double vInter = linearinterpolation(rotwCVector,abs00(angleDelta*180/M_PI));
		  vInter = vInter/180*M_PI;
                  if(angleDelta < 0 )
                  {
                	  vInter = -vInter;
                  }

                  double error = abs00(vInter - vrot);

                  if(error > 1.0){
                	  costValue = min00(1.0/error,1.00);
                  } else {
			  
                	  costValue = 100.0 + (1.0 - error);
			
                  }
//		  std::cout<<"AngleDelta: "<<angleDelta<<" vInter: "<<vInter<<" vrot: "<<vrot<<" error: "<<error<<" costValue: "<<costValue<<std::endl;

//                  if (abs00(vrot) <= vRotStepHelp)
//                    {
//                      //
//                      // no movement at all
//                      //
//                      brakeAngle = 0.0;
//                      posAstop   = posA;
//                    }
//                  else
//                    {
//                      //
//                      // robot turns in place
//                      //
//                      brakeAngle = (vrot*vrot)/(2*wAccMax);
//                      if (vrot < 0.0) brakeAngle *= -1.0;
//                      // factor 2.0 is used to damp the rotational velocity
//                      posAstop   = angle00(posA+10.0*brakeAngle);
//                    }
//                  hitAngle = angle00(desiredHeading-posAstop);
//
//                  if ((vrot > 0.0) && (hitAngle >= 0.0))
//                    {
//                      // turning left
//                      costHeading = 1.0-abs00(hitAngle)/M_PI;
//                    }
//                  else if ((vrot < 0.0) && (hitAngle <= 0.0))
//                    {
//                      // turning right
//                      costHeading = 1.0-abs00(hitAngle)/M_PI;
//                    }
//                  else
//                    {
//                      // overshooting
//                      costHeading = 0.0;
//                    }
//		  std::cout<<"vrot: "<<vrot<<" costValue: "<<costValue<<std::endl;
                  //costValue = costHeading;

//                  evaluation[vi][wi].costHit      = 0.0;
//                  evaluation[vi][wi].costPassing  = 0.0;
//                  evaluation[vi][wi].costSpeed    = 0.0;
//                  evaluation[vi][wi].costDistance = 0.0;
//                  evaluation[vi][wi].costHeading  = costHeading;
//                  evaluation[vi][wi].costValue    = costValue;

#if DEBUG_VW_WINDOW
                  if (file!=NULL)
                    {
                      fprintf(file,"rotate in place\n");
                      fprintf(file,"pos  x y a %f %f %f\n",posX,posY,deg00(posA));
                      fprintf(file,"goal x y   %f %f\n",goalX,goalY);
                      fprintf(file,"desiredHeading %f\n",deg00(desiredHeading));
                      fprintf(file,"brakeAngle  %f\n",deg00(brakeAngle));
                      fprintf(file,"posAstop    %f\n",deg00(posAstop));
                      fprintf(file,"hitAngle    %f\n",deg00(hitAngle));
                      fprintf(file,"costHeading %f\n",costHeading);
                    }
#endif
                }
                break;

//<startegy added by asteck, mlutz; 2009-10-23>
                case CDL_STRATEGY_12:
                {
                  //double free = 1000.0;
                  double angle = (60.0*M_PI/180.0);

                  double deltaX = goalX - posX;
                  double deltaY = goalY - posY;
                  double hitDistance = sqrt( deltaX*deltaX + deltaY*deltaY );
                  double hitAngle = angle00( atan2( deltaY, deltaX) - posA );


                  if (hitAngle < -angle) hitAngle=-angle;
                  if (hitAngle >  angle) hitAngle= angle;
                  costHeading=1.0-abs00(angle00(hitAngle-vrot))/(2*angle);

                  costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;
                  costDist=dist/cdl_max_distance; // CDL_MAX_DISTANCE = 5000.0

                  if (costDist>1.0) costDist=1.0;

                  double gainHitDist = hitDistance / 1500.0; //hitDistance / 1000.0;
                  if(gainHitDist >1.0) gainHitDist = 1.0;

                  if(evalFunc == CDL_EVAL_PASSING) gainHitDist = 1.0;

                  alpha1 = 20.0 - (1-gainHitDist) *  19.0; // 20.0 - 19.0
                  alpha2 =  2.0 - (1-gainHitDist) *   1.0; // 2.0  - 1.0
                  alpha3 =  1.0 + (1-gainHitDist) * 100.0; // 1.0  + 100.0
                  costValue = alpha1*costSpeed + alpha2*costDist + alpha3*costHeading;

                  //std::cout << "vtrans = " << vtrans << "; vrot = " << vrot/M_PI*180.0 <<  "; costSpeed = " << costSpeed << " ; costDist = " << costDist << " ; costHeading = " << costHeading << " ; costValue = " << costValue << std::endl;

/*
                  //if ((dist > free) && (vtrans>100.0))
                  if ((hitDistance > free) && (vtrans>100.0))
                  {
                    alpha1=5.0;
                    alpha2=15.0;
                    alpha3=2.0;
                    costValue=5+alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                  }
                  else
                  {
                    alpha1=2.0;
                    alpha2=1.0;
                    alpha3=8.0;
                    costValue=alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                  }
*/
                  break;
                }
//</strategy 12>


		case CDL_STRATEGY_13:
                {
                  costHeading=1.0-abs00(angle00(vrot));

                  costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;

                  //std::cout<<"vrot: "<<vrot<<" costHeading: "<<costHeading<<" vtrans: "<<vtrans<<" costSpeed: "<<costSpeed;

                  if(abs00(vrot)> 0.10 )
                  {
						 costValue=0;
                  }else{
                     costValue = costSpeed + costHeading;
                  }

		  //std::cout<<" costValue: "<<costValue<<std::endl;
                  break;
                }
//</strategy 13>

                //<startegy added by mlutz; 2011-06-08>
		case CDL_STRATEGY_14: {

			//double free = 1000.0;
			double angle = (60.0*M_PI/180.0);

			double deltaX = goalX - posX;
			double deltaY = goalY - posY;
			double hitAngle = angle00( atan2( deltaY, deltaX) - posA );
			if (hitAngle < -angle) hitAngle=-angle;
			if (hitAngle >  angle) hitAngle= angle;
			costHeading=1.0-abs00(angle00(hitAngle-vrot))/(2*angle);

			costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;
			costDist=dist/cdl_max_distance; // CDL_MAX_DISTANCE = 5000.0
			costDist = 0;
			if (costDist>1.0) costDist=1.0;

			double gainHitDist = hitDistance / 1500.0; //hitDistance / 1000.0;
			if(gainHitDist >1.0) gainHitDist = 1.0;

			if(evalFunc == CDL_EVAL_PASSING) gainHitDist = 1.0;

			// alpha1 = 20.0 - (1-gainHitDist) *  19.0; // 20.0 - 19.0
			// alpha2 =  2.0 - (1-gainHitDist) *   1.0; // 2.0  - 1.0
			// alpha3 =  1.0 + (1-gainHitDist) * 100.0; // 1.0  + 100.0
			alpha1 = 1;
			alpha2 = 1;
			alpha3 = 2;
			// costValue = alpha1*costSpeed + alpha2*costDist + alpha3*costHeading;
			costValue = alpha1*costSpeed + alpha3*costHeading;

			  //std::cout <<" vtrans = " << vtrans << "; vrot = " << vrot <<  "; costSpeed = " << costSpeed << " ; costDist = " << costDist << " ; costHeading = " << costHeading << " ; costValue = " << costValue << std::endl;

		  //}

		  break;
		}
                //</strategy 14>

                 case CDL_STRATEGY_2:
                  // ----------------------------------------------------
                  // - akzeptiert Geschwindigkeitsvorgabe von aussen
                  // ----------------------------------------------------
                {
                  double free = 1000.0;
                  double angle = (60.0*M_PI/180.0);

                  if (goalHeadingRelative < -angle) goalHeadingRelative=-angle;
                  if (goalHeadingRelative >  angle) goalHeadingRelative= angle;
                  costHeading=1.0-abs00(angle00(goalHeadingRelative-vrot))/(2*angle);

                  costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;
                  costDist=dist/maxDistance;
                  if (costDist>1.0) costDist=1.0;
                  if ((dist > free) && (vtrans>100.0))
                    {
                      alpha1=1.0; //1
                      alpha2=1.0; //1
                      alpha3=2.0; //2
                      costValue=5+alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                    }
                  else
                    {
                      alpha1=6.0; // 6
                      alpha2=3.0; // 3
                      alpha3=1.0; // 1
                      costValue=alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                    }

                }
                break;

                case CDL_STRATEGY_3:
                  // ----------------------------------------------------
                  // - macht Geschwindigkeitsregelung selbstaendig
                  // - muss man noch fertig implementieren
                  // ----------------------------------------------------
                {
                  double angle = (40.0*M_PI/180.0);

                  if (goalHeadingRelative < -angle) goalHeadingRelative=-angle;
                  if (goalHeadingRelative >  angle) goalHeadingRelative= angle;

                  if ((dist < 1000.0) || (abs00(vrot) > angle) || (abs00(vtrans)<50.0))
                    {
                      // mindestens Restwegstrecke von 1000 mm
                      // mindestens Translationsgeschwindigkeit 100 mm/s
                      // maximal Drehung um 40 Grad/s
                      costValue = 0.0;
                    }
                  else
                    {
                      costHeading=1.0-abs00(angle00(goalHeadingRelative-vrot))/(2*angle);

                      costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;

                      costDist=dist/maxDistance;
                      if (costDist>1.0) costDist=1.0;
                      costValue=alpha1*costSpeed+alpha2*costDist+alpha3*costHeading;
                    }
                }
                break;
                case CDL_STRATEGY_4:
                  // -------------------------------------------------------
                  // used for testing purposes
                  // - je groesser die Abweichung von der Zielrichtung, desto hoeher w
                  // - je groesser die Entfernung zu Ziel, desto hoeher v
                  // -------------------------------------------------------
// vtrans, vrot         aktuell betrachtete Geschwindigkeit
// dist                 verbleibende Reststrecke
// goalHeadingRelative  Ausrichtungsfehler bezueglich Ziel
//
                {
                  double angle = (90*M_PI/180.0);
                  double mind  = 1000.0;
                  double minv  = 50.0;
                  double k     = 2.0;               // Linearitaetsfaktor
                  double wmax  = (50.0*M_PI/180.0);   // maximal erlaubte Rotationsgeschwindigkeit
                  double amax  = 30.0;              // zur Berechnung der maximal erlaubten Geschwindigkeit

                  double wsoll;
                  double vsoll;

                  // Begrenzung des Winkelfehlers
                  if (goalHeadingRelative < -angle) goalHeadingRelative=-angle;
                  if (goalHeadingRelative >  angle) goalHeadingRelative= angle;

                  // Bestimmung der Solldrehgeschwindigkeit, begrenzt durch wmax
//            wsoll = sqrt(2*goalHeadingRelative*wmax);
                  wsoll = goalHeadingRelative*k;
                  if (wsoll > wmax) wsoll =  wmax;
                  if (wsoll <-wmax) wsoll = -wmax;

                  // Bestimmung der Sollfahrgeschwindigkeit
                  if (maxDistance < 200.0) vsoll=0.0;
                  else vsoll=sqrt(2*maxDistance*amax);

                  if ((dist < mind) || (abs00(vrot) > wsoll) || (abs00(vtrans) < minv))
                    {
                      costValue = 0.0;
                    }
                  else
                    {
                      costHeading = 1.0 - min00(abs00(wsoll-vrot)/(2*wmax),1.0);
                      costSpeed   = 1.0 - min00(abs00(vsoll-vtrans)/vmaxIni,1.0);
                      costDist    = min00(dist/maxDistance,1.0);
// vtrans=0.0;
                      costValue = 1.0 * costSpeed + 1.0 * costDist + 1.0 * costHeading;
                    }
                  break;
                }


//<asteck>
                case CDL_STRATEGY_5:
                {
                  // ------------------------------------------------------------------------
                  // used for v-w commands from "outside" (for example from joystick)
                  // commands should be set with setDesiredTranslationSpeed and setHeading
                  // ------------------------------------------------------------------------

                  double free   = 1000.0;
                  double angle  = (60.0*M_PI/180.0);

                  if (goalHeadingRelative < -angle) goalHeadingRelative=-angle;
                  if (goalHeadingRelative >  angle) goalHeadingRelative= angle;

                  costHeading = 1.0 - abs00( angle00(goalHeadingRelative-vrot)) / (2*angle);
                  costSpeed   = 1.0 - abs00( desiredTranslationalSpeed-vtrans) / vmaxIni;

                  costDist    = 1; //dist/maxDistance;

                  alpha1 = 1.0;
                  alpha2 = 1.0;
                  alpha3 = 3.0;

                  //costValue =  alpha1*costSpeed + alpha3*costHeading;
                  costValue = alpha1*costSpeed + alpha3*costHeading;

                  //if( vtrans == 350 )
                  //  printf("STRAT v ; w ; costValue ; dist -- %5.2f ; %5.2f ; %10.2f ; %6.2f\n", vtrans , vrot, costValue, dist);

                  break;
                } // CDL_STRATEGY_5

//</asteck>

                case CDL_STRATEGY_6:
                {
                  // ------------------------------------------------
                  // - assumes absolute current position
                  // - assumes absolute goal position
                  //
                  // - calculate position when assuming maximum
                  //   allowed decceleration on selected curvature
                  //   for determining all the relevant cost values
                  // ------------------------------------------------

                  //
                  // calculate stop position
                  //
                  if (abs00(vrot) <= vRotStepHelp)
                    {
                      //
                      //
                      // drive straight forward
                      //
                      //
                      if (abs00(vtrans) <= vTransStepHelp)
                        {
                          // no movement at all, but set brakeDist to an
                          // uncritical value to get the heading direction
                          // for determining the costGoal value
                          brakeDist  = 1.0;
                          brakeAngle = 0.0;
                        }
                      else
                        {
                          // robot moves straight forward
                          brakeDist  = BRAKE_SECURITY_FACTOR*(vtrans*vtrans)/(2*vAccMax);
                          brakeAngle = 0.0;
                        }
                      posXstop     = posX+brakeDist*cos(posA);
                      posYstop     = posY+brakeDist*sin(posA);
                      posAstop     = angle00(posA);

                      // intersection of circle around goal position
                      // and line through current position and goal position
                      status = intersectionsCircleLine(
                                 goalX,goalY,cdl_capture_distance,
                                 posX,posY,posXstop,posYstop,
                                 x1i,y1i,x2i,y2i,
                                 nrIntersections);
                      if ((status == 0) && (nrIntersections > 0))
                        {
                          // now check whether the intersections are in
                          // driving direction
                          double alpha1,alpha2,alpha;

                          alpha1 = atan2(posYstop-posY,posXstop-posX);
                          alpha2 = atan2(y1i-posY,x1i-posX);
                          alpha  = angle00(alpha2 - alpha1);

                          if ((alpha > -M_PI) && (alpha < M_PI))
                            {
                              // correct heading to hit the goal point therefore
                              // driving straight forward allows to hit the goal
                              costPassing = 1;
                            };
                        }
                      else
                        {
                          // goal point is in reverse direction to current heading or
                          // is too far away from driving direction
                          costPassing = 0;
                        }

                      // calculate whether goal point hits the goal position
                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }


                    }
                  else if (abs00(vtrans) <= vTransStepHelp)
                    {
                      //
                      //
                      // turn in place
                      //
                      //
                      brakeDist       = 0.0;
                      brakeAngle      = BRAKE_SECURITY_FACTOR*(vrot*vrot)/(2*wAccMax);
                      if (vrot < 0.0) brakeAngle *= -1.0;
                      posXstop        = posX;
                      posYstop        = posY;
                      posAstop        = angle00(posA+brakeAngle);
                      deltaX          = goalX - posXstop;
                      deltaY          = goalY - posYstop;
                      passingDistance = sqrt(deltaX*deltaX+deltaY*deltaY);

                      if (passingDistance<cdl_capture_distance)
                        {
                          // turning in place allows to hit the goal since the
                          // robot is already within the circle around the goal
                          costPassing = 1;
                        }
                      else
                        {
                          costPassing = 0;
                        }

                      // calculate whether goal point hits the goal position
                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }

                    }
                  else if (vrot > 0.0)
                    {
                      //
                      //
                      // drive on curvature, left
                      //
                      //
                      brakeAngle      = BRAKE_SECURITY_FACTOR*(vrot*vrot)/(2*wAccMax);
                      radius          = vtrans/vrot;
                      posXm           = posX  - radius * sin(posA);
                      posYm           = posY  + radius * cos(posA);
                      posXstop        = posXm + radius * sin(posA+brakeAngle);
                      posYstop        = posYm - radius * cos(posA+brakeAngle);
                      posAstop        = angle00(posA+brakeAngle);

                      deltaX          = posXm - goalX;
                      deltaY          = posYm - goalY;
                      passingDistance = abs00(sqrt(deltaX*deltaX+deltaY*deltaY)-radius);
                      if (passingDistance<cdl_capture_distance)
                        {
                          // the considered path goes through the specified
                          // goal point within a reasonable bound
                          costPassing = 1;
                        }
                      else
                        {
                          costPassing = 0;
                        }

                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }

                    }
                  else
                    {
                      //
                      //
                      // drive on curvature, right
                      //
                      //
                      brakeAngle      = -BRAKE_SECURITY_FACTOR*(vrot*vrot)/(2*wAccMax);
                      radius          = -vtrans/vrot;   // assume positive radius
                      posXm           = posX  + radius * sin(posA);
                      posYm           = posY  - radius * cos(posA);
                      posXstop        = posXm - radius * sin(posA+brakeAngle);
                      posYstop        = posYm + radius * cos(posA+brakeAngle);
                      posAstop        = angle00(posA+brakeAngle);
                      deltaX          = posXm - goalX;
                      deltaY          = posYm - goalY;
                      passingDistance = abs00(sqrt(deltaX*deltaX+deltaY*deltaY)-radius);

                      if (passingDistance<cdl_capture_distance)
                        {
                          // the considered path goes through the specified
                          // goal point within a reasonable bound
                          costPassing = 1;
                        }
                      else
                        {
                          costPassing = 0;
                        }

                      deltaX      = goalX - posXstop;
                      deltaY      = goalY - posYstop;
                      hitDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                      if (hitDistance < cdl_capture_distance)
                        {
                          costHit = 1;
                        }
                      else
                        {
                          costHit = 0;
                        }

                      // calculate the alignment of the stop position with
                      // the goal point
                      if (hitDistance < cdl_capture_distance)
                        {
                          // distance is too small, therefore heading cannot be
                          // calculated correctly. Since heading in goal point
                          // is not relevant in this mode, set goalAngle to 0.
                          hitAngle = 0.0;
                        }
                      else
                        {
                          hitAngle = angle00(atan2(deltaY,deltaX)-posAstop);
                        }
                    }

                  deltaX = posX - goalX;
                  deltaY = posY - goalY;
                  goalDistance = sqrt(deltaX * deltaX + deltaY * deltaY);

                  //
                  //
                  // evaluation functions
                  //
                  //
                  switch (evalFunc)
                    {
                    case CDL_EVAL_STANDARD:
                      //
                      //
                      //
                      break;
                    case CDL_EVAL_APPROACH_OBJECT:
                      //
                      //
                      //
                      break;
                    case CDL_EVAL_PASSING:
                      //
                      // std::cout << "CDL_EVAL_PASSING: posX = " << posX << "; goalX = " << goalX <<"; posY = "<<posY<< "; goalY = "<< goalY <<  "; goalDistance = " << goalDistance << std::endl;
                      //
                      if (goalDistance > rangeFarPassing)
                        {
                          desiredTranslationalSpeed = speedFarPassing;
                        }
                      else if (goalDistance > rangeMediumPassing)
                        {
                          desiredTranslationalSpeed = speedMediumPassing;
                        }
                      else
                        {
                          desiredTranslationalSpeed = speedNearPassing;
                        }
                      costHeading = 1.0-abs00(hitAngle)/M_PI;

                      costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                      if (costSpeed < 0.0) costSpeed = 0.0;

                      costDist=dist/cdl_max_distance;
                      if (costDist > 1.0) costDist = 1.0;

                      costValue = 0.0;
//                costValue += 1.0*costSpeed+1.0*costDist+2000.0*costHeading;
                      costValue += 1.0*costSpeed+1.0*costDist+200.0*costHeading;
                      costValue += 2*costPassing;

                      if (dist > bonusDistPassing)
                        {
                          costValue += 5000.0;
                        }

                      if ((vtrans > 5.0) && (vtrans < 100.0)) costValue = 0.0;
                      if (fabs(vtrans) < 5.0)
                        {
                          if (vrot >  rad00(10)) vrot =  rad00(10);
                          if (vrot < -rad00(10)) vrot = -rad00(10);
                        }

                      alpha  = angle00(posA - atan2(goalY-posY,goalX-posX));
                      if ((fabs(alpha) > rad00(45)) && (vtrans > 400.0)) costValue=0;


                      break;
                    case CDL_EVAL_STOPPING:
                      if (goalDistance > rangeFarStopping)
                        {
                          desiredTranslationalSpeed = speedFarStopping;

                          costHeading = 1.0-abs00(hitAngle)/M_PI;
                          costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                          if (costSpeed < 0.0) costSpeed = 0.0;

                          costDist=dist/cdl_max_distance;
                          if (costDist > 1.0) costDist = 1.0;

                          costValue = 0.0;
                          costValue += 1.0*costSpeed+1.0*costDist+10.0*costHeading;
                          costValue += 2*costHit;

                        }
                      else if (goalDistance > rangeMediumStopping)
                        {
                          desiredTranslationalSpeed = speedMediumStopping;

                          costHeading = 1.0-abs00(hitAngle)/M_PI;
                          costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                          if (costSpeed < 0.0) costSpeed = 0.0;

                          costDist=dist/cdl_max_distance;
                          if (costDist > 1.0) costDist = 1.0;

                          costValue = 0.0;
                          costValue += 1.0*costSpeed+1.0*costDist+50.0*costHeading;
                          costValue += 2*costHit;

                        }
                      else
                        {
                          desiredTranslationalSpeed = speedNearStopping;

                          costHeading = 1.0-abs00(hitAngle)/M_PI;
                          costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/1000.0;
                          if (costSpeed < 0.0) costSpeed = 0.0;

                          costDist=dist/cdl_max_distance;
                          if (costDist > 1.0) costDist = 1.0;

                          costValue = 0.0;
                          costValue += 1.0*costSpeed+1.0*costDist+200.0*costHeading;
                          costValue += 2*costHit;

                        }





                      if (dist > bonusDistStopping)
                        {
                          costValue += 5000.0;
                        }

                      if ((vtrans > 5.0) && (vtrans < 50.0)) costValue = 0.0;
                      if (fabs(vtrans) < 5.0)
                        {
                          if (vrot >  rad00(10)) vrot =  rad00(10);
                          if (vrot < -rad00(10)) vrot = -rad00(10);
                        }
                      alpha  = angle00(posA - atan2(goalY-posY,goalX-posX));
                      if ((fabs(alpha) > rad00(45)) && (vtrans > 400.0)) costValue=0;

                      break;

                    default:
                      costValue = 0.0;
                      break;
                    }

		case CDL_STRATEGY_7:
                {
                  // ------------------------------------------------------------------------
                  // follow
                  // ------------------------------------------------------------------------



                	if(indexWantedVWCheck[ci]==1){
                		costHeading=1.0-abs00(angle00(desiredRotationalSpeed-vrot))/wmaxIni;
                		costSpeed=1.0-abs00(desiredTranslationalSpeed-vtrans)/vmaxIni;
                		alpha1 = 1;
                		alpha2 = 1;
                		alpha3 = 1;
                		costValue = alpha1*costSpeed + alpha3*costHeading;
//                		std::cout<<ci<<" - v|desv: "<<vtrans<<"|"<<desiredTranslationalSpeed<<" w|desw: "<<vrot<<"|"<<desiredRotationalSpeed<<" cost: "<<costValue<<"              v:"<<v<<" w:"<<w<<std::endl;
                	} else {
                		//					  std::cout<<ci<<" not wanted - v|desv: "<<vtrans<<"|"<<desiredTranslationalSpeed<<" w|desw: "<<vrot<<"|"<<desiredRotationalSpeed<<std::endl;
                		costValue = 0;
                	}

                	break;



// ALT 22.02.2018
//                  double free   = 1000.0;
//                  double angle  = (60.0*M_PI/180.0);
//
//                  if (goalHeadingRelative < -angle) goalHeadingRelative=-angle;
//                  if (goalHeadingRelative >  angle) goalHeadingRelative= angle;
//
//                  //gain function for goalheadingrelative
//                  // f(x) = 5+ 0.06* x^2
//                  double deg_goalHeadingRelative = goalHeadingRelative * (180.0/M_PI);
//                  double heading_gain_rad = 0.0;
//
//                  if(goalHeadingRelative >0.0)
//                  {
//                  double heading_gain_deg = 5 + (0.06 * deg_goalHeadingRelative * deg_goalHeadingRelative);
//                  heading_gain_rad = heading_gain_deg * M_PI/180.0;
//                  }else
//                  {
//                   double heading_gain_deg = -1.0 *(5 + (0.06 * deg_goalHeadingRelative * deg_goalHeadingRelative));
//                   heading_gain_rad = heading_gain_deg * M_PI/180.0;
//                  }
//
//                  heading_gain_rad = 0;
//
//                  costHeading = 1.0 - abs00( angle00(goalHeadingRelative + heading_gain_rad - vrot)) / (2*angle);
//                  costSpeed   = 1.0 - abs00( desiredTranslationalSpeed - vtrans) / vmaxIni;
//                  costDist    = dist/maxDistance;
//
//                  alpha1 = 1.0;
//                  alpha2 = 2.0;
//                  alpha3 = 8.0;
//
//
//                  costValue = alpha1*costSpeed + alpha2*costDist + alpha3*costHeading;

//                  break;
                } // CDL_STRATEGY_7



                  break;
                 }
                default:
                  // ----------------------------------------------------
                  //
                  // ----------------------------------------------------
                  std::cout<<"Debug CDL_Lookup in default!!\n";
                  costValue = 0.0;
                  break;
                } // end switch()

              if (costValue > costResult)
                {
            	  ciResult   = ci;
                  vResult    = vtrans;
                  wResult    = vrot;
                  vAccResult = vAccMax;
                  wAccResult = wAccMax;
                  costResult = costValue;
                }
            }
          else
            {

              // these velocities are not allowed
#if DEBUG_VW_WINDOW
              if (file!=NULL)
                {
                  fprintf(file,"=> these velocities are not allowed\n");
                }
#endif
            }
        }
    } // end for (vi=ivmin;vi<=ivmax;vi++)

    //std::cout << "cost = " << costResult << "; alpha1 = " << alpha1 << " ; alpha2 = " << alpha2 << " ; alpha3 = " << alpha3 << std::endl;


#if DEBUG_VW_WINDOW
  if (file!=NULL)
    {
      fprintf(file,"Selected:\n");
      fprintf(file,"  vResult    %f\n",vResult);
      fprintf(file,"  wResult    %f\n",deg00(wResult));
      fprintf(file,"  vAccResult %f\n",vAccResult);
      fprintf(file,"  wAccResult %f\n",deg00(wAccResult));
      fprintf(file,"  cost       %f\n",costResult);
      fclose(file);
    }

  eval = fopen(filenameEval,"w");
  if (eval != NULL)
    {
      fprintf(eval,"costHit\n");
      for (i=ivmin;i<=ivmax;i++)
        {
          for (j=iwmin;j<=iwmax;j++)
            {
              fprintf(eval,"%f ",evaluation[i][j].costHit);
            }
          fprintf(eval,"\n");
        }
      fprintf(eval,"costPassing\n");
      for (i=ivmin;i<=ivmax;i++)
        {
          for (j=iwmin;j<=iwmax;j++)
            {
              fprintf(eval,"%f ",evaluation[i][j].costPassing);
            }
          fprintf(eval,"\n");
        }
      fprintf(eval,"costSpeed\n");
      for (i=ivmin;i<=ivmax;i++)
        {
          for (j=iwmin;j<=iwmax;j++)
            {
              fprintf(eval,"%f ",evaluation[i][j].costSpeed);
            }
          fprintf(eval,"\n");
        }
      fprintf(eval,"costDistance\n");
      for (i=ivmin;i<=ivmax;i++)
        {
          for (j=iwmin;j<=iwmax;j++)
            {
              fprintf(eval,"%f ",evaluation[i][j].costDistance);
            }
          fprintf(eval,"\n");
        }
      fprintf(eval,"costHeading\n");
      for (i=ivmin;i<=ivmax;i++)
        {
          for (j=iwmin;j<=iwmax;j++)
            {
              fprintf(eval,"%f ",evaluation[i][j].costHeading);
            }
          fprintf(eval,"\n");
        }
      fprintf(eval,"costValue\n");
      for (i=ivmin;i<=ivmax;i++)
        {
          for (j=iwmin;j<=iwmax;j++)
            {
              fprintf(eval,"%f ",evaluation[i][j].costValue);
            }
          fprintf(eval,"\n");
        }
      fclose(eval);
    }
#endif

#if DEBUG_INTERACTIVE
  cout << "CDL debug interactive result\n\n";
  cout << "  v w       " << vResult << " " << deg00(wResult) << "\n";
  cout << "  vAcc wAcc " << vAccResult << " " << deg00(wAccResult) << "\n\n";
#endif

#if DEBUG_ONLY_ONE_CYCLE
  cout << "CDL: Debug only one cycle\n";
  cout << "-1- next cycle\n";
  cout << "-2- exit\n";
  cin >> i;
  if (i==2) exit(0);
#endif


#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
std::cout<<"ciRes: "<< ciResult <<" vResult|desv: ("<<vResult<<"|"<<desiredTranslationalSpeed<<") wResult|desW: ("<<wResult<<"|"<<desiredRotationalSpeed<<")"<<std::endl;
std::cout<<"Current Limits: vmin|vmax: ("<<vmin<<"|"<<vmax<<") wmin|wmax: ("<<wmin<<"|"<<wmax<<")"<<std::endl;
if( strategy  == CDL_STRATEGY_1){
   if(desiredRotationalSpeed !=0){
	  double r = (desiredTranslationalSpeed/1000.0) / desiredRotationalSpeed;
			  r = r*1000.0;
	  double icc_x = 0;
	  double icc_y = r;

	if(icc_y<0){
	  r = abs00(r);
	  cvEllipse(img, cvPoint((-cdl_c_y_min + -icc_y) * cdl_to_pixel_scale, (cdl_c_x_max + -icc_x) * cdl_to_pixel_scale), cvSize(r * cdl_to_pixel_scale,r * cdl_to_pixel_scale), 0,  180,  270, CV_RGB(0, 0, 255), 1);

	} else {
	  cvEllipse(img, cvPoint((-cdl_c_y_min + -icc_y) * cdl_to_pixel_scale, (cdl_c_x_max + -icc_x) * cdl_to_pixel_scale), cvSize(r * cdl_to_pixel_scale,r * cdl_to_pixel_scale), 0,  270, 360 , CV_RGB(0, 0, 255), 1);
	}
   }

   else {

   	cvLine(img,
   		cvPoint((-cdl_c_y_min + -0) * cdl_to_pixel_scale, (cdl_c_x_max + -0) * cdl_to_pixel_scale),
   		cvPoint((-cdl_c_y_min + -0) * cdl_to_pixel_scale, (cdl_c_x_max + -1000) * cdl_to_pixel_scale),
   		CV_RGB(0, 0, 255),
   		2);
     }


   char buffer1[255];
   char buffer2[255];
   sprintf(buffer1, "distO|dist: (%f|%f)",dist_lookup, dist );
   sprintf(buffer2, "angleO|angle: (%f|%f)",angle_lookup, angle );
   CvFont font;
   cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.75, 0.75);
   if(dist == 0){
	   cvPutText(img, buffer1, cvPoint(-cdl_c_y_min * cdl_to_pixel_scale,
				cdl_c_x_max * cdl_to_pixel_scale), &font, CV_RGB(255, 0, 0));

   } else {
	   cvPutText(img, buffer1, cvPoint(-cdl_c_y_min * cdl_to_pixel_scale,
	   				cdl_c_x_max * cdl_to_pixel_scale), &font, CV_RGB(0, 0, 0));
   }

   if(angle == 0){
	   cvPutText(img, buffer2, cvPoint(-cdl_c_y_min * cdl_to_pixel_scale,
	       			(cdl_c_x_max -200) * cdl_to_pixel_scale), &font, CV_RGB(255, 0, 0));

   } else {
   cvPutText(img, buffer2, cvPoint(-cdl_c_y_min * cdl_to_pixel_scale,
    			(cdl_c_x_max -200) * cdl_to_pixel_scale), &font, CV_RGB(0, 0, 0));
   }

   if(vResult==0 && wResult == 0){
	   char buffer[255];
	      sprintf(buffer, "STOP ROBOT!");
	      CvFont font;
	      cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
	      cvPutText(img, buffer, cvPoint(-cdl_c_y_min * cdl_to_pixel_scale,
	    			(cdl_c_x_max + 200) * cdl_to_pixel_scale), &font, CV_RGB(255, 0, 0));
   }


}


//for person following
if( strategy  == CDL_STRATEGY_7){

	  show_px_color(goalX, goalY, CV_RGB(255, 0, 255)); // alte transformiert = ungerastert/kontinuierliche werte

}


  if(wResult !=0){
	  double r = (vResult/1000.0) / wResult;
			  r = r*1000.0;
	  double icc_x = 0;
	  double icc_y = r;

	if(icc_y<0){
	  r = abs00(r);
	  cvEllipse(img, cvPoint((-cdl_c_y_min + -icc_y) * cdl_to_pixel_scale, (cdl_c_x_max + -icc_x) * cdl_to_pixel_scale), cvSize(r * cdl_to_pixel_scale,r * cdl_to_pixel_scale), 0,  180,  270, CV_RGB(255, 0, 0), 1);

	} else {
	  cvEllipse(img, cvPoint((-cdl_c_y_min + -icc_y) * cdl_to_pixel_scale, (cdl_c_x_max + -icc_x) * cdl_to_pixel_scale), cvSize(r * cdl_to_pixel_scale,r * cdl_to_pixel_scale), 0,  270, 360 , CV_RGB(255, 0, 0), 1);
	}


	//calculate point on line
	double s, alpha, h,ys, r_m;
	r_m = r/1000.0;
	s = vResult * deltat;
	alpha = s/r;
	h = sin(alpha) * r_m;
	ys = cos(alpha) * r_m;


	double res_x,res_y;
	res_x = h*1000;
	if(icc_y<0){
		res_y = icc_y + (ys*1000.0);
	} else {
		res_y = icc_y - (ys*1000.0);
	}


	show_px_color(res_x, res_y, CV_RGB(128, 128, 128));

  } else {

	cvLine(img, 
		cvPoint((-cdl_c_y_min + -0) * cdl_to_pixel_scale, (cdl_c_x_max + -0) * cdl_to_pixel_scale),
		cvPoint((-cdl_c_y_min + -0) * cdl_to_pixel_scale, (cdl_c_x_max + -1000) * cdl_to_pixel_scale),
		CV_RGB(255, 0, 0), 
		1);	
	double s;
	s = vResult * deltat;
	show_px_color(s, 0, CV_RGB(128, 128, 128));
  }

  // FOR Debuging only Plot current speed!
  if(fabs(w) > 0.01){
	  std::cout << "v/w: " << v << "/" << w << std::endl;
	  double r = (v/1000.0) / w;
			  r = r*1000.0;
	  double icc_x = 0;
	  double icc_y = r;

	if(icc_y<0){
	  r = abs00(r);
	  cvEllipse(img, cvPoint((-cdl_c_y_min + -icc_y) * cdl_to_pixel_scale, (cdl_c_x_max + -icc_x) * cdl_to_pixel_scale), cvSize(r * cdl_to_pixel_scale,r * cdl_to_pixel_scale), 0,  180,  270, CV_RGB(0, 0, 255), 1);

	} else {
	  cvEllipse(img, cvPoint((-cdl_c_y_min + -icc_y) * cdl_to_pixel_scale, (cdl_c_x_max + -icc_x) * cdl_to_pixel_scale), cvSize(r * cdl_to_pixel_scale,r * cdl_to_pixel_scale), 0,  270, 360 , CV_RGB(0, 0, 255), 1);
	}


	//calculate point on line
	double s, alpha, h,ys, r_m;
	r_m = r/1000.0;
	s = v * deltat;
	alpha = s/r;
	h = sin(alpha) * r_m;
	ys = cos(alpha) * r_m;


	double res_x,res_y;
	res_x = h*1000;
	if(icc_y<0){
		res_y = icc_y + (ys*1000.0);
	} else {
		res_y = icc_y - (ys*1000.0);
	}


	show_px_color(res_x, res_y, CV_RGB(0, 0, 255));

  } else {

	cvLine(img,
		cvPoint((-cdl_c_y_min + -0) * cdl_to_pixel_scale, (cdl_c_x_max + -0) * cdl_to_pixel_scale),
		cvPoint((-cdl_c_y_min + -0) * cdl_to_pixel_scale, (cdl_c_x_max + -1000) * cdl_to_pixel_scale),
		CV_RGB(0, 0, 255),
		1);
	double s;
	s = v * deltat;
	show_px_color(s, 0, CV_RGB(0, 0, 255));
  }

 
	   char buffer[255];
	      
	      sprintf(buffer, "V: %f W: %f A: %f:", v, w, posA*180.0/M_PI);
	      
	      CvFont font;
	      cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
	      cvPutText(img, buffer, cvPoint(-cdl_c_y_min * cdl_to_pixel_scale,
	    			(cdl_c_x_max + 200) * cdl_to_pixel_scale), &font, CV_RGB(0, 0, 255));
  
#endif


#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
  //TODO
  	  	  if( strategy  == CDL_STRATEGY_14){
  	  		double lx,ly;
  	  		CommBasicObjects::CommBasePose basePosition = scan.get_base_state().get_base_position();

  	  		transformWorldPointToRobot_LookUP(basePosition.get_x(1),basePosition.get_y(1),basePosition.get_base_azimuth(),goalX/1000.0,goalY/1000.0,lx,ly);
			show_px_color(lx*1000, ly*1000, CV_RGB(111, 0, 255));

			transformWorldPointToRobot_LookUP(basePosition.get_x(1),basePosition.get_y(1),basePosition.get_base_azimuth(),finalGoalX/1000.0,finalGoalY/1000.0,lx,ly);
			show_px_color(lx*1000, ly*1000, CV_RGB(159, 0, 255));

  	  	  }
#endif

#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
cvShowImage("cdldebug", img);
cvWaitKey(33);
//dbg_pause("cdl wait");

cvReleaseImage(&img);
#endif


#if DEBUG_CDL_LOOKUP_FILE_OUT
	if(COMP->cdlLookupdebug.is_open()){

		COMP->cdlLookupdebug<<"result: V: "<<vResult<<" W: "<<wResult<<" Cost: "<<costResult<<std::endl;
		COMP->cdlLookupdebug <<" =END -- CALL CALCULATE====================================="<<std::endl;
	}
#endif


  return 0;
};

//int CdlLookupClass::setLaserscan(SmartLaserScanClass &laserscan)
int CdlLookupClass::setLaserscan( CommBasicObjects::CommMobileLaserScan &laserscan)
{
  //laserscan.getCartesianRobot(scan);

  //<asteck: date="17.07.2008" other Laserscan-Object>
  scan = laserscan;
  //<asteck>

#if DEBUG_VW_WINDOW
  currentScan = laserscan;
#endif

  return 0;
};

int CdlLookupClass::setIrScan( CommBasicObjects::CommMobileIRScan &irscan){
	this->irScan = irscan;

	return 0;
}

int CdlLookupClass::setSecondLaserscan( CommBasicObjects::CommMobileLaserScan &laserscan)
{
  scan2 = laserscan;
  return 0;
};


int CdlLookupClass::setParameterWeighting(double a1,double a2,double a3)
{
  alpha1 = a1;
  alpha2 = a2;
  alpha3 = a3;

  return 0;
};

int CdlLookupClass::setEvalPassing(double a1,double a2,double a3,double a4,double a5,double a6)
{
  rangeFarPassing    = a1;
  rangeMediumPassing = a2;
  speedFarPassing    = a3;
  speedMediumPassing = a4;
  speedNearPassing   = a5;
  bonusDistPassing   = a6;

  return 0;
};

int CdlLookupClass::setEvalStopping(double a1,double a2,double a3,double a4,double a5,double a6)
{
  rangeFarStopping    = a1;
  rangeMediumStopping = a2;
  speedFarStopping    = a3;
  speedMediumStopping = a4;
  speedNearStopping   = a5;
//  bonusDistStopping   = 16;
  bonusDistStopping   = a6;
  return 0;
};

int CdlLookupClass::setParameterRobot(double dt,double transacc,double rotacc)
{
  deltat = dt;
  vacc   = transacc;
  wacc   = rotacc;

  return 0;
};

int CdlLookupClass::setHeading(double heading)
{
  goalHeadingRelative = heading;

  return 0;
};

double CdlLookupClass::getHeading()
{
  return goalHeadingRelative;
};

void CdlLookupClass::setSafetyClearance(double sc)
{
  safetyClearance = sc;
}


int CdlLookupClass::setMaxDistance(double distance)
{
  maxDistance = distance;
  if (maxDistance > cdl_max_distance) maxDistance=cdl_max_distance;

  return 0;
};

int CdlLookupClass::setDesiredTranslationalSpeed(double speed)
{
  desiredTranslationalSpeed = speed;

  return 0;
};

int CdlLookupClass::setDesiredRotationalSpeed(double speed){
	desiredRotationalSpeed = speed;

	return 0;
}

double CdlLookupClass::getDesiredTranslationalSpeed()
{
  return desiredTranslationalSpeed;
};

int CdlLookupClass::setPathNavGoal(double startX,double startY, double goalX, double goalY){

	usePathNav = true;
	pathNavGoalX = goalX;
	pathNavGoalY = goalY;
	pathNavStartX = startX;
	pathNavStartY = startY;

	//TODO remove this?!
	setGoalPosition(pathNavGoalX,pathNavGoalY);

	return 0;
}

void CdlLookupClass::setFinalGoalPosition(double x,double y){
	  finalGoalX = x;
	  finalGoalY = y;

	  return;
}

int CdlLookupClass::setGoalPosition(double x,double y)
{
  goalX = x;
  goalY = y;

  return 0;
};

int CdlLookupClass::setRotDevSpeed(double rotDev1, double rotSpeed1, double rotDev2, double rotSpeed2,
		                           double rotDev3, double rotSpeed3, double rotDev4, double rotSpeed4) {


	rotwCVector.clear();

	rotwCVector.push_back( std::make_pair(rotDev1,rotSpeed1));
	rotwCVector.push_back( std::make_pair(rotDev2,rotSpeed2));
	rotwCVector.push_back( std::make_pair(rotDev3,rotSpeed3));
	rotwCVector.push_back( std::make_pair(rotDev4,rotSpeed4));

	return 0;
}

int CdlLookupClass::loadCurvatureIndexAscii(char *filename)
{
  FILE   *file;
  double help;
  int    num,format;
  int    vi,wi;

  file=fopen(filename,"r");
  if (file==NULL)
    {
      fprintf(stderr,"Unable to open data file %s for indices.\n",filename);
      return 1;
    }
  else
    {
      // read first line
      num=fscanf(file,"Format : SFB527 (%d)\n",&format);
      if (num!=1)
        {
          fprintf(stderr,"Error in line 1: unknown file format\n");
          fclose(file);
          return 1;
        }

      // read second line
      fscanf(file,"Content : CDL CURVATURE INDICES\n");

      // read third line
      fscanf(file,"Comment : ---\n");

      // read fourth line
      fscanf(file,"VSpaceNumberCellsTrans : %lf\n",&help);
      if ((num!=1) || (help!=cdl_tra_cells))
        {
          fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read fifth line
      fscanf(file,"VSpaceNumberCellsRot : %lf\n",&help);
      if ((num!=1) || (help!=cdl_rot_cells))
        {
          fprintf(stderr,"Error in line 5: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read data area
      for (vi=0;vi<cdl_tra_cells;vi++)
        {
          for (wi=0;wi<cdl_rot_cells;wi++)
            {
              num=fscanf(file,"%d ",&(indexVW[vi][wi]));
              if (num!=1)
                {
                  fprintf(stderr,"Error in data area: Wrong data format\n");
                  fclose(file);
                  return 1;
                }
            }
          fscanf(file,"\n");
        }
      fclose(file);
      return 0;
    }
};

int CdlLookupClass::loadDistAngleLookupAscii(char *filename)
{
  FILE   *file;
  int    xi,yi,ci;
  double help;
  int    num,format;

  file=fopen(filename,"r");
  if (file==NULL)
    {
      fprintf(stderr,"Unable to open data file %s for lookup-table.\n",filename);
      return 1;
    }
  else
    {
      // read first line
      num=fscanf(file,"Format : SFB527 (%d)\n",&format);
      if (num!=1)
        {
          fprintf(stderr,"Error in line 1: unknown file format\n");
          fclose(file);
          return 1;
        }

      // read second line
      fscanf(file,"Content : CDL LOOKUP TABLE\n");

      // read third line
      fscanf(file,"Comment : ---\n");

      // read fourth line
      num=fscanf(file,"VSpaceMaxTransVel [mm/s] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_v_tra_max))
        {
          fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read fifth line
      num=fscanf(file,"VSpaceMinTransVel [mm/s] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_v_tra_min))
        {
          fprintf(stderr,"Error in line 5: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read sixth line
      num=fscanf(file,"VSpaceMaxRotVel [deg/s] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_v_rot_max))
        {
          fprintf(stderr,"Error in line 6: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read seventh line
      num=fscanf(file,"VSpaceMinRotVel [deg/s] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_v_rot_min))
        {
          fprintf(stderr,"Error in line 7: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read eigth line
      num=fscanf(file,"VSpaceResTrans  [mm/s] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_v_tra_step))
        {
          fprintf(stderr,"Error in line 8: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read ninth line
      num=fscanf(file,"VSpaceResRot [deg/s] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_v_rot_step))
        {
          fprintf(stderr,"Error in line 9: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read tenth line
      num=fscanf(file,"VSpaceNumberCellsTrans : %lf\n",&help);
      if ((num!=1) || (help!=cdl_tra_cells))
        {
          fprintf(stderr,"Error in line 10: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read eleventh line
      num=fscanf(file,"VSpaceNumberCellsRot : %lf\n",&help);
      if ((num!=1) || (help!=cdl_rot_cells))
        {
          fprintf(stderr,"Error in line 11: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read twelfth line
      num=fscanf(file,"CartMinX [mm] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_c_x_min))
        {
          fprintf(stderr,"Error in line 12: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read thirteenth line
      num=fscanf(file,"CartMaxX [mm] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_c_x_max))
        {
          fprintf(stderr,"Error in line 13: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read fourteenth line
      num=fscanf(file,"CartMinY [mm] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_c_y_min))
        {
          fprintf(stderr,"Error in line 14: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read fifteenth line
      num=fscanf(file,"CartMaxY [mm] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_c_y_max))
        {
          fprintf(stderr,"Error in line 15: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read sixteenth line
      num=fscanf(file,"CartCellSize [mm] : %lf\n",&help);
      if ((num!=1) || (help!=cdl_c_res))
        {
          fprintf(stderr,"Error in line 16: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read seventeenth line
      num=fscanf(file,"CartNumberCellsX : %lf\n",&help);
      if ((num!=1) || (help!=cdl_c_x_cells))
        {
          fprintf(stderr,"Error in line 17: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read eighteenth line
      num=fscanf(file,"CartNumberCellsY : %lf\n",&help);
      if ((num!=1) || (help!=cdl_c_y_cells))
        {
          fprintf(stderr,"Error in line 18: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read nineteenth line
      num=fscanf(file,"CurvNumberValues : %lf\n",&help);
      if ((num!=1) || (help!=cdl_curvature_indices))
        {
          fprintf(stderr,"Error in line 19: Wrong format or wrong value\n");
          fclose(file);
          return 1;
        }

      // read 20. line
      fscanf(file,"Comment : ---\n");

      for (xi=0;xi<cdl_c_x_cells;xi++)
        {
          for (yi=0;yi<cdl_c_y_cells;yi++)
            {
              for (ci=0;ci<cdl_curvature_indices;ci++)
                {
                  num=fscanf(file,"%d ",&(distLookup[xi][yi][ci]));
                  if (num!=1)
                    {
                      fprintf(stderr,"Error in data area: Wrong data format\n");
                      fclose(file);
                      return 1;
                    }
                  num=fscanf(file,"%d ",&(alphaLookup[xi][yi][ci]));
                  if (num!=1)
                    {
                      fprintf(stderr,"Error in data area: Wrong data format\n");
                      fclose(file);
                      return 1;
                    }
                }
              fscanf(file,"\n");
            }
        }
      fclose(file);
      return 0;
    }
};

int CdlLookupClass::loadDistAngleLookupBin(char *filename)
{
  FILE *file;
  int counter,size;
  double a;

  file = fopen(filename,"rb");
  if (file!=NULL)
    {
      counter=0;
      size=0;

      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_v_tra_max)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_v_tra_min)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_v_rot_max)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_v_rot_min)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_v_tra_step)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_v_rot_step)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_tra_cells)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_rot_cells)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_c_x_min)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_c_x_max)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_c_y_min)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_c_y_max)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_c_res)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_c_x_cells)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_c_y_cells)
        {
          fclose(file);
          return 1;
        };
      size+=fread(&a,sizeof(a),1,file);
      counter++;
      if (a!=(double)cdl_curvature_indices)
        {
          fclose(file);
          return 1;
        };

      if (size != counter)
        {
          fclose(file);
          return 1;
        };


      for (unsigned int i = 0; i < cdl_c_x_cells; i++)
          for (unsigned int j = 0; j < cdl_c_y_cells; j++)
        	size = fread(distLookup[i][j], sizeof(int), cdl_curvature_indices, file);

//      size=fread(&(distLookup[0][0][0]),
//                 sizeof(int),
//                 (int)(cdl_c_x_cells*cdl_c_y_cells*cdl_curvature_indices),
//                 file);
//      if (size != (cdl_c_x_cells*cdl_c_y_cells*cdl_curvature_indices))
//        {
//          fclose(file);
//          return 1;
//        };



      for (unsigned int i = 0; i < cdl_c_x_cells; i++)
          for (unsigned int j = 0; j < cdl_c_y_cells; j++)
        	size = fread(alphaLookup[i][j], sizeof(int), cdl_curvature_indices, file);

//      size=fread(&(alphaLookup[0][0][0]),
//                 sizeof(int),
//                 (int)(cdl_c_x_cells*cdl_c_y_cells*cdl_curvature_indices),
//                 file);
//      if (size != (cdl_c_x_cells*cdl_c_y_cells*cdl_curvature_indices))
//        {
//          fclose(file);
//          return 1;
//        };

      fclose(file);
      return 0;
    }
  return 1;
}


int CdlLookupClass::save_file_uncompressed(const char *infilename, const char *filename){
	gzFile infile = gzopen(infilename, "rb");
	FILE *outfile = fopen(filename, "wb");
	if (!infile || !outfile) return -1;

	char buffer[128];
	int num_read = 0;
	while ((num_read = gzread(infile, buffer, sizeof(buffer))) > 0) {
		fwrite(buffer, 1, num_read, outfile);
	}

	gzclose(infile);
	fclose(outfile);

	return 0;
}



int CdlLookupClass::loadAccLookupBin(char *filename)
{
  FILE *file;
  int  size;
  int  a;

  file = fopen(filename,"rb");
  if (file!=NULL)
    {
      size=fread(&a,sizeof(a),1,file);
      if ((size!=1) || (a!=(int)cdl_max_curvature))
        {
          fclose(file);
          return 1;
        };
      size=fread(&(transAccLookup[0]),
                 sizeof(double),
                 cdl_max_curvature,
                 file);
      if (size != cdl_max_curvature)
        {
          fclose(file);
          return 1;
        };
      size=fread(&(rotAccLookup[0]),
                 sizeof(double),
                 cdl_max_curvature,
                 file);
      if (size != cdl_max_curvature)
        {
          fclose(file);
          return 1;
        };
      fclose(file);
      return 0;
    }
  return 1;
}


bool CdlLookupClass::pathNavCheckIFRobotIsInCollisionWithPathBorder(){


	std::vector< std::pair<double,double> > points_list;

	calculatePathBorderPoints(points_list);

	std::cout<<__FUNCTION__<<" PointsList_size: "<<points_list.size()<<std::endl;

	std::vector< std::pair<double,double> >::const_iterator iter;
	for(iter = points_list.begin(); iter != points_list.end(); iter++){
		//TODO limit the check to the size of the window!
		if(cdl_check_point_inside_polygon(iter->first,iter->second) == true){
			return true;
		}
	}


	return false;


}


int CdlLookupClass::pathNavRecoverRobotToPathCenter(double& vX, double& vY , double& vW, bool& done){


//	//1. Detect if robot is standing in collision with path-border
//	if(pathNavCehckIFRobotIsInCollisionWithPathBorder() == true){
//		std::cout<<"Path border intersects with robot contour"<<std::endl;

	vX = 0;
	vY = 0;
	vW = 0;
	done = false;

	//2. Calculate closest point on path center

	double closestPoint_x,closestPoint_y;
	calculateClosestPointFromPointToLineSegment(0,0,pathNavStartX,pathNavStartY,pathNavGoalX,pathNavGoalY,closestPoint_x,closestPoint_y);
	double distToGoalPoint = std::sqrt( square(closestPoint_x)+square(closestPoint_y) );


#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	  show_px_color(closestPoint_x, closestPoint_y, CV_RGB(255, 0, 255)); // alte transformiert = ungerastert/kontinuierliche werte
#endif

	std::cout<<"closest point on center robot path : "<<closestPoint_x<<" : "<<closestPoint_y<<" dist: "<<distToGoalPoint<<std::endl;
	if(distToGoalPoint> pathNavRecover_max_dist )
	{
	 std::cout<<"Robot too far away from path --> do not recover!"<<std::endl;
	 return -1;
	}

	if(distToGoalPoint<50){
		std::cout<<"Robot back on track!"<<std::endl;
		done = true;
	}

	//3. Calculate speed vector to reach path center

    std::vector<std::pair<double, double> > _vxControl;
    std::vector<std::pair<double, double> > _vyControl;

    _vxControl.push_back( std::make_pair( 10.0, 0 ) );
    _vxControl.push_back( std::make_pair( 10.0, 20 ) );
    _vxControl.push_back( std::make_pair( 100.0, 100 ) );
    _vxControl.push_back( std::make_pair( 400.0, 300 ) );

    _vyControl.push_back( std::make_pair( 10.0, 0 ) );
    _vyControl.push_back( std::make_pair( 10.0, 20) );
    _vyControl.push_back( std::make_pair( 100.0, 100 ) );
    _vyControl.push_back( std::make_pair( 400.0, 300 ) );


    vX = linearinterpolation(_vxControl, abs00(closestPoint_x));
    if( closestPoint_x < 0 )
	{
			vX = -vX;
	}

	vY = linearinterpolation( _vyControl, abs00(closestPoint_y) );
	if( closestPoint_y < 0 )
	{
			vY = -vY;
	}

	std::cout<<"Calculated speed: "<<vX<<" : "<<vY<<std::endl;

//	} else
//	{
//		std::cout<<"Robot is not in collision with path-border --> unwise to move robot without collision checking now!"<<std::endl;
//		return -1;
//	}
	return 0;

}

