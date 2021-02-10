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

#ifndef _SMARTCDLLOOKUP_HH
#define _SMARTCDLLOOKUP_HH

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>

#include "smartCdlDebug.h"

//#include "smartLaserClient.hh"
#include "CommBasicObjects/CommMobileLaserScan.hh"
#include "CommBasicObjects/CommMobileIRScan.hh"

#include "smartCdlTypes.h"
//#include "smartCdlTags.h"
#include "smartCdlDefine.h"


#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	#ifdef WITH_OPENCV_4_2_VERSION
		#include <opencv4/opencv2/highgui.hpp>
		#include <opencv4/opencv2/core.hpp>
		#include <opencv4/opencv2/imgproc.hpp>
	#else
		#include "cv.h"
		#include "highgui.h"
	#endif
#endif

class CdlLookupClass
  // = TITLE
  //     This is the online part of the new steer angle field
  //     algorithm which needs access to the precalculated
  //     values of the curvature index and the distance values.
{
protected:

#if DEBUG_VW_WINDOW
//  SmartLaserScanClass currentScan;
#endif

  int*** distLookup;
  // this array contains the remaining distance information

  int*** alphaLookup;
  // this array contains the remaining angle information in rad*1000.0

  double* rotAccLookup;
  // this array contains the maximum allowed rotational acceleration

  double* transAccLookup;
  // this array contains the maximum allowed translational acceleration

  int** indexVW;
  // contains curvature index corresponding to v,w

  struct cdl_polygon_struct polygon; 
  // will hold the cdl contur

#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
	#ifdef WITH_OPENCV_4_2_VERSION
  		void drawCdlContour(cv::Mat img, int cdl_x_max, int cdl_y_min, double cdl_to_pixel_scale, cv::Scalar color);
	#else
  		void drawCdlContour(IplImage *img, int cdl_x_max, int cdl_y_min, double cdl_to_pixel_scale, CvScalar color);
	#endif
#endif

  bool cdl_check_point_inside_polygon(double x_p,double y_p);

  struct evaluationStruct {
    double costHit;
    double costPassing;
    double costSpeed;
    double costDistance;
    double costHeading;
    double costValue;
  } ** evaluation;
  // contains calculated costs of this curvature

  int* indexFlag;
  // indicates whether this index has already been inserted into indexList

  int* indexWanted;
  // for test purposes only (CDL_EVAL_APPROACH_EXACT) !!!!
  // 0 this curvature is not wanted
  // 1 this curvature is ok
  // this array is used to indicate, whether this curvature should be used
  // or not


  int* indexWantedVWCheck;
  //this is used for VEL CHECK and will mark those curvature
  //representing the desired one

  int* indexList;
  // list of indizes in the current velocity window

  int* distanceList;
  // the remaining distances with respect to the curvature index

  int* alphaList;
  // the remaining rotation angle with respect to the curvature index
  // in [rad * 1000]

  //DEFINE VALUES FROM CDL-CALCULATE - smartCdlCons.hh
  unsigned int cdl_ok;  
  unsigned int cdl_inf;
  unsigned int cdl_no;
  unsigned int cdl_nok;
  float cdl_accuracy;

  float cdl_v_tra_max;
  float cdl_v_tra_min;
  float cdl_v_rot_max;
  float cdl_v_rot_min;

  float cdl_v_tra_step;
  float cdl_v_rot_step;

  float cdl_a_rot_max;
  float cdl_a_tra_max;

  float cdl_tra_cells;
  float cdl_rot_cells;

  float cdl_c_x_min;
  float cdl_c_x_max;
  float cdl_c_y_min;
  float cdl_c_y_max;
  float cdl_c_res;

  float cdl_max_distance;
  float cdl_capture_distance;

  float cdl_c_x_cells;
  float cdl_c_y_cells;

  float cdl_curvature_indices;
  float cdl_angle_step;

  unsigned int cdl_max_lines;
  unsigned int cdl_max_tra_cells;
  unsigned int cdl_max_rot_cells;
  unsigned int cdl_max_x_cells;
  unsigned int cdl_max_y_cells;
  unsigned int cdl_max_curvature;

  //LaserScanCartesianType scan;
  CommBasicObjects::CommMobileLaserScan scan;
  // Parameter: the current laserscan used for obstacle avoidance
  CommBasicObjects::CommMobileLaserScan scan2;
  CommBasicObjects::CommMobileIRScan irScan;

  double rangeFarPassing, rangeMediumPassing;
  double speedFarPassing, speedMediumPassing, speedNearPassing;
  double bonusDistPassing;
  double rangeFarStopping, rangeMediumStopping;
  double speedFarStopping, speedMediumStopping, speedNearStopping;
  double bonusDistStopping;
  // Parameter: distance dependent velocities, used within
  // CDL_EVAL_PASSING, CDL_EVAL_STOPPING

  double goalHeadingRelative;
  // Parameter: the driving direction relative to the robot frame

  double maxDistance;
  // Parameter: the maximum considered remaining free path length
  // which yields to a rating of 1.0 in the objective function
  // for the distance term. Maximum value is CDL_MAX_DISTANCE

  double desiredTranslationalSpeed;
  // Parameter: the translational speed which should be selected

  double alpha1,alpha2,alpha3;
  // Parameter: the weighting of the different components of the
  // objective function

  double deltat;
  // Parameter: time intervall for the next speed selection

  double vacc,wacc;
  // Parameter: the allowed accelerations

  double goalX, goalY;
  // Parameter: the goal point which should be reached

  double finalGoalX, finalGoalY;
  //used for debugging

  double pathNavGoalX, pathNavGoalY, pathNavStartX, pathNavStartY;
  bool usePathNav;
  bool usePathBorders;
  double pathNavGoalPoint_x,pathNavGoalPoint_y;
  std::vector< std::pair<double, double> > pathNavList;
  std::vector < double > pathWithList;

  std::vector<std::pair<double, double> > _distlookControl;
  double pathNavPredictedGoalPose_minDist;
  double pathNavRecover_max_dist;

  double desiredRotationalSpeed;
  // Parameter: the goal point which should be reached

  double desiredCurvature;


  double safetyClearance;
  // Parameter: the global safety clearance

  std::vector<std::pair<double, double> > rotwCVector;

#ifdef WITH_OPENCV_CDL_LOOKUP_DEBUG
  double cdl_to_pixel_scale;
  int dotsize;
  int fill;

  int pixelX;
  int pixelY;
  int gridsize;
	#ifdef WITH_OPENCV_4_2_VERSION
  	  	cv::Mat img;
	#else
  		IplImage* img;
	#endif
#endif

  int cdl_line_intersection(double a1,double b1,double c1,
                            double a2,double b2,double c2,
                            double &x,double &y);

  int cdl_ab_axbyc(double pax,double pay,double pbx,double pby,
                   double &a,double &b,double &c);

  int intersectionsCircleLine(double,double,double,
                              double,double,double,double,
                              double&,double&,double&,double&,int&);
  // calculate number of intersections and points of intersection
  // of a circle with a line. This function is needed internally
  // to check whether the selected curvature hits the goal point.

  double min00(double,double);
  double max00(double,double);
  double rad00(double);
  double deg00(double);
  double abs00(double);
  double angle00(double);


public:
  // = Initialization and termination
  bool use_obstacle_history;

  CdlLookupClass();
  // Default constructor

  ~CdlLookupClass();
  // Implicitly destroy the class

  //init the class, memory allocation etc.
  void initLookup(double pathNavRecover_max_dist, double pathNavPredictedGoalPose_minDist,
		            std::pair<double,double> PathNavPredictedGoalPose_controll1,
		            std::pair<double,double> PathNavPredictedGoalPose_controll2,
		            std::pair<double,double> PathNavPredictedGoalPose_controll3);

  // = User interface functions.
  // All methods return status information. If status if 0, everything
  // is OK, else an error occured.

  int setParameterWeighting(double alpha1,double alpha2,double alpha3);
  // Sets the weighting parameters for the different components of
  // the objective function

  int setParameterRobot(double deltat,double vacc,double wacc);
  // Sets the robots parameter which are relevant for the lookup-phase

  int setEvalPassing(double,double,double,double,double,double);
  // rangeFar, rangeMedium, speedFar, speedMedium, speedNear, bonusDist

  int setEvalStopping(double,double,double,double,double,double);
  // rangeFar, rangeMedium, speedFar, speedMedium, speedNear, bonusDist

  int setHeading(double heading);
  // Set the goal heading relative to the robot frame.
  // (Attention: is it different to the SmartSafAmos ?)

  double getHeading();
  // Get the goal heading relative to the robot frame.

  void setSafetyClearance(double sc);
  // Set the global safety clearance.

  int setMaxDistance(double distance);
  // Set the maximum considered free path length for the distance
  // term of the objective function

  int setDesiredRotationalSpeed(double speed);

  int setDesiredTranslationalSpeed(double speed);
  // Set the desired speed which should be chosen by the objective function

  double getDesiredTranslationalSpeed();
  // Get the desired speed which is chosen by the objective function

  //int setLaserscan(SmartLaserScanClass &laserscan);
  int setLaserscan( CommBasicObjects::CommMobileLaserScan &laserscan);
  // Put current laserscan into object to calculate remaining distances

  int setSecondLaserscan( CommBasicObjects::CommMobileLaserScan &laserscan);
  // Put current laserscan into object to calculate remaining distances

  int setIrScan( CommBasicObjects::CommMobileIRScan &irscan);
  // Put current irscan into object to calculate remaining distances

  int setGoalPosition(double x,double y);
  // Set the goal coordinates which are used to calculate the deviation
  // within the cost function

  void setFinalGoalPosition(double x,double y);
  // Set the Final goal coordinates ONLY used for DEBUGGING AND VIS

  int setPathNavGoal(double startX,double startY, double goalX, double goalY);
  // Set the goal for path based navigation

  void setUsePathBorders(bool set){
	  usePathBorders = set;
  }
  void setUsePathNav(bool set){
	usePathNav = set;
  }

  void setPathNavPathList(std::vector< std::pair<double, double> > pathList, std::vector< double > pathWidth);
  // set the complete path to goal

  int setRotDevSpeed(double rotDev1, double rotSpeed1, double rotDev2, double rotSpeed2,
		  	  	  	 double rotDev3, double rotSpeed3, double rotDev4, double rotSpeed4);

  template<class T>
  inline T square(const T x)    { return x*x; }

  void calculateClosestPointFromPointToLineSegment(
                  const double &  px,
                  const double &  py,
                  const double &  x1,
                  const double &  y1,
                  const double &  x2,
                  const double &  y2,
                  double  &outx,
                  double  &outy);

  void mapPointToCDLGrid(double x, double y, int & ix,int& iy, int& px, int& py);

  int calculateSpeedValues(double v,double w,
                           double posX,double posY,double posA,
                           double vmin,double vmax,double wmin,double wmax,
                           CdlStrategyType strategy,
                           CdlEvalFunctionType evalFunc,
                           double &vResult,double &wResult,
                           double &vAccResult,double &wAccResult);
  // calculates the next speed values taking into account the allowed
  // min/max velocities

  int freeBehavior(double,int&);
  // Used to free the robot when stalled.
  // securityDist, turnDirection

  int loadCurvatureIndexAscii(char *filename);
  // load curvature index file

  int loadDistAngleLookupAscii(char *filename);
  // load lookup file for the distance and angle values

  int loadDistAngleLookupBin(char *filename);
  // load lookup file for the distance and angle values

  int save_file_uncompressed(const char *infilename, const char *filename);
  // uncompress files

  int loadAccLookupBin(char *filename);
  // load lookup file for the maximum accelerations with respect
  // to the curvature

  //insertScan as obstacle
  void insertLaserScanAsObstacle(const CommBasicObjects::CommMobileLaserScan& scan, int indexCurvature);

  void insertIRScanAsObstacle(const CommBasicObjects::CommMobileIRScan& scan, int indexCurvature);

  //void bresenhamProjectPath(int x0,int y0,int x1,int y1,int width, double alpha, std::list< std::pair<double,double> >& points_list);
  void bresenham(double x0,double y0,double x1,double y1, std::vector< std::pair<double,double> >& points_list);
  //void insertPathBordersAsObstacle(int xs, int ys, int xg, int yg, int width, int indexCurvature);
  void insertPathBordersAsObstacleNew(int xs, int ys, int xg, int yg, int indexCurvature);
  void calculatePathBorderPoints(std::vector< std::pair<double,double> >& points_list);

  void projectPointAngleDistance(double x,double y, double distance, double angle, double& outX, double& outY);
  void projectPoseAngleDistance(double x,double y,double yaw, double distance, double angle, double& outX, double& outY);

  bool cdl_load_contour_ascii(char *filename);

  inline float getCDL_MAX_DISTANCE(){
	  return this->cdl_max_distance;
  }

  inline float getCDL_V_TRA_MIN(){
	  return this->cdl_v_tra_min;
  }

  inline float getCDL_V_TRA_MAX(){
	  return this->cdl_v_tra_max;
  }

  inline float getCDL_V_ROT_MIN(){
	  return this->cdl_v_rot_min;
  }
  inline float getCDL_V_ROT_MAX(){
	  return this->cdl_v_rot_max;
  }


  double linearinterpolation(const std::vector<std::pair<double, double> >& vec, const double x );

  bool pathNavCheckIFRobotIsInCollisionWithPathBorder();

  int pathNavRecoverRobotToPathCenter(double& vX, double& vY , double& vW, bool& done);

};

#endif

