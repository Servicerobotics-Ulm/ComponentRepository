#ifndef _MVBB_
#define _MVBB_

#include <vector>
#include <string>
#include <string.h>
#include <sstream>
#include <set>
#include <list>

#include "Point.h"
#include "ColorDebug.h"
#include "gdiam.h"
#include "point.h"
#include "_generic.h"

using namespace std;

/// A structure to group the MVBB core parameters.
struct MvbbParameters
{	
  /// Gain value for the decomposition
  double Gain;
  /// Minimum number of points value for the acceptance of boxes
  int    MinPoints;
  /// Sample value for the decomposition
  int    Samples;
  /// Grid value for the decomposition
  int    Grid;
  /// Minimum cut cell dimension for the 2D grid splitting
  int    MinCutCells;
 	
  /// Empty constructor with default setup
  MvbbParameters() : Gain(0.9), MinPoints(4), Samples(200), Grid(3), MinCutCells(50) {};
  
  /**
   * Constructor with specified setup
   * @param gain Gain value for the decomposition
   * @param min Minimum number of points value for the acceptance of boxes
   * @param samples Sample value for the decomposition
   * @param grid Grid value for the decomposition
   * @param cutcellmin Minimum cut cell dimension for the 2D grid splitting
   */
  MvbbParameters(double gain, int min, int samples, int grid, int cutcellmin) : 
  Gain(gain), MinPoints(min), Samples(samples), Grid(grid), MinCutCells(cutcellmin) {};
};

/// A Bounding Box container structure
struct bbCont 
{
  /// The static ID of the current bounding box
  static int _ID;
  
  /// Empty constructor
  bbCont() 
  {
    _ID++;
    id = _ID;
    parentIdx = -1;
    hLevel=0;
  }

  /// The points included as a vector of doubles
  vector<double> points;
  /// A gdiam struct to work with the MVBB code
  gdiam_bbox bb;
  /// This box's ID
  int id;
  /// The ID of this box's parent box
  int parentIdx;
  /// The hierarchy level of this box (in the decomposition tree)
  int hLevel;
  /// The gain value under which this box was split
  float gain;  
  
  /**
   * Multisets are by default ordered in strict less than order, so this operator is needed. 
   * @param ref Another bbContainer
   * @return If this one is smaller (in volume) than the ref box.
   */
  bool operator< (const bbCont& ref) const {
    return bb.volume() < ref.bb.volume();
  }
};

/// A structure for a dismissed Bounding Box pair
struct bbDismissed {
  /// The left dismissed container
  bbCont l;
  /// The right dismissed container
  bbCont r;
  /// The ID of the BB container that was dismissed
  int parentIdx;
};

/// A simplistic structure for a Bounding Box
struct BoxInfo {
  /// The ID of the box
  int id;
  /// The ID of the box's parent
  int parentId;
  /// The hierarchy level of this box (in the decomposition tree)
  int hLevel;
  /// The gain value under which this box was split
  float gain;
  /// The points included as a vector of doubles
  vector<double> coords;
};

/**
 * This class controls the MVBB decomposition algorithm.
 * It is a wrapper around the MVBB approximation algorithm
 * by Har-Peled and Barequet, which is available in 
 * BoxGrasping/3rd-party/MVBB. 
 */
class MVBB
{
 public:

  /**
   * Constructor of MVBB decomposition class from a vector of doubles.
   * Triples of double values are interpreted as points.
   * @param coords The coordinate values.
   * @param bbivfile The file where the special BB model is saved.
   * @param redux  If points shall be reduced in the decomposition stage.
   */
  MVBB(vector<double>& coords, const char* bbivfile=NULL, bool redux=false);

  /**
   * Constructor of MVBB decomposition class from a vector of points.
   * @param points The points.
   * @param bbivfile The file where the special BB model is saved.
   * @param redux  If points shall be reduced in the decomposition stage.
   */
  MVBB(vector<Point>& points, const char* bbivfile=NULL, bool redux=false);

  /// Empty desctructor
  ~MVBB();
  
  /**
   * The main function that starts iterative approximation and 
   * decomposition of minimum volume bounding boxes.
   * @return Resulting vector of simplistic box informations.
   */
  vector<BoxInfo> iterate();

  /**
   * Set the gain value for the MVBB decomposition.
   * @param g Gain value.
   */
  inline void setGain(double g)    {m_params.Gain = g;};

  /**
   * Set the sample value for the MVBB approximation.
   * @param s Number of samples.
   */
  inline void setSamples(int s)    {m_params.Samples = s;};

  /**
   * Set the minimum number of points value for the MVBB decomposition.
   * @param m Min point value.
   */
  inline void setMinPoints(int m)  {m_params.MinPoints = m;};

  /**
   * Set the grid value for the MVBB approximation.
   * @param g Grid value.
   */
  inline void setGrid(int g)       {m_params.Grid = g;};

  /**
   * Set the minimum number of cutting cells value for the MVBB decomposition.
   * @param g Cut cells value.
   */
  inline void setCutCellMin(int g) {m_params.MinCutCells=g;};

  /**
   * Query if the MVBB decomposition is finished.
   * @return If algorithm is finished or not.
   */
  inline bool finished() const     {return _finished;};
  
 private:
    
  /// Put the current decomposition into an inventor file
  void dumpCurrentState();
  
  /**
   * Perform a cut on the current state of boxes.
   * @return A vector of simplistic box info after performing the cut.
   */
  vector<BoxInfo> cut();
  
  /**
   * Main function to trigger a decomposition of a point cloud into two.
   * @param points The input point cloud.
   * @param X The x-axis of the point cloud's frame (usually the bounding box frame).
   * @param Y The y-axis of the point cloud's frame (usually the bounding box frame).
   * @param Z The z-axis of the point cloud's frame (usually the bounding box frame).
   * @param pointsl Reference to the first output point cloud.
   * @param pointsr Reference to the second output point cloud.
   */
  void divide(vector<double> points, double X[3], double Y[3], double Z[3],
	      vector<double>& pointsl, vector<double>& pointsr);
  
  /**
   * Convert a point representation of a vector of doubles into a vector of 3d points.
   * @param points The input point cloud as doubles.
   * @param points3d Reference to the output point cloud as Points.
   */
  void argVector2point3dVector(vector<double> points, vector<Point3d> &points3d);
  
  /**
   * Convert a vector of 3d points into a point representation of a vector of doubles.
   * @param points3d The input point cloud as Points.
   * @param points Reference to the output point cloud as doubles.
   */
  void point3dVector2DoubleVector(vector<Point3d> points3d, vector<double> &points);
   
  /**
   * Transform 3d points into a new frame given new axes (usually the bounding box frame).
   * @param points3d The input point cloud as Points; will be overwritten with transformed points.
   * @param x_norm The normalized x-axis to transform to (usually of the bounding box frame).
   * @param y_norm The normalized y-axis to transform to (usually of the bounding box frame).
   * @param z_norm The normalized z-axis to transform to (usually of the bounding box frame).
   */
  void transformCordinates2BBCordinates(vector<Point3d> &points3d,
					const Point3d &x_norm,
					const Point3d &y_norm,
					const Point3d &z_norm);
  
  /**
   * This function computes the X/Y/Z bound of an input point cloud.
   * @param points3d The input point cloud as Points.
   * @param xmin The minimum x value detected
   * @param xmax The maximum x value detected
   * @param ymin The minimum y value detected
   * @param ymax The maximum y value detected
   * @param zmin The minimum z value detected
   * @param zmax The maximum z value detected
   */
  void calMaxMin(vector<Point3d> &points3d,
		 real_mvbb &xmin, real_mvbb &xmax,
		 real_mvbb &ymin, real_mvbb &ymax,
		 real_mvbb &zmin, real_mvbb &zmax);

  /**
   * This is the top function to trigger the best cut computation.
   * It determines the best cut based on 2D projections to minimize the volume.
   * @param points3d The input point cloud as Points.
   * @param xmin The minimum x value
   * @param xmax The maximum x value
   * @param ymin The minimum y value
   * @param ymax The maximum y value
   * @param zmin The minimum z value
   * @param zmax The maximum z value
   * @param opt_face Will include the face ID of the optimal cut
   * @param opt_p Will include the start 2D coordinate of the optimal cut
   * @param opt_q Will include the end 2D coordinate of the optimal cut
   */  
  void findBestCut(vector<Point3d> &points3d,
		   const real_mvbb &xmin, const real_mvbb &xmax,
		   const real_mvbb &ymin, const real_mvbb &ymax,
		   const real_mvbb &zmin, const real_mvbb &zmax,
		   int &opt_face, Point2d &opt_p, Point2d &opt_q);

  /**
   * To find the best hull is a subfunction called by findBestCut.
   * It determines based on a 2D projection the two points that minimize the 2D volume.
   * It also returns this minimum volume value for comparison with splits from other projections.
   * @param points The input 2D points.
   * @param opt_p Will include the start 2D coordinate of the optimal cut
   * @param opt_q Will include the end 2D coordinate of the optimal cut
   * @param minV Will include the volume that is generated by a split through opt_p and opt_q
   * @return The number of 'cracks' in the hull, i.e. those points that are possible split points
   */
  int findBestHull(vector<Point2d> points, Point2d &opt_p, Point2d &opt_q, float &minV);

  /**
   * If the minimizing split parameters are found, this function finally performs the split.
   * @param in The input 3D point cloud (of box-normalized points)
   * @param cut_idx An identifier to tell which of the box facets 1-3 should be used.
   * @param cut_p The determined first cutting point of the cutting line.
   * @param cut_q The determined second cutting point of the cutting line.
   * @param original The original 3D point cloud (splitted output should not keep box-normalized ones)
   * @param left The first new point cloud after splitting.
   * @param right The second new point cloud after splitting.
   */
  void makeCut(const vector<Point3d> &in, const int &cut_idx, const Point2d &cut_p, const Point2d &cut_q,
	       const vector<Point3d> &original,
	       vector<Point3d> &left, vector<Point3d> &right);

  /**
   * Returns the current overall volume of the decomposition state.
   * @return Overall volume value of the current state.
   */
  double calVol();

  /// A set of MVBB parameters
  MvbbParameters m_params;
  /// The points that the MVBB approximation works on
  double* _points;
  /// The number of points
  int  _numPoints;
  /// The number of iterations
  int  _iterations;
  /// If decomposition is started
  bool _started;
  /// If decomposition is finished
  bool _finished;
  /// If point reduction is enabled
  bool _redux;

  /// We store the original points in a vector
  vector<Point3d> _original;
  /// A vector of resulting boxes
  vector<gdiam_bbox> bbVec;
  /// A vector including info about which boxes are going to be used later on
  vector<bool> saveIdx;
  /// A vector of string for the final output data
  vector<string> out_bbiv;
  /// A list of dismissed bounding boxes (which could intersting to decompose at a later point)
  list<bbDismissed> bbD;
  /// A set of bounding box container structures
  multiset<bbCont> bbCSet;

  /**
   * This is the file where the special BB model is saved.
   * It can be used (default) to be loaded into the scene
   * in order to visualize the decomposition process.
   */
  char m_bbivfile[128];
};

#endif


