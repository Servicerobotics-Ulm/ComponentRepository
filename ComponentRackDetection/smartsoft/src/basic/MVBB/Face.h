// -*- mode:C++; tab-width:2; c-basic-offset:2; indent-tabs-mode:nil -*- 

/**
 * @file Face.h
 * Geometrical declaration of a box Face for the BoxGrasping Framework.
 */

#ifndef _FACE_
#define _FACE_

#include "Point.h"
#include "globals.h"
#include <vector>
#include <assert.h>

/**
 * The number of face grasps. It is set manually in the constructor,
 * usually to 4, as a box face easily allows 4 different orientation
 * grasps, differing by 90 deg. 2 might be used for symmetrical hands.
 */
static const unsigned int GRASPS_PER_FACE = 4;

/// The class Face represents the geometrical declaration of a box face.

class Face
{
  /// Makes the face sendable to an output stream (console output).
  friend ostream& operator << (ostream& os, const Face& face);
  
 public:
  
  /**
   * Empty constructor
   */
  Face();

  /**
   * Constructor with four corner points and a "depth", which in context of boxes 
   * is the distance between the face center and the box center. As this is independent of 
   * the face itself, it has to come from another source, e.g. the box.
   * @note This constructor automatically computes area, normal, center, aso.
   */
  Face(const Point& a, const Point& b, const Point& c, const Point& d, double depth=1);

  /**
   * Destructor
   */
  ~Face() {};
  
  /**
   * Get the face area.
   * @return The area of the face (in point units).
   */
  double  getArea()   const {return m_area;};

  /**
   * Get the "depth" of the face. The depth is seen as the distance between 
   * the face center and its opposing face in the box.
   * @return The depth of the face (in point units).
   */
  double  getDepth()  const {return m_depth;};

  /**
   * Get the width of the face.
   * @return The width of the face (in point units).
   */
  double  getWidth()  const {return m_edges[0].length();};

  /**
   * Get the height of the face.
   * @return The height of the face (in point units).
   */
  double  getHeight() const {return m_edges[1].length();};

  /**
   * Get the normal vector of the face.
   * @return The normal vector of the face (as a Vector of point units).
   */
  Vector  getNormal() const {return m_normal;};

  /**
   * Get the center point of the face.
   * @return The center point of the face (as a Point of point units).
   */
  Point   getCenter() const {return m_center;};
  
  /**
   * Get the first corner point of the face.
   * @return The first corner point of the face (as a Point of point units).
   */
  Point c1() const {return m_points[0];};

  /**
   * Get the second corner point of the face.
   * @return The second corner point of the face (as a Point of point units).
   */
  Point c2() const {return m_points[1];};

  /**
   * Get the third corner point of the face.
   * @return The third corner point of the face (as a Point of point units).
   */
  Point c3() const {return m_points[2];};

  /**
   * Get the fourth corner point of the face.
   * @return The fourth corner point of the face (as a Point of point units).
   */
  Point c4() const {return m_points[3];};

  /**
   * Get the face ID of the face. According to its position in the box,
   * each face can have an ID between 0 and 5.
   * @return The face ID of the face.
   */
  int getFaceID() const {return m_faceID;};

  /**
   * Get the box ID of the face. It is the link to the box to which this face belongs.
   * @return The box ID of the face.
   */
  int getBoxID() const {return m_boxID;};

  /**
   * Get all the corner points as a vector of Points.
   * @return The corner points of the face (as a vector of Point).
   */
  vector<Point> getCorners() const {return m_points;};

  /**
   * Get an edge vector.
   * @param id Index of the edge (has to be in [0..3]
   * @return The idth edge vector
   */
  Vector getEdge(int id) const {assert (id>-1 && id <4); return m_edges[id];};

  /**
   * Get all the edge vectors as a vector of Vectors.
   * @return The edge vectors of the face (as a vector of Vector).
   */
  //vector<Vector> getEdges() const {return m_edges;};
  
  /**
   * Check if this face is valid. Invalidity might be caused by occlusion by other boxes, for example.
   * @return TRUE if this face is valid, FALSE otherwise.  
   */
  bool isValid() const;

  /**
   * Set the ID of this face, according to its position in the box (face ID) and
   * the ID of the box that this face belongs to.
   * @param id The box ID this face belongs to.
   * @param fid The face ID that this face identifies in its box.
   */
  void setBoxID(int id, int fid) {m_boxID = id; m_faceID = fid;};

  /**
   * setBlocked tests for collinearity of the face's edge vectors to a given vector.
   * In case of collinearity, these grasping directions are to be blocked. Assume
   * the normal vector of a face of box A pointing towards a face of box B.
   * The face of box B gets invalidated due to occlusion; however, also the neighboring
   * boxes are partially blocked for grasping in the direction of A. This partial 
   * blocking is done by setBlocked. 
   * @param n The vector that should be tested for collinearity with the face edge vectors.
   * @param d not used yet. It was planned to make the partial block also dependent on
   *          the distance between boxes A and B.
   */
  void setBlocked(Vector n, double d);

  /**
   * Return if (and in which distance) this face is blocked at one of its four edges.
   * @param edge The edge index that should be checked (must be between 0 and 3, as a face always has 4 edges).
   * @return A value of the distance. If this value is >0, the face is blocked at the given edge. Otherwise, it is not.
   */
  inline double getBlocked(int edge) {if (edge < 0 || edge > 3) return -1; return m_edgeblocks[edge];};

  /**
   * Set a block directly for a given edge of this face.
   * @param edge The edge index that should be checked (must be between 0 and 3, as a face always has 4 edges).
   * @param v A value of the distance. If this value is >0, the face is blocked at the given edge. Otherwise, it is not.
   */
  inline void setBlocked(int edge, double v) {if (edge < 0 || edge > 3) return; m_edgeblocks[edge] = v;};

  /**
   * Set occlusion to this face.
   * @param o A bool to set this face occluded and thereby not care about it for grasping.
   */
  void setOccluded(bool o) {m_bOccluded = o;};

  /**
   * Check the occlusion of this face.
   * @return TRUE if this face is occluded, FALSE otherwise.
   */
  inline bool getOccluded() {return m_bOccluded;};

  /**
   * Switch the normal in its opposite direction. This is necessary, as the mvbb algorithm seems 
   * not generally to produce box edge points in an order so that a calculated normal from the first 
   * three points of a face is turned outwards. For these cases (when the normal points inwards the box),
   * the normal has to be switched, as the BoxGrasping framework and other applications define normals
   * as pointing outwards.
   */
  void switchNormal();
  
  /**
   * Return the (shortest) distance of a point to the plane that is spanned by the face.
   * @param p The point for which the distance should be computed for.
   * @return The distance between p and the face.
   */
  double distanceToPoint(const Point p);

  /**
   * This function returns true if the normal of the face intersects another face.
   * @param other The other face.
   * @param t The distance between the faces.
   * @return true if the intersection is on the face, else false
   */
  bool intersects(const Face& other, double &t);

  /**
   * This function returns as argument a pointer to the intersection of the object's normal with
   * another face. If no intersection exists, a zero pointer is returned.
   * @param other The other face.
   * @param intersection Will keep the point of intersection or 0 if none exists.
   * @return The distance between the faces.
   */
  double getIntersection(const Face& other, Point &intersection);

  /**
   * This function checks if the edge along which the face might be graspable is too long or not.
   * @param mode Signifies along which edges of the two possible choices the test shall be done (0 to 3).
   * @param max_edge_length Additional input to return false (=not graspable) if the edge is longer than max_edge_length.
   *        Given a maximum opening distance of a gripper, this checks if the box can be grasped by it or not.
   * @return true if graspable, else false
   */
  bool isGraspableWithHand(unsigned int mode, double max_edge_length=0);

  /**
   * Without this operator, the generic find()-algorithm in BoxApproximation::getGraspableFaces() didn't work.
   * @param other another face.
   * @return true, if the 4 vertices are the same, else false
   */
  bool operator== (const Face& other) const;

  /**
   * Set the shape classification identifier for this face.
   * Each face can carry a 2.5D shape classifier (see ShapeClassAut).
   * @param s The shape class identifier that shall be assigned to this face.
   */
  void  setShapeClass(ShapeClassAut s) {shape_2d = s;};

  /**
   * Get the shape classification identifier for this face.
   * Each face can carry a 2.5D shape classifier (see ShapeClassAut).
   * @return The shape class identifier assigned to this face.
   */
  ShapeClassAut getShapeClass() {return shape_2d;};
  
 private:
	
  /// The vector or corner points.
  vector<Point> m_points;
  /// The vector of edge vectors.
  vector<Vector> m_edges;
  /// The vector of edge blocks (probably including their block distances, i.e. distances to faces that block them)
  vector<double> m_edgeblocks;
  /// TRUE if this face is occluded, FALSE otherwise
  bool m_bOccluded;
  /// The center point of the face
  Point m_center;
  /// The normal of the face
  Vector m_normal;
  /// The area of the face
  double  m_area;
  /// The depth of the face, i.e. the distance from the face center to the center of the box that the face belongs to.
  double  m_depth;
  /// The box ID of this face (link to the box that this face belongs to)
  int m_boxID;
  /// The identification of this face in its box (so necessarily 0-5)
  int m_faceID;
  /// The assigned shape class identifier
  ShapeClassAut shape_2d;  
};

#endif //_FACE_
