// -*- mode:C++; tab-width:2; c-basic-offset:2; indent-tabs-mode:nil -*-

/**
 * @file BoundingBox.h
 * BoxFaceProjection declares a BoundingBox for the BoxGrasping Framework.
 */

#ifndef _BBOX_
#define _BBOX_

#include <string>
#include <iostream>
#include <vector>
#include <assert.h>
#include "Face.h"
//#include "Interface.h"

using namespace std;

/**
 * SIDE is an enumeration used to define a correlation between faces in the box.
 * Each face correlates to the 5 other faces in the box by being to its LEFT, to its RIGHT,
 * on its TOP or BOTTOM, or describing the opposing BACK. This is what SIDE defines.
 */
typedef enum
{
  LEFT,
  RIGHT,
  TOP,
  BOTTOM,
  BACK
} SIDE;

/// BoundingBox declares a BoundingBox for the BoxGrasping Framework.
/**
 * BoundingBox is a representation for a box. It holds a bunch of information,
 * from the faces, over volume and center point, parent and child boxes in the
 * box decomposition hierarchy, as much more.
 */
class BoundingBox
{
  /// A streaming operator to print information about the BoundingBox
  friend ostream& operator << (ostream& os, const BoundingBox& box);

 public:

  /**
   * Empty constructor. Sets _id to -1 which is used to represent an invalid bounding box
   */
  BoundingBox();

  /**
   * Filled constructor, given the faces, an id, and a parent id to define the box.
   * The constructor is computing additional information, e.g. volume, center point, aso.
   * @param f A vector of faces that build the box (should/must be 6).
   * @param id The new ID for this box.
   * @param parentId The parent ID. Each box has a parent in the box decomposition hierarchy (except the root node).
   */
  BoundingBox(const vector<Face>& f, int id = -1, int parentId = -1);

  /**
   * Destructor
   */
  ~BoundingBox(){};

  /**
   * Create a new box from a vector of its (corner) coordinates and necessary ids.
   * @param id The id of this box
   * @param parent The id of the parent box
   * @param coords The corner coordinates as a vector of doubles
   */
  void   createBoxFromCoords(int id, int parent, const vector<double>& coords);

  /**
   * Initialization that also "cleans" the box by setting most member variables to -1.
   */
  void   initialize();

  // Access functions

  /**
   * Return the size of the root box as a 3D point.
   * @return The size of the root box.
   */
  Point getSize()
  {
    return Point (m_faces[0].getWidth(),
                  m_faces[0].getHeight(),
                  m_faces[0].getDepth());
  };

  /**
   * Return a face of the current box.
   * @param id Index of the face (between 0 and 5).
   * @return The idth faces of the box.
   */
  Face getFace(int id) const {assert (id>-1 && id <6); return m_faces[id];}

  /**
   * Returns the faces of the current box as a vector of Face.
   * @return The (usually 6-elemented) vector of Faces.
   */
  vector<Face> getFaces() const {return m_faces;}

  /**
   * Returns the a pointer to the faces of the current box.
   * @return Pointer to the (usually 6-elemented) vector of Faces.
   */
  vector<Face>* getFacePtr() {return &m_faces;}

  /**
   * Returns the center point of the box.
   * @return 3d center point of the box.
   */
  Point  getCenter() const {return m_center;}

  /**
   * Returns the volume of the box
   * @return Double value representing the box volume
   */
  double getVolume() const {return m_volume;}

  /**
   * Returns the box ID of the left (or first) child.
   * Each box will have either 0 or two childs due to the decomposition.
   * @return ID of the first child box.
   */
  int    getChild1() const {return m_childId1;}

  /**
   * Returns the box ID of the right (or second) child.
   * Each box will have either 0 or two childs due to the decomposition.
   * @return ID of the second child box.
   */
  int    getChild2() const {return m_childId2;}

  /**
   * Returns the box ID of the parent box.
   * Each box will have a parent ID, except the root box, where parent ID is -1.
   * @return ID of the parent box.
   */
  int    getParent() const {return m_parentId;}

  /**
   * The ID of this box.
   * @return The ID of this box.
   */
  int    getId() const {return m_id;}

  /**
   * The hierarchy level of the box. The root box is on level 0, its children on level 1, aso.
   * @return The hierarchy level in which this box is placed.
   */
  int    getHierarchyLevel() const {return m_hierarchyLevel;}

  /**
   * The gain that this box reached. It represents the rate between the volume of this box and
   * the volume of a previous stage during the decomposition. One can compare to the gain parameter
   * g used for the box decomposition. A Box b should only be split if g is larger than the sum of
   * gains of b's two children.
   * @return Gain value for this box.
   */
  float  getGain() const {return m_gain;}

  /**
   * Returns if this box is selected for a grasping task. Boxes might be deselected for several
   * reasons, e.g. not fitting to the task at hand.
   * @return TRUE if box is selected for grasping, FALSE otherwise.
   */
  bool   isSelected() const {return m_bSelected > -1;}

  /**
   * Returns if this box is selected for a grasping task with a given rank. Boxes might be deselected for several
   * reasons, e.g. not fitting to the task at hand. By an index, it is possible to parametrize if this box also
   * corresponds to a given ranking. Thus, one can loop through the ranking, starting at 0.
   * @return TRUE if box is selected for grasping, FALSE otherwise.
   */
  bool   isSelected(int rank) const {return m_bSelected == rank;}

  /**
   * Returns if this box is a leaf box of the decomposition tree. Only leaf boxes
   * are parts of the final decomposition and have no children.
   * @return TRUE if box is leaf box, FALSE otherwise.
   */
  bool   isLeafBox() const {return m_childId1 == -1 && m_childId2 == -1;}

  /**
   * Checks if this box envelopes a given 3d point.
   * @return TRUE if point lies in the box, FALSE otherwise.
   */
  bool   envelopesPoint(Point p);

//  /**
//   * Checks if this box envelopes a given 6d contact point.
//   * @return TRUE if point lies in the box, FALSE otherwise.
//   */
//  bool   envelopesPoint(IF_contact c);

  /**
   * This function is helping to correlate faces to other faces on the box surface.
   * Each face in the box has an id in [0..5]. The correlation of which box is
   * LEFT, RIGHT, TOP, BOTTOM, or BACK (see enum SIDE) of a given face is given by
   * this function. To resolve redundancy, a reference dir can be set (default LEFT).
   * @param face_id The related face_id
   * @param side The side of the related face that should be returned.
   * @param ref_dir a reference direction to resolve redundancy.
   * @return The index of the face that is on :side: of face :face_id:.
   */
  int    getSideOfFace(unsigned int face_id, SIDE side, SIDE ref_dir=LEFT);

  // Setter functions

  /**
   * Set the box ID, giving an id.
   * @param id ID of this box.
   */
  void   setId(int id) {m_id = id;}

  /**
   * Set the first child, giving an id.
   * @param child1 ID of the first child.
   */
  void   setChild1 (int child1) {m_childId1 = child1;}

  /**
   * Set the second child, giving an id.
   * @param child2 ID of the second child.
   */
  void   setChild2(int child2) {m_childId2 = child2;}

  /**
   * Set the hierarchy level of the box. The root box shall be level 0, its children on level 1, aso.
   * @param hLevel The hierarchy level on which this box shall be placed.
   */
  void   setHierarchyLevel(int hLevel) {m_hierarchyLevel = hLevel;}

  /**
   * The gain that this box reached. It represents the rate between the volume of this box and
   * the volume of a previous stage during the decomposition. One can compare to the gain parameter
   * g used for the box decomposition. A Box b should only be split if g is larger than the sum of
   * gains of b's two children.
   * @param gain Gain value for this box.
   */
  void   setGain(float gain) {if (m_gain == -1) m_gain = gain;}

  /**
   * Set this box is selected for a grasping task. Boxes might be deselected for several
   * reasons, e.g. not fitting to the task at hand. In this case, the setting is -1.
   * In each other case, the given number assigns a rank to the box in which
   * grasping might be applied
   * @param b The rank in which this box shall be selected for grasping; -1 for disabled.
   */
  void   setSelected(int b) {m_bSelected = b;}

  /**
   * Return if this box is selected for a grasping task. Boxes might be deselected for several
   * reasons, e.g. not fitting to the task at hand. In this case, -1 will be returned.
   * In each other case, the given number corresponds to a ranking in which grasping might be applied to this box.
   * @return The rank in which this box is selected for grasping; -1 for disabled.
   */
  inline int getSelected() {return m_bSelected;}

  /**
   * Other functions
   */
  bool   isValid() const {return m_id != -1;}

  /**
   * These are the basic face and corner indices. To have an order for faces in the box
   * and how to produce 6 faces from 8 corner points, the list is a 24 element array,
   * where each 4 correspond to corner indices for one face.
   * E.g., {2, 6, 4, 0,} as the first four tell that the corner points 2,6,4,0 build the
   * first face. This structure is highly important, as many procedures belong to it.
   */
  static int faceIndices[24];

  /**
   * The number of indices in faceIndices.
   */
  static int numIndices;

protected:

  /**
   * This holds the fixed relations between a face and its face to the left.
   * E.g., lftFaces[0] is the index of the face that is left to face 0.
   */
  static int lftFaces[6];

  /**
   * This holds the fixed relations between a face and its face to the right.
   * E.g., rgtFaces[0] is the index of the face that is right to face 0.
   */
  static int rgtFaces[6];

  /**
   * This holds the fixed relations between a face and its face to the top.
   * E.g., topFaces[0] is the index of the face that is top to face 0.
   */
  static int topFaces[6];

  /**
   * This holds the fixed relations between a face and its face to the bottom.
   * E.g., botFaces[0] is the index of the face that is bottom to face 0.
   */
  static int botFaces[6];

  /**
   * This holds the fixed relations between a face and its face to the back.
   * E.g., bckFaces[0] is the index of the face that is back to face 0.
   */
  static int bckFaces[6];

 private:
  /// Vector of faces that build the box
  vector<Face> m_faces;
  /// The center point of the box
  Point  m_center;
  /// The ID of the box
  int    m_id;
  /// The hierarchy level of the box (in the decomposition hierarchy)
  int    m_hierarchyLevel;
  /// The ID of the parent of this box
  int    m_parentId;
  /// The ID of the first child of this box
  int    m_childId1;
  /// The ID of the second child of this box
  int    m_childId2;
  /// The volume of this box
  double m_volume;
  /// The gain value of this box
  float  m_gain;
  /// If this box is selected for grasping or not
  int   m_bSelected;
};

#endif
