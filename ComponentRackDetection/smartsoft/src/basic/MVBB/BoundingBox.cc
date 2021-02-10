#include "BoundingBox.h"

// The ordering of points in the faces
int BoundingBox::numIndices = 24;
int BoundingBox::faceIndices [24] = {2, 6, 4, 0,
				     1, 5, 7, 3,
				     6, 7, 5, 4,
				     2, 3, 7, 6,
				     0, 1, 3, 2,
				     4, 5, 1, 0};

// These indices give back the indices of faces left from a face.
int BoundingBox::bckFaces[6] = {1,0,4,5,2,3};
int BoundingBox::lftFaces[6] = {4,2,0,4,1,4};
int BoundingBox::topFaces[6] = {5,5,5,0,5,1};
int BoundingBox::rgtFaces[6] = {2,4,1,2,0,2};
int BoundingBox::botFaces[6] = {3,3,3,1,3,0};

ostream& operator<< (ostream& os, const BoundingBox& box)
{
  int i;
  for (i = 0; i < 6; ++i)
  {
    os << "Face #" << i+1 << ":" << endl;
    os << box.m_faces[i] << endl;
  }
  return os;
}

BoundingBox::BoundingBox()
{
  initialize();
}

BoundingBox::BoundingBox(const vector<Face>& f, int id, int parentId)
{
  initialize();

  assert(f.size() >= 6);
  for (unsigned int i = 0; i < 6; ++i)
  {
    m_faces.push_back(f[i]);
    m_volume *= f[i].getArea();
  }
  m_volume = sqrt(sqrt(m_volume));
  m_id = id;
  m_parentId = parentId;
  m_center = ( f[0].getCenter() + f[1].getCenter() ) / 2;
}

void BoundingBox::initialize()
{
  m_id = -1;
  m_childId1  = -1;
  m_childId2 = -1;
  m_volume = 1;
  m_hierarchyLevel = -1;
  m_gain = -1;
  m_bSelected = -1;
}

bool BoundingBox::envelopesPoint(Point p)
{
  int inside = 0;
  unsigned int i;
  for (i=0; i<6; i++)
    if (m_faces[i].distanceToPoint(p) <= 0)
      inside++;
  return (inside == 6);
}

//bool BoundingBox::envelopesPoint(IF_contact c)
//{
//  int inside = 0;
//  unsigned int i;
//  Point p(c.x(), c.y(), c.z());
//  for (i=0; i<6; i++)
//    if (m_faces[i].distanceToPoint(p) <= 0)
//      inside++;
//  return (inside == 6);
//}

int BoundingBox::getSideOfFace(unsigned int face_id, SIDE side, SIDE ref_dir)
{
  if (face_id > 5)
  {
    cout << "Failure: FACE_ID must be between 0 and 5!" << endl;
    return -1;
  }

  // We can resolve redundancy by giving a reference side that is LEFT.
  switch (ref_dir)
  {
    case TOP:
      switch (side)
      {
        case LEFT:   side = TOP; break;
        case TOP:    side = RIGHT; break;
        case RIGHT:  side = BOTTOM; break;
        case BOTTOM: side = LEFT; break;
        default: break;
      }
    case RIGHT:
      switch (side)
      {
        case LEFT:   side = RIGHT; break;
        case TOP:    side = BOTTOM; break;
        case RIGHT:  side = LEFT; break;
        case BOTTOM: side = TOP; break;
        default: break;
      }
    case BOTTOM:
      switch (side)
      {
        case LEFT:   side = BOTTOM; break;
        case TOP:    side = LEFT; break;
        case RIGHT:  side = TOP; break;
        case BOTTOM: side = RIGHT; break;
        default: break;
      }
    default: break;
  }

  // Depending on the unified face, we return the related one.
  switch(side)
  {
    case LEFT:   return lftFaces[face_id];
    case RIGHT:  return rgtFaces[face_id];
    case TOP:    return topFaces[face_id];
    case BOTTOM: return botFaces[face_id];
    case BACK:   return bckFaces[face_id];
    default: return -1;
  }
}

void BoundingBox::createBoxFromCoords(int id, int parent, const vector<double>& coords)
{
  vector<Point>  points;

  assert (coords.size() == 24); //8 vertices of box * 3 coordinates

  //Create vector of points from vector of double
  for (std::size_t i = 0; i < coords.size(); i += 3)
    points.push_back(Point(coords[i], coords[i+1], coords[i+2]));

  unsigned int limit = numIndices;
  bool boxOk = true;

  unsigned int i,a,b,c,d;
  Vector center;

  // Find the center of the box for depth calculation
  // (later on, each face's depth, i.e. the depth of the box to this face, is needed).
  center = (points[0] + points[7])/2;

  int fid=0;
  //_faces.clear();
  for (i = 0; i < limit; i += 4)
  {
    a = faceIndices[i];
    b = faceIndices[i+1];
    c = faceIndices[i+2];
    d = faceIndices[i+3];

    Face f(points[a], points[b], points[c], points[d], 2*(center - (points[a] + points[c])/2).length());

    if (f.isValid())
    {
      if (f.distanceToPoint(Point(center.x(), center.y(), center.z())) > 0)
	f.switchNormal();
      f.setBoxID(id,fid++);
      m_faces.push_back(f);
    } else {
      boxOk = false;
      break;
    }
  }

  // Initialize if valid
  if (boxOk)
  {
    m_volume = 1;
    //_faces.insert(_faces.end(), faces.begin(), faces.end());
    for (i = 0; i < 6; ++i)
      m_volume *= m_faces[i].getArea();
    m_volume = sqrt(sqrt(m_volume));
    m_center = ( m_faces[0].getCenter() + m_faces[1].getCenter() ) / 2;
    m_id = id;
    m_parentId = parent;
  }

  //BoundingBox bb = boxOk ? BoundingBox(faces, id, parent) : BoundingBox();
  //cout << "center " << id << ": " << bb.center() << endl;
  //return bb;
}


