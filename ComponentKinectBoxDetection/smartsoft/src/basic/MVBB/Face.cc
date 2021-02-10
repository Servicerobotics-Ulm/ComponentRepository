/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

#include "Face.h"
#include <algorithm>

ostream& operator<< (ostream& os, const Face& face) 
{
  os << face.m_points[0] << " " << face.m_points[1] << " " << face.m_points[2] << " " << face.m_points[3];
  return os;
}

Face::Face() : 
  m_points(vector<Point>(4)),
  m_edges (vector<Vector>(4)),
  m_edgeblocks (vector<double>(4))
{
};

Face::Face(const Point& a, const Point& b, const Point& c, const Point& d, double depth) : 
  m_points(vector<Point>(4)),
  m_edges (vector<Vector>(4)),
  m_edgeblocks (vector<double>(4))
{
  m_points[0] = a;
  m_points[1] = b;
  m_points[2] = c;
  m_points[3] = d;
  m_depth = depth;	
  m_center = (a + c) / 2;
  m_boxID = -1;
  m_bOccluded = false;

  unsigned int i;
  for (i=0; i<3; i++)
    m_edges[i] = m_points[i+1] - m_points[i];
  m_edges[3] = m_points[0] - m_points[3];
  
  for (i=0; i<4; i++)
    m_edgeblocks[i] = 0;//-1;
  
  m_normal = m_edges[0].crossProduct(m_edges[1]);
  m_normal.normalize();  
  m_area = m_edges[0].length() * m_edges[1].length();
}

void Face::switchNormal()
{
  m_normal = -m_normal;
}

double Face::distanceToPoint(const Point p)
{
  return (m_normal.x()*(p.x() - m_points[0].x()) + 
	  m_normal.y()*(p.y() - m_points[0].y()) + 
	  m_normal.z()*(p.z() - m_points[0].z())) / m_normal.length();
}

bool Face::intersects(const Face& other, double &t) 
{
  if (*this == other) return false;
	
  double s, u;  
  Vector m = other.c2() - other.c1(); //these 3
  Vector n = other.c4() - other.c1(); //vectors define
  Vector o = m_center    - other.c1(); //the plane!!!
  Vector r = m_normal; //r = ray
  
  Vector temp1 = r.crossProduct(n);
  Vector temp2 = o.crossProduct(m);
  
  double x = 1 / (temp1 * m);
  
  t = x * ( temp2 * n );
  s = x * ( temp1 * o );
  u = x * ( temp2 * r );
  
  return (s >= 0 && s <= 1 && u >= 0 && u <= 1 && t > 0);
}


double Face::getIntersection(const Face& other, Point &intersection) 
{
  if (*this == other) return 0;
  
  double t, s, u;
  Vector m = other.c2() - other.c1(); //these 3
  Vector n = other.c4() - other.c1(); //vectors define
  Vector o = m_center    - other.c1(); //the plane!!!
  Vector r = m_normal; //r = ray
  
  Vector temp1 = r.crossProduct(n);
  Vector temp2 = o.crossProduct(m);
  
  double x = 1 / (temp1 * m);
  
  t = x * ( temp2 * n );
  s = x * ( temp1 * o );
  u = x * ( temp2 * r );
  
  bool success = (s >= 0 && s <= 1 && u >= 0 && u <= 1 && t > 0);

  if (success) 
  {
    Vector c = m_center;
    Vector res = c + (r * t);
    intersection.set(res.x(), res.y(), res.z());
    return t;
  }

  return -1;
}

bool Face::isValid() const 
{
  vector<Point> tmp(m_points);
  sort(tmp.begin(), tmp.end());
  
  // 	for (std::size_t i = 0; i < tmp.size(); ++i) {
  // 		cout << tmp[i] << endl;
  // 	}
  
  vector<Point>::iterator pos = unique(tmp.begin(), tmp.end());
	
  // 	if (pos != tmp.end()) {
  // 		for (vector<Point>::iterator it = tmp.begin(); it != pos; ++it) {
  // 			cout << *it << endl;
  // 		}
  // 		int d;
  // 		cin >> d;
  // 		cout << d*d << endl;
  // 	}
  
  return (pos == tmp.end());
}

bool Face::isGraspableWithHand(unsigned int mode, double max_edge_length) 
{
  assert(mode >= 0 && mode < GRASPS_PER_FACE);
  
  double length;
  Vector v;
  
  if (mode == 0 || mode == 2)
  {
    length = m_edges[0].length();
    if (m_edgeblocks[0] > 0.00001 || m_edgeblocks[2] > 0.00001)
      return false;
  }
  else if (mode == 1 || mode == 3)
  {
    length = m_edges[1].length();
    if (m_edgeblocks[1] > 0.00001 || m_edgeblocks[3] > 0.00001)
      return false;
  }
  /*
  else // if 45 deg ... 3 < mode < 8
  {
    bool is0min = (m_edges[0].length() < m_edges[1].length());
    length = sqrt(2)*min(m_edges[0].length(), m_edges[1].length());
    if (is0min && (m_edgeblocks[0] > 0.00001 || m_edgeblocks[2] > 0.00001))
      return false;
    if (!is0min && (m_edgeblocks[1] > 0.00001 || m_edgeblocks[3] > 0.00001))
      return false;
  }
  */
  return (length <= max_edge_length);
}

void Face::setBlocked(Vector n, double d)
{
  unsigned int i=0;
  for (i=0; i<4; i++)
  {
    // We test for collinearity of the face's edge vectors to the given vector
    // so that in case of collinearity, these grasping directions are to be blocked.
    if (m_edges[i] * n  > 0.99 * (m_edges[i].length() * n.length()))
      m_edgeblocks[i] = 1;
  }
}

bool Face::operator== (const Face& other) const 
{
  return (m_points[0] == other.m_points[0] && 
	  m_points[1] == other.m_points[1] && 
	  m_points[2] == other.m_points[2] && 
	  m_points[3] == other.m_points[3]);
}
	
