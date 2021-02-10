/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

#include "Mvbb.h"

//#define DEBUG
//#define _DEBUG_

#define NONE (-1)
const int CONVHULL  = 30;
const double MY_EPS = 0.005;

int bbCont::_ID = -1;

/**
 * Tests if a point is Left|On|Right of an infinite line.
 * @param P0 first point
 * @param P1 second point
 * @param P2 third point
 * @return >0 for P2 left of the line through P0 and P1,
 *         =0 for P2 on the line
 *         <0 for P2 right of the line
 */
inline float isLeft( Point2d P0, Point2d P1, Point2d P2 )
{
  return (float)((P1.x() - P0.x())*(P2.y() - P0.y()) - (P2.x() - P0.x())*(P1.y() - P0.y()));
};
  
/**
 * Bin struct for the BFP fast approximate 2D convex hull algorithm.
 */
class Bin 
{
public:
  /// Index of min point P[] in bin (>=0 or NONE)
  int min;    
  /// index of max point P[] in bin (>=0 or NONE)
  int max;    
};

// area2D_Polygon(): computes the area of a 2D polygon
//    Input:  int n = the number of vertices in the polygon
//            Point* V = an array of n+2 vertices
//                       with V[n]=V[0] and V[n+1]=V[1]
//    Return: the (float) area of the polygon
float area2D_Polygon( vector<Point2d> P )
{
  int n = P.size();
  P.push_back(P[0]);
  P.push_back(P[1]);
  
  float area = 0;
  
  int i,j,k;
  for (i=1, j=2, k=0; i<=n; i++, j++, k++) 
    area += P[i].x() * (P[j].y() - P[k].y());  
  return area / 2.0;
};

int nearHull_2D( vector< Point2d > P, unsigned int k, vector< Point2d > &H, bool convex=true )
{
  if (P.size() < 1) return 0;

  H.clear();
  int    minmin=0,  minmax=0;
  int    maxmin=0,  maxmax=0;
  float  xmin = P[0].x(),  xmax = P[0].x();
  Point2d cP;                 // the current point being considered
  int    bot=0, top=(-1);  // indices for bottom and top of the stack
  
  // Get the points with (1) min-max x-coord, and (2) min-max y-coord
  for (unsigned int i=1; i<P.size(); i++) 
  {
    cP = P[i];
    if (cP.x() <= xmin) 
    {
      if (cP.x() < xmin) 
      {
        // new xmin
        xmin = cP.x();
        minmin = minmax = i;
      }
      else 
      {
        // another xmin
        if (cP.y() < P[minmin].y())
          minmin = i;
        else if (cP.y() > P[minmax].y())
          minmax = i;
      }
    }
    if (cP.x() >= xmax) 
    {
      if (cP.x() > xmax) 
      {
        // new xmax
        xmax = cP.x();
        maxmin = maxmax = i;
      }
      else 
      {
        // another xmax
        if (cP.y() < P[maxmin].y())
          maxmin = i;
        else if (cP.y()> P[maxmax].y())
          maxmax = i;
      }
    }
  }
  
  if (xmin == xmax) 
  {
    // degenerate case: all x-coords == xmin
    {H.push_back(P[minmin]); top++;}
    //H[++top] = P[minmin];           // a point, or
    if (minmax != minmin)           // a nontrivial segment
      {H.push_back(P[minmax]); top++;}//H[++top] = P[minmax];
    return top+1;                   // one or two points
  }
  
  // Next, get the max and min points in the k range bins
  Bin*   B = new Bin[k+2];   // first allocate the bins
  B[0].min = minmin;         B[0].max = minmax;        // set bin 0
  B[k+1].min = maxmin;       B[k+1].max = maxmax;      // set bin k+1
  for (unsigned int b=1; b<=k; b++) 
  {
    // initially nothing is in the other bins
    B[b].min = B[b].max = NONE;
  }
  for (unsigned int b, i=0; i<P.size(); i++) 
  {
    cP = P[i];
    if (cP.x() == xmin || cP.x() == xmax) // already have bins 0 and k+1
      continue;
    // check if a lower or upper point
    if (isLeft( P[minmin], P[maxmin], cP) < 0) // below lower line
    {  
      b = (int)( k * (cP.x() - xmin) / (xmax - xmin) ) + 1;  // bin #
      if (B[b].min == NONE)       // no min point in this range
        B[b].min = i;           // first min
      //else if (cP.x() < B[b].min)
      else if (cP.y() < P[B[b].min].y())
        B[b].min = i;           // new min
      continue;
    }
    if (isLeft( P[minmax], P[maxmax], cP) > 0) // above upper line
    {  
      b = (int)( k * (cP.x() - xmin) / (xmax - xmin) ) + 1;  // bin #
      if (B[b].max == NONE)       // no max point in this range
        B[b].max = i;           // first max
      //else if (cP.x() < B[b].min)
      else if (cP.y() > P[B[b].max].y())
        B[b].max = i;           // new max
      continue;
    }
  }
  
  // Now, use the chain algorithm to get the lower and upper hulls
  // the output array H[] will be used as the stack
  
  // First, compute the lower hull on the stack H
  for (unsigned int i=0; i <= k+1; ++i)
  {
    if (B[i].min == NONE)  // no min point in this range
      continue;
    cP = P[ B[i].min ];    // select the current min point
    
    while (top > 0)        // there are at least 2 points on the stack
    {
      // test if current point is left of the line at the stack top
      if (isLeft( H[top-1], H[top], cP) > 0)
        break;         // cP is a new hull vertex
      else 
      {
        if (convex) H.pop_back();
        top--;         // pop top point off stack
      }
    }
    {H.push_back(cP); top++;}
    //H[++top] = *cP;        // push current point onto stack
  }
    
  // Next, compute the upper hull on the stack H above the bottom hull
  if (maxmax != maxmin)      // if distinct xmax points
    {H.push_back(P[maxmax]); top++;}
  //H[++top] = P[maxmax];  // push maxmax point onto stack
  bot = top;                 // the bottom point of the upper hull stack
  for (int i=k; i >= 0; --i)
  {
    if (B[i].max == NONE)  // no max point in this range
      continue;
    cP = P[ B[i].max ];   // select the current max point
    
    while (top > bot)      // at least 2 points on the upper stack
    {
      // test if current point is left of the line at the stack top
      if (isLeft( H[top-1], H[top], cP) > 0)
        break;         // current point is a new hull vertex
      else
      {
        if (convex) H.pop_back();
        top--;         // pop top point off stack
      }
    }
    {H.push_back(cP); top++;}
    //H[++top] = *cP;        // push current point onto stack
  }
  if (minmax != minmin)
    {H.push_back(P[minmin]); top++;}
  //H[++top] = P[minmin];  // push joining endpoint onto stack

  delete[] B;                  // free bins before returning
  return top+1;              // # of points on the stack
};

MVBB::MVBB(vector<double>& points, const char* bbivfile, bool redux) 
{  
  _numPoints = points.size();
  _points = new double[_numPoints];
  _started = false;
  _finished = false;
  _redux = redux;
  sprintf(m_bbivfile,"%s", bbivfile);

  for (int i = 0; i < _numPoints; ++i)
    _points[i] = points[i];

  _numPoints /= 3;
  _iterations = 0;
  bbCont::_ID = -1;  
};

MVBB::MVBB(vector<Point>& points, const char* bbivfile, bool redux) 
{  
  //
  _numPoints = points.size();
  _points = new double[3*_numPoints];
  _started = false;
  _finished = false;
  _redux = redux;
  sprintf(m_bbivfile,"%s", bbivfile);

  for (int i = 0; i < _numPoints; i++)
  {
    _points[3*i]   = points[i].x();
    _points[3*i+1] = points[i].y();
    _points[3*i+2] = points[i].z();  
  }

  _iterations = 0;
  bbCont::_ID = -1;  
};

MVBB::~MVBB() 
{  
  delete[] _points;
};

vector<BoxInfo> MVBB::iterate() 
{
#ifdef _DEBUG_
  cout << "*** Entering MVBB::iterate() ***" << endl;
#endif
  
  vector<BoxInfo> retVal;	
  
  if (!_started) 
  {
    int num = _numPoints;   
    gdiam_point* pnt_arr = gdiam_convert( _points, num );
    
    stringstream ss;
    ss << "[" << num << " points] [grid = " << m_params.Grid << "] [samples = " 
       << m_params.Samples << "] [gain = " << m_params.Gain << "]";
    ColorDebug::tabPlot(" Computing Root MVBB: ", ss, 0, GREEN);

    // 1st approximation
    bbCont bbC;
    bbC.bb = gdiam_approx_mvbb_grid_sample( pnt_arr, num, m_params.Grid, m_params.Samples );
    delete[] pnt_arr;

    bbC.parentIdx = -1;
    
    for (int i = 0; i < num*3; ++i)
      bbC.points.push_back(_points[i]);
    
    bbCSet.insert(bbC);
    
    _started = true;
    
    BoxInfo info;          
    vector<double> bbCoords;
    bbC.bb.dumpToBBVec(bbCoords);
    
    info.id = bbC.id;
    info.parentId = bbC.parentIdx;
    info.hLevel = 0;
    info.coords = bbCoords;
    retVal.push_back(info);
          
    ++_iterations;
    if (strcmp(m_bbivfile, "") != 0)
      dumpCurrentState();          
    return retVal;         
  } 
  else 
  {          
    if (bbCSet.size() == 0) 
    {
      cerr << "No need to Iterate, algorithm is done..." << endl;
      _finished = true;
      return retVal;
    }
    
    retVal = cut();
    ++_iterations;
    if (strcmp(m_bbivfile, "") != 0)
      dumpCurrentState();
    
#ifdef _DEBUG_
    cout << "*** Leaving MVBB::iterate() ***" << endl;
#endif    
    return retVal;
  }
}

void MVBB::dumpCurrentState() 
{
#ifdef _DEBUG_
  cout << "*** Entering MVBB::dumpCurrentState() *** ";
#endif
	
  if ( bbCSet.empty() && bbVec.empty()) 
  {
    cerr << "Unexpected emptiness in vector :-/. Nothing to dump... " << endl;
    return;
  }

  FILE *fp;

  multiset<bbCont>::iterator iter = bbCSet.begin();
  for (; iter != bbCSet.end(); ++iter) 
  {
    bbCont bbC = *iter;
    if (iter == bbCSet.begin())
      { 
        //fp = fopen(m_bbivfile, "w");
        sprintf(bbC.bb.bbFile,"%s", m_bbivfile);
        bbC.bb.dump("w");
        //fclose(fp);
      }// Dump first one seperately in "w" mode
    else
      {
        //fp = fopen(m_bbivfile, "a");
        sprintf(bbC.bb.bbFile,"%s", m_bbivfile);
        bbC.bb.dump("a");
        //fclose(fp);
      } //append all the others with "a" mode
  }
                
  bool firstDump = bbCSet.empty();	
  for (unsigned int i = 0; i < bbVec.size(); ++i) 
  {
    if (firstDump) 
    {
      if (saveIdx[i]) 
      {
        //fp = fopen(m_bbivfile, "w");
        sprintf(bbVec[i].bbFile, "%s", m_bbivfile);
        bbVec[i].dump("w"); // Dump first one seperately in "w" mode
        //fclose(fp);
        firstDump = false;
      }
    } else if(saveIdx[i]) {
      //fp = fopen(m_bbivfile, "a");
      sprintf(bbVec[i].bbFile, "%s", m_bbivfile);
      bbVec[i].dump("a"); //append all the others with "a" mode
      //fclose(fp);
    }
  }
	
  // do not forget the final bracket in the .iv file
  fp = fopen(m_bbivfile, "a");
  fprintf(fp, "}\n");
  fclose(fp); 
  out_bbiv.push_back("}");
  
#ifdef _DEBUG_
  cout << "*** Leaving MVBB::dumpCurrentState() ***" << endl;
#endif	
}

vector<BoxInfo> MVBB::cut()
{
#ifdef _DEBUG_
  cout << "*** Entering MVBB::cut() ***" << endl;
#endif
  
  int grid = m_params.Grid;
  int samples = m_params.Samples;

  vector<BoxInfo> retVal;  
  vector<double> plv, prv;
  bbCont bbC = *bbCSet.begin();
  int s = (*bbCSet.begin()).points.size() / 3; 
  bbCSet.erase(bbCSet.begin());
  if (s <= m_params.MinPoints)
    return retVal;

  double altVol = calVol();
  gdiam_bbox bb = bbC.bb;

  stringstream ss;
  ss << "[" << bbC.id << "] ";
  ss << ColorDebug::color(GREEN) << "[parent = " << bbC.parentIdx << "] [hierarchy level = " << bbC.hLevel << "]";
  ColorDebug::tabPlot(" Current Box: ", ss, CYAN, CYAN);
  
  divide(bbC.points, bb.get_dir(0), bb.get_dir(1), bb.get_dir(2), plv, prv);
  
  int numl = (int)plv.size()/3;
  int numr = (int)prv.size()/3;
  
  if (numl != 0 && numr != 0) 
  {      
    gdiam_real *pointsl = new gdiam_real[numl*3];
    gdiam_real *pointsr = new gdiam_real[numr*3];
    
    for(int i=0; i < numl*3; ++i) 
      pointsl[i] = plv[i];
    
    for(int i = 0; i < numr*3; ++i) 
      pointsr[i] = prv[i];
      
    try 
    {
      gdiam_point *pnt_arr = NULL;
      gdiam_bbox bbl, bbr;
        
      stringstream ss;
      bool bl = numl > m_params.MinPoints;
      if (bl) 
      {
        pnt_arr = gdiam_convert( pointsl, numl );
        bbl = gdiam_approx_mvbb_grid_sample(pnt_arr, numl, grid, samples);
        delete[] pnt_arr; pnt_arr = NULL;
        ss << "New MVBB with " << numl << " points";
        ColorDebug::tabPlot(" MVBB from subset A: ", ss, 0, GREEN);
      }
      else 
      {
        ss << "Dropped. Too few points [" << numl << "]";
        ColorDebug::tabPlot(" MVBB from subset A: ", ss, 0, RED);
      }      
      
      ss.str("");
      bool br = numr > m_params.MinPoints;
      if (br) 
      {
        pnt_arr = gdiam_convert( pointsr, numr );
        bbr = gdiam_approx_mvbb_grid_sample(pnt_arr, numr, grid, samples);
        delete[] pnt_arr; pnt_arr = NULL;
        ss << "New MVBB with " << numr << " points";
        ColorDebug::tabPlot(" MVBB from subset B: ", ss, 0, GREEN);
      }
      else 
      {
        ss << "Dropped. Too few points [" << numr << "]";
        ColorDebug::tabPlot(" MVBB from subset B: ", ss, 0, RED);
      }      
 	         
      float gain = (bbl.volume() + bbr.volume() + altVol) / (bb.volume() + altVol);
      //cout << endl << ColorDebug::color(GREEN, " \xab Checking the 3D volume gain after splitting:") << endl
      //<< "    New volume: " << (bbl.volume() + bbr.volume() + altVol) 
      //   << " (" << bbl.volume() << "(L) + " << bbr.volume() << "(R) + " 
      //   << altVol << "(A))" << endl << "    Old volume: " 
      //   << (bb.volume() + altVol) << " (" << bb.volume() 
      //   << "(O) + " << altVol << "(A))" << endl;
           
      if (gain < m_params.Gain)
      {     
        char txt[16];
        sprintf(txt, "%f", gain);
        ColorDebug::tabPlot(" Gain: ", txt, 0, CYAN);
        
        if (bl) 
        {            
          bbCont bbCl;
          bbCl.points = plv;
          bbCl.bb = bbl;
          bbCl.parentIdx = bbC.id;
          bbCl.hLevel = bbC.hLevel+1;
          bbCl.gain = (bbl.volume() + altVol/2) / (bb.volume() + altVol);
          bbCSet.insert(bbCl);
          
          BoxInfo info;					
          vector<double> bbCoords;
          bbCl.bb.dumpToBBVec(bbCoords);            
          info.id = bbCl.id;
          info.parentId = bbCl.parentIdx;
          info.hLevel = bbC.hLevel + 1;
          info.gain = (bbl.volume() + altVol/2) / (bb.volume() + altVol);
          info.coords = bbCoords;
          
          retVal.push_back(info);
        }
        if (br) 
        {
          bbCont bbCr;
          bbCr.points = prv;
          bbCr.bb = bbr;
          bbCr.parentIdx = bbC.id;
          bbCr.hLevel = bbC.hLevel+1;
          bbCr.gain = (bbr.volume() + altVol/2) / (bb.volume() + altVol);
          bbCSet.insert(bbCr);
          
          BoxInfo info;            
          vector<double> bbCoords;
          bbCr.bb.dumpToBBVec(bbCoords);            
          info.id = bbCr.id;
          info.parentId = bbCr.parentIdx;
          info.hLevel = bbC.hLevel + 1;
          info.gain = (bbr.volume() + altVol/2) / (bb.volume() + altVol);
          info.coords = bbCoords;
          
          retVal.push_back(info);
        }
        if (bl || br) 
        {
          /*
          multiset<bbCont> tmpSet;
          checkDismissedBB(tmpSet);
          
          if (!tmpSet.empty()) 
          {              
            cout << "checkDismissedBB() was successful. " << tmpSet.size() << " box(es) added." << endl;
            multiset<bbCont>::iterator iter = tmpSet.begin();
            while (iter != tmpSet.end()) 
            {               
              BoxInfo info;              
              vector<double> bbCoords;
              (*iter).bb.dumpToBBVec(bbCoords);
              
              info.id = (*iter).id;
              info.parentId = (*iter).parentIdx;
              info.hLevel = bbC.hLevel;
              //info.gain = bbC.gain;
              info.coords = bbCoords;
              
              retVal.push_back(info);
              bbCSet.insert(*iter);
              ++iter;
            }
          }
          */
        }
      }
      else 
      {
        char txt[16];
        sprintf(txt, "%f", gain);
        ColorDebug::tabPlot(" Gain: ", txt, 0, RED);
        
        bbDismissed bbDt;
        //bbDt.l = bbCl;
        bbDt.l.points = plv;
        bbDt.l.bb = bbl;
        bbDt.l.parentIdx = bbC.id;
        bbDt.l.hLevel = bbC.hLevel;
        bbDt.l.gain = (bbl.volume() + altVol/2) / (bb.volume() + altVol);
        
        //bbDt.r = bbCr;
        bbDt.r.points = prv;
        bbDt.r.bb = bbr;
        bbDt.r.parentIdx = bbC.id;
        bbDt.r.hLevel = bbC.hLevel;
        bbDt.r.gain = (bbr.volume() + altVol/2) / (bb.volume() + altVol);
        
        bbDt.parentIdx = bbVec.size();
        bbD.push_back(bbDt);
        bbVec.push_back(bb);
        saveIdx.push_back(true);
      }
      //delete(pointsl);
      //delete(pointsr);
    }
    catch(...) 
    {
      cout << "Error: failed to calculate new bounding boxes, " << "pushing current bounding box." << endl;
      bbVec.push_back(bb);
      saveIdx.push_back(true);
      return retVal;
    }
  }
  else 
  {
    ColorDebug::tabPlot(" Stop: ", "One of the new containers is empty", RED, RED);
    bbVec.push_back(bb);
    saveIdx.push_back(true);
    //return retVal;
  }
  
  if (bbCSet.empty()) 
    _finished = true;  
  
#ifdef _DEBUG_
  cout << "*** Leaving MVBB::cut() ***" << endl;
#endif
  
  return retVal;
}

double MVBB::calVol() 
{
#ifdef _DEBUG_
  cout << "*** Entering MVBB::calVol() ***" << endl;
#endif
	
  double vol = 0;
  //cout << "bbCSet size " << bbCSet.size() << ", bbVec size " << bbVec.size() << endl;
	
  for (multiset<bbCont>::const_iterator ic = bbCSet.begin(); ic != bbCSet.end(); ++ic)
    vol += (*ic).bb.volume();
	
  for (unsigned int i = 0; i < bbVec.size(); ++i)
    if (saveIdx[i])
      vol += bbVec[i].volume();
        
#ifdef _DEBUG_
  cout << "calVol returns " << vol << endl;
  cout << "*** Leaving MVBB::calVol() ***" << endl;
#endif
	
  return vol;
}
  
void MVBB::divide(vector<double> points, double X[3], double Y[3], double Z[3],
                  vector<double> &pointsl, vector<double> &pointsr) 
{
#ifdef _DEBUG_
  cout << "*** Entering MVBB::divide() ***" << endl;
#endif
	
  Point3d x_norm(X[0], X[1], X[2]);
  //cout << "X: " << X[0] << "," << X[1] << "," << X[2] << endl;
  x_norm.normalize();
  Point3d y_norm(Y[0], Y[1], Y[2]);
  //cout << "Y: " << Y[0] << "," << Y[1] << "," << Y[2] << endl;
  y_norm.normalize();
  Point3d z_norm(Z[0], Z[1], Z[2]);
  //cout << "Z: " << Z[0] << "," << Z[1] << "," << Z[2] << endl;
  z_norm.normalize();
  
  vector<Point3d> points3d;
  argVector2point3dVector(points, points3d);
  _original = points3d;
  transformCordinates2BBCordinates(points3d, x_norm, y_norm, z_norm);
  
  int cutface_idx;
  real_mvbb xmin, xmax, ymin, ymax, zmin, zmax;
  Point2d p,q;
  vector<Point3d> left, right;
  calMaxMin(points3d, xmin, xmax, ymin, ymax, zmin, zmax);	
  findBestCut(points3d, xmin, xmax, ymin, ymax, zmin, zmax, cutface_idx, p, q);
  makeCut(points3d, cutface_idx, p, q, _original, left, right);
  
#ifdef _DEBUG_
  cout << " points in left container " << left.size() << endl;
  cout << " points in right container " << right.size() << endl;
#endif
  
  point3dVector2DoubleVector(left, pointsl);
  point3dVector2DoubleVector(right, pointsr);
  
#ifdef DEBUG
  //dumpVector2File(left, "left.crd");
  //dumpVector2File(right, "right.crd");
#endif
  
#ifdef _DEBUG_
  cout << "*** Leaving MVBB::divide() ***" << endl;
#endif
}

void MVBB::argVector2point3dVector(vector<double> points, vector<Point3d> &points3d) 
{	
#ifdef _DEBUG_
  cout << "*** Entering MVBB::argVector2point3dVector() ***" << endl;
#endif
  
  Point3d pnt;
  for(unsigned int i = 0; i < points.size()/3; ++i, points3d.push_back(pnt)) 
  {
    pnt[0] = points[i*3];
    pnt[1] = points[i*3 + 1];
    pnt[2] = points[i*3 +2];
  }
  
#ifdef _DEBUG_
  cout << "*** Leaving MVBB::argVector2point3dVector() ***" << endl;
#endif
}
  
void MVBB::point3dVector2DoubleVector(vector<Point3d> points3d, vector<double> &points) 
{					     
#ifdef _DEBUG_
  cout << "*** Entering MVBB::point3dVector2DoubleVector() ***" << endl;
#endif
	
  points.clear();
  for(unsigned int i = 0; i < points3d.size();++i) 
  {
    points.push_back(points3d[i][0]);
    points.push_back(points3d[i][1]);
    points.push_back(points3d[i][2]);
  }
#ifdef _DEBUG_
  cout << "*** Leaving MVBB::point3dVector2DoubleVector() ***" << endl;
#endif
}

void MVBB::transformCordinates2BBCordinates(vector<Point3d> &points3d, 
                                            const Point3d &x_norm, const Point3d &y_norm, const Point3d &z_norm) 
{							  
#ifdef _DEBUG_
  cout << "*** Entering MVBB::transformCordinates2BBCordinates() ***" << endl;
#endif
							  
  real_mvbb x,y,z;
  for(unsigned int i=0 ; i< points3d.size(); i++) 
  {
    //cout << points3d[i].coords[0] << "," << points3d[i].coords[1] << "," << points3d[i].coords[2] << endl;
    x = points3d[i].dot_prod(x_norm);
    y = points3d[i].dot_prod(y_norm);
    z = points3d[i].dot_prod(z_norm);
    points3d[i].coords[0] = x;
    points3d[i].coords[1] = y;
    points3d[i].coords[2] = z;
    //cout << x << "," << y << "," << z << endl;
  }
  /*
    cout << "X: " << x_norm.coords[0] << "," << x_norm.coords[1] << "," << x_norm.coords[2] << endl;
    cout << "Y: " << y_norm.coords[0] << "," << y_norm.coords[1] << "," << y_norm.coords[2] << endl;
    cout << "Z: " << z_norm.coords[0] << "," << z_norm.coords[1] << "," << z_norm.coords[2] << endl;
  */
  
#ifdef _DEBUG_
  cout << "*** Leaving MVBB::transformCordinates2BBCordinates() ***" << endl;
#endif	
}
  
void MVBB::calMaxMin(vector<Point3d> &points3d,
		   real_mvbb &xmin, real_mvbb &xmax,
		   real_mvbb &ymin, real_mvbb &ymax,
		   real_mvbb &zmin, real_mvbb &zmax) 
{    
#ifdef _DEBUG_
  cout << "*** Entering MVBB::calMaxMin() ***" << endl;
#endif
  
  xmin=points3d[0].coords[0];
  xmax=points3d[0].coords[0];
  ymin=points3d[0].coords[1];
  ymax=points3d[0].coords[1];
  zmin=points3d[0].coords[2];
  zmax=points3d[0].coords[2];
  
  for (unsigned int i=0; i<points3d.size(); i++) 
  {		
    if      (points3d[i].coords[0] < xmin) xmin = points3d[i].coords[0];
    else if (points3d[i].coords[0] > xmax) xmax = points3d[i].coords[0];
    
    if      (points3d[i].coords[1] < ymin) ymin = points3d[i].coords[1];
    else if (points3d[i].coords[1] > ymax) ymax = points3d[i].coords[1];
    
    if      (points3d[i].coords[2] < zmin) zmin = points3d[i].coords[2];
    else if (points3d[i].coords[2] > zmax) zmax = points3d[i].coords[2];  
  }

#ifdef _DEBUG_
  cout << "*** Leaving MVBB::calMaxMin() ***" << endl;
#endif
}

int MVBB::findBestHull(vector<Point2d> points, Point2d &opt_p, Point2d &opt_q, float &minV) 
{
  unsigned int RES = 4;
  unsigned int i,j,k;
  vector<Point2d> hull;
  nearHull_2D(points, CONVHULL, hull);
  
  if (hull.size() < 3) 
  {
    cout << "Error: mvbb.cpp, hull size < 3 (" << hull.size() << ")" << endl;
    return 0;
  }
  
  hull.push_back(hull[1]);
  
#ifdef DEBUG
  FILE *fp = fopen(ColorDebug::logfile("Mvbb-points"), "w");
  for (j=0; j<points.size(); j++)
    fprintf(fp,"%f %f\n", points[j].x(), points[j].y());
  fclose(fp);    
  fp = fopen(ColorDebug::logfile("Mvbb-concaves"), "w");
#endif
   
  Point2d d1, d2;
  vector<unsigned int> cracks;
  for (i=0; i<hull.size()-1; i++)
  {      
    // Length cracks
    d1 = hull[i]; d1.subtract(hull[i+1]);
    if (d1.length() > 15)
    {
      for (j=0; j<=RES; j++)
      {
        d2.setCoord(0, hull[i].x() - j * d1.x() / RES);
        d2.setCoord(1, hull[i].y() - j * d1.y() / RES);	  
        cracks.push_back(i+j);
        hull.insert(hull.begin()+i+j, d2);	  
#ifdef DEBUG
        fprintf(fp,"%f %f\n", d2.x(), d2.y());
#endif	  	  
      }
      i += RES+1;
    }      
  }
#ifdef DEBUG
  fclose(fp);   

  fp = fopen(ColorDebug::logfile("Mvbb-hull"), "w");
  for (j=0; j<hull.size(); j++)
    fprintf(fp,"%f %f\n", hull[j].x(), hull[j].y());
  fclose(fp);
#endif	  	  
  
  float area, d;
  minV = 100000;
  vector<Point2d> p1,p2;
  vector<Point2d> h1,h2;
  for (i=0; i<cracks.size(); i++)
  {
    for (j=i+1; j<cracks.size(); j++)
    {
      p1.clear(); p2.clear();
      for(k=0; k<points.size(); k++)
      {
        d = isLeft(hull[cracks[i]], hull[cracks[j]], points[k]);
        if ( d <  5) p1.push_back(points[k]);
        if ( d > -5) p2.push_back(points[k]);
      }

      nearHull_2D(p1, 10, h1);
      nearHull_2D(p2, 10, h2);   
      area = area2D_Polygon(h1) + area2D_Polygon(h2) - area2D_Polygon(hull);	
      
      if (area < minV)
      {
        minV  = area;
        opt_p = hull[cracks[i]];
        opt_q = hull[cracks[j]];
#ifdef DEBUG
        fp = fopen(ColorDebug::logfile("Mvbb-cut"), "w");
        fprintf(fp,"%f %f\n", opt_p.x(), opt_p.y());
        fprintf(fp,"%f %f\n", opt_q.x(), opt_q.y());
        fclose(fp);
        fp = fopen(ColorDebug::logfile("Mvbb-hull1"), "w");
        for (k=0; k<h1.size(); k++)
          fprintf(fp,"%f %f\n", h1[k].x(), h1[k].y());
        fclose(fp);
        fp = fopen(ColorDebug::logfile("Mvbb-hull2"), "w");
        for (k=0; k<h2.size(); k++)
          fprintf(fp,"%f %f\n", h2[k].x(), h2[k].y());
        fclose(fp);
#endif
      }
    }
  }
  return cracks.size();
};

void MVBB::findBestCut(vector<Point3d> &points3d,
                       const real_mvbb &xmin, const real_mvbb &xmax,
                       const real_mvbb &ymin, const real_mvbb &ymax,
                       const real_mvbb &zmin, const real_mvbb &zmax,
                       int &opt_face, Point2d &opt_p, Point2d &opt_q) 
{
  //cout << ColorDebug::color(GREEN, " \xab Checking three projections for convex hull cuts:\n");
  
  double XYZsideLengths[3];  

  // 	The grid cell size, take cell_max cells along the longest side
  XYZsideLengths[0] = xmax-xmin;
  XYZsideLengths[1] = ymax-ymin;
  XYZsideLengths[2] = zmax-zmin;
  
  real_mvbb cell_s= min(min(XYZsideLengths[0], XYZsideLengths[1]),  XYZsideLengths[2])/m_params.MinCutCells;
  
  if (cell_s - MY_EPS < 0)
    cell_s += 0.05;	
  assert (cell_s > 0);
  
  // 	Numbers of cells depending on min/max values
  unsigned int cell_x = (int)(.5+ (xmax-xmin)/cell_s)+1; 
  unsigned int cell_y = (int)(.5+ (ymax-ymin)/cell_s)+1; 
  unsigned int cell_z = (int)(.5+ (zmax-zmin)/cell_s)+1; 
  
  // 	Grids for projection
  vector<int> grid_xy(cell_x * cell_y, -1);
  vector<int> grid_xz(cell_x * cell_z, -1);
  vector<int> grid_yz(cell_y * cell_z, -1);
  vector<bool> grid_xyz(cell_x * cell_y * cell_z, false);
  
  vector<Point2d> points_xy;
  vector<Point2d> points_xz;
  vector<Point2d> points_yz;
  
  int ix, iy, iz;
    
  // 	generate grids
  unsigned int i,j;
  vector<Point3d> redux, oredux;
  if (_redux)
  {
    int oldsize = points3d.size();
    //for (it = points3d.begin(); it != points3d.end(); it++)
    for (i=0; i<points3d.size(); i++)
    {
      ix = (int)((points3d[i].coords[0]-xmin)/cell_s);
      iy = (int)((points3d[i].coords[1]-ymin)/cell_s);
      iz = (int)((points3d[i].coords[2]-zmin)/cell_s);
      
      grid_xy[iy*cell_x+ix] = max(grid_xy[iy*cell_x+ix], iz);
      grid_xz[iz*cell_x+ix] = max(grid_xz[iz*cell_x+ix], iy);
      grid_yz[iz*cell_y+iy] = max(grid_yz[iz*cell_y+iy], ix);
      
      if (!grid_xyz[(iy*cell_x+ix)*cell_z + iz])
      {
        grid_xyz[(iy*cell_x+ix)*cell_z + iz] = true;
        redux.push_back(points3d[i]);
        oredux.push_back(_original[i]);
      }      
    }
    points3d.clear();
    points3d.insert(points3d.begin(), redux.begin(), redux.end());
    _original.clear();
    _original.insert(_original.begin(), oredux.begin(), oredux.end());
    
    stringstream ss;
    ss << oldsize << " to " << points3d.size() << " points [" << 100*points3d.size()/oldsize << "\%]"; 
    ColorDebug::tabPlot(" Point reduction: ", ss, 0, GREEN);
  }
  else for (i=0; i<points3d.size(); i++)
  {
    ix = (int)((points3d[i].coords[0]-xmin)/cell_s);
    iy = (int)((points3d[i].coords[1]-ymin)/cell_s);
    iz = (int)((points3d[i].coords[2]-zmin)/cell_s);
    
    grid_xy[iy*cell_x+ix] = max(grid_xy[iy*cell_x+ix], iz);
    grid_xz[iz*cell_x+ix] = max(grid_xz[iz*cell_x+ix], iy);
    grid_yz[iz*cell_y+iy] = max(grid_yz[iz*cell_y+iy], ix);
  }
  
  for(i=0; i<cell_y; i++)
    for(j=0; j<cell_x; j++)
      if (grid_xy[i*cell_x+j]>0)
        points_xy.push_back(Point2d(j,i));

  for(i=0; i<cell_z; i++)
    for(j=0; j<cell_x; j++)
      if (grid_xz[i*cell_x+j]>0)
        points_xz.push_back(Point2d(j,i));

  for(i=0; i<cell_z; i++)
    for(j=0; j<cell_y; j++)
      if (grid_yz[i*cell_y+j]>0)
        points_yz.push_back(Point2d(j,i));
  
  // 	optimal values initiation
  
  float opt_vol = 100, vol, vol_perc;
  Point2d p, q;
  int numberOfCHcuts[3];
  char volOfCHcuts[3][256];
 
  // 	======= Find the best cut in xy plane =====
    
  numberOfCHcuts[0] = findBestHull(points_xy, p, q, vol);
  vol_perc = 100*vol/(cell_x*cell_y);
  if (vol_perc > 0) sprintf(volOfCHcuts[0], "NoCut");
  else sprintf(volOfCHcuts[0], "%.2f%%", vol_perc);
  
#ifdef DEBUG
  cout << "1) Best cut in xy is from point (" << p[0] << "," << p[1]
       << ") to point (" << q[0]<< "," << q[1] << ")" << endl 
       << "point (" << p[0]*cell_s+xmin << "," << p[1]*cell_s+ymin
       << ") to point (" << q[0]*cell_s+xmin << "," << q[1]*cell_s+ymin << ") (vol "
       << vol << ", " << vol_perc << "%)" << endl;
  cin >> i;
#endif
	
  if (vol_perc < opt_vol) 
  {
    opt_vol  = vol_perc;
    opt_face = 1;
    opt_p[0] = p[0]*cell_s+xmin;
    opt_p[1] = p[1]*cell_s+ymin;
    opt_q[0] = q[0]*cell_s+xmin;
    opt_q[1] = q[1]*cell_s+ymin;
  }	
  
  // 	======= Find the best cut in xz plane ======
  
  numberOfCHcuts[1] = findBestHull(points_xz, p, q, vol);
  vol_perc = 100*vol/(cell_x*cell_z);
  if (vol_perc > 0) sprintf(volOfCHcuts[1], "NoCut");
  else sprintf(volOfCHcuts[1], "%.2f%%", vol_perc);
  
#ifdef DEBUG
  cout << "2) Best cut in xz is from point (" << p[0] << "," << p[1]
       << ") to point (" << q[0]<< "," << q[1] << ")" << endl 
       << "point (" << p[0]*cell_s+xmin << "," << p[1]*cell_s+zmin
       << ") to point (" << q[0]*cell_s+xmin << "," << q[1]*cell_s+zmin << ") (vol "
       << vol << ", " << vol_perc << "%)" << endl;
  cin >> i;
#endif
	    
  if (vol_perc < opt_vol) 
  {
    opt_vol  = vol_perc;
    opt_face = 2;
    opt_p[0] = p[0]*cell_s+xmin;
    opt_p[1] = p[1]*cell_s+zmin;
    opt_q[0] = q[0]*cell_s+xmin;
    opt_q[1] = q[1]*cell_s+zmin;
  }

  // 	======= Find the best cut in yz plane ======
  
  numberOfCHcuts[2] = findBestHull(points_yz, p, q, vol);
  vol_perc = 100*vol/(cell_y*cell_z);
  if (vol_perc > 0) sprintf(volOfCHcuts[2], "noCut");
  else sprintf(volOfCHcuts[2], "%.2f%%", vol_perc);

#ifdef DEBUG
  cout << "3) Best cut in yz is from point (" << p[0] << "," << p[1]
       << ") to point (" << q[0]<< "," << q[1] << ")" << endl 
       << "point (" << p[0]*cell_s+ymin << "," << p[1]*cell_s+zmin
       << ") to point (" << q[0]*cell_s+ymin << "," << q[1]*cell_s+zmin << ")"
       << "(vol " << vol << ", " << vol_perc << "%)" << endl;	
  cin >> i;
#endif

  if (vol_perc < opt_vol) 
  {
    opt_vol  = vol_perc;
    opt_face = 3;
    opt_p[0] = p[0]*cell_s+ymin;
    opt_p[1] = p[1]*cell_s+zmin;
    opt_q[0] = q[0]*cell_s+ymin;
    opt_q[1] = q[1]*cell_s+zmin;
  }

  stringstream ss;
  ss << (opt_face == 1 ? ColorDebug::color(CYAN) : ColorDebug::color(GREEN))
     << "[XY: " << volOfCHcuts[0] << "] "; 
  ss << (opt_face == 2 ? ColorDebug::color(CYAN) : ColorDebug::color(GREEN))
     << "[XZ: " << volOfCHcuts[1] << "] ";
  ss << (opt_face == 3 ? ColorDebug::color(CYAN) : ColorDebug::color(GREEN))
     << "[YZ: " << volOfCHcuts[2] << "]";
  ColorDebug::tabPlot(" 2D Gain estimates: ", ss, 0, RED); 
} 

void MVBB::makeCut(const vector<Point3d> &in, const int &cut_idx, const Point2d &cut_p, const Point2d &cut_q,
                   const vector<Point3d> &original,
                   vector<Point3d> &left, vector<Point3d> &right)
{
  if(in.size() != original.size())
  {
    cout << "computed points and original points are of different size" << std::endl;
    exit(-1);
  }
    
  //cout << in.size() << " points to divide in plane " << cut_idx << endl;    

  Point2d p;
  if (cut_idx == 1)  
  {
    //cout << " cutting xy plane with line " << cut_p[0] << "," << cut_p[1] << " to " << cut_q[0] << "," << cut_q[1] << endl;
    for(size_t i = 0; i < in.size(); ++i)
    {
      p[0] = in[i][0];
      p[1] = in[i][1];
      
      if(isLeft(cut_p, cut_q, p)<0) left.push_back(original[i]);	
      else right.push_back(original[i]);	
    }
  }
  else if (cut_idx == 2)      
  {
    //cout << " cutting xz plane with line " << cut_p[0] << "," << cut_p[1] << " to " << cut_q[0] << "," << cut_q[1] << endl;
    for(size_t i = 0; i < in.size(); ++i)
    {
      p[0] = in[i][0];
      p[1] = in[i][2];
      if(isLeft(cut_p, cut_q, p)<0) left.push_back(original[i]);	
      else right.push_back(original[i]);	
    }
  }
  else if (cut_idx == 3)      
  {
    //cout << " cutting yz plane with line " << cut_p[0] << "," << cut_p[1] << " to " << cut_q[0] << "," << cut_q[1] << endl;
    for(size_t i = 0; i < in.size(); ++i)
    {
      p[0] = in[i][1];
      p[1] = in[i][2];
      
      if(isLeft(cut_p, cut_q, p)<0) left.push_back(original[i]);	
      else right.push_back(original[i]);	
    }
  }

#ifdef DEBUG
  unsigned int i;
  FILE *fp = fopen(ColorDebug::logfile("Mvbb-left"), "w");
  sort(left.begin(), left.end());
  for (i=0; i<left.size(); i++)    
    fprintf(fp,"%f %f %f\n", left[i][0], left[i][1], left[i][2]);
  fclose(fp);
  
  fp = fopen(ColorDebug::logfile("Mvbb-right"), "w");
  sort(right.begin(), right.end());
  for (i=0; i<right.size(); i++)   
    fprintf(fp,"%f %f %f\n", right[i][0], right[i][1], right[i][2]);
  fclose(fp);
#endif
}
  
