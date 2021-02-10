// -*- mode:C++; tab-width:2; c-basic-offset:2; indent-tabs-mode:nil -*- 

/**
 * @file Point.h
 * Declaration of a usual 3D point.
 */
#ifndef _POINT_
#define _POINT_

#include <stdlib.h>
#include <iostream>
#include <math.h>   

using namespace std;
/// The class Point represents a basic 3D point class.

class Point 
{
  /// Makes the face sendable to an output stream (console output).
  friend ostream& operator<< (ostream& os, const Point& point);
	
 public:
  
  /**
   * Empty constructor
   */
  Point() : _x(0), _y(0), _z(0) 
  {
    _px = _py = -1;
    _r = _g = 0; _b = 1;
  };
  
  /**
   * Coordinate constructor.
   * @param a Initial x-coordinate of the point.
   * @param b Initial y-coordinate of the point.
   * @param c Initial z-coordinate of the point.
   */
  Point(const double a, const double b, const double c) : _x(a), _y(b), _z(c) 
  {
    _px = _py = -1;
    _r = _g = 0; _b = 1;
    _v = true;
  };
  
  /**
   * Destructor
   */
  ~Point() {};
  
	/**
	 * Return the Point as a VVector
	 */
	//operator VVector();

  /**
   * Equality operator
   * @param other The other point to test if this is equal.
   * @return TRUE if Points are equal, FALSE otherwise.
   */
	bool operator== (const Point& other) const;

  /**
   * Smaller operator
   * @param other The other point to test if this is smaller.
   * @return TRUE if this is smaller than other, FALSE otherwise.
   */
	bool operator<  (const Point& other) const;
	
  /**
   * Value addition operator
   * @param val The value gets added to all elements of this point.
   * @return The addition of this point and a point (val,val,val).
   */
	Point operator+ (double val) const;

  /**
   * Point addition operator
   * @param other The other point to add.
   * @return The addition of this point and the other point.
   */
	Point operator+ (const Point& other) const;

  /**
   * Point addition operator
   * @param other The other point to add.
   */
  void operator+= (const Point& other);

  /**
   * Value substraction operator
   * @param val The value gets substracted from all elements of this point.
   * @return The substraction of this point and a point (val,val,val).
   */
	Point operator- (double val) const;

  /**
   * Point substraction operator
   * @param other The other point to subtract.
   * @return The subtraction of this point and the other point.
   */
	Point operator- (const Point& other) const;

  /**
   * Point substraction operator
   * @param other The other point to subtract.
   */
  void operator-= (const Point& other);

  /**
   * Value division operator
   * @param val Each elements of this point get divided by this value.
   * @return The division of this point and a the value val.
   */
	Point operator/ (double val) const;

  /**
   * Value multiplication operator
   * @param val Each elements of this point get multiplied by this value.
   * @return The multiplication of this point and a the value val.
   */
	Point operator* (double val) const;

  /**
   * Value multiplication operator
   * @param val Each elements of this point get multiplied by this value.
   */
  void operator*= (double val);

  /**
   * Negation operator
   * @return The negation of this point.
   */
  Point operator-() const { return (*this)*(-1);};

  /**
	 * Operator that returns a coordinate.
   * @param ind coordinate to return.
   * @return i. coordinate of this point.
	 */
  double& operator[]( int ind )
  {
    switch(ind)
    {
    case 0: return _x; break;
    case 1: return _y; break;
    case 2: return _z; break;
    default: {cout << "Out of range error in Point[]." << endl; exit(0);}
    }
  }

	/**
	 * Access function that returns the x-coordinate.
   * @return x-coordinate of this point.
	 */
	double x() const {return _x;}

  /**
	 * Access function that returns the y-coordinate.
   * @return y-coordinate of this point.
	 */
	double y() const {return _y;}

  /**
	 * Access function that returns the z-coordinate.
   * @return z-coordinate of this point.
	 */
	double z() const {return _z;}

  /**
	 * Access function that returns the visibility.
   * @return TRUE if this point is visible.
	 */
	bool v() const {return _v;}

  /**
	 * Access function that sets the x-coordinate.
   * @param x New x-coordinate of this point.
	 */
	void setX(double x) {_x = x;}

  /**
	 * Access function that sets the y-coordinate.
   * @param y New y-coordinate of this point.
	 */
	void setY(double y) {_y = y;}

  /**
	 * Access function that sets the z-coordinate.
   * @param z New z-coordinate of this point.
	 */
	void setZ(double z) {_z = z;}

  /**
	 * Access function that sets the visibility.
   * @param v TRUE to set this point visible.
	 */
	void setV(bool v) {_v = v;}

  /**
	 * Access function that sets all three coordinates.
   * @param x New x-coordinate of this point.
   * @param y New y-coordinate of this point.
   * @param z New z-coordinate of this point.
	 */
	void set(const double x, const double y, const double z) {_x = x; _y = y; _z = z;}

  /**
	 * Access function that sets all three coordinates to one value.
   * @param v New x/y/z-coordinate of this point.
	 */
  void setAll(double v) {_x = v; _y = v; _z = v;}

  /**
   * Set all the coordinates to a given value (default 0).
   * @param val The new value of all coordinates.
   */
	void clear(double val=0) {set(val,val,val);};

  /**
   * Return the length of this Point (seen as a vector).
   * @return The length of the vector of this point (= the distance to (0,0,0))
   */
	double length() const;
   
  /**
   * Normalizes this point seen as a vector by dividing by its length.
   */
	void normalize() {(*this)=(*this)/length();};

  /**
   * Return the dot product with another point.
   * @param other The other point to compute the dot product with.
   * @return The dot product (double) of this and the other point.
   */
	double dotProduct(const Point& other) const;

  /**
   * Return the dot product with another point (* operator).
   * @param other The other point to compute the dot product with.
   * @return The dot product (double) of this and the other point.
   */
	double operator* (const Point &other) const {return this->dotProduct(other);};

  /**
   * Return the cross product with another point.
   * @param other The other point to compute the cross product with.
   * @return The cross product (Point) of this and the other point.
   */
	Point crossProduct(const Point& other) const;
  
  /**
   * Assign image pixel values to a 3D point. By this assignment,
   * information from the image can be tracked for each 3D point.
   * @param x The x value of this point in the 2D image.
   * @param y The y value of this point in the 2D image.
   */
  void setPixel(int x, int y) {_px = x; _py = y;};

  /**
   * Assign image pixel colors to a 3D point. By this assignment,
   * color information from the image can be tracked for each 3D point.
   * @param ir The R value of this point in the 2D image.
   * @param ig The G value of this point in the 2D image.
   * @param ib The B value of this point in the 2D image.
   */
  void setColor(int ir, int ig, int ib) {_r = ir; _g = ig; _b = ib;};

  /**
   * Get the x coordinate of this 3D point in the image.
   * @return x-value of P in the 2D image.
   */
  int getPx() const {return _px;};

  /**
   * Get the y coordinate of this 3D point in the image.
   * @return y-value of P in the 2D image.
   */
  int getPy() const {return _py;};

  /**
   * Get the R color value of this 3D point in the image.
   * @return r-value of P in the 2D image.
   */
  int getCr() const {return _r;};
  
  /**
   * Get the G color value of this 3D point in the image.
   * @return g-value of P in the 2D image.
   */
  int getCg() const {return _g;};

  /**
   * Get the B color value of this 3D point in the image.
   * @return b-value of P in the 2D image.
   */
  int getCb() const {return _b;};
  
  /**
   * Rotate the point around the z-axis.
   * @param r Angle of rotation (in rad).
   * @return Rotated point
   */
  Point rotZ (double r)
  {
    double c = cos(r);
    double s = sin(r);    
    Point tmp(this->x() * c - this->y() * s,
              this->x() * s + this->y() * c,
              this->z());
    return tmp;
  };

  /**
   * Rotate the point around another z-axis.
   * @param p Vector to rotate around.
   * @param r Angle of rotation (in rad).
   * @return Rotated point
   */
  Point rotN (Point p, double r)
  {
    double c = cos(r);
    double s = sin(r);    
    double t = 1-c;

    double M[3][3];
    M[0][0] = (t * p.x() * p.x()) + c;
    M[0][1] = (t * p.x() * p.y()) - (s * p.z());
    M[0][2] = (t * p.x() * p.z()) + (s * p.y());
    M[1][0] = (t * p.x() * p.y()) + (s * p.z());
    M[1][1] = (t * p.y() * p.y()) + c;
    M[1][2] = (t * p.y() * p.z()) - (s * p.x());
    M[2][0] = (t * p.x() * p.z()) - (s * p.y());
    M[2][1] = (t * p.y() * p.z()) + (s * p.x());
    M[2][2] = (t * p.z() * p.z()) + c;
   
    Point tmp(M[0][0]*this->x() + M[0][1]*this->y() + M[0][2]*this->z(),
              M[1][0]*this->x() + M[1][1]*this->y() + M[1][2]*this->z(),
              M[2][0]*this->x() + M[2][1]*this->y() + M[2][2]*this->z());
    return tmp;
  };

  /// Print xyz point information.
  void print() const
  {
    std::cout << "[" << _x << "," << _y << "," << _z << "]" << std::endl;
  };

 private:
	
  /// The x-coordinate value;
	double _x;
  /// The y-coordinate value;
  double _y;
  /// The z-coordinate value;
  double _z;
  /// x value in 2D image
  int _px;
  /// y value in 2D image
  int _py;
  /// r color value in 2D image
  int _r;
  /// g color value in 2D image
  int _g;
  /// b color value in 2D image
  int _b;
  /// v value for visibility
  bool _v;
};

/// We handle a vector simple the same way as a point.
typedef Point Vector;

#endif //_POINT_
