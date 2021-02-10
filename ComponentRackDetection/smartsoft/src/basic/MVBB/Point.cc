// -*- mode:C++; tab-width:2; c-basic-offset:2; indent-tabs-mode:nil -*- 

#include "Point.h"
#include <algorithm>
#include <cmath>

const double EPSILON = 0.0001;

ostream& operator<< (ostream& os, const Point& point) 
{
  os << "[" << point._x << ", " << point._y << ", " << point._z << "]";
  return os;
}

//Point::operator VVector() 
//{
//  return VVector(_x, _y, _z);
//}

bool Point::operator== (const Point& other) const {
	//
// 	return (_x == other._x && _y == other._y && _z == other._z);
// 	bool equal = abs(_x - other._x) < EPSILON && abs(_y - other._y) < EPSILON && abs(_z - other._z) < EPSILON;
// 	if (equal) {
// 		cout << "EQUAL!!!" << endl;
// 		int fff;
// 		cin >> fff;
// 	}
// 	return equal;
	return ( abs(_x - other._x) < EPSILON &&
			abs(_y - other._y) < EPSILON &&
			abs(_z - other._z) < EPSILON );
}

bool Point::operator< (const Point& other) const {
	//
	return ( (_x < other._x) ||
		   (_x == other._x && _y < other._y) ||
		   (_x == other._x && _y == other._y && _z < other._z)
		 );
}


// Point operator+ (const Point& op1, const Point& op2) {
// 	Point p(op1);
// 	p._x += op2._x;
// 	p._y += op2._y;
// 	p._z += op2._z;
// 	return p;
// }
// 
// Point operator- (const Point& op1, const Point& op2) {
// 	Point p(op1);
// 	p._x -= op2._x;
// 	p._y -= op2._y;
// 	p._z -= op2._z;
// 	return p;
// }
// 
// Point operator+ (const Point& op1, double val) {
// 	Point p(op1);
// 	p._x += val;
// 	p._y += val;
// 	p._z += val;
// 	return p;
// }
// 
// Point operator- (const Point& op1, double val) {
// 	Point p(op1);
// 	p._x -= val;
// 	p._y -= val;
// 	p._z -= val;
// 	return p;
// }

Point Point::operator+ (double val) const {
	//
	Point p(*this);
	p._x += val;
	p._y += val;
	p._z += val;
	return p;
}

Point Point::operator- (double val) const {
	//
	Point p(*this);
	p._x -= val;
	p._y -= val;
	p._z -= val;
	return p;
}

Point Point::operator+ (const Point& other) const {
	//
	Point p(*this);
	p._x += other._x;
	p._y += other._y;
	p._z += other._z;
	return p;
}

void Point::operator+= (const Point& other) 
{
	this->_x = this->_x + other._x;
	this->_y = this->_y + other._y;
	this->_z = this->_z + other._z;
}

Point Point::operator- (const Point& other) const {
	//
	Point p(*this);
	p._x -= other._x;
	p._y -= other._y;
	p._z -= other._z;
	return p;
}

void Point::operator-= (const Point& other) 
{
	this->_x = this->_x - other._x;
	this->_y = this->_y - other._y;
	this->_z = this->_z - other._z;
}

Point Point::operator* (double val) const {
	//
	Point p(*this);
	p._x *= val;
	p._y *= val;
	p._z *= val;
	return p;
}

void Point::operator*= (double val) 
{
	this->_x *= val;
	this->_y *= val;
	this->_z *= val;
}

Point Point::operator/ (double val) const {
	//
	Point p(*this);
	p._x /= val;
	p._y /= val;
	p._z /= val;
	return p;
}

double Point::dotProduct (const Point& other) const 
{
  return  (  _x * other._x 
	   + _y * other._y 
	   + _z * other._z);
}

double Point::length() const 
{
  return sqrt(dotProduct(*this));
}

Point Point::crossProduct(const Point &other) const
{
    Point  c;
    c.set ( _y * other._z - _z * other._y,
	    _z * other._x - _x * other._z,
	    _x * other._y - _y * other._x);
    return  c;
}
