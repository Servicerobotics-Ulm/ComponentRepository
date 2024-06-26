//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
// --------------------------------------------------------------------------
//
//  Copyright (C) 2021 Timo Blender
//
//
//        timo.blender@thu.de
//        christian.schlegel@thu.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
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
#ifndef _NAVPATHSERVICEINHANDLER_HH
#define _NAVPATHSERVICEINHANDLER_HH

#include "NavPathServiceInHandlerCore.hh"

#include "ComponentNavigationGraph.hh"

#include <CGAL/intersections.h>
#include <CGAL/Simple_cartesian.h>
	
class NavPathServiceInHandler  : public NavPathServiceInHandlerCore
{		
public:
	NavPathServiceInHandler(Smart::InputSubject<DomainRobotFleetNavigation::CommNavPath> *subject, const int &prescaleFactor=1);
	virtual ~NavPathServiceInHandler();
	
	virtual void on_NavPathServiceIn(const DomainRobotFleetNavigation::CommNavPath &input);

	double sld(DomainRobotFleetNavigation::CommNode from, DomainRobotFleetNavigation::CommNode to);

	double sld(double x1, double y1, double x2, double y2);

	int cdl_ab_axbyc(double pax,double pay,double pbx,double pby,
	                 double *a,double *b,double *c);

	int cdl_line_intersection(double a1,double b1,double c1,
	                          double a2,double b2,double c2,
	                          double *x,double *y);

	bool intersectionTest(CGAL::Segment_2<CGAL::Simple_cartesian<double>> seg1, CGAL::Segment_2<CGAL::Simple_cartesian<double>> seg2, double &x, double &y);
};

#endif
