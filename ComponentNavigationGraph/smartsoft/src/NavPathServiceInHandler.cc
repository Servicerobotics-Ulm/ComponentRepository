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
#include "NavPathServiceInHandler.hh"

#include <iostream>

#include <DomainRobotFleetNavigation/CommNode.hh>
#include "PathSegment.hh"

#define CDL_OK 		0
#define CDL_INF         1
#define CDL_NO          2
#define CDL_NOK		1000
#define CDL_ACCURACY    1.0e-05

#define INTERSECTION_ID_START 100

NavPathServiceInHandler::NavPathServiceInHandler(Smart::InputSubject<DomainRobotFleetNavigation::CommNavPath> *subject, const int &prescaleFactor)
:	NavPathServiceInHandlerCore(subject, prescaleFactor)
{
	std::cout << "constructor NavPathServiceInHandler\n";
}
NavPathServiceInHandler::~NavPathServiceInHandler() 
{
	std::cout << "destructor NavPathServiceInHandler\n";
}

struct IntersectionInformation {
	int segmentIndex;
	double ix;
	double iy;
};

void NavPathServiceInHandler::on_NavPathServiceIn(const DomainRobotFleetNavigation::CommNavPath &input)
{
	// implement business logic here
	// (do not use blocking calls here, otherwise this might block the InputPort NavPathServiceIn)

	// Create a graph from the received navigation path(s)

	COMP->nodeMapInt.clear();
	COMP->nodeMap.clear();
	//COMP->nodeStatusMap.clear();
	COMP->g.clear();

	std::cout << "NavPath received!" << std::endl;

	std::cout << "SIZE: " << COMP->g.m_vertices.size() << std::endl;

	std::vector<DomainRobotFleetNavigation::CommPath> path = input.getPathCopy();

	std::vector<PathSegment> pathSegments;

	double a1, b1, c1, a2, b2, c2;

	int id = INTERSECTION_ID_START;

	double dist;

	for (int i = 0; i < path.size(); i++) {
		for (int j = 0; j < path[i].getNodeSize(); j++) {
			COMP->nodeIdPathIdAssociation.insert(std::make_pair(path[i].getNodeElemAtPos(j).getId(), path[i].getId()));
		}
		COMP->pathMapInt.insert(std::make_pair(path[i].getId(), path[i]));
	}

	for (int i = 0; i < path.size(); i++) {

		for (int j = 0; j < path[i].getNodeSize()-1; j++) {



			//COMP->nodeMapInt.insert(std::make_pair(id, path[i].getNodeElemAtPos(j)));
			COMP->nodeMapInt.insert(std::make_pair(path[i].getNodeElemAtPos(j).getId(), path[i].getNodeElemAtPos(j)));

//			cdl_ab_axbyc(path[i].getNodeElemAtPos(j).getX(), path[i].getNodeElemAtPos(j).getY(),
//					     path[i].getNodeElemAtPos(j+1).getX(), path[i].getNodeElemAtPos(j+1).getY(),
//						 &a1, &b1, &c1);

			CGAL::Segment_2<CGAL::Simple_cartesian<double>> seg1(CGAL::Simple_cartesian<double>::Point_2(path[i].getNodeElemAtPos(j).getX(),
					path[i].getNodeElemAtPos(j).getY()), CGAL::Simple_cartesian<double>::Point_2(path[i].getNodeElemAtPos(j+1).getX(), path[i].getNodeElemAtPos(j+1).getY()));

			bool intersection = false;

			std::vector<IntersectionInformation> intersectionInformationList;

//			std::cout << "Next edge" << std::endl;

			for (int k = 0; k < pathSegments.size(); k++) {
//				cdl_ab_axbyc(pathSegments[i].getNode1().getX(), pathSegments[i].getNode1().getY(),
//							 pathSegments[i].getNode2().getX(), pathSegments[i].getNode2().getY(),
//							 &a2, &b2, &c2);

//				std::cout << "pathSeg: " << pathSegments[k].getNode1().getId() << "/" << pathSegments[k].getNode2().getId()
//						<< pathSegments[k].getNode1() << "/" << pathSegments[k].getNode2() << std::endl;

				CGAL::Segment_2<CGAL::Simple_cartesian<double>> seg2(CGAL::Simple_cartesian<double>::Point_2(pathSegments[k].getNode1().getX(),
						pathSegments[k].getNode1().getY()), CGAL::Simple_cartesian<double>::Point_2(pathSegments[k].getNode2().getX(), pathSegments[k].getNode2().getY()));

//				std::cout << "L11: " << path[i].getNodeElemAtPos(j).getX() << "/" << path[i].getNodeElemAtPos(j).getY() << std::endl;
//				std::cout << "L12: " << path[i].getNodeElemAtPos(j+1).getX() << "/" << path[i].getNodeElemAtPos(j+1).getY() << std::endl;
//
//				std::cout << "L11: " << pathSegments[k].getNode1().getX() << "/" << pathSegments[k].getNode1().getY() << std::endl;
//				std::cout << "L12: " << pathSegments[k].getNode2().getX() << "/" << pathSegments[k].getNode2().getY() << std::endl;

//				std::cout << "--------------------------" << std::endl;

				double ix, iy;

				if (intersectionTest(seg1, seg2, ix, iy)) {

				//if (cdl_line_intersection(a1, b1, c1, a2, b2, c2, &ix, &iy) == CDL_OK) {
//											std::cout << "INTERSECTION AT" << ix << "/" << iy << std::endl;
//											std::cout << "--------------------------" << std::endl;


					bool isNewIntersection = true;

					if ((fabs(ix-path[i].getNodeElemAtPos(j).getX()) < 0.000001 && fabs(iy-path[i].getNodeElemAtPos(j).getY()) < 0.000001) ||
					    (fabs(ix-path[i].getNodeElemAtPos(j+1).getX()) < 0.000001 && fabs(iy-path[i].getNodeElemAtPos(j+1).getY()) < 0.000001) ||
						(fabs(ix-pathSegments[k].getNode1().getX()) < 0.000001 && fabs(iy-pathSegments[k].getNode1().getY()) < 0.000001) ||
						(fabs(ix-pathSegments[k].getNode2().getX()) < 0.000001 && fabs(iy-pathSegments[k].getNode2().getY()) < 0.000001)) {

						isNewIntersection = false;
					}

					//std::cout << "isNewIntersection: " << isNewIntersection << std::endl;

					if (isNewIntersection) {

						std::cout << "INTERSECTION AT" << ix << "/" << iy << std::endl;
						std::cout << "--------------------------" << std::endl;

						std::cout << "L11: " << path[i].getNodeElemAtPos(j).getX() << "/" << path[i].getNodeElemAtPos(j).getY() << std::endl;
						std::cout << "L12: " << path[i].getNodeElemAtPos(j+1).getX() << "/" << path[i].getNodeElemAtPos(j+1).getY() << std::endl;

						std::cout << "L11: " << pathSegments[k].getNode1().getX() << "/" << pathSegments[k].getNode1().getY() << std::endl;
						std::cout << "L12: " << pathSegments[k].getNode2().getX() << "/" << pathSegments[k].getNode2().getY() << std::endl;

						IntersectionInformation intersectionInformation;

						intersectionInformation.segmentIndex = k;
						intersectionInformation.ix = ix;
						intersectionInformation.iy = iy;

						intersectionInformationList.push_back(intersectionInformation);
					}
				}
			}

			if (intersectionInformationList.size() == 0) {
				dist = sld(path[i].getNodeElemAtPos(j), path[i].getNodeElemAtPos(j+1));
				//boost::add_edge(id, id+1, dist, COMP->g);
				boost::add_edge(path[i].getNodeElemAtPos(j).getId(), path[i].getNodeElemAtPos(j+1).getId(), dist, COMP->g);

				PathSegment ps;
				ps.setNode1(path[i].getNodeElemAtPos(j));
				ps.setRefNode1(path[i].getNodeElemAtPos(j).getId());
				ps.setNode2(path[i].getNodeElemAtPos(j+1));
				ps.setRefNode2(path[i].getNodeElemAtPos(j+1).getId());
				pathSegments.push_back(ps);
			}
			else if (intersectionInformationList.size() == 1) {
				// Create new node in the navigation graph
				DomainRobotFleetNavigation::CommNode newNode;
				newNode.setId(id);
				newNode.setX(intersectionInformationList.front().ix);
				newNode.setY(intersectionInformationList.front().iy);
				std::cout << "NEW NODE WITH ID: " << id << ":" << newNode << std::endl;
				COMP->nodeMapInt.insert(std::make_pair(id, newNode));




				std::cout << "from: " << path[i].getNodeElemAtPos(j).getId() << "to: " << id << std::endl;
				dist = sld(path[i].getNodeElemAtPos(j).getX(), path[i].getNodeElemAtPos(j).getY(), intersectionInformationList.front().ix, intersectionInformationList.front().iy);
				boost::add_edge(path[i].getNodeElemAtPos(j).getId(), id, dist, COMP->g);

				PathSegment ps1;
				ps1.setNode1(path[i].getNodeElemAtPos(j));
				ps1.setRefNode1(path[i].getNodeElemAtPos(j).getId());
				ps1.setNode2(newNode);
				ps1.setRefNode2(id);
				pathSegments.push_back(ps1);





				std::cout << "from: " << id << "to: " << path[i].getNodeElemAtPos(j+1).getId() << std::endl;
				dist = sld(intersectionInformationList.front().ix, intersectionInformationList.front().iy, path[i].getNodeElemAtPos(j+1).getX(), path[i].getNodeElemAtPos(j+1).getY());
				boost::add_edge(id, path[i].getNodeElemAtPos(j+1).getId(), dist, COMP->g);

				PathSegment ps2;
				ps2.setNode1(newNode);
				ps2.setRefNode1(id);
				ps2.setNode2(path[i].getNodeElemAtPos(j+1));
				ps2.setRefNode2(path[i].getNodeElemAtPos(j+1).getId());
				pathSegments.push_back(ps2);





				std::cout << "from: " << pathSegments[intersectionInformationList.front().segmentIndex].getRefNode1() << "to: " << id << std::endl;
				dist = sld(pathSegments[intersectionInformationList.front().segmentIndex].getNode1().getX(), pathSegments[intersectionInformationList.front().segmentIndex].getNode1().getY(), intersectionInformationList.front().ix, intersectionInformationList.front().iy);
				boost::add_edge(pathSegments[intersectionInformationList.front().segmentIndex].getRefNode1(), id, dist, COMP->g);

				PathSegment ps3;
				ps3.setNode1(pathSegments[intersectionInformationList.front().segmentIndex].getNode1());
				ps3.setRefNode1(pathSegments[intersectionInformationList.front().segmentIndex].getRefNode1());
				ps3.setNode2(newNode);
				ps3.setRefNode2(id);
				pathSegments.push_back(ps3);




				std::cout << "from: " << id << "to: " << pathSegments[intersectionInformationList.front().segmentIndex].getRefNode2() << std::endl;
				dist = sld(intersectionInformationList.front().ix, intersectionInformationList.front().iy, pathSegments[intersectionInformationList.front().segmentIndex].getNode2().getX(), pathSegments[intersectionInformationList.front().segmentIndex].getNode2().getY());
				boost::add_edge(id, pathSegments[intersectionInformationList.front().segmentIndex].getRefNode2(), dist, COMP->g);

//				std::cout << "from: " << id << "to: " << pathSegments[intersectionInformationList.front().segmentIndex].getRefNode2() << " add "
//						<< newNode << " and " << pathSegments[intersectionInformationList.front().segmentIndex].getNode2() << std::endl;

				PathSegment ps4;
				ps4.setNode1(newNode);
				ps4.setRefNode1(id);
				ps4.setNode2(pathSegments[intersectionInformationList.front().segmentIndex].getNode2());
				ps4.setRefNode2(pathSegments[intersectionInformationList.front().segmentIndex].getRefNode2());
				pathSegments.push_back(ps4);




				boost::remove_edge(pathSegments[intersectionInformationList.front().segmentIndex].getRefNode1(),pathSegments[intersectionInformationList.front().segmentIndex].getRefNode2(), COMP->g);
				std::cout << "remove from: " << pathSegments[intersectionInformationList.front().segmentIndex].getRefNode1() << "to: " << pathSegments[intersectionInformationList.front().segmentIndex].getRefNode2() << std::endl;
				pathSegments.erase(pathSegments.begin()+intersectionInformationList.front().segmentIndex);

				id++;
			}

			else {
				std::cout << "TODO: More than one intersection!" << std::endl;

				for (int k = 0; k < intersectionInformationList.size(); k++) {
					std::cout << intersectionInformationList[k].segmentIndex << std::endl;
				}
			//	std::swap(test[0], test[1]);
			//	std::swap(intersectionInformationList[0], intersectionInformationList[1]);
				// sort intersectionInformationList according to the distance to the first point of the current edge
				for (int k = 0; k < intersectionInformationList.size(); k++) {
					for (int l = k+1; l < intersectionInformationList.size(); l++) {
						if (sld(path[i].getNodeElemAtPos(j).getX(), path[i].getNodeElemAtPos(j).getY(), intersectionInformationList[k].ix, intersectionInformationList[k].iy) > sld(path[i].getNodeElemAtPos(j).getX(), path[i].getNodeElemAtPos(j).getY(), intersectionInformationList[l].ix, intersectionInformationList[l].iy)) {
							std::swap(intersectionInformationList[k], intersectionInformationList[l]);
						}
					}
				}

				std::cout << "after sort" << std::endl;
				for (int k = 0; k < intersectionInformationList.size(); k++) {
					std::cout << intersectionInformationList[k].segmentIndex << std::endl;
				}

				std::cout << "for all the other segments:" << std::endl;

				for (int k = 0, lid = id; k < intersectionInformationList.size(); k++, lid++) {
					DomainRobotFleetNavigation::CommNode newNode;
					newNode.setId(lid);
					newNode.setX(intersectionInformationList[k].ix);
					newNode.setY(intersectionInformationList[k].iy);
					std::cout << "NEW NODE WITH ID: " << lid << ":" << newNode << std::endl;
					COMP->nodeMapInt.insert(std::make_pair(lid, newNode));




					std::cout << "from: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1() << "to: " << lid << std::endl;
					dist = sld(pathSegments[intersectionInformationList[k].segmentIndex].getNode1().getX(), pathSegments[intersectionInformationList[k].segmentIndex].getNode1().getY(), intersectionInformationList[k].ix, intersectionInformationList[k].iy);
					boost::add_edge(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1(), lid, dist, COMP->g);

					PathSegment ps4;
					ps4.setNode1(pathSegments[intersectionInformationList[k].segmentIndex].getNode1());
					ps4.setRefNode1(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1());
					ps4.setNode2(newNode);
					ps4.setRefNode2(lid);
					pathSegments.push_back(ps4);

					std::cout << "from: " << lid << "to: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2() << std::endl;
					std::cout << "id is" << id << std::endl;
					dist = sld(intersectionInformationList[k].ix, intersectionInformationList[k].iy, pathSegments[intersectionInformationList[k].segmentIndex].getNode2().getX(), pathSegments[intersectionInformationList[k].segmentIndex].getNode2().getY());
					boost::add_edge(lid, pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2(), dist, COMP->g);

					PathSegment ps5;
					ps5.setNode1(newNode);
					ps5.setRefNode1(lid);
					ps5.setNode2(pathSegments[intersectionInformationList[k].segmentIndex].getNode2());
					ps5.setRefNode2(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2());
					pathSegments.push_back(ps5);


					boost::remove_edge(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1(),pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2(), COMP->g);
					std::cout << "remove from: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1() << "to: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2() << std::endl;
					pathSegments.erase(pathSegments.begin()+intersectionInformationList[k].segmentIndex);

				}

				std::cout << "for the current segment:" << std::endl;

				// from first of current node to first new node

				std::cout << "from: " << path[i].getNodeElemAtPos(j).getId() << "to: " << id << std::endl;

				dist = sld(path[i].getNodeElemAtPos(j).getX(), path[i].getNodeElemAtPos(j).getY(), intersectionInformationList.front().ix, intersectionInformationList.front().iy);
				boost::add_edge(path[i].getNodeElemAtPos(j).getId(), id, dist, COMP->g);

				DomainRobotFleetNavigation::CommNode newNodeFirst;
				newNodeFirst.setId(id);
				newNodeFirst.setX(intersectionInformationList.front().ix);
				newNodeFirst.setY(intersectionInformationList.front().iy);

				PathSegment ps1;
				ps1.setNode1(path[i].getNodeElemAtPos(j));
				ps1.setRefNode1(path[i].getNodeElemAtPos(j).getId());
				ps1.setNode2(newNodeFirst);
				ps1.setRefNode2(id);
				pathSegments.push_back(ps1);

				int oldid = id;

				// between all new nodes

				for (int k = 0; k < intersectionInformationList.size()-1; k++) {
					dist = sld(intersectionInformationList[k].ix, intersectionInformationList[k].iy, intersectionInformationList[k+1].ix, intersectionInformationList[k+1].iy);
					boost::add_edge(id, id+1, dist, COMP->g);

					std::cout << "from: " << id << "to: " << id+1 << std::endl;

					DomainRobotFleetNavigation::CommNode newNode1;
					newNode1.setId(id);
					newNode1.setX(intersectionInformationList[k].ix);
					newNode1.setY(intersectionInformationList[k].iy);

					DomainRobotFleetNavigation::CommNode newNode2;
					newNode2.setId(id+1);
					newNode2.setX(intersectionInformationList[k+1].ix);
					newNode2.setY(intersectionInformationList[k+1].iy);

					PathSegment ps2;
					ps2.setNode1(newNode1);
					ps2.setRefNode1(id);
					ps2.setNode2(newNode2);
					ps2.setRefNode2(id+1);
					pathSegments.push_back(ps2);

					id++;
				}

				// from last new to second current

				std::cout << "from: " << id << "to: " << path[i].getNodeElemAtPos(j+1).getId() << std::endl;

				dist = sld(intersectionInformationList.back().ix, intersectionInformationList.back().iy, path[i].getNodeElemAtPos(j+1).getX(), path[i].getNodeElemAtPos(j+1).getY());
				boost::add_edge(id, path[i].getNodeElemAtPos(j+1).getId(), dist, COMP->g);

				DomainRobotFleetNavigation::CommNode newNodeSecond;
				newNodeSecond.setId(id);
				newNodeSecond.setX(intersectionInformationList.back().ix);
				newNodeSecond.setY(intersectionInformationList.back().iy);

				PathSegment ps3;
				ps3.setNode1(newNodeSecond);
				ps3.setRefNode1(id);
				ps3.setNode2(path[i].getNodeElemAtPos(j+1));
				ps3.setRefNode2(path[i].getNodeElemAtPos(j+1).getId());
				pathSegments.push_back(ps3);





				// Now check all the other segements that need to be modified due to the intersections

//				for (int k = 0; k < intersectionInformationList.size(); k++) {
//					std::cout << "from: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1() << "to: " << oldid << std::endl;
//					dist = sld(pathSegments[intersectionInformationList[k].segmentIndex].getNode1().getX(), pathSegments[intersectionInformationList[k].segmentIndex].getNode1().getY(), intersectionInformationList[k].ix, intersectionInformationList[k].iy);
//					boost::add_edge(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1(), oldid, dist, COMP->g);
//
//					DomainRobotFleetNavigation::CommNode newNodeNew;
//					newNodeNew.setId(oldid);
//					newNodeNew.setX(intersectionInformationList[k].ix);
//					newNodeNew.setY(intersectionInformationList[k].iy);
//
//					PathSegment ps4;
//					ps4.setNode1(pathSegments[intersectionInformationList[k].segmentIndex].getNode1());
//					ps4.setRefNode1(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1());
//					ps4.setNode2(newNodeNew);
//					ps4.setRefNode2(oldid);
//					pathSegments.push_back(ps4);
//
//					std::cout << "from: " << oldid << "to: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2() << std::endl;
//					dist = sld(intersectionInformationList[k].ix, intersectionInformationList[k].iy, pathSegments[intersectionInformationList[k].segmentIndex].getNode2().getX(), pathSegments[intersectionInformationList[k].segmentIndex].getNode2().getY());
//					boost::add_edge(id, pathSegments[intersectionInformationList.front().segmentIndex].getRefNode2(), dist, COMP->g);
//
//
//					PathSegment ps5;
//					ps5.setNode1(newNodeNew);
//					ps5.setRefNode1(oldid);
//					ps5.setNode2(pathSegments[intersectionInformationList[k].segmentIndex].getNode2());
//					ps5.setRefNode2(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2());
//					pathSegments.push_back(ps5);
//
//
//					boost::remove_edge(pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1(),pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2(), COMP->g);
//					std::cout << "remove from: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode1() << "to: " << pathSegments[intersectionInformationList[k].segmentIndex].getRefNode2() << std::endl;
//					pathSegments.erase(pathSegments.begin()+intersectionInformationList[k].segmentIndex);
//
//					oldid++;
//				}

			}

		}
		//COMP->nodeMapInt.insert(std::make_pair(id, path[i].getNodeElemAtPos(path[i].getNodeSize()-1)));
		COMP->nodeMapInt.insert(std::make_pair(path[i].getNodeElemAtPos(path[i].getNodeSize()-1).getId(), path[i].getNodeElemAtPos(path[i].getNodeSize()-1)));
	}

	for (std::map<int, DomainRobotFleetNavigation::CommNode>::iterator it = COMP->nodeMapInt.begin(); it != COMP->nodeMapInt.end(); it++)
	{
		std::cout << "STATUS: " << COMP->nodeStatusMap[it->first] << std::endl;
		COMP->nodeStatusMap.insert(std::make_pair(it->first, COMP->NodeStatus::FREE));
	}

	std::cout << "NavPath graph created!" << std::endl;

	std::cout << "SIZE: " << COMP->g.m_vertices.size() << std::endl;

	for (int i = 0; i < COMP->g.m_vertices.size(); i++) {
		std::cout << "Vertex " << i << ": ";
		for (int j = 0; j < COMP->g.m_vertices[i].m_out_edges.size(); j++) {
			std::cout << COMP->g.m_vertices[i].m_out_edges[j].m_target;
			std::cout << " ";
		}
		std::cout << std::endl;
	}

	// Was ist mit Knoten im gleichen/in verschiedenen Pfaden die eigentlich die gleiche Position haben?
	// Was ist mit Schnittpunkten zwischen Pfadsegementen? (sowohl innerhalb eines Pfads als auch zwischen verschiedenen Pfaden)
}

double NavPathServiceInHandler::sld(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

double NavPathServiceInHandler::sld(DomainRobotFleetNavigation::CommNode from, DomainRobotFleetNavigation::CommNode to) {
	return sqrt(pow(from.getX()-to.getX(),2)+pow(from.getY()-to.getY(),2));
}

// -----------------------------------------------------------------
// Transformation Line AB => ax+by=c
// Gradient          : b/a
// y-axis segment    : c
// return value      : OK/NOK
// -----------------------------------------------------------------
int NavPathServiceInHandler::cdl_ab_axbyc(double pax,double pay,double pbx,double pby,
                 double *a,double *b,double *c)
{
  double      deltax,deltay;

  deltax = pbx - pax;
  deltay = pby - pay;
  if (fabs(deltax) < CDL_ACCURACY) {
    // parallel to y
    *a = 1.0;
    *b = 0.0;
    *c = pax;
  } else if (fabs(deltay) < CDL_ACCURACY) {
    // parallel to x
    *a = 0.0;
    *b = 1.0;
    *c = pay;
  } else if ((fabs(deltay) < CDL_ACCURACY) && (fabs(deltax) < CDL_ACCURACY)) {
    // distance A <-> B too small
    return CDL_NOK;
  } else {
    *a = -deltay/deltax;
    *b = 1.0;
    *c = *a * pax + *b * pay;
  }
  return CDL_OK;
}

// -----------------------------------------------------------------
// Calculates intersection of two lines if available
//
// input lines are assumed as ax+by+c=0
// input    a1,b1,c1         Parameter line 1
// input    a2,b2,c2         Parameter line 2
// output   x,y              line intersection point
// output   status  CDL_OK   found intersection
//                  CDL_INF  lines are equal, infinity intersections
//                  CDL_NO   lines are parallel, no intersection
// -----------------------------------------------------------------
int NavPathServiceInHandler::cdl_line_intersection(double a1,double b1,double c1,
                          double a2,double b2,double c2,
                          double *x,double *y)
{
  int status;

  if ((a1*b2-a2*b1) == 0.0) {
    // parallel lines
    status = CDL_NO;
  } else if ((a2==0.0) && (b2==0.0) && (c2==0.0)) {
    // equal lines
    status = CDL_INF;
  } else if ((a1==0.0) && (b1==0.0) && (c1==0.0)) {
    // equal lines
    status = CDL_INF;
  } else if ((a1/a2==b1/b2) && (a1/a2==c1/c2)) {
    // equal lines
    status = CDL_INF;
  } else {
    // lines not parallel
    *x = (b1*c2-b2*c1)/(a1*b2-a2*b1);
    *y = (c1*a2-c2*a1)/(a1*b2-a2*b1);
    status = CDL_OK;
  }
  return status;
}

bool NavPathServiceInHandler::intersectionTest(CGAL::Segment_2<CGAL::Simple_cartesian<double>> seg1, CGAL::Segment_2<CGAL::Simple_cartesian<double>> seg2, double &x, double &y) {
    CGAL::Object result;
    CGAL::Point_2<CGAL::Simple_cartesian<double>> ipoint;
    CGAL::Segment_2<CGAL::Simple_cartesian<double>> iseg;

    result = CGAL::intersection(seg1, seg2);
    if (CGAL::assign(ipoint, result)) {
        x = ipoint.x();
        y = ipoint.y();
        return 1;
    }

    return 0;
}