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

#ifndef _PATHSEGMENT_HH
#define _PATHSEGMENT_HH

#include <DomainRobotFleetNavigation/CommNode.hh>

class PathSegment {

	private:
	DomainRobotFleetNavigation::CommNode node1;
		int refNode1;
		DomainRobotFleetNavigation::CommNode node2;
		int refNode2;

	public:
		void setNode1(DomainRobotFleetNavigation::CommNode node1) {
			this->node1 = node1;
		}
		void setRefNode1(int refNode1) {
			this->refNode1 = refNode1;
		}
		void setNode2(DomainRobotFleetNavigation::CommNode node2) {
			this->node2 = node2;
		}
		void setRefNode2(int refNode2) {
			this->refNode2 = refNode2;
		}

		DomainRobotFleetNavigation::CommNode getNode1() {
			return this->node1;
		}
		int getRefNode1() {
			return this->refNode1;
		}
		DomainRobotFleetNavigation::CommNode getNode2() {
			return this->node2;
		}
		int getRefNode2() {
			return this->refNode2;
		}
};

#endif
