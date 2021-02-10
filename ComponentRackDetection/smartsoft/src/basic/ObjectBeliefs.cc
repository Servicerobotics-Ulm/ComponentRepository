//------------------------------------------------------------------------
//
//  Copyright (C) 2011 Manuel Wopfner
//
//        wopfner@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartColorToFObjectRecognition".
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
//--------------------------------------------------------------------------

#include "ObjectBeliefs.hh"
#include <iostream>
#include <vector>
#include <algorithm>

bool candidateCompBelief(ObjectBeliefs::ObjectBelief c1, ObjectBeliefs::ObjectBelief c2) {
	// This function is used for comparison purpose
	// in order to sort the vector of candidates
	return (c1.getBelief() > c2.getBelief());
}

ObjectBeliefs::ObjectBeliefs() {
}

ObjectBeliefs::~ObjectBeliefs() {
}

void ObjectBeliefs::clear() {
	beliefs.clear();
}

size_t ObjectBeliefs::size() const {
	return beliefs.size();
}

bool ObjectBeliefs::hasBelief(const std::string& objectName) const {
	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::const_iterator iter = this->beliefs.find(objectName);
	return (iter != beliefs.end());
}

double ObjectBeliefs::getBelief(const std::string& objectName) const {
	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::const_iterator iter = this->beliefs.find(objectName);

	if (iter != beliefs.end()) {
		return iter->second.getBelief();
	}

	return 0.0;
}

void ObjectBeliefs::setBelief(const std::string& objectName, double belief) {
	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::iterator iter = beliefs.find(objectName);

	if (iter != beliefs.end()) {
		iter->second.integrateBelief(belief);
		std::cout<<"ObjectBeliefs::integrateBelief SOULD NOT BE USED!!!! Line 86 --> exit(0) "<<std::endl;
		exit(0);
	} else {
		ObjectBelief& b = beliefs[objectName];
		b.setName(objectName);
		b.setBelief(belief);
	}
}

void ObjectBeliefs::setPosePDF(const std::string& objectName, double x, double y,double z, double yaw, double pitch, double roll, arma::mat cov){
	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::iterator iter = beliefs.find(objectName);

		if (iter != beliefs.end()) {
			std::cout<<"ERROR THIS SOULD NOT BE THE CASE"<<std::endl;
		} else {

			ObjectBelief& b = beliefs[objectName];

			CPose3D init_Mean(x,y,z,yaw,pitch,roll);
			CMatrixDouble66 init_Cov;

			for (size_t i = 0; i < 6; ++i)
			{
				for (size_t j = 0; j < 6; j++)
				{
					init_Cov(i, j) = cov(i,j);
				}
			}
			CPose3DPDFGaussian pose(init_Mean, init_Cov);

			b.setPosePDF(pose);

		}

	}

//void ObjectBeliefs::integrateBelief(const std::string& objectName, double belief) {
//	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::iterator iter = beliefs.find(objectName);
//
//	if (iter != beliefs.end()) {
//		//TODO
//		iter->second.integrateBelief(belief);
//		std::cout<<"ObjectBeliefs::integrateBelief SOULD NOT BE USED!!!! Line 86 --> exit(0) "<<std::endl;
//		exit(0);
//	} else {
//		ObjectBelief& b = beliefs[objectName];
//		b.setName(objectName);
//		b.integrateBelief(belief);
//	}
//}

const double ObjectBeliefs::getBestBelief() const {
	double belief = 0;

	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::const_iterator iter = beliefs.begin();
	for (; iter != beliefs.end(); iter++) {
		if (iter->second.getBelief() > belief) {
			belief = iter->second.getBelief();
		}
	}

	return belief;
}

const std::string ObjectBeliefs::getBestBeliefName() const {

	double belief = 0;
	std::string objName;

	std::map<std::string, ObjectBelief,std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::const_iterator iter = beliefs.begin();
	for (; iter != beliefs.end(); iter++) {
		if (iter->second.getBelief() > belief) {
			objName = iter->first;
			belief = iter->second.getBelief();
		}
	}

	return objName;
}

bool ObjectBeliefs::getBestBeliefPosePDF(CPose3DPDFGaussian & pose ) const{
		double belief = 0;
		bool has_Pose = false;

		std::map<std::string, ObjectBelief,std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::const_iterator iter = beliefs.begin();
		for (; iter != beliefs.end(); iter++) {
			if (iter->second.getBelief() > belief ) {
				if(iter->second.hasPose() == true){
					pose = iter->second.getPosePDF();
					has_Pose = true;
				}else {
					has_Pose = false;
				}
				belief = iter->second.getBelief();
			}
		}

		return has_Pose;
}

std::map<std::string, ObjectBeliefs::ObjectBelief,std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > > ObjectBeliefs::getAllBeliefs() const {
	return beliefs;
}

void ObjectBeliefs::setAllBeliefs(const std::map<std::string, ObjectBeliefs::ObjectBelief,std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >bel){
	this->beliefs = bel;
}

void ObjectBeliefs::print(std::ostream &os) const {

	std::vector<ObjectBelief, Eigen::aligned_allocator<ObjectBeliefs::ObjectBelief> > b;
	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > >::const_iterator iter = beliefs.begin();

	for (; iter != beliefs.end(); iter++) {
		b.push_back(iter->second);
	}

	std::sort(b.begin(), b.end(), candidateCompBelief);

	os << "[";
	for (size_t i = 0; i < b.size(); ++i) {
		os << b[i].getName() << "=" << b[i].getBelief() * 100 << "%, ";
	}
	os << "]";
}
