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

#ifndef OBJECTBELIEFS_H_
#define OBJECTBELIEFS_H_

#include <map>
#include <string>
#include <ostream>
#include <iostream>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include "armadillo.hh"

#include <eigen3/Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;

class ObjectBeliefs {

public:
	class ObjectBelief {
	private:
		//EIGEN_MAKE_ALIGNED_OPERATOR_NEW // http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.htmls

		std::string name;
		double belief;
		CPose3DPDFGaussian posePDF;
		bool has_Pose;

	public:
		ObjectBelief() :
			name(""), belief(0), has_Pose(false) {
		}

		void setName(const std::string& name) {
			this->name = name;
		}

		const std::string& getName() const {
			return this->name;
		}

		void integrateBelief(double belief) {
			std::cout<<"ObjectBelief::integrateBelief SOULD NOT BE USED!!!! Line 67 --> exit(0) "<<std::endl;
			std::cout<<"BEFOR THIS->Belief: "<<this->belief<<std::endl;
			std::cout<<"intergarte bel: "<<belief<<std::endl;
			this->belief = (this->belief + belief) - (this->belief * belief);
			std::cout<<"PAST THIS->Belief: "<<this->belief<<std::endl;
			std::cout<<"Return to continue!"<<std::endl;
			exit(0);
		}

		double getBelief() const {
			return this->belief;
		}

		void setBelief(double belief) {
			this->belief = belief;
		}

		void setPosePDF(CPose3DPDFGaussian pose){
			 posePDF = pose;
			 has_Pose = true;
		}

		bool hasPose() const{
			return has_Pose;
		}

		CPose3DPDFGaussian getPosePDF() const {
			return this->posePDF;
		}
	};

private:

public:
	std::map<std::string, ObjectBeliefs::ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > > beliefs;
	ObjectBeliefs();
	virtual ~ObjectBeliefs();

	void clear();

	size_t size() const;

	bool hasBelief(const std::string& objectName) const;

	double getBelief(const std::string& objectName) const;

	void setBelief(const std::string& objectName, double belief);

	void setPosePDF(const std::string& objectName, double x, double y,double z, double yaw, double pitch, double roll, arma::mat cov);

//	void integrateBelief(const std::string& objectName, double belief);

	const double getBestBelief() const;

	const std::string getBestBeliefName() const;

	bool getBestBeliefPosePDF(CPose3DPDFGaussian & pose) const;

	std::map<std::string, ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > > getAllBeliefs() const;

	void setAllBeliefs(const std::map<std::string, ObjectBeliefs::ObjectBelief, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string,ObjectBeliefs::ObjectBelief> > > bel);

	void print(std::ostream &os = std::cout) const;



	ObjectBeliefs(const ObjectBeliefs& p) {
		beliefs = p.getAllBeliefs();

	    //std::cout << "beliefs in copy constr.: " << this << std::endl;
	    //print(std::cout);
	}

	ObjectBeliefs& operator=(const ObjectBeliefs& p) {
		beliefs = p.getAllBeliefs();

	    //std::cout << "beliefs in = operator.: " << this << std::endl;
		///print(std::cout);
	    return *this;
	}
};

inline std::ostream &operator<<(std::ostream &os, const ObjectBeliefs &b) {
	b.print(os);
	return os;
}

#endif /* OBJECTBELIEFS_H_ */
