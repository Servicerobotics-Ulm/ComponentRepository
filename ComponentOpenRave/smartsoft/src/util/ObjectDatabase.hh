//--------------------------------------------------------------------------
//  Copyright (C) 2010 Jonas Brich
//
//        brich@mail.hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartOpenRave component".
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

#ifndef OBJECTDATABASE_HH_
#define OBJECTDATABASE_HH_

#include <string>
#include <vector>

class ObjectDatabase {
public:
	enum ObjectShape {
		CYLINDER, BOX, MESH, SPHERE
	};

public:
	ObjectDatabase();
	void init(const std::string& filename);

	virtual ~ObjectDatabase();

	/**
	 * This method should be called first. It searches the database for the given object.
	 *
	 * return: Found object if it is true otherwise false
	 */
	bool isKnownObject(const std::string& type);

	/**
	 * This method can be called after "isKnownObject" to get the ObjectShape.
	 */
	ObjectShape getObjectShape(const std::string& type);

	/**
	 * This methods can be called after "isKnownObject" to get the dimensions and/or location of the model.
	 */
	void getObjectBox(double& x, double&y, double& z);

	void getObjectCylinder(double& r, double& h);

	void getObjectMeshFilename(std::string& filename);

	void getObjectSphere(double& r);

private:
	struct ObjectContainer {
		std::string type;
		ObjectShape shape;
		double x;
		double y;
		double z;

		double r;
		double h;

		std::string meshFilename;
	};

	std::vector<ObjectContainer> objects;

	int position;

	void readObjectDatabase(const std::string& filename);
};

#endif /* OBJECTDATABASE_HH_ */
