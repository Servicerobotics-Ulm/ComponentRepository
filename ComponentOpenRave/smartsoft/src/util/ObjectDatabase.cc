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

#include "ObjectDatabase.hh"
#include "openRaveObjectsDB.hh"
#include <iostream>

//////////////////////////////////////////////////
//
//	Constructors
//
//////////////////////////////////////////////////

ObjectDatabase::ObjectDatabase() {
	this->position = -1;
}

void ObjectDatabase::init(const std::string& filename) {
	this->position = -1;
	this->readObjectDatabase(filename);
}

ObjectDatabase::~ObjectDatabase() {
}

//////////////////////////////////////////////////
//
//	public methods
//
//////////////////////////////////////////////////

bool ObjectDatabase::isKnownObject(const std::string& type) {
	for (size_t i = 0; i < this->objects.size(); ++i) {
		if (this->objects[i].type == type) {
			this->position = i;
			return true;
		}
	}
	return false;
}

ObjectDatabase::ObjectShape ObjectDatabase::getObjectShape(const std::string& type) {
	return this->objects[this->position].shape;
}

void ObjectDatabase::getObjectBox(double& x, double&y, double& z) {
	x = this->objects[this->position].x;
	y = this->objects[this->position].y;
	z = this->objects[this->position].z;
}

void ObjectDatabase::getObjectCylinder(double& r, double& h) {
	r = this->objects[this->position].r;
	h = this->objects[this->position].h;
}

void ObjectDatabase::getObjectMeshFilename(std::string& filename) {
	filename = this->objects[this->position].meshFilename;
}

void ObjectDatabase::getObjectSphere(double& r) {
	r = this->objects[this->position].r;
}

//////////////////////////////////////////////////
//
//	private methods
//
//////////////////////////////////////////////////

void ObjectDatabase::readObjectDatabase(const std::string& filename) {
	  std::cout << "Reading Database of known Objects..." << std::endl;
	  try
	  {
		  std::auto_ptr<objectRecDb_t> objRecDbTmp(objectRecDb(filename));
		  objectRecDb_t::object_sequence objectSeq = objRecDbTmp->object();

		  int n = 0;
		  for (objectRecDb_t::object_iterator i(objectSeq.begin()); i != objectSeq.end(); ++i) {
				++n;
				object_t::shape_type shape = i->shape();
				if(shape.mesh())
				{
					ObjectContainer meshObj;
					meshObj.type = i->type();
					meshObj.shape = ObjectDatabase::MESH;
					meshObj.meshFilename = shape.mesh()->filename();
					this->objects.push_back(meshObj);
				}else if (shape.cylinder())
				{
					ObjectContainer cylObj;
					cylObj.type = i->type();
					cylObj.shape = ObjectDatabase::CYLINDER;
					cylObj.r = shape.cylinder()->radius();
					cylObj.h = shape.cylinder()->height();
					this->objects.push_back(cylObj);
				}else if (shape.box())
				{
					ObjectContainer boxObj;
					boxObj.type = i->type();
					boxObj.shape = ObjectDatabase::BOX;
					boxObj.x = shape.box()->sizeX();
					boxObj.y = shape.box()->sizeY();
					boxObj.z = shape.box()->sizeZ();
					this->objects.push_back(boxObj);
				}else if (shape.sphere())
				{
					ObjectContainer sphereObj;
					sphereObj.type = i->type();
					sphereObj.shape = ObjectDatabase::SPHERE;
					sphereObj.r = shape.sphere()->radius();
					this->objects.push_back(sphereObj);
				}
		  }
	  std::cout << "finished reading object database, found " << n << " objects " << std::endl;
	  }catch(...)
	  {
		  std::cerr << "----------------------------------" << std::endl;
		  std::cerr << "Was unable to read Object database" << std::endl;
		  std::cerr << "Specified filename of the XML file was " << filename.c_str() << std::endl;
		  std::cerr << "if the file exists, please make sure that it is not malformed" << std::endl;
		  std::cerr << "----------------------------------" << std::endl;
	  }
}
