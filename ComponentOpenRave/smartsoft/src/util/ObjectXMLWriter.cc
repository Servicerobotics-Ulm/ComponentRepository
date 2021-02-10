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

#include <sstream>

#include "ObjectXMLWriter.hh"
#include "ComponentOpenRave.hh"

const string ObjectXMLWriter::KINBODY_OPEN = "<KinBody ";
const string ObjectXMLWriter::KINBODY_OPEN_CLOSE = ">";
const string ObjectXMLWriter::KINBODY_CLOSE = "</KinBody>";

const string ObjectXMLWriter::BODY_OPEN = "<Body ";
const string ObjectXMLWriter::BODY_OPEN_CLOSE = ">";
const string ObjectXMLWriter::BODY_CLOSE = "</Body>";

const string ObjectXMLWriter::NAME = "name=";
const string ObjectXMLWriter::TYPE_DYNAMIC = "type=\"dynamic\"";

const string ObjectXMLWriter::GEOM_OPEN_BOX = "<Geom type=\"box\">";
const string ObjectXMLWriter::GEOM_OPEN_CYLINDER = "<Geom type=\"cylinder\">";
const string ObjectXMLWriter::GEOM_OPEN_TRIMESH = "<Geom type=\"trimesh\">";
const string ObjectXMLWriter::GEOM_OPEN_SPHERE = "<Geom type=\"sphere\">";
const string ObjectXMLWriter::GEOM_CLOSE = "</Geom>";

const string ObjectXMLWriter::RENDER_OPEN = "<Render>";
const string ObjectXMLWriter::RENDER_CLOSE = "</Render>";

const string ObjectXMLWriter::DATA_OPEN = "<data>";
const string ObjectXMLWriter::DATA_CLOSE = "</data>";

const string ObjectXMLWriter::EXTENTS_OPEN = "<Extents>";
const string ObjectXMLWriter::EXTENTS_CLOSE = "</Extents>";

const string ObjectXMLWriter::DIFFUSECOLOR_OPEN = "<DiffuseColor>";
const string ObjectXMLWriter::DIFFUSECOLOR_CLOSE = "</DiffuseColor>";

const string ObjectXMLWriter::TRANSLATION_OPEN = "<Translation>";
const string ObjectXMLWriter::TRANSLATION_CLOSE = "</Translation>";

const string ObjectXMLWriter::ROTATIONAXIS_OPEN = "<RotationAxis>";
const string ObjectXMLWriter::ROTATIONAXIS_CLOSE = "</RotationAxis>";

const string ObjectXMLWriter::RADIUS_OPEN = "<Radius>";
const string ObjectXMLWriter::RADIUS_CLOSE = "</Radius>";

const string ObjectXMLWriter::HEIGHT_OPEN = "<Height>";
const string ObjectXMLWriter::HEIGHT_CLOSE = "</Height>";

//////////////////////////////////////////////////
//
//	public methods
//
//////////////////////////////////////////////////

ObjectXMLWriter::ObjectXMLWriter() {
	this->lastFileName = "";
}

ObjectXMLWriter::~ObjectXMLWriter() {
}

void ObjectXMLWriter::writeKinBodyBox(const string& name, const double& x, const double& y, const double& z, string& output,
		bool writeToFile) {

	std::stringstream stream;

	stream << KINBODY_OPEN << NAME << "\"" << name << "\"" << KINBODY_OPEN_CLOSE << std::endl;
	stream << "\t" << BODY_OPEN << NAME << "\"" << name << "\" " << TYPE_DYNAMIC << BODY_OPEN_CLOSE << std::endl;
	stream << "\t\t" << GEOM_OPEN_BOX << std::endl;
	stream << "\t\t\t" << EXTENTS_OPEN << x / 2 << " " << y / 2 << " " << z / 2 << EXTENTS_CLOSE << std::endl;
	stream << "\t\t" << GEOM_CLOSE << std::endl;
	stream << "\t" << BODY_CLOSE << std::endl;
	stream << KINBODY_CLOSE << std::endl;

	output = stream.str();

	if (COMP->getGlobalState().getOpenRave().getSaveObjectsToFile() || writeToFile) {
		this->saveKinBodyToFile(name, stream.str());
	}
}

void ObjectXMLWriter::writeKinBodyCylinder(const string& name, const double& radius, const double& height, string& output,
		bool writeToFile) {

	std::stringstream stream;

	stream << KINBODY_OPEN << NAME << "\"" << name << "\"" << KINBODY_OPEN_CLOSE << std::endl;
	stream << "\t" << BODY_OPEN << NAME << "\"" << name << "\" " << TYPE_DYNAMIC << BODY_OPEN_CLOSE << std::endl;
	stream << "\t\t" << GEOM_OPEN_CYLINDER << std::endl;
	stream << "\t\t\t" << RADIUS_OPEN << radius << RADIUS_CLOSE << std::endl;
	stream << "\t\t\t" << HEIGHT_OPEN << height << HEIGHT_CLOSE << std::endl;
	stream << "\t\t" << GEOM_CLOSE << std::endl;
	stream << "\t" << BODY_CLOSE << std::endl;
	stream << KINBODY_CLOSE << std::endl;

	output = stream.str();

	if (COMP->getGlobalState().getOpenRave().getSaveObjectsToFile() || writeToFile) {
		this->saveKinBodyToFile(name, stream.str());
	}
}

void ObjectXMLWriter::writeKinBodyMesh(const string& name, const string& meshFilename, string& output, bool writeToFile) {

	std::stringstream stream;

	stream << KINBODY_OPEN << NAME << "\"" << name << "\"" << KINBODY_OPEN_CLOSE << std::endl;
	stream << "\t" << BODY_OPEN << NAME << "\"" << name << "\" " << TYPE_DYNAMIC << BODY_OPEN_CLOSE << std::endl;
	stream << "\t\t" << GEOM_OPEN_TRIMESH << std::endl;
	stream << "\t\t" << RENDER_OPEN << meshFilename << RENDER_CLOSE << std::endl;
	stream << "\t\t" << DATA_OPEN << meshFilename << DATA_CLOSE << std::endl;
	stream << "\t\t" << GEOM_CLOSE << std::endl;
	stream << "\t" << BODY_CLOSE << std::endl;
	stream << KINBODY_CLOSE << std::endl;

	output = stream.str();

	if (COMP->getGlobalState().getOpenRave().getSaveObjectsToFile() || writeToFile) {
		this->saveKinBodyToFile(name, stream.str());
	}
}

void ObjectXMLWriter::writeKinBodySphere(const string& name, const double& radius, string& output,
		bool writeToFile) {

	std::stringstream stream;

	stream << KINBODY_OPEN << NAME << "\"" << name << "\"" << KINBODY_OPEN_CLOSE << std::endl;
	stream << "\t" << BODY_OPEN << NAME << "\"" << name << "\" " << TYPE_DYNAMIC << BODY_OPEN_CLOSE << std::endl;
	stream << "\t\t" << GEOM_OPEN_SPHERE << std::endl;
	stream << "\t\t\t" << RADIUS_OPEN << radius << RADIUS_CLOSE << std::endl;
	stream << "\t\t" << GEOM_CLOSE << std::endl;
	stream << "\t" << BODY_CLOSE << std::endl;
	stream << KINBODY_CLOSE << std::endl;

	output = stream.str();

	if (COMP->getGlobalState().getOpenRave().getSaveObjectsToFile() || writeToFile) {
		this->saveKinBodyToFile(name, stream.str());
	}
}

std::string ObjectXMLWriter::getLastFileName() {
	return this->lastFileName;
}

//////////////////////////////////////////////////
//
//	private methods
//
//////////////////////////////////////////////////

void ObjectXMLWriter::saveKinBodyToFile(const string& name, const string& input) {
	this->lastFileName.clear();
	this->lastFileName.append("OpenRaveObject_");
	this->lastFileName.append(name);
	this->lastFileName.append(".kinbody.xml");
	this->filestream.open(this->lastFileName.c_str(), std::ios_base::out | std::ios_base::trunc);
	this->filestream << input;
	this->filestream.close();
	std::cout << "wrote OpenRAve-XML description of required kinbody to " << this->lastFileName.c_str() << endl;
}
