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

#ifndef OBJECTXMLWRITER_HH_
#define OBJECTXMLWRITER_HH_

#include <fstream>
#include <string>

using std::string;

class ObjectXMLWriter {
private:
	std::ofstream filestream;
	std::string lastFileName;

	/**
	 * Strings which represent the structure of the OpenRAVE xml format
	 */
	static const string KINBODY_OPEN;
	static const string KINBODY_OPEN_CLOSE;
	static const string KINBODY_CLOSE;

	static const string BODY_OPEN;
	static const string BODY_OPEN_CLOSE;
	static const string BODY_CLOSE;

	static const string NAME;
	static const string TYPE_DYNAMIC;

	static const string GEOM_OPEN_BOX;
	static const string GEOM_OPEN_CYLINDER;
	static const string GEOM_OPEN_SPHERE;
	static const string GEOM_OPEN_TRIMESH;
	static const string GEOM_CLOSE;

	static const string RENDER_OPEN;
	static const string RENDER_CLOSE;

	static const string DATA_OPEN;
	static const string DATA_CLOSE;

	static const string EXTENTS_OPEN;
	static const string EXTENTS_CLOSE;

	static const string DIFFUSECOLOR_OPEN;
	static const string DIFFUSECOLOR_CLOSE;

	static const string TRANSLATION_OPEN;
	static const string TRANSLATION_CLOSE;

	static const string ROTATIONAXIS_OPEN;
	static const string ROTATIONAXIS_CLOSE;

	static const string RADIUS_OPEN;
	static const string RADIUS_CLOSE;

	static const string HEIGHT_OPEN;
	static const string HEIGHT_CLOSE;

public:
	ObjectXMLWriter();
	virtual ~ObjectXMLWriter();

	/**
	 * Writes a cuboid which is represented by a KinBody Box into a string.
	 *
	 * name:		name of the object
	 * x:			this length will be multiplied by two
	 * y:			this length will be multiplied by two
	 * z:			this length will be multiplied by two
	 * output:		output string which includes the XML-data stream
	 * writeToFile:	true if the object should be written to a file
	 */
	void writeKinBodyBox(const string& name, const double& x, const double& y, const double& z, string& output, bool writeToFile =
			false);

	/**
	 * Writes a cylinder which is represented by a Kinbody Cylinder into a string.
	 *
	 * name:		name of the object
	 * radius:		radius of the object
	 * height:		height of the object
	 * output: 		output string which includes the XML-data stream
	 * writeToFile:	true if the object should be written to a file
	 */
	void writeKinBodyCylinder(const string& name, const double& radius, const double& height, string& output, bool writeToFile =
			false);

	/**
	 * Writes a mesh which is represented by a Kinbody mesh into a string.
	 *
	 * name: 			name of the object
	 * meshFilenname:	filename of the mesh where it is stored on the hard drive
	 * output:			output string which includes the XML-data stream
	 * writeToFile:	true if the object should be written to a file
	 */
	void writeKinBodyMesh(const string& name, const string& meshFilename, string& output, bool writeToFile = false);


	/**
	 * Writes a sphere which is represented by a KinBody sphere into a string.
	 *
	 * name:		name of the object
	 * r:			radius of the sphere
	 * output:		output string which includes the XML-data stream
	 * writeToFile:	true if the object should be written to a file
	 */
	void writeKinBodySphere(const string& name, const double& radius, string& output, bool writeToFile = false);

	/**
	 * Returns the last file name.
	 */
	std::string getLastFileName();

private:
	/**
	 * Saves a string which represents the XML-data into a file.
	 *
	 * name:	name of the object
	 * input:	input string which includes the XML-data stream
	 */
	void saveKinBodyToFile(const string& name, const string& input);
};

#endif /* OBJECTXMLWRITER_HH_ */
