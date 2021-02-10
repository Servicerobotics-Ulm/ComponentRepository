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
//---------------------------------------------------------------------

#ifndef _MANIPULATORFACTORY_HH
#define _MANIPULATORFACTORY_HH

#include <string>

#include "Manipulator.hh"

#define MANIPULATORFACTORY SpecificManipulator::ManipulatorFactory::instance()

namespace SpecificManipulator {

/**
 * This class creates the manipulator specific class out of a given string.
 *
 * If a new manipulator specific class is introduced, the createManipulatorClass method has to be changed.
 */
class ManipulatorFactory {

public:
	virtual ~ManipulatorFactory();

	/**
	 * Returns the singleton instance of the ManipulatorFactory
	 */
	static ManipulatorFactory* instance();

	/**
	 * This method creates the specific manipulator given by the string value.
	 * The specific manipulator class can only be accessed over the Interface.
	 */
	Manipulator* createManipulatorClass(const std::string& name);

private:
	ManipulatorFactory();

	// Singleton Instance
	static ManipulatorFactory *_instance;
};

}

#endif /* _MANIPULATORFACTORY_HH */
