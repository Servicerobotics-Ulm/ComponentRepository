//--------------------------------------------------------------------------
//
//  Copyright (C) 	1997-2000 Christian Schlegel
// 					2009/2010 Andreas Steck
//
//      steck@hs-ulm.de
//		schlegel@hs-ulm.de
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
//
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
//--------------------------------------------------------------------------

#include "eventInterface.hh"
#include "sstream"

EventInterface::EventInterface()
{
}

int EventInterface::append(const std::string& newString)
{
  int status;
  lock.acquire();
  this->internList.push_back(newString);
  lock.release();
  return status;
}

int EventInterface::get(std::string& liste)
{
	std::ostringstream stream;
	lock.acquire();
	for (std::list<std::string>::const_iterator it = this->internList.begin(); it != this->internList.end(); ++it) {
		stream << *it<< " ";
	}
	liste = stream.str();
	this->internList.clear();
	lock.release();

	return 0;
}

