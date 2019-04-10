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

#ifndef _EVENTINTERFACE_HH
#define _EVENTINTERFACE_HH

#include <string>
#include <list>

#include "aceSmartSoft.hh"

class EventInterface
{
private:
  std::list<std::string> internList;
  SmartACE::SmartMutex lock;
public:
  EventInterface();

  int append(const std::string& newString);
  // Add new entry at end of the current list.
  // The resulting list has complete lisp notation.
  // Returns 0 if everything is OK, 1 otherwise.

  int get(std::string& list);
  // Get current entries and delete current list.
  // As soon as the list is read it is reset to
  // the empty list to prevent the lisp process
  // to read entries more than once.
  // Returns 0 if everything is OK, 1 otherwise.
};

#endif

