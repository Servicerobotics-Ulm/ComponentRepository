/*--------------------------------------------------------------------------

 Copyright (C) 2017

 Created on: Oct 27, 2017
 Author    : Nayabrasul Shaik (shaik@hs-ulm.de)

 ZAFH Servicerobotik Ulm
 University of Applied Sciences
 Prittwitzstr. 10
 D-89075 Ulm
 Germany

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2.1
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this library; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

 --------------------------------------------------------------------------*/

#ifndef DEFAULTVISUALIZATION_H_
#define DEFAULTVISUALIZATION_H_

#include "AbstractVisualization.hh"

#define TEXT_COLOR_RESET   "\033[0m"
#define TEXT_COLOR_GREEN   "\033[32m"      /* Green */
#include <iostream>
#include <iomanip>

class DefaultVisualization: public AbstractVisualization {
private:

public:
	DefaultVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~DefaultVisualization();
};

#endif /* DEFAULTVISUALIZATION_H_ */
