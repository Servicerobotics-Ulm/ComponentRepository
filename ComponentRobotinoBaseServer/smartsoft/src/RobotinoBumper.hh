//--------------------------------------------------------------------------
//
//  Copyright (C) 2015 Matthias Lutz
//
//        lutz@hs-ulm.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
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

#ifndef ROBOTINOBUMPER_HH
#define ROBOTINOBUMPER_HH

#include "rec/robotino/api2/all.h"


class RobotinoBumper : public  rec::robotino::api2::Bumper
{
public:
		RobotinoBumper()
                : bumped( false )
        {
        }

        void bumperEvent( bool hasContact );

        bool bumped;
};




#endif
