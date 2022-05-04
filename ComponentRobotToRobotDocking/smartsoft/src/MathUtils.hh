//--------------------------------------------------------------------------
//
//  Copyright (C)  2018 Matthias Lutz
//
//              lutz@hs-ulm.de
//              schlegel@hs-ulm.de
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
//-------------------------------------------------------------------------

#ifndef SMARTSOFT_SRC_MATHUTILS_HH_
#define SMARTSOFT_SRC_MATHUTILS_HH_

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

inline double deg2rad(double degrees){return M_PI * degrees / 180.0f;}
inline double rad2deg(double radians){return 180.0f * radians / M_PI;}

/**
 * Linear approximation of function using first-order Taylor polynomial
 * Ref: https://en.wikipedia.org/wiki/Linear_approximation
 *
 * @param fxy contains collection of points(where y is the function value at x).
 * @param x for which function value has to be approximated
 * @return approximated function value y ~= f(x)
 * */
inline double linearapproximation(std::vector<Eigen::Vector2d> fxy, const double x)
{
	double y = 0.0;
	size_t input_size = fxy.size();
	for(size_t i =0; i< input_size; ++i)
	{
		Eigen::Vector2d curr_fxy = fxy[i];

		// x < x_0 then y = 0.0
		if((i == 0) && (x < curr_fxy[0]))
		{
			y = curr_fxy[1];
			return y;
		}else
		{
			if(x >= curr_fxy[0] && i < input_size-1)
			{
				Eigen::Vector2d  next_fxy = fxy[i+1];
				if( x < next_fxy[0])
				{
					Eigen::Vector2d dxy = next_fxy - curr_fxy;
					if(dxy[0] == 0.0){
						y = curr_fxy[1];
					}else
					{
					 //f1(x) = f(x0)       + (     x      - x0)f'(x0)
						y    = curr_fxy[0] + (curr_fxy[0] - x)* (dxy[1]/dxy[0]);
					}
				}

			}else{
				y = curr_fxy[1];
			}
		}

	}
	return y;

}

#endif /* SMARTSOFT_SRC_MATHUTILS_HH_ */
