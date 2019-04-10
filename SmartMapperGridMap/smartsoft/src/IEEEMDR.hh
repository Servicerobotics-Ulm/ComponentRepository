/*
 * IEEEMDR.hh
 *
 *  Created on: Mar 11, 2019
 *      Author: shaik
 */

#ifndef IEEEMDR_HH_
#define IEEEMDR_HH_
#include "smartLtmGridMap.hh"
#include "pugixml1.9/pugixml.hh"
using namespace Smart;
class np_uncertainty{
public:
	static const char* uncertainty;
	static const char* covariance_xx;
	static const char* covariance_yy;
	static const char* covariance_thetatheta;
    static const char* covariance_xy;
    static const char* covariance_xtheta;
    static const char* covariance_ytheta;
};


class np_cells{
public:
	static const char* cells;
	};

class np_cell{
public:
	static const char* cell;
	static const char* x;
	static const char* y;

	static const char* height;
	static const char* width;
	static const char* value;
};

class np_offset{
public:
	static const char* offset;
	static const char* offset_x;
	static const char* offset_y;
	static const char* theta;
};

class np_coordinate_system{
public:
	static const char* coordinate_system;

};

class np_grid_map{
public:
	static const char* id;
	static const char* grid_map;
	static const char* maptype;
	static const char* num_cells_x;
	static const char* num_cells_y;
	static const char* resolution;
};

class np_mdr_maps{
public:
	static const char* mdr_maps;
	static const char* xmlns_mdr;
	static const char* xmlns_xsi;
	static const char* xsi_schemaLocation;
};

class IEEEMDR {
public:
	IEEEMDR();
	virtual ~IEEEMDR();
	void ltm_gridmap_to_ieeemdr(SmartLtmGridMap& ltm_grid, const std::string filename);
	void ieeemdr_to_ltm_gridmap(const std::string filename, SmartLtmGridMap& ltm_grid);
};

#endif /* IEEEMDR_HH_ */
