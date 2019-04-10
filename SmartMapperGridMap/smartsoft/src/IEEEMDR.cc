/*
 * IEEEMDR.cc
 *
 *  Created on: Mar 11, 2019
 *      Author: shaik
 */

#include "IEEEMDR.hh"

 const char* np_uncertainty::uncertainty = "uncertainty";
 const char* np_uncertainty::covariance_xx = "covariance_xx";
 const char* np_uncertainty::covariance_yy = "covariance_yy";
 const char* np_uncertainty::covariance_thetatheta = "covariance_thetatheta";
 const char* np_uncertainty::covariance_xy = "covariance_xy";
 const char* np_uncertainty::covariance_xtheta = "covariance_xtheta";
 const char* np_uncertainty::covariance_ytheta = "covariance_ytheta";


 const char* np_cell::cell = "cell";
 const char* np_cell::x = "x";
 const char* np_cell::y = "y";
 const char* np_cell::height = "height";
 const char* np_cell::width = "width";
 const char* np_cell::value = "value";

 const char* np_offset::offset = "offset";
 const char* np_offset::offset_x = "offset_x";
 const char* np_offset::offset_y = "offset_y";
 const char* np_offset::theta = "theta";

 const char* np_grid_map::id = "id";
 const char* np_grid_map::grid_map = "grid_map";
 const char* np_grid_map::maptype = "maptype";
 const char* np_grid_map::num_cells_x = "num_cells_x";
 const char* np_grid_map::num_cells_y = "num_cells_y";
 const char* np_grid_map::resolution = "resolution";

 const char* np_coordinate_system::coordinate_system = "coordinate_system";
 const char* np_cells::cells = "cells";

 const char* np_mdr_maps::mdr_maps = "mdr:maps";
 const char* np_mdr_maps::xmlns_mdr = "xmlns:mdr";
 const char* np_mdr_maps::xmlns_xsi = "xmlns:xsi";
 const char* np_mdr_maps::xsi_schemaLocation = "xsi:schemaLocation";


IEEEMDR::IEEEMDR() {
	// TODO Auto-generated constructor stub

}

IEEEMDR::~IEEEMDR() {
	// TODO Auto-generated destructor stub
}
void IEEEMDR::ltm_gridmap_to_ieeemdr(SmartLtmGridMap& ltm_grid, const std::string filename)
{

	pugi::xml_document doc;

	//xml declaration
	pugi::xml_node declaration_node = doc.append_child(pugi::node_declaration);
	declaration_node.append_attribute("version") = "1.0";
	declaration_node.append_attribute("encoding") = "UTF-8";

	//mdr maps node
	pugi::xml_node mdr_node = doc.append_child(np_mdr_maps::mdr_maps);
	mdr_node.append_attribute(np_mdr_maps::xmlns_mdr) = "http://www.example.org/mdr";
	mdr_node.append_attribute(np_mdr_maps::xmlns_xsi) = "http://www.w3.org/2001/XMLSchema-instance";
	mdr_node.append_attribute(np_mdr_maps::xsi_schemaLocation) = "http://www.example.org/mdr . MDR_schema.xsd";


	//grid_map node
	pugi::xml_node gm_node = mdr_node.append_child(np_grid_map::grid_map);
	gm_node.append_attribute(np_grid_map::id) = "GridMap";
	gm_node.append_attribute(np_grid_map::maptype) = "1";
	gm_node.append_attribute(np_grid_map::num_cells_x) = ltm_grid.getXSizeCells();
	gm_node.append_attribute(np_grid_map::num_cells_y) = ltm_grid.getYSizeCells();
	gm_node.append_attribute(np_grid_map::resolution) = ltm_grid.getCellSizeMM()/1000.0f; // conversion mm to meters

	// coordinate systems
	gm_node.append_child(np_coordinate_system::coordinate_system);

	// offset
	pugi::xml_node offset_node = gm_node.append_child(np_offset::offset);
	offset_node.append_attribute(np_offset::offset_x) = ltm_grid.getXOffsetCells();
	offset_node.append_attribute(np_offset::offset_y) = ltm_grid.getYOffsetCells();
	offset_node.append_attribute(np_offset::theta) = 0.0f;

	//uncertainty
	pugi::xml_node uncertainty_node = offset_node.append_child(np_uncertainty::uncertainty);
	uncertainty_node.append_attribute(np_uncertainty::covariance_xx) = 0.0f;
	uncertainty_node.append_attribute(np_uncertainty::covariance_yy) = 0.0f;
	uncertainty_node.append_attribute(np_uncertainty::covariance_thetatheta) = 0.0f;

	uncertainty_node.append_attribute(np_uncertainty::covariance_xy) = 0.0f;
	uncertainty_node.append_attribute(np_uncertainty::covariance_xtheta) = 0.0f;
	uncertainty_node.append_attribute(np_uncertainty::covariance_ytheta) = 0.0f;

	//palette_elements (optional)

	//cells
	pugi::xml_node cells_node = gm_node.append_child(np_cells::cells);

	// cell
    const std::vector<unsigned char>& cellList = ltm_grid.getCellRef();
    std::cout << " size = " << cellList.size() <<std::endl;
    size_t i =0;
    int map_width  = ltm_grid.getXSizeCells();
    int map_height = ltm_grid.getYSizeCells();
    for(std::vector<unsigned char>::const_iterator it = cellList.begin(); it!= cellList.end(); ++it)
    {
    	pugi::xml_node current_cell_node = cells_node.append_child(np_cell::cell);

    	// grid map used in smartsoft uses range from 0 to 127
    	// so converting the cell values back to the range 0 - 255
    	int v = 255 - 2 * ltm_grid.getCellElemAtPos(i);

    	current_cell_node.append_attribute(np_cell::x) = std::floor(i/map_width);
    	current_cell_node.append_attribute(np_cell::y) = i%map_width;
    	current_cell_node.append_attribute(np_cell::height) = 1;
    	current_cell_node.append_attribute(np_cell::width) = 1;
    	current_cell_node.append_attribute(np_cell::value) = v;
    	++i;
    }

	// save document to file
	std::cout << "Saving result: " << doc.save_file(filename.c_str()) << std::endl;

}

void IEEEMDR::ieeemdr_to_ltm_gridmap(const std::string filename, SmartLtmGridMap& ltm_grid)
{

	pugi::xml_document doc;

	pugi::xml_parse_result result = doc.load_file(filename.c_str());
	std::cout << " doc name = " <<doc.name() <<std::endl;

	if(!result)
	{
		 std::cout << "Parse error: " << result.description() << ", character pos= " << result.offset;
		return;
	}
	else
	{

		std::cout << "Parse result: " << result.description() <<std::endl;

		pugi::xml_node mdr_node = doc.child(np_mdr_maps::mdr_maps);
		pugi::xml_node gm_node = mdr_node.child(np_grid_map::grid_map);


		ltm_grid.setXSizeCells(gm_node.attribute(np_grid_map::num_cells_x).as_uint());
		ltm_grid.setYSizeCells(gm_node.attribute(np_grid_map::num_cells_y).as_uint());
		ltm_grid.setCellSizeMM(gm_node.attribute(np_grid_map::resolution).as_double()*1000); // in mm


		pugi::xml_node offset_node = gm_node.child(np_offset::offset);

		ltm_grid.setXOffsetCells(offset_node.attribute(np_offset::offset_x).as_int());
		ltm_grid.setYOffsetCells(offset_node.attribute(np_offset::offset_y).as_int());

		//cells
	    pugi::xml_node cells_node = gm_node.child(np_cells::cells);


	    int map_width  = ltm_grid.getXSizeCells();
	    int map_height = ltm_grid.getYSizeCells();

	    int i =0;
	    for(pugi::xml_node current_cell : cells_node.children(np_cell::cell))
	    {

			unsigned int x 		= current_cell.attribute(np_cell::x).as_uint();
	    	unsigned int y 		= current_cell.attribute(np_cell::y).as_uint();
	    	unsigned int value 	= current_cell.attribute(np_cell::value).as_uint();

	    	//std::cout << " x,y,value =" << x << ", "<< y << ", "<< value <<  std::endl;
	    	++i;
	    	ltm_grid.setCellElemAtPos(x+ y*map_width, value);
	    }

	}

}

