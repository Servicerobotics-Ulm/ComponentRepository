//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs.
// If you want the toolchain to re-generate this file, please
// delete it before running the code generator.
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//
//  Copyright (C) 2020 Nayabrasul Shaik
//
//        christian.schlegel@thu.de
//        nayabrasul.shaik@thu.de
//
//        ZAFH Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2.1
//  of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this library; if not, write to the Free Software Foundation, Inc.,
//  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//  This work is based on previous work by the folks from PlayerStage.
//
//--------------------------------------------------------------------------

#ifndef SMARTSOFT_SRC_JSONPARSER_HH_
#define SMARTSOFT_SRC_JSONPARSER_HH_

#include <fstream>
#include <vector>
#include <CommTrackingObjects/CommDetectedMarker.hh>
#include <CommTrackingObjects/CommDetectedMarkerList.hh>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
class JsonParser {
public:
	JsonParser(std::string json_file_name);
	virtual ~JsonParser();
	std::vector<CommTrackingObjects::CommDetectedMarker> get_markers_info();
private:
	std::ifstream input_stream;
	json json_string;
	std::vector<CommTrackingObjects::CommDetectedMarker> marker_list;
	bool is_parsed;
};

#endif /* SMARTSOFT_SRC_JSONPARSER_HH_ */
