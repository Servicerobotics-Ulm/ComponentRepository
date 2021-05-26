/*--------------------------------------------------------------------------

 Copyright (C) 2011 

 Created on: Aug 25, 2017
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

#include "SmartSoft2RealSenseSdkConverter.hh"
#include<map>
#include<memory>

SmartSoft2RealSenseSdkConverter::SmartSoft2RealSenseSdkConverter() {
	// TODO Auto-generated constructor stub

}

SmartSoft2RealSenseSdkConverter::~SmartSoft2RealSenseSdkConverter() {
	// TODO Auto-generated destructor stub
}
rs::core::image_interface* SmartSoft2RealSenseSdkConverter::CreateSdkImage(rs::core::stream_type streamType, uchar* framedata, rs::core::image_info &info){

	return rs::core::image_interface::create_instance_from_raw_data(
	           &info,
	           framedata,
	           streamType,
	           rs::core::image_interface::flag::any,
	           0,
	           0);


}

rs::core::correlated_sample_set SmartSoft2RealSenseSdkConverter::CreateSdkSampleSet(uchar* color_framedata, rs::core::image_info &color_info,uchar* depth_framedata, rs::core::image_info &depth_info){
	rs::core::correlated_sample_set sampleSet;
	  sampleSet[rs::core::stream_type::color] = CreateSdkImage(rs::core::stream_type::color, color_framedata, color_info);
	  sampleSet[rs::core::stream_type::depth] = CreateSdkImage(rs::core::stream_type::depth, depth_framedata, depth_info);
	  return sampleSet;

}

rs::core::video_module_interface::actual_module_config SmartSoft2RealSenseSdkConverter::CreateSdkModuleConfig(rs::core::intrinsics  &colorIntrinsics, rs::core::intrinsics  &depthIntrinsics,rs::core::extrinsics  &extrinsics){
	rs::core::video_module_interface::actual_module_config actualModuleConfig = {};
	  std::vector<rs::core::stream_type> possible_streams = {rs::core::stream_type::depth,
	                                                         rs::core::stream_type::color //person tracking uses only color & depth
	                                                        };
	  std::map<rs::core::stream_type, rs::core::intrinsics> intrinsics;
	  intrinsics[rs::core::stream_type::depth] = depthIntrinsics;
	  intrinsics[rs::core::stream_type::color] = colorIntrinsics;

	  //fill stream configurations
	  for (auto &stream : possible_streams)
	  {
	    rs::core::video_module_interface::actual_image_stream_config &actualStreamConfig = actualModuleConfig[stream];
	    actualStreamConfig.size.width = intrinsics[stream].width;
	    actualStreamConfig.size.height = intrinsics[stream].height;
	    actualStreamConfig.frame_rate = 30;
	    actualStreamConfig.intrinsics = intrinsics[stream];
	    actualStreamConfig.extrinsics = extrinsics;
	    actualStreamConfig.is_enabled = true;
	  }
	  actualModuleConfig.projection = rs::core::projection_interface::create_instance(&colorIntrinsics, &depthIntrinsics, &extrinsics);
	  return actualModuleConfig;

}

