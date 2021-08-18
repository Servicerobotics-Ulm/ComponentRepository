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
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#ifndef _PARAMETERSTATESTRUCTCORE_HH
#define _PARAMETERSTATESTRUCTCORE_HH

#include "aceSmartSoft.hh"

#include "nlohmann/json.hpp"

#include <list>
#include <iostream>

// forward declaration (in order to define validateCOMMIT(ParameterStateStruct) which is implemented in derived class)
class ParameterStateStruct;

class ParameterStateStructCore
{
	friend class ParamUpdateHandler;
public:
	
		///////////////////////////////////////////
		// Internal params
		///////////////////////////////////////////
		
		/**
		 * Definition of Parameter webots
		 */
		class webotsType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			std::string cameraName;
			std::string rangeFinderName;
			std::string robotName;
		
		public:
			// default constructor
			webotsType() {
				cameraName = "kinect_v2_color";
				rangeFinderName = "kinect_v2_range";
				robotName = "kinect_v2";
			}
		
			/**
			 * here are the public getters
			 */
			inline std::string getCameraName() const { return cameraName; }
			inline std::string getRangeFinderName() const { return rangeFinderName; }
			inline std::string getRobotName() const { return robotName; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "webots(";
				os << "cameraName = " << cameraName; os << ", ";
				os << "rangeFinderName = " << rangeFinderName; os << ", ";
				os << "robotName = " << robotName;
				os << ")\n";
			}
			
		}; // end class webotsType
		
		/**
		 * Definition of Parameter settings
		 */
		class settingsType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			bool debug_info;
			bool pushnewest_color_image;
			bool pushnewest_depth_image;
			bool pushnewest_rgbd_image;
		
		public:
			// default constructor
			settingsType() {
				debug_info = true;
				pushnewest_color_image = true;
				pushnewest_depth_image = false;
				pushnewest_rgbd_image = false;
			}
		
			/**
			 * here are the public getters
			 */
			inline bool getDebug_info() const { return debug_info; }
			inline bool getPushnewest_color_image() const { return pushnewest_color_image; }
			inline bool getPushnewest_depth_image() const { return pushnewest_depth_image; }
			inline bool getPushnewest_rgbd_image() const { return pushnewest_rgbd_image; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "settings(";
				os << "debug_info = " << debug_info; os << ", ";
				os << "pushnewest_color_image = " << pushnewest_color_image; os << ", ";
				os << "pushnewest_depth_image = " << pushnewest_depth_image; os << ", ";
				os << "pushnewest_rgbd_image = " << pushnewest_rgbd_image;
				os << ")\n";
			}
			
		}; // end class settingsType
		
		/**
		 * Definition of Parameter sensor_pose
		 */
		class sensor_poseType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			double azimuth;
			double elevation;
			double roll;
			double x;
			double y;
			double z;
		
		public:
			// default constructor
			sensor_poseType() {
				azimuth = 1.5707963;
				elevation = 3.14159265;
				roll = 1.32;
				x = 90;
				y = 0;
				z = 350;
			}
		
			/**
			 * here are the public getters
			 */
			inline double getAzimuth() const { return azimuth; }
			inline double getElevation() const { return elevation; }
			inline double getRoll() const { return roll; }
			inline double getX() const { return x; }
			inline double getY() const { return y; }
			inline double getZ() const { return z; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "sensor_pose(";
				os << "azimuth = " << azimuth; os << ", ";
				os << "elevation = " << elevation; os << ", ";
				os << "roll = " << roll; os << ", ";
				os << "x = " << x; os << ", ";
				os << "y = " << y; os << ", ";
				os << "z = " << z;
				os << ")\n";
			}
			
		}; // end class sensor_poseType
		
		/**
		 * Definition of Parameter base
		 */
		class baseType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			double base_a;
			bool on_base;
			bool on_ptu;
			bool on_ur;
			int x;
			int y;
			int z;
		
		public:
			// default constructor
			baseType() {
				base_a = 0;
				on_base = false;
				on_ptu = false;
				on_ur = false;
				x = 0;
				y = 0;
				z = 0;
			}
		
			/**
			 * here are the public getters
			 */
			inline double getBase_a() const { return base_a; }
			inline bool getOn_base() const { return on_base; }
			inline bool getOn_ptu() const { return on_ptu; }
			inline bool getOn_ur() const { return on_ur; }
			inline int getX() const { return x; }
			inline int getY() const { return y; }
			inline int getZ() const { return z; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "base(";
				os << "base_a = " << base_a; os << ", ";
				os << "on_base = " << on_base; os << ", ";
				os << "on_ptu = " << on_ptu; os << ", ";
				os << "on_ur = " << on_ur; os << ", ";
				os << "x = " << x; os << ", ";
				os << "y = " << y; os << ", ";
				os << "z = " << z;
				os << ")\n";
			}
			
		}; // end class baseType
		
		/**
		 * Definition of Parameter hardware_properties
		 */
		class hardware_propertiesType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			double max_distance;
			double min_distance;
		
		public:
			// default constructor
			hardware_propertiesType() {
				max_distance = 4500;
				min_distance = 500;
			}
		
			/**
			 * here are the public getters
			 */
			inline double getMax_distance() const { return max_distance; }
			inline double getMin_distance() const { return min_distance; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "hardware_properties(";
				os << "max_distance = " << max_distance; os << ", ";
				os << "min_distance = " << min_distance;
				os << ")\n";
			}
			
		}; // end class hardware_propertiesType
		
	
		///////////////////////////////////////////
		// External params
		///////////////////////////////////////////
		
	
		///////////////////////////////////////////
		// Instance params
		///////////////////////////////////////////
		
	
protected:

	// Internal params
	baseType base;
	hardware_propertiesType hardware_properties;
	sensor_poseType sensor_pose;
	settingsType settings;
	webotsType webots;
	
	// External params
	
	// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	

	void setContent(const ParameterStateStructCore &commit) {
		// External params
	
	}

	// special trigger method (user upcall) called before updating parameter global state
	virtual SmartACE::ParamResponseType handleCOMMIT(const ParameterStateStruct &commitState) = 0;
public:
	ParameterStateStructCore() {  }
	virtual ~ParameterStateStructCore() {  }
	
	// internal param getters
	baseType getBase() const {
		return base;
	}
	hardware_propertiesType getHardware_properties() const {
		return hardware_properties;
	}
	sensor_poseType getSensor_pose() const {
		return sensor_pose;
	}
	settingsType getSettings() const {
		return settings;
	}
	webotsType getWebots() const {
		return webots;
	}
	
	// external param getters
	
	// repo wrapper class getter(s)
	
	// helper method to easily implement output stream in derived classes
	void to_ostream(std::ostream &os = std::cout) const
	{
		// Internal params
		base.to_ostream(os);
		hardware_properties.to_ostream(os);
		sensor_pose.to_ostream(os);
		settings.to_ostream(os);
		webots.to_ostream(os);
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	}
	
	std::string getAsJSONString() {
		nlohmann::json param;
	
		param["base"] = nlohmann::json {
			{"base_a" , getBase().getBase_a()},
			{"on_base" , getBase().getOn_base()},
			{"on_ptu" , getBase().getOn_ptu()},
			{"on_ur" , getBase().getOn_ur()},
			{"x" , getBase().getX()},
			{"y" , getBase().getY()},
			{"z" , getBase().getZ()}
		};
		param["hardware_properties"] = nlohmann::json {
			{"max_distance" , getHardware_properties().getMax_distance()},
			{"min_distance" , getHardware_properties().getMin_distance()}
		};
		param["sensor_pose"] = nlohmann::json {
			{"azimuth" , getSensor_pose().getAzimuth()},
			{"elevation" , getSensor_pose().getElevation()},
			{"roll" , getSensor_pose().getRoll()},
			{"x" , getSensor_pose().getX()},
			{"y" , getSensor_pose().getY()},
			{"z" , getSensor_pose().getZ()}
		};
		param["settings"] = nlohmann::json {
			{"debug_info" , getSettings().getDebug_info()},
			{"pushnewest_color_image" , getSettings().getPushnewest_color_image()},
			{"pushnewest_depth_image" , getSettings().getPushnewest_depth_image()},
			{"pushnewest_rgbd_image" , getSettings().getPushnewest_rgbd_image()}
		};
		param["webots"] = nlohmann::json {
			{"cameraName" , getWebots().getCameraName()},
			{"rangeFinderName" , getWebots().getRangeFinderName()},
			{"robotName" , getWebots().getRobotName()}
		};
	
		
		return param.dump();
	}
};

#endif
