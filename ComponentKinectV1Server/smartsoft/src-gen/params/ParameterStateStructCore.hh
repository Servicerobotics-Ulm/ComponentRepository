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
			unsigned char depth_mode;
			bool high_resolution;
			bool pushnewest_color_image;
			bool pushnewest_depth_image;
			bool pushnewest_rgbd_image;
			unsigned char rgb_mode;
			bool undistort_image;
			double valid_image_time;
		
		public:
			// default constructor
			settingsType() {
				debug_info = true;
				depth_mode = 1;
				high_resolution = false;
				pushnewest_color_image = false;
				pushnewest_depth_image = false;
				pushnewest_rgbd_image = true;
				rgb_mode = 1;
				undistort_image = true;
				valid_image_time = 1.0;
			}
		
			/**
			 * here are the public getters
			 */
			inline bool getDebug_info() const { return debug_info; }
			inline unsigned char getDepth_mode() const { return depth_mode; }
			inline bool getHigh_resolution() const { return high_resolution; }
			inline bool getPushnewest_color_image() const { return pushnewest_color_image; }
			inline bool getPushnewest_depth_image() const { return pushnewest_depth_image; }
			inline bool getPushnewest_rgbd_image() const { return pushnewest_rgbd_image; }
			inline unsigned char getRgb_mode() const { return rgb_mode; }
			inline bool getUndistort_image() const { return undistort_image; }
			inline double getValid_image_time() const { return valid_image_time; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "settings(";
				os << "debug_info = " << debug_info << ", ";
				os << "depth_mode = " << depth_mode << ", ";
				os << "high_resolution = " << high_resolution << ", ";
				os << "pushnewest_color_image = " << pushnewest_color_image << ", ";
				os << "pushnewest_depth_image = " << pushnewest_depth_image << ", ";
				os << "pushnewest_rgbd_image = " << pushnewest_rgbd_image << ", ";
				os << "rgb_mode = " << rgb_mode << ", ";
				os << "undistort_image = " << undistort_image << ", ";
				os << "valid_image_time = " << valid_image_time << ", ";
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
				azimuth = -1.57079632;
				elevation = 0.174532925;
				roll = -1.57079632;
				x = 21;
				y = 0;
				z = 80;
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
				os << "azimuth = " << azimuth << ", ";
				os << "elevation = " << elevation << ", ";
				os << "roll = " << roll << ", ";
				os << "x = " << x << ", ";
				os << "y = " << y << ", ";
				os << "z = " << z << ", ";
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
			double steer_a;
			int x;
			int y;
			int z;
		
		public:
			// default constructor
			baseType() {
				base_a = 0;
				on_base = false;
				on_ptu = false;
				steer_a = 0;
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
			inline double getSteer_a() const { return steer_a; }
			inline int getX() const { return x; }
			inline int getY() const { return y; }
			inline int getZ() const { return z; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "base(";
				os << "base_a = " << base_a << ", ";
				os << "on_base = " << on_base << ", ";
				os << "on_ptu = " << on_ptu << ", ";
				os << "steer_a = " << steer_a << ", ";
				os << "x = " << x << ", ";
				os << "y = " << y << ", ";
				os << "z = " << z << ", ";
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
				max_distance = 8000;
				min_distance = 320;
			}
		
			/**
			 * here are the public getters
			 */
			inline double getMax_distance() const { return max_distance; }
			inline double getMin_distance() const { return min_distance; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "hardware_properties(";
				os << "max_distance = " << max_distance << ", ";
				os << "min_distance = " << min_distance << ", ";
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
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	}
};

#endif
