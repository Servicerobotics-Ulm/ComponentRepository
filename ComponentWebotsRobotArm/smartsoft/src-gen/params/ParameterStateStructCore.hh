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
			std::string robotName;
		
		public:
			// default constructor
			webotsType() {
				robotName = "UR5e";
			}
		
			/**
			 * here are the public getters
			 */
			inline std::string getRobotName() const { return robotName; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "webots(";
				os << "robotName = " << robotName;
				os << ")\n";
			}
			
		}; // end class webotsType
		
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
			bool on_base;
		
		public:
			// default constructor
			baseType() {
				on_base = false;
			}
		
			/**
			 * here are the public getters
			 */
			inline bool getOn_base() const { return on_base; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "base(";
				os << "on_base = " << on_base;
				os << ")\n";
			}
			
		}; // end class baseType
		
		/**
		 * Definition of Parameter manipulator
		 */
		class manipulatorType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			double azimuth;
			double elevation;
			double roll;
			double tcp_velocity;
			bool verbose;
			int x;
			int y;
			int z;
		
		public:
			// default constructor
			manipulatorType() {
				azimuth = 0;
				elevation = 0;
				roll = 0;
				tcp_velocity = 1.0;
				verbose = true;
				x = 0;
				y = 0;
				z = 0;
			}
		
			/**
			 * here are the public getters
			 */
			inline double getAzimuth() const { return azimuth; }
			inline double getElevation() const { return elevation; }
			inline double getRoll() const { return roll; }
			inline double getTcp_velocity() const { return tcp_velocity; }
			inline bool getVerbose() const { return verbose; }
			inline int getX() const { return x; }
			inline int getY() const { return y; }
			inline int getZ() const { return z; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "manipulator(";
				os << "azimuth = " << azimuth; os << ", ";
				os << "elevation = " << elevation; os << ", ";
				os << "roll = " << roll; os << ", ";
				os << "tcp_velocity = " << tcp_velocity; os << ", ";
				os << "verbose = " << verbose; os << ", ";
				os << "x = " << x; os << ", ";
				os << "y = " << y; os << ", ";
				os << "z = " << z;
				os << ")\n";
			}
			
		}; // end class manipulatorType
		
	
		///////////////////////////////////////////
		// External params
		///////////////////////////////////////////
		
	
		///////////////////////////////////////////
		// Instance params
		///////////////////////////////////////////
		
		/**
		 * Definition of instantiated ParameterRepository CommManipulatorObjects
		 */
		class CommManipulatorObjectsType {
			friend class ParamUpdateHandler;
			public:
			/**
			 * Definition of instantiated ParameterSet ManipulatorParameter
			 */
			class ManipulatorParameterType {
				friend class ParamUpdateHandler;
				public:
				protected:
					/**
					 * internal members
					 */
					
				public:
					/**
					 * public getter methods
					 */
					
					void to_ostream(std::ostream &os = std::cout) const
					{
						os << "ManipulatorParameter(\n";
						os << ")";
					}
			}; // end of parameter-set class ManipulatorParameterType
			
			protected:
				/**
				 * internal members
				 */
				ManipulatorParameterType ManipulatorParameter;
			
			public:
				/**
				 * public getter methods
				 */
				inline ManipulatorParameterType getManipulatorParameter() const { return ManipulatorParameter; }
				
				void to_ostream(std::ostream &os = std::cout) const
				{
					os << "CommManipulatorObjects(\n";
					ManipulatorParameter.to_ostream(os);
					os << ")";
				}
		}; // end of parameter-repository wrapper class CommManipulatorObjectsType
	
protected:

	// Internal params
	baseType base;
	manipulatorType manipulator;
	webotsType webots;
	
	// External params
	
	// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	CommManipulatorObjectsType CommManipulatorObjects;
	

	void setContent(const ParameterStateStructCore &commit) {
		// External params
	
		this->CommManipulatorObjects = commit.getCommManipulatorObjects();
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
	manipulatorType getManipulator() const {
		return manipulator;
	}
	webotsType getWebots() const {
		return webots;
	}
	
	// external param getters
	
	// repo wrapper class getter(s)
	CommManipulatorObjectsType getCommManipulatorObjects() const {
		return CommManipulatorObjects;
	}
	
	// helper method to easily implement output stream in derived classes
	void to_ostream(std::ostream &os = std::cout) const
	{
		// Internal params
		base.to_ostream(os);
		manipulator.to_ostream(os);
		webots.to_ostream(os);
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
		CommManipulatorObjects.to_ostream(os);
	}
	
	std::string getAsJSONString() {
		nlohmann::json param;
	
		param["base"] = nlohmann::json {
			{"on_base" , getBase().getOn_base()}
		};
		param["manipulator"] = nlohmann::json {
			{"azimuth" , getManipulator().getAzimuth()},
			{"elevation" , getManipulator().getElevation()},
			{"roll" , getManipulator().getRoll()},
			{"tcp_velocity" , getManipulator().getTcp_velocity()},
			{"verbose" , getManipulator().getVerbose()},
			{"x" , getManipulator().getX()},
			{"y" , getManipulator().getY()},
			{"z" , getManipulator().getZ()}
		};
		param["webots"] = nlohmann::json {
			{"robotName" , getWebots().getRobotName()}
		};
	
		param["ManipulatorParameter"] = nlohmann::json {
		};
		
		return param.dump();
	}
};

#endif