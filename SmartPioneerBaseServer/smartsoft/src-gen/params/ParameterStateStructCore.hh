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
		 * Definition of Parameter Robot
		 */
		class RobotType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			bool enable_motors;
			bool enable_sonar;
			int maxRotVel;
			int maxRotVelAcc;
			int maxRotVelDecel;
			int maxVel;
			int maxVelAcc;
			int maxVelDecel;
			std::string robotType;
			std::string serialport;
		
		public:
			// default constructor
			RobotType() {
				enable_motors = true;
				enable_sonar = false;
				maxRotVel = 300;
				maxRotVelAcc = 100;
				maxRotVelDecel = 100;
				maxVel = 1000;
				maxVelAcc = 300;
				maxVelDecel = 300;
				robotType = "p3dx";
				serialport = "/dev/ttyS0";
			}
		
			/**
			 * here are the public getters
			 */
			inline bool getEnable_motors() const { return enable_motors; }
			inline bool getEnable_sonar() const { return enable_sonar; }
			inline int getMaxRotVel() const { return maxRotVel; }
			inline int getMaxRotVelAcc() const { return maxRotVelAcc; }
			inline int getMaxRotVelDecel() const { return maxRotVelDecel; }
			inline int getMaxVel() const { return maxVel; }
			inline int getMaxVelAcc() const { return maxVelAcc; }
			inline int getMaxVelDecel() const { return maxVelDecel; }
			inline std::string getRobotType() const { return robotType; }
			inline std::string getSerialport() const { return serialport; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "Robot(";
				os << "enable_motors = " << enable_motors << ", ";
				os << "enable_sonar = " << enable_sonar << ", ";
				os << "maxRotVel = " << maxRotVel << ", ";
				os << "maxRotVelAcc = " << maxRotVelAcc << ", ";
				os << "maxRotVelDecel = " << maxRotVelDecel << ", ";
				os << "maxVel = " << maxVel << ", ";
				os << "maxVelAcc = " << maxVelAcc << ", ";
				os << "maxVelDecel = " << maxVelDecel << ", ";
				os << "robotType = " << robotType << ", ";
				os << "serialport = " << serialport << ", ";
				os << ")\n";
			}
			
		}; // end class RobotType
		
	
		///////////////////////////////////////////
		// External params
		///////////////////////////////////////////
		
	
		///////////////////////////////////////////
		// Instance params
		///////////////////////////////////////////
		
		/**
		 * Definition of instantiated ParameterRepository CommBasicObjects
		 */
		class CommBasicObjectsType {
			friend class ParamUpdateHandler;
			public:
			/**
			 * Definition of instantiated ParameterSet BaseParams
			 */
			class BaseParamsType {
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
						os << "BaseParams(\n";
						os << ")";
					}
			}; // end of parameter-set class BaseParamsType
			
			protected:
				/**
				 * internal members
				 */
				BaseParamsType BaseParams;
			
			public:
				/**
				 * public getter methods
				 */
				inline BaseParamsType getBaseParams() const { return BaseParams; }
				
				void to_ostream(std::ostream &os = std::cout) const
				{
					os << "CommBasicObjects(\n";
					BaseParams.to_ostream(os);
					os << ")";
				}
		}; // end of parameter-repository wrapper class CommBasicObjectsType
	
protected:

	// Internal params
	RobotType Robot;
	
	// External params
	
	// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	CommBasicObjectsType CommBasicObjects;
	

	void setContent(const ParameterStateStructCore &commit) {
		// External params
	
		this->CommBasicObjects = commit.getCommBasicObjects();
	}

	// special trigger method (user upcall) called before updating parameter global state
	virtual SmartACE::ParamResponseType handleCOMMIT(const ParameterStateStruct &commitState) = 0;
public:
	ParameterStateStructCore() {  }
	virtual ~ParameterStateStructCore() {  }
	
	// internal param getters
	RobotType getRobot() const {
		return Robot;
	}
	
	// external param getters
	
	// repo wrapper class getter(s)
	CommBasicObjectsType getCommBasicObjects() const {
		return CommBasicObjects;
	}
	
	// helper method to easily implement output stream in derived classes
	void to_ostream(std::ostream &os = std::cout) const
	{
		// Internal params
		Robot.to_ostream(os);
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
		CommBasicObjects.to_ostream(os);
	}
	
	std::string getAsJSONString() {
		nlohmann::json param;
	
		param["Robot"] = nlohmann::json {
			{"enable_motors" , getRobot().getEnable_motors()},
			{"enable_sonar" , getRobot().getEnable_sonar()},
			{"maxRotVel" , getRobot().getMaxRotVel()},
			{"maxRotVelAcc" , getRobot().getMaxRotVelAcc()},
			{"maxRotVelDecel" , getRobot().getMaxRotVelDecel()},
			{"maxVel" , getRobot().getMaxVel()},
			{"maxVelAcc" , getRobot().getMaxVelAcc()},
			{"maxVelDecel" , getRobot().getMaxVelDecel()},
			{"robotType" , getRobot().getRobotType()},
			{"serialport" , getRobot().getSerialport()}
		};
	
		param["BaseParams"] = nlohmann::json {
		};
		
		return param.dump();
	}
};

#endif
