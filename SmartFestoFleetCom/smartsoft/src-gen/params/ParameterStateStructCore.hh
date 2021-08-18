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
		 * Definition of Parameter Settings
		 */
		class SettingsType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			unsigned short basePortsSubcriptionInterval;
			std::string ip;
			std::string port;
			bool pushCyclicSystemState;
			std::string robotname;
			unsigned int socket_timeout_s;
			bool use_socket_timeout;
			bool verbose;
		
		public:
			// default constructor
			SettingsType() {
				basePortsSubcriptionInterval = 1;
				ip = "0.0.0.0";
				port = "20021";
				pushCyclicSystemState = true;
				robotname = "1";
				socket_timeout_s = 5;
				use_socket_timeout = true;
				verbose = false;
			}
		
			/**
			 * here are the public getters
			 */
			inline unsigned short getBasePortsSubcriptionInterval() const { return basePortsSubcriptionInterval; }
			inline std::string getIp() const { return ip; }
			inline std::string getPort() const { return port; }
			inline bool getPushCyclicSystemState() const { return pushCyclicSystemState; }
			inline std::string getRobotname() const { return robotname; }
			inline unsigned int getSocket_timeout_s() const { return socket_timeout_s; }
			inline bool getUse_socket_timeout() const { return use_socket_timeout; }
			inline bool getVerbose() const { return verbose; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "Settings(";
				os << "basePortsSubcriptionInterval = " << basePortsSubcriptionInterval; os << ", ";
				os << "ip = " << ip; os << ", ";
				os << "port = " << port; os << ", ";
				os << "pushCyclicSystemState = " << pushCyclicSystemState; os << ", ";
				os << "robotname = " << robotname; os << ", ";
				os << "socket_timeout_s = " << socket_timeout_s; os << ", ";
				os << "use_socket_timeout = " << use_socket_timeout; os << ", ";
				os << "verbose = " << verbose;
				os << ")\n";
			}
			
		}; // end class SettingsType
		
		/**
		 * Definition of Parameter RobotinoIO
		 */
		class RobotinoIOType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			short conveyerBelt_BoxPresentDIn;
			short laser_SafteyFieldDIn;
			short laser_WarningFieldDIn;
		
		public:
			// default constructor
			RobotinoIOType() {
				conveyerBelt_BoxPresentDIn = 2;
				laser_SafteyFieldDIn = 0;
				laser_WarningFieldDIn = 1;
			}
		
			/**
			 * here are the public getters
			 */
			inline short getConveyerBelt_BoxPresentDIn() const { return conveyerBelt_BoxPresentDIn; }
			inline short getLaser_SafteyFieldDIn() const { return laser_SafteyFieldDIn; }
			inline short getLaser_WarningFieldDIn() const { return laser_WarningFieldDIn; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "RobotinoIO(";
				os << "conveyerBelt_BoxPresentDIn = " << conveyerBelt_BoxPresentDIn; os << ", ";
				os << "laser_SafteyFieldDIn = " << laser_SafteyFieldDIn; os << ", ";
				os << "laser_WarningFieldDIn = " << laser_WarningFieldDIn;
				os << ")\n";
			}
			
		}; // end class RobotinoIOType
		
	
		///////////////////////////////////////////
		// External params
		///////////////////////////////////////////
		
	
		///////////////////////////////////////////
		// Instance params
		///////////////////////////////////////////
		
		/**
		 * Definition of instantiated ParameterRepository DomainRobotFleet
		 */
		class DomainRobotFleetType {
			friend class ParamUpdateHandler;
			public:
			/**
			 * Definition of instantiated ParameterSet FleetManagerParameter
			 */
			class FleetManagerParameterType {
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
						os << "FleetManagerParameter(\n";
						os << ")";
					}
			}; // end of parameter-set class FleetManagerParameterType
			
			protected:
				/**
				 * internal members
				 */
				FleetManagerParameterType FleetManagerParameter;
			
			public:
				/**
				 * public getter methods
				 */
				inline FleetManagerParameterType getFleetManagerParameter() const { return FleetManagerParameter; }
				
				void to_ostream(std::ostream &os = std::cout) const
				{
					os << "DomainRobotFleet(\n";
					FleetManagerParameter.to_ostream(os);
					os << ")";
				}
		}; // end of parameter-repository wrapper class DomainRobotFleetType
	
protected:

	// Internal params
	RobotinoIOType RobotinoIO;
	SettingsType Settings;
	
	// External params
	
	// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	DomainRobotFleetType DomainRobotFleet;
	

	void setContent(const ParameterStateStructCore &commit) {
		// External params
	
		this->DomainRobotFleet = commit.getDomainRobotFleet();
	}

	// special trigger method (user upcall) called before updating parameter global state
	virtual SmartACE::ParamResponseType handleCOMMIT(const ParameterStateStruct &commitState) = 0;
public:
	ParameterStateStructCore() {  }
	virtual ~ParameterStateStructCore() {  }
	
	// internal param getters
	RobotinoIOType getRobotinoIO() const {
		return RobotinoIO;
	}
	SettingsType getSettings() const {
		return Settings;
	}
	
	// external param getters
	
	// repo wrapper class getter(s)
	DomainRobotFleetType getDomainRobotFleet() const {
		return DomainRobotFleet;
	}
	
	// helper method to easily implement output stream in derived classes
	void to_ostream(std::ostream &os = std::cout) const
	{
		// Internal params
		RobotinoIO.to_ostream(os);
		Settings.to_ostream(os);
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
		DomainRobotFleet.to_ostream(os);
	}
	
	std::string getAsJSONString() {
		nlohmann::json param;
	
		param["RobotinoIO"] = nlohmann::json {
			{"conveyerBelt_BoxPresentDIn" , getRobotinoIO().getConveyerBelt_BoxPresentDIn()},
			{"laser_SafteyFieldDIn" , getRobotinoIO().getLaser_SafteyFieldDIn()},
			{"laser_WarningFieldDIn" , getRobotinoIO().getLaser_WarningFieldDIn()}
		};
		param["Settings"] = nlohmann::json {
			{"basePortsSubcriptionInterval" , getSettings().getBasePortsSubcriptionInterval()},
			{"ip" , getSettings().getIp()},
			{"port" , getSettings().getPort()},
			{"pushCyclicSystemState" , getSettings().getPushCyclicSystemState()},
			{"robotname" , getSettings().getRobotname()},
			{"socket_timeout_s" , getSettings().getSocket_timeout_s()},
			{"use_socket_timeout" , getSettings().getUse_socket_timeout()},
			{"verbose" , getSettings().getVerbose()}
		};
	
		param["FleetManagerParameter"] = nlohmann::json {
		};
		
		return param.dump();
	}
};

#endif
