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
		 * Definition of Parameter options
		 */
		class optionsType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			std::string log_path;
		
		public:
			// default constructor
			optionsType() {
				log_path = "~/Desktop";
			}
		
			/**
			 * here are the public getters
			 */
			inline std::string getLog_path() const { return log_path; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "options(";
				os << "log_path = " << log_path << ", ";
				os << ")\n";
			}
			
		}; // end class optionsType
		
	
		///////////////////////////////////////////
		// External params
		///////////////////////////////////////////
		
	
		///////////////////////////////////////////
		// Instance params
		///////////////////////////////////////////
		
		/**
		 * Definition of instantiated ParameterRepository CommTrackingObjects
		 */
		class CommTrackingObjectsType {
			friend class ParamUpdateHandler;
			public:
			/**
			 * Definition of instantiated ParameterSet Markers
			 */
			class MarkersType {
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
						os << "Markers(\n";
						os << ")";
					}
			}; // end of parameter-set class MarkersType
			
			protected:
				/**
				 * internal members
				 */
				MarkersType Markers;
			
			public:
				/**
				 * public getter methods
				 */
				inline MarkersType getMarkers() const { return Markers; }
				
				void to_ostream(std::ostream &os = std::cout) const
				{
					os << "CommTrackingObjects(\n";
					Markers.to_ostream(os);
					os << ")";
				}
		}; // end of parameter-repository wrapper class CommTrackingObjectsType
	
protected:

	// Internal params
	optionsType options;
	
	// External params
	
	// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	CommTrackingObjectsType CommTrackingObjects;
	

	void setContent(const ParameterStateStructCore &commit) {
		// External params
	
		this->CommTrackingObjects = commit.getCommTrackingObjects();
	}

	// special trigger method (user upcall) called before updating parameter global state
	virtual SmartACE::ParamResponseType handleCOMMIT(const ParameterStateStruct &commitState) = 0;
public:
	ParameterStateStructCore() {  }
	virtual ~ParameterStateStructCore() {  }
	
	// internal param getters
	optionsType getOptions() const {
		return options;
	}
	
	// external param getters
	
	// repo wrapper class getter(s)
	CommTrackingObjectsType getCommTrackingObjects() const {
		return CommTrackingObjects;
	}
	
	// helper method to easily implement output stream in derived classes
	void to_ostream(std::ostream &os = std::cout) const
	{
		// Internal params
		options.to_ostream(os);
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
		CommTrackingObjects.to_ostream(os);
	}
	
	std::string getAsJSONString() {
		nlohmann::json param;
	
		param["options"] = nlohmann::json {
			{"log_path" , getOptions().getLog_path()}
		};
	
		param["Markers"] = nlohmann::json {
		};
		
		return param.dump();
	}
};

#endif
