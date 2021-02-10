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
#ifndef _TRIGGERHANDLERCORE_HH
#define _TRIGGERHANDLERCORE_HH


#include <string>
#include <iostream>
#include <list>

class TriggerHandlerCore
{
	friend class ParamUpdateHandler;
	
public:
	TriggerHandlerCore() 
	{  
	}
	virtual ~TriggerHandlerCore() {  }

	// trigger user methods
	
		
		virtual void handleCommLocalizationObjects_LocalizationParameter_GLOBALLOCALIZATION() = 0;
	
		
		virtual void handleCommLocalizationObjects_LocalizationParameter_INITIALPOSE(const int &x, const int &y, const double &a) = 0;
	
		
		virtual void handleCommLocalizationObjects_LocalizationParameter_LOADMAP(const std::string &filename) = 0;
	
		
		virtual void handleCommLocalizationObjects_LocalizationParameter_SENSORSTOUSE(const unsigned short &sensorsToUse) = 0;
	
	// extended trigger user methods
	
private:
	// trigger internal methods
	void handleCommLocalizationObjects_LocalizationParameter_GLOBALLOCALIZATIONCore();
	void handleCommLocalizationObjects_LocalizationParameter_INITIALPOSECore(const int &x, const int &y, const double &a);
	void handleCommLocalizationObjects_LocalizationParameter_LOADMAPCore(const std::string &filename);
	void handleCommLocalizationObjects_LocalizationParameter_SENSORSTOUSECore(const unsigned short &sensorsToUse);
	
	// extended trigger internal methods 
};

#endif // _TRIGGERHANDLERCORE_HH