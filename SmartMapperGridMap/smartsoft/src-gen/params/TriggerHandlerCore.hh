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
	
		
		virtual void handleCommNavigationObjects_MapperParams_CURLOAD(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_CURLOADLTM() = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_CURPARAMETER(const int &xsize, const int &ysize, const int &xpos, const int &ypos, const int &id) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_CURSAVE(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_CURSAVEXPM(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMINITIALIZE(const int &value) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMLOAD(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMLOADIEEESTD(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMLOADYAML(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMPARAMETER(const int &xsize, const int &ysize, const int &xpos, const int &ypos, const int &id) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMSAVE(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMSAVEIEEESTD(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMSAVEXPM(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMSAVEYAMLPGM(const std::string &filename) = 0;
	
		
		virtual void handleCommNavigationObjects_MapperParams_LTMSAVEYAMLPPM(const std::string &filename) = 0;
	
	// extended trigger user methods
	
private:
	// trigger internal methods
	void handleCommNavigationObjects_MapperParams_CURLOADCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_CURLOADLTMCore();
	void handleCommNavigationObjects_MapperParams_CURPARAMETERCore(const int &xsize, const int &ysize, const int &xpos, const int &ypos, const int &id);
	void handleCommNavigationObjects_MapperParams_CURSAVECore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_CURSAVEXPMCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMINITIALIZECore(const int &value);
	void handleCommNavigationObjects_MapperParams_LTMLOADCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMLOADIEEESTDCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMLOADYAMLCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMPARAMETERCore(const int &xsize, const int &ysize, const int &xpos, const int &ypos, const int &id);
	void handleCommNavigationObjects_MapperParams_LTMSAVECore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMSAVEIEEESTDCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMSAVEXPMCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMSAVEYAMLPGMCore(const std::string &filename);
	void handleCommNavigationObjects_MapperParams_LTMSAVEYAMLPPMCore(const std::string &filename);
	
	// extended trigger internal methods 
};

#endif // _TRIGGERHANDLERCORE_HH
