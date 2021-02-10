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

#ifndef _PRODUCTIONSTATION_HH
#define _PRODUCTIONSTATION_HH

// use the generic client implmentation from the Open62541 C++ Wrapper Library
#include <OpcUaGenericClient.hh>

// implement the abstract interface
#include "ProductionStationInterface.hh"

namespace OPCUA {

/** This class wraps OPC UA related communication to an OPC UA Device.
 *
 * This class internally implements an OPC UA client and provides a C++ API
 * based on a provided XML file that contains the device's information model.
 * 
 * In case where no XML file is provided, this class can still be used generically
 * by using two generic connect methods and the generic template getter/setter/caller methods.
 * 
 */
class ProductionStation : public GenericClient, public ProductionStationInterface
{
protected:
	// method implementing the XML-specific client space
	virtual bool createClientSpace(const bool activateUpcalls=true) override;
	
	// generic upcall method called whenever one of the ntity's values is changed
	virtual void handleVariableValueUpdate(const std::string &variableName, const OPCUA::Variant &value) override;
	
	// specific method to handle value updates for isBoxPresent
	void handleIsBoxPresent(const bool &value);
	
	// specific method to handle value updates for LED_RED
	void handleLED_RED(const bool &value);
	
	// specific method to handle value updates for LED_YELLOW
	void handleLED_YELLOW(const bool &value);
	
	// specific method to handle value updates for LED_GREEN
	void handleLED_GREEN(const bool &value);
	

public:
	// Constructor
	ProductionStation();
	
	// Destructor
	virtual ~ProductionStation();
	
	/** XML Specific Getter function for variable isBoxPresent
	 *
	 *  This function gets isBoxPresent  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *
	 *  @return the new value (or a default value like 0 in case of errors)
	 */
	virtual bool getIsBoxPresent() const;
	
	/** XML Specific Getter function for variable isBoxPresent
	 *
	 *  This function gets isBoxPresent  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *	
	 *  @param value	:output parameter, returns the new value if StatusCode is ALL_OK
	 *
	 *  @return status code
	 *	- ALL_OK
	 *  - DISCONNECTED
	 *  - ERROR_COMMUNICATION
	 */
	virtual OPCUA::StatusCode getIsBoxPresent(bool &isBoxPresent) const override;
	virtual OPCUA::StatusCode getIsBoxPresentWait(bool &isBoxPresent);
	
	/** XML Specific Getter function for variable LED_RED
	 *
	 *  This function gets LED_RED  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *
	 *  @return the new value (or a default value like 0 in case of errors)
	 */
	virtual bool getLED_RED() const;
	
	/** XML Specific Getter function for variable LED_RED
	 *
	 *  This function gets LED_RED  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *	
	 *  @param value	:output parameter, returns the new value if StatusCode is ALL_OK
	 *
	 *  @return status code
	 *	- ALL_OK
	 *  - DISCONNECTED
	 *  - ERROR_COMMUNICATION
	 */
	virtual OPCUA::StatusCode getLED_RED(bool &lED_RED) const override;
	virtual OPCUA::StatusCode getLED_REDWait(bool &lED_RED);
	
	/** XML Specific Setter function for entity LED_RED
	 *
	 *  This function sets LED_RED  at the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *
	 *  @param value			:Value to be set
	 * 
	 *  @return status code
	 *    - true    : Entity was found and the value was set correctly
	 *    - false   : Entity was not found or the value was not set correctly
	 */
	virtual OPCUA::StatusCode setLED_RED(const bool &value) override;
	
	/** XML Specific Getter function for variable LED_YELLOW
	 *
	 *  This function gets LED_YELLOW  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *
	 *  @return the new value (or a default value like 0 in case of errors)
	 */
	virtual bool getLED_YELLOW() const;
	
	/** XML Specific Getter function for variable LED_YELLOW
	 *
	 *  This function gets LED_YELLOW  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *	
	 *  @param value	:output parameter, returns the new value if StatusCode is ALL_OK
	 *
	 *  @return status code
	 *	- ALL_OK
	 *  - DISCONNECTED
	 *  - ERROR_COMMUNICATION
	 */
	virtual OPCUA::StatusCode getLED_YELLOW(bool &lED_YELLOW) const override;
	virtual OPCUA::StatusCode getLED_YELLOWWait(bool &lED_YELLOW);
	
	/** XML Specific Setter function for entity LED_YELLOW
	 *
	 *  This function sets LED_YELLOW  at the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *
	 *  @param value			:Value to be set
	 * 
	 *  @return status code
	 *    - true    : Entity was found and the value was set correctly
	 *    - false   : Entity was not found or the value was not set correctly
	 */
	virtual OPCUA::StatusCode setLED_YELLOW(const bool &value) override;
	
	/** XML Specific Getter function for variable LED_GREEN
	 *
	 *  This function gets LED_GREEN  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *
	 *  @return the new value (or a default value like 0 in case of errors)
	 */
	virtual bool getLED_GREEN() const;
	
	/** XML Specific Getter function for variable LED_GREEN
	 *
	 *  This function gets LED_GREEN  from the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *	
	 *  @param value	:output parameter, returns the new value if StatusCode is ALL_OK
	 *
	 *  @return status code
	 *	- ALL_OK
	 *  - DISCONNECTED
	 *  - ERROR_COMMUNICATION
	 */
	virtual OPCUA::StatusCode getLED_GREEN(bool &lED_GREEN) const override;
	virtual OPCUA::StatusCode getLED_GREENWait(bool &lED_GREEN);
	
	/** XML Specific Setter function for entity LED_GREEN
	 *
	 *  This function sets LED_GREEN  at the Server
	 *  When class ProductionStation is used with a Specific XML file to connect to
	 *  SeRoNet Servers which implements the device information model.
	 *
	 *  @param value			:Value to be set
	 * 
	 *  @return status code
	 *    - true    : Entity was found and the value was set correctly
	 *    - false   : Entity was not found or the value was not set correctly
	 */
	virtual OPCUA::StatusCode setLED_GREEN(const bool &value) override;
	
	
	/** XML Specific Caller function for method loadbox
	 *
	 *  This function calls the method loadbox at the Server
	 *
	*  @param timeout			: Input | DataTypeIdentifier:6(int) ValueRank:-1 ArrayDimensions:0			 	
	*  @param result			: Output| DataTypeIdentifier:12(std::string) ValueRank:-1 ArrayDimensions:0			 	
	 * 
	 *  @return status code
	 *    - true    : Method was found and the method call was completed correctly
	 *    - false   : Method was not found or the method call was not completed correctly
	 */
	 virtual OPCUA::StatusCode callLoadbox(const int &timeout, std::string &result) override;
	 
	/** XML Specific Caller function for method start_unloading
	 *
	 *  This function calls the method start_unloading at the Server
	 *
	*  @param XtimeoutX			: Input | DataTypeIdentifier:6(int) ValueRank:-1 ArrayDimensions:0			 	
	*  @param result			: Output| DataTypeIdentifier:12(std::string) ValueRank:-1 ArrayDimensions:0			 	
	 * 
	 *  @return status code
	 *    - true    : Method was found and the method call was completed correctly
	 *    - false   : Method was not found or the method call was not completed correctly
	 */
	 virtual OPCUA::StatusCode callStart_unloading(const int &XtimeoutX, std::string &result) override;
	 
	/** XML Specific Caller function for method stop_unloading
	 *
	 *  This function calls the method stop_unloading at the Server
	 *
	*  @param XtimeoutX			: Input | DataTypeIdentifier:6(int) ValueRank:-1 ArrayDimensions:0			 	
	*  @param result			: Output| DataTypeIdentifier:12(std::string) ValueRank:-1 ArrayDimensions:0			 	
	 * 
	 *  @return status code
	 *    - true    : Method was found and the method call was completed correctly
	 *    - false   : Method was not found or the method call was not completed correctly
	 */
	 virtual OPCUA::StatusCode callStop_unloading(const int &XtimeoutX, std::string &result) override;
	 
};

} /* namespace OPCUA */

#endif // _PRODUCTIONSTATION_HH