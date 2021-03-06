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

#ifndef SMARTJOYSTICKNAVIGATION_ACE_PORTFACTORY_HH_
#define SMARTJOYSTICKNAVIGATION_ACE_PORTFACTORY_HH_

// include ACE/SmartSoft component implementation
#include "SmartJoystickNavigationImpl.hh"

// include the main component-definition class
#include "SmartJoystickNavigationPortFactoryInterface.hh"

class SmartJoystickNavigationAcePortFactory: public SmartJoystickNavigationPortFactoryInterface
{
private:
	SmartJoystickNavigationImpl *componentImpl;
public:
	SmartJoystickNavigationAcePortFactory();
	virtual ~SmartJoystickNavigationAcePortFactory();

	virtual void initialize(SmartJoystickNavigation *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommJoystick> * createJoystickServiceIn() override;
	virtual Smart::ISendClientPattern<CommBasicObjects::CommNavigationVelocity> * createNavVelServiceOut() override;
	
	
	// get a pointer to the internal component implementation
	SmartACE::SmartComponent* getComponentImpl();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* SMARTJOYSTICKNAVIGATION_ACE_PORTFACTORY_HH_ */
