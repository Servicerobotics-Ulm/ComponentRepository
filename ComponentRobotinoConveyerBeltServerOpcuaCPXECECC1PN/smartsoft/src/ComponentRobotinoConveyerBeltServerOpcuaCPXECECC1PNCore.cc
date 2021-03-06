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
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#include "ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNCore.hh"

// constructor
ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNCore::ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNCore():
triggerLoad(0),
	triggerUnLoad(0),
	ack_triggered(false)
{
	std::cout << "constructor SmartRobotinoConveyerBeltServerCore\n";
}


bool ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNCore::getAckTiggered(){
	SmartACE::SmartGuard guard(lock);
	return ack_triggered;
}


void ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNCore::setAckTiggered(bool state){
	SmartACE::SmartGuard guard(lock);
	ack_triggered = state;
}
