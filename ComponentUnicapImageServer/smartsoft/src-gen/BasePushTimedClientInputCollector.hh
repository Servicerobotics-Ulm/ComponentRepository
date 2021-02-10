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
#ifndef _BASEPUSHTIMEDCLIENT_INPUT_COLLECTOR_HH
#define _BASEPUSHTIMEDCLIENT_INPUT_COLLECTOR_HH

#include "aceSmartSoft.hh"

/** BasePushTimedClientInputCollector colelcts inputs from multiple input-port instances and acts as a single source
 *
 * This class implements an InputHandler for the InputPort basePushTimedClient and at the same time acts itself
 * as an InputSubject for other InputHnalder implementations. The reason is that multiple instances
 * of an input port might be created (e.g. when the same imput port is instantiated with different
 * communciation technologies). In this case multiple input sources have to be uniformely collected.
 * This class collects the inputs from multiple sources and provides a uniform source by itself for all
 * the other related input handlders (e.g. for related UpcallManagers and InputTaskTriggers).
 */
class BasePushTimedClientInputCollector
:	public Smart::IInputHandler<CommBasicObjects::CommBaseState>
,	public Smart::InputSubject<CommBasicObjects::CommBaseState>
{
public:
	BasePushTimedClientInputCollector(Smart::InputSubject<CommBasicObjects::CommBaseState> *subject)
	:	Smart::IInputHandler<CommBasicObjects::CommBaseState>(subject)
	{  }
	virtual ~BasePushTimedClientInputCollector() = default;
	
	// this method is implemented public and can be called from multiple sources
	virtual void handle_input(const CommBasicObjects::CommBaseState &input) {
		this->notify_input(input);
	}
};

#endif // _BASEPUSHTIMEDCLIENT_INPUT_COLLECTOR_HH
