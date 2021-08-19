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
#ifndef _JOBSOURCE2EVENTCLIENT_INPUT_COLLECTOR_HH
#define _JOBSOURCE2EVENTCLIENT_INPUT_COLLECTOR_HH

#include "aceSmartSoft.hh"

/** JobSource2EventClientInputCollector colelcts inputs from multiple input-port instances and acts as a single source
 *
 * This class implements an InputHandler for the InputPort jobSource2EventClient and at the same time acts itself
 * as an InputSubject for other InputHnalder implementations. The reason is that multiple instances
 * of an input port might be created (e.g. when the same imput port is instantiated with different
 * communciation technologies). In this case multiple input sources have to be uniformely collected.
 * This class collects the inputs from multiple sources and provides a uniform source by itself for all
 * the other related input handlders (e.g. for related UpcallManagers and InputTaskTriggers).
 */
class JobSource2EventClientInputCollector
:	public Smart::IInputHandler<Smart::EventInputType<CommBasicObjects::CommTaskMessage>>
,	public Smart::InputSubject<Smart::EventInputType<CommBasicObjects::CommTaskMessage>>
{
public:
	JobSource2EventClientInputCollector(Smart::InputSubject<Smart::EventInputType<CommBasicObjects::CommTaskMessage>> *subject)
	:	Smart::IInputHandler<Smart::EventInputType<CommBasicObjects::CommTaskMessage>>(subject)
	{  }
	virtual ~JobSource2EventClientInputCollector() = default;
	
	// this method is implemented public and can be called from multiple sources
	virtual void handle_input(const Smart::EventInputType<CommBasicObjects::CommTaskMessage> &input) {
		this->notify_input(input);
	}
};

#endif // _JOBSOURCE2EVENTCLIENT_INPUT_COLLECTOR_HH