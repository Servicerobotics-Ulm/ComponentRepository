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

#include "TriggerHandlerCore.hh"

int TriggerHandlerCore::on_execute()
{
	// blocking wait until some active trigger request(s) come in
	sema.acquire();
	
	{
		SmartACE::SmartGuard g(mutex);
		
		// get the top trigger from the queue
		current_trigger_enumerator = trigger_queue.front();
		trigger_queue.pop();
		
		// retrieve the corresponding trigger attributes from the trigger specific queue
		switch(current_trigger_enumerator) {
		case TriggerEnumerators::COMMBASICOBJECTS_SEQ2SEQCOMPARAM_SEND:
			current_CommBasicObjects_Seq2SeqComParam_Send = CommBasicObjects_Seq2SeqComParam_Send_queue.front();
			CommBasicObjects_Seq2SeqComParam_Send_queue.pop();
			break;
		}
	} // mutex release

	// now call the corresponding trigger handler
	// (releasing the mutex before, allows to store newly incoming trigger commands on the queue in parallel)
	switch(current_trigger_enumerator) {
	case TriggerEnumerators::COMMBASICOBJECTS_SEQ2SEQCOMPARAM_SEND:
		this->handleCommBasicObjects_Seq2SeqComParam_Send(current_CommBasicObjects_Seq2SeqComParam_Send.message);
		break;
	}
	return 0;
}

//
// trigger internal handler methods
//

	// handle Send
	void TriggerHandlerCore::handleCommBasicObjects_Seq2SeqComParam_SendCore(const std::string &message)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		CommBasicObjects_Seq2SeqComParam_SendAttributes attr;
		attr.message = message;
		CommBasicObjects_Seq2SeqComParam_Send_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::COMMBASICOBJECTS_SEQ2SEQCOMPARAM_SEND);
		
		// signal the task, in case it is waiting
		sema.release();
	}

//
// extended trigger internal handler methods
//
