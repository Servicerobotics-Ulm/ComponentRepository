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

#include "ComponentExerciseRGBDApproachCylinderAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentExerciseRGBDApproachCylinderAcePortFactory acePortFactory;

ComponentExerciseRGBDApproachCylinderAcePortFactory::ComponentExerciseRGBDApproachCylinderAcePortFactory()
{  
	componentImpl = 0;
	ComponentExerciseRGBDApproachCylinder::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentExerciseRGBDApproachCylinderAcePortFactory::~ComponentExerciseRGBDApproachCylinderAcePortFactory()
{  }

void ComponentExerciseRGBDApproachCylinderAcePortFactory::initialize(ComponentExerciseRGBDApproachCylinder *component, int argc, char* argv[])
{
	if(component->connections.component.defaultScheduler != "DEFAULT") {
		ACE_Sched_Params sched_params(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(component->connections.component.defaultScheduler == "FIFO") {
			sched_params.policy(ACE_SCHED_FIFO);
			#if defined(ACE_HAS_PTHREADS)
				sched_params.priority(ACE_THR_PRI_FIFO_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				sched_params.priority(THREAD_PRIORITY_IDLE);
			#endif
		} else if(component->connections.component.defaultScheduler == "RR") {
			sched_params.policy(ACE_SCHED_RR);
			#if defined(ACE_HAS_PTHREADS)
				sched_params.priority(ACE_THR_PRI_RR_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				sched_params.priority(THREAD_PRIORITY_IDLE);
			#endif
		}
		// create new instance of the SmartSoft component with customized scheuling parameters 
		componentImpl = new ComponentExerciseRGBDApproachCylinderImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentExerciseRGBDApproachCylinderImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentExerciseRGBDApproachCylinderAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::ISendClientPattern<CommBasicObjects::CommNavigationVelocity> * ComponentExerciseRGBDApproachCylinderAcePortFactory::createNavigationVelocityServiceOut()
{
	return new SmartACE::SendClient<CommBasicObjects::CommNavigationVelocity>(componentImpl);
}

Smart::IPushClientPattern<DomainVision::CommRGBDImage> * ComponentExerciseRGBDApproachCylinderAcePortFactory::createRGBDImagePushServiceIn()
{
	return new SmartACE::PushClient<DomainVision::CommRGBDImage>(componentImpl);
}



SmartACE::SmartComponent* ComponentExerciseRGBDApproachCylinderAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentExerciseRGBDApproachCylinderAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentExerciseRGBDApproachCylinderAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
