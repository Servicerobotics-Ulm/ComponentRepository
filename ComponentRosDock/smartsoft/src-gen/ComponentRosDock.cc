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
#include "ComponentRosDock.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentRosDockAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentRosDock* ComponentRosDock::_componentRosDock = 0;

// constructor
ComponentRosDock::ComponentRosDock()
{
	std::cout << "constructor of ComponentRosDock\n";
	
	// set all pointer members to NULL
	baseStateServiceOut = NULL;
	dockActivity = NULL;
	dockActivityTrigger = NULL;
	//twist_pub = NULL;
	//twist_sub = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentRosDock";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.baseStateServiceOut.serviceName = "BaseStateServiceOut";
	connections.baseStateServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.dockActivity.minActFreq = 0.0;
	connections.dockActivity.maxActFreq = 0.0;
	connections.dockActivity.trigger = "PeriodicTimer";
	connections.dockActivity.periodicActFreq = 1.0;
	// scheduling default parameters
	connections.dockActivity.scheduler = "DEFAULT";
	connections.dockActivity.priority = -1;
	connections.dockActivity.cpuAffinity = -1;
	
	// initialize members of ComponentRosDockROSExtension
	rosPorts = 0;
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentRosDockExtension
	
}

void ComponentRosDock::addPortFactory(const std::string &name, ComponentRosDockPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentRosDock::addExtension(ComponentRosDockExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentRosDock::getComponentImpl()
{
	return dynamic_cast<ComponentRosDockAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentRosDock::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}




/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentRosDock::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentRosDock::startAllTasks() {
	// start task DockActivity
	if(connections.dockActivity.scheduler != "DEFAULT") {
		ACE_Sched_Params dockActivity_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.dockActivity.scheduler == "FIFO") {
			dockActivity_SchedParams.policy(ACE_SCHED_FIFO);
			dockActivity_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.dockActivity.scheduler == "RR") {
			dockActivity_SchedParams.policy(ACE_SCHED_RR);
			dockActivity_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		dockActivity->start(dockActivity_SchedParams, connections.dockActivity.cpuAffinity);
	} else {
		dockActivity->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentRosDock::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentRosDock::getInputTaskTriggerFromString(const std::string &client)
{
	
	return NULL;
}


void ComponentRosDock::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		
		// initializations of ComponentRosDockROSExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentRosDockExtension
		
		
		// initialize all registered port-factories
		for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
		{
			portFactory->second->initialize(this, argc, argv);
		}
		
		// initialize all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->initialize(this, argc, argv);
		}
		
		ComponentRosDockPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentRosDockAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentRosDock is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		baseStateServiceOut = portFactoryRegistry[connections.baseStateServiceOut.roboticMiddleware]->createBaseStateServiceOut(connections.baseStateServiceOut.serviceName);
		
		// create client ports
		
		// create InputTaskTriggers and UpcallManagers
		
		// create input-handler
		
		// create request-handlers
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		
		
		
		// create Task DockActivity
		dockActivity = new DockActivity(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.dockActivity.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.dockActivity.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(dockActivity);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				dockActivityTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task DockActivity" << std::endl;
			}
		} else if(connections.dockActivity.trigger == "DataTriggered") {
			dockActivityTrigger = getInputTaskTriggerFromString(connections.dockActivity.inPortRef);
			if(dockActivityTrigger != NULL) {
				dockActivityTrigger->attach(dockActivity, connections.dockActivity.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.dockActivity.inPortRef << " as activation source for Task DockActivity" << std::endl;
			}
		} else
		{
			// setup default task-trigger as PeriodicTimer
			Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
			int microseconds = 1000*1000 / 1.0;
			if(microseconds > 0) {
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				triggerPtr->attach(dockActivity);
				// store trigger in class member
				dockActivityTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task DockActivity" << std::endl;
			}
		}
		
		
		// link observers with subjects
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std exception" << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}

// run the component
void ComponentRosDock::run()
{
	stateSlave->acquire("init");
	// startup all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onStartup();
	}
	
	// startup all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onStartup();
	}
	stateSlave->release("init");
	
	// do not call this handler within the init state (see above) as this handler internally calls setStartupFinished() (this should be fixed in future)
	compHandler.onStartup();
	
	// this call blocks until the component is commanded to shutdown
	stateSlave->acquire("shutdown");
	
	// shutdown all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onShutdown();
	}
	
	// shutdown all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onShutdown();
	}
	
	if(connections.component.useLogger == true) {
		//FIXME: use logging
		//Smart::LOGGER->stopLogging();
	}
	
	compHandler.onShutdown();
	
	stateSlave->release("shutdown");
}

// clean-up component's resources
void ComponentRosDock::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(dockActivityTrigger != NULL){
		dockActivityTrigger->detach(dockActivity);
		delete dockActivity;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers

	// destroy client ports

	// destroy server ports
	delete baseStateServiceOut;
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	
	delete stateSlave;
	// destroy state-change-handler
	delete stateChangeHandler;
	
	// destroy all master/slave ports
	delete wiringSlave;
	

	// destroy all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->destroy();
	}

	// destroy all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->destroy();
	}
	
	// destruction of ComponentRosDockROSExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentRosDockExtension
	
}

void ComponentRosDock::loadParameter(int argc, char *argv[])
{
	/*
	 Parameters can be specified via command line --filename=<filename> or -f <filename>

	 With this parameter present:
	   - The component will look for the file in the current working directory,
	     a path relative to the current directory or any absolute path
	   - The component will use the default values if the file cannot be found

	 With this parameter absent:
	   - <Name of Component>.ini will be read from current working directory, if found there
	   - $SMART_ROOT/etc/<Name of Component>.ini will be read otherwise
	   - Default values will be used if neither found in working directory or /etc
	 */
	SmartACE::SmartIniParameter parameter;
	std::ifstream parameterfile;
	bool parameterFileFound = false;

	// load parameters
	try
	{
		// if paramfile is given as argument
		if(parameter.tryAddFileFromArgs(argc,argv,"filename", 'f'))
		{
			parameterFileFound = true;
			std::cout << "parameter file is loaded from an argv argument \n";
		} else if(parameter.searchFile("ComponentRosDock.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentRosDock.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentRosDock.ini parameter file not found! (using default values or command line arguments)\n";
		}
		
		// add command line arguments to allow overwriting of parameters
		// from file
		parameter.addCommandLineArgs(argc,argv,"component");
		
		// initialize the naming service using the command line parameters parsed in the
		// SmartIniParameter class. The naming service parameters are expected to be in
		// the "component" parameter group.
		SmartACE::NAMING::instance()->checkForHelpArg(argc,argv);
		if(parameterFileFound) 
		{
			if(SmartACE::NAMING::instance()->init(parameter.getAllParametersFromGroup("component")) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		} else {
			if(SmartACE::NAMING::instance()->init(argc, argv) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		}
			
		// print all known parameters
		// parameter.print();
		
		//--- server port // client port // other parameter ---
		// load parameter
		parameter.getString("component", "name", connections.component.name);
		parameter.getString("component", "initialComponentMode", connections.component.initialComponentMode);
		if(parameter.checkIfParameterExists("component", "defaultScheduler")) {
			parameter.getString("component", "defaultScheduler", connections.component.defaultScheduler);
		}
		if(parameter.checkIfParameterExists("component", "useLogger")) {
			parameter.getBoolean("component", "useLogger", connections.component.useLogger);
		}
		
		
		// load parameters for server BaseStateServiceOut
		parameter.getString("BaseStateServiceOut", "serviceName", connections.baseStateServiceOut.serviceName);
		if(parameter.checkIfParameterExists("BaseStateServiceOut", "roboticMiddleware")) {
			parameter.getString("BaseStateServiceOut", "roboticMiddleware", connections.baseStateServiceOut.roboticMiddleware);
		}
		
		// load parameters for task DockActivity
		parameter.getDouble("DockActivity", "minActFreqHz", connections.dockActivity.minActFreq);
		parameter.getDouble("DockActivity", "maxActFreqHz", connections.dockActivity.maxActFreq);
		parameter.getString("DockActivity", "triggerType", connections.dockActivity.trigger);
		if(connections.dockActivity.trigger == "PeriodicTimer") {
			parameter.getDouble("DockActivity", "periodicActFreqHz", connections.dockActivity.periodicActFreq);
		} else if(connections.dockActivity.trigger == "DataTriggered") {
			parameter.getString("DockActivity", "inPortRef", connections.dockActivity.inPortRef);
			parameter.getInteger("DockActivity", "prescale", connections.dockActivity.prescale);
		}
		if(parameter.checkIfParameterExists("DockActivity", "scheduler")) {
			parameter.getString("DockActivity", "scheduler", connections.dockActivity.scheduler);
		}
		if(parameter.checkIfParameterExists("DockActivity", "priority")) {
			parameter.getInteger("DockActivity", "priority", connections.dockActivity.priority);
		}
		if(parameter.checkIfParameterExists("DockActivity", "cpuAffinity")) {
			parameter.getInteger("DockActivity", "cpuAffinity", connections.dockActivity.cpuAffinity);
		}
		
		// load parameters for ComponentRosDockROSExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentRosDockExtension
		
		
		// load parameters for all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->loadParameters(parameter);
		}
		
	
	} catch (const SmartACE::IniParameterError & e) {
		std::cerr << e.what() << std::endl;
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std::exception: " << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}
