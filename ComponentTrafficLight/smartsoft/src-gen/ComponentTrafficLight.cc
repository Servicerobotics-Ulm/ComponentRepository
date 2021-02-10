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
#include "ComponentTrafficLight.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentTrafficLightAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentTrafficLight* ComponentTrafficLight::_componentTrafficLight = 0;

// constructor
ComponentTrafficLight::ComponentTrafficLight()
{
	std::cout << "constructor of ComponentTrafficLight\n";
	
	// set all pointer members to NULL
	trafficLightServiceIn = NULL;
	trafficLightServiceInInputTaskTrigger = NULL;
	trafficLightServiceInUpcallManager = NULL;
	trafficLightServiceInInputCollector = NULL;
	trafficLightServiceInHandler = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentTrafficLight";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.trafficLightServiceIn.serviceName = "TrafficLightServiceIn";
	connections.trafficLightServiceIn.roboticMiddleware = "ACE_SmartSoft";
	connections.trafficLightServiceInHandler.prescale = 1;
	
	// initialize members of ComponentTrafficLightROS1InterfacesExtension
	
	// initialize members of ComponentTrafficLightROSExtension
	
	// initialize members of ComponentTrafficLightRestInterfacesExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentTrafficLightExtension
	
}

void ComponentTrafficLight::addPortFactory(const std::string &name, ComponentTrafficLightPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentTrafficLight::addExtension(ComponentTrafficLightExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentTrafficLight::getComponentImpl()
{
	return dynamic_cast<ComponentTrafficLightAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentTrafficLight::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}




/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentTrafficLight::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentTrafficLight::startAllTasks() {
}

/**
 * Start all timers contained in this component
 */
void ComponentTrafficLight::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentTrafficLight::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "TrafficLightServiceIn") return trafficLightServiceInInputTaskTrigger;
	
	return NULL;
}


void ComponentTrafficLight::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		
		// initializations of ComponentTrafficLightROS1InterfacesExtension
		
		// initializations of ComponentTrafficLightROSExtension
		
		// initializations of ComponentTrafficLightRestInterfacesExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentTrafficLightExtension
		
		
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
		
		ComponentTrafficLightPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentTrafficLightAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentTrafficLight is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		trafficLightServiceIn = portFactoryRegistry[connections.trafficLightServiceIn.roboticMiddleware]->createTrafficLightServiceIn(connections.trafficLightServiceIn.serviceName);
		
		// create client ports
		
		// create InputTaskTriggers and UpcallManagers
		trafficLightServiceInInputCollector = new TrafficLightServiceInInputCollector(trafficLightServiceIn);
		trafficLightServiceInInputTaskTrigger = new Smart::InputTaskTrigger<DomainHMI::CommTrafficLight>(trafficLightServiceInInputCollector);
		trafficLightServiceInUpcallManager = new TrafficLightServiceInUpcallManager(trafficLightServiceInInputCollector);
		
		// create input-handler
		trafficLightServiceInHandler = new TrafficLightServiceInHandler(trafficLightServiceIn, connections.trafficLightServiceInHandler.prescale);
		
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
		
		
		
		
		// link observers with subjects
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std exception" << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}

// run the component
void ComponentTrafficLight::run()
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
void ComponentTrafficLight::fini()
{
	// unlink all observers
	
	// destroy all task instances

	// destroy all input-handler
	delete trafficLightServiceInHandler;

	// destroy InputTaskTriggers and UpcallManagers
	delete trafficLightServiceInInputTaskTrigger;
	delete trafficLightServiceInUpcallManager;
	delete trafficLightServiceInInputCollector;

	// destroy client ports

	// destroy server ports
	delete trafficLightServiceIn;
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
	
	// destruction of ComponentTrafficLightROS1InterfacesExtension
	
	// destruction of ComponentTrafficLightROSExtension
	
	// destruction of ComponentTrafficLightRestInterfacesExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentTrafficLightExtension
	
}

void ComponentTrafficLight::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentTrafficLight.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentTrafficLight.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentTrafficLight.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		
		// load parameters for server TrafficLightServiceIn
		parameter.getString("TrafficLightServiceIn", "serviceName", connections.trafficLightServiceIn.serviceName);
		if(parameter.checkIfParameterExists("TrafficLightServiceIn", "roboticMiddleware")) {
			parameter.getString("TrafficLightServiceIn", "roboticMiddleware", connections.trafficLightServiceIn.roboticMiddleware);
		}
		
		if(parameter.checkIfParameterExists("TrafficLightServiceInHandler", "prescale")) {
			parameter.getInteger("TrafficLightServiceInHandler", "prescale", connections.trafficLightServiceInHandler.prescale);
		}
		
		// load parameters for ComponentTrafficLightROS1InterfacesExtension
		
		// load parameters for ComponentTrafficLightROSExtension
		
		// load parameters for ComponentTrafficLightRestInterfacesExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentTrafficLightExtension
		
		
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
