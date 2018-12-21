# ComponentRepository

This repository contains robotics software-component projects. See README.md in the individual component subfolders for a detailed explanation of the individual component's services and configurations options.

All components depend on the [**ACE/SmartSoft Framework**](https://github.com/Servicerobotics-Ulm/AceSmartSoftFramework) and have component-specific (i.e. individual) dependendies to further libraries according to the componen's internal implementation.

All the components have been modelled and developed with the [**SmartMDSD Toolchain v3**](http://robmosys.eu/wiki/baseline:environment_tools:smartsoft:smartmdsd-toolchain:start).

Individual components instantiate and implement modelled service-definitions that are specified independently within the [**DomainModelsRepositories**](https://github.com/Servicerobotics-Ulm/DomainModelsRepositories).

License: Each component has its individual license depending on the internally used algorithms and libraries.

This repository is maintained by Servicerobotik Ulm. For more information see:

* Big picture: relation of repositories: https://wiki.servicerobotik-ulm.de/download
* SRRC Technical Wiki on SmartSoft and SmartMDSD Toolchain: https://wiki.servicerobotik-ulm.de

## Installation and compilation instructions

For compiling and installing the components, please checkout and install the [**ACE/SmartSoft Framework**](https://github.com/Servicerobotics-Ulm/AceSmartSoftFramework) (following [these installation instructions](https://github.com/Servicerobotics-Ulm/AceSmartSoftFramework/blob/master/README.md)) and make sure that the required [**DomainModelsRepositories**](https://github.com/Servicerobotics-Ulm/DomainModelsRepositories) are checked out.

The CMake scripts of individual components automatically search for the required **DomainModels** by using the environment variable named **SMART_PACKAGE_PATH**. In other words, the environment variable **SMART_PACKAGE_PATH** should point to the root folders (separated by a **:** if you have several) of your local copies of [**DomainModelsRepositories**](https://github.com/Servicerobotics-Ulm/DomainModelsRepositories).

After the [ACE/SmartSoft Framework](https://github.com/Servicerobotics-Ulm/AceSmartSoftFramework) has been installed and the required [DomainModelsRepositories](https://github.com/Servicerobotics-Ulm/DomainModelsRepositories) have been checked out, the individual component can be compiled like this (e.g. for the component SmartJoystickServer):

```
> cd <path-to-checkout>/SmartJoystickServer/smartsoft
> mkdir build; cd build
> cmake ..
> make
```

The CMake script of a component automatically searches for the required DomainModels, automatically builds them (if needed) and then builds the component itself. In addition, component-specific libraries are searched and the CMake script will print an error message if the required libraries are not found in your system.

After compiling, the component binary is automatically installed into the location specified by the environment variable named **SMART_ROOT_ACE**. For executing the component, you first need to start the naming-service like this:

```
> cd $SMART_ROOT_ACE
> ./startSmartSoftNamingService
```

After that, you can start the individual components (each in a separate terminal) like this:

```
> cd $SMART_ROOT_ACE
> ./bin/SmartJoystickServer
```

Be aware that components have default configuration sets that are used if no configuration files (i.e. ini-files) are found. At startup a component searches for an according configuration file within a local folder named **etc**. For instance, for the SmartJoystickServer component, an ini-file named **SmartJoystickServer.ini** can be created within the folder **$SMART_ROOT_ACE/etc**. For convenience reasons, a file named **SmartJoystickServer.ini.template** is automatically generated into the **$SMART_ROOT_ACE/etc** folder. This file can be renamed (by removing the **.template** extension) and adjusted to change the initial configuration parameters of a component. After restarting a component, you can see that the new parameters are automatically loaded. Please note, that the ini-files only specify the startup parameters. A component additionally has run-time parameters that can be dynamically changed using the component's generic **parameter service**.
