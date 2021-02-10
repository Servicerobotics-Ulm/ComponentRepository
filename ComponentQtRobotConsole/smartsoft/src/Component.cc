/*
 * Component.cc
 *
 *  Created on: Nov 19, 2019
 *      Author: shaikv3
 */

#include <Component.hh>

Component::Component(std::string n): name(n) {
	current_state ="Unknown";
	has_parammeter_slave = false;
	has_state_slave = false;
}

Component::Component()
{
	current_state ="Unknown";
	has_parammeter_slave = false;
	has_state_slave = false;
	name = "notset";
}

Component::~Component() {
}
void Component::set_state(std::string new_state)
{
	if(current_state != new_state)
		current_state = new_state;
}
std::string Component::get_state()
{
	return current_state;
}

void Component::add_state(std::string new_state)
{
	states_list.push_back(QString::fromStdString(new_state));

}
std::string Component::get_name()
{
return name;
}

void Component::set_name(std::string& comp_name)
{
	name = comp_name;
}

QStringList Component::get_stateList()
{
//	QVariant var;
//	    var.setValue(states_list);
//	    return var;
	return states_list;
}

bool Component::is_current_state(QString state)
{
	return (current_state == state.toStdString());
}

void Component::set_has_state_slave(bool value)
{
	has_state_slave = value;
}
bool Component::get_has_state_slave()
{
    return has_state_slave;
}

void Component::set_has_parammeter_slave(bool value)
{
	has_parammeter_slave = value;
}

bool Component::get_has_parammeter_slave()
{
	return has_parammeter_slave;
}
