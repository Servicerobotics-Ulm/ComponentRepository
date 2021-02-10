/*
 * Component.hh
 *
 *  Created on: Nov 19, 2019
 *      Author: shaikv3
 */

#ifndef SMARTSOFT_SRC_COMPONENT_HH_
#define SMARTSOFT_SRC_COMPONENT_HH_
#include <string>
#include <vector>
#include <QList>
#include <QVariant>
class Component {

	std::string name;
	QStringList states_list;
	std::string current_state;
	bool has_parammeter_slave;
	bool has_state_slave;
public:
	Component(std::string n);
	Component();
	virtual ~Component();
	void set_state(std::string new_state);
	std::string get_state();
	std::string get_name();
	void set_name(std::string& comp_name);
	void add_state(std::string new_state);
	QStringList get_stateList();
	bool is_current_state(QString state);
	void set_has_state_slave(bool value);
	bool get_has_state_slave();

	void set_has_parammeter_slave(bool value);
	bool get_has_parammeter_slave();
};

#endif /* SMARTSOFT_SRC_COMPONENT_HH_ */
