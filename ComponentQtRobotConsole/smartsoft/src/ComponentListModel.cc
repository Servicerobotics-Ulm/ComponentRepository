/*
 * ComponentListModel.cpp
 *
 *  Created on: Mar 26, 2019
 *      Author: shaikv3
 */

#include <ComponentListModel.hh>
#include <algorithm>
ComponentListModel::ComponentListModel(SmartACE::StateMaster *stateMaster, SmartACE::ParameterMaster *paramMaster, QObject* parent):
mstateMaster(stateMaster), mparamMaster(paramMaster) ,QAbstractListModel(parent)
{

}

ComponentListModel::~ComponentListModel() {
	// TODO Auto-generated destructor stub
}

int ComponentListModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return mComponents.size();
}


QVariant ComponentListModel::data(const QModelIndex &index, int role) const
{
    if (!isIndexValid(index)) {
        return QVariant();
    }
    Component comp = mComponents.at(index.row());

    switch (role) {

    case Roles::ComponentNameRole:
        return QString::fromStdString(comp.get_name());

    case Roles::ComponentCurrentStateRole:
    	return QString::fromStdString(comp.get_state());

    case Roles::ComponentStateListRole:
        	return comp.get_stateList();

    case Roles::ComponentHasParameterSlave:
            	return comp.get_has_parammeter_slave();
    default:
        return QVariant();
    }
}

bool ComponentListModel::setData(const QModelIndex& index, const QVariant& value, int role)
{

	if (!isIndexValid(index)) {
	        return false;
	    }
	    Component& current_comp = mComponents.at(index.row());
	    current_comp.set_state(value.toString().toStdString());
	    emit dataChanged(index, index);
	    return true;
}

bool ComponentListModel::removeRows(int row, int count, const QModelIndex& parent)
{
	if (row < 0
	            || row >= rowCount()
	            || count < 0
	            || (row + count) > rowCount()) {
	        return false;
	    }
	    beginRemoveRows(parent, row, row + count - 1);
	    int countLeft = count;
	    while (countLeft--) {
	        const Component& current_comp = mComponents.at(row + countLeft);
	        //mDb->shelfDao.removeShelf(current_shelf.id());
	    }
	    mComponents.erase(mComponents.begin() + row,
	    		mComponents.begin() + row + count);
	    endRemoveRows();
	    return true;
}

QHash<int, QByteArray> ComponentListModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[Roles::ComponentNameRole] = "name";
    roles[Roles::ComponentCurrentStateRole] = "current_state";
    roles[Roles::ComponentStateListRole] = "state_list";
    roles[Roles::ComponentHasParameterSlave] = "has_param_slave";

    return roles;
}

bool ComponentListModel::isIndexValid(const QModelIndex& index) const
{
    if (index.row() < 0
            || index.row() >= rowCount()
            || !index.isValid()) {
        return false;
    }
    return true;
}

bool ComponentListModel::set_state(QString comp, QString state)
{
	std::cout << "setting state comp:" <<comp.toStdString() << ", " <<state.toStdString()  <<std::endl;
	Smart::StatusCode s =  mstateMaster->setWaitState(state.toStdString(), comp.toStdString(), "state");

	if(s!= Smart::SMART_OK)
	{
		std::cout << "setting state Fail :  " << Smart::StatusCodeConversion(s)<<std::endl;
	}else{

	std::cout << "setting state done "<<std::endl;
	}

	return true;
}

bool ComponentListModel::refresh()
{
	mComponents.clear();


	// get components with state clients
	SmartACE::NSKeyType searchPattern;
	searchPattern.names[SmartACE::NSKeyType::PATTERN_NAME] = ACE_TEXT("*");

	ACE_Unbounded_Queue<SmartACE::NSKeyType> sate_comps = SmartACE::NAMING::instance()->getEntriesForMatchingPattern(searchPattern);

	for (ACE_Unbounded_Queue_Iterator<SmartACE::NSKeyType> iter (sate_comps); !iter.done (); iter.advance ())
	{
		SmartACE::NSKeyType *comp = 0;
		iter.next (comp);

		std::string current_servername = comp->names[SmartACE::NSKeyType::COMP_NAME].c_str();
		std::string service_name = comp->names[SmartACE::NSKeyType::SERVICE_NAME].c_str();
		std::string pattern_name = comp->names[SmartACE::NSKeyType::PATTERN_NAME].c_str();
		std::string commobj1_name = comp->names[SmartACE::NSKeyType::COMMOBJ1_NAME].c_str();
		std::string commobj2_name = comp->names[SmartACE::NSKeyType::COMMOBJ2_NAME].c_str();

		std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++\n";
		std::cout << "Component    : " << current_servername << "\n";
		std::cout << "Service Name : " << service_name << "\n";
		std::cout << "Pattern      : " << pattern_name << "\n";
		std::cout << "ComObj1      : " << commobj1_name << "\n";
		std::cout << "ComObj2      : " << commobj2_name << std::endl;

		uint8_t component_index = get_component_index(mComponents, current_servername);

		Component& current_component = mComponents.at((unsigned int)component_index);
		std::string current_state;

		//check if it is state query pattern
		if(!service_name.compare(std::string("state")))
		{
			current_component.set_has_state_slave(true);
			std::string current_state;
			// get current state of component
			Smart::StatusCode s_c = mstateMaster->getCurrentMainState(current_state, current_servername);
			if (s_c == Smart::SMART_OK)
			{
				current_component.set_state(current_state);
			} else
			{
				std::cout <<current_servername <<" failed to respond, Error: " << Smart::StatusCodeConversion(s_c) <<std::endl;

			}

            //get all the states of component
			std::list<std::string> mainstates;
			Smart::StatusCode status = mstateMaster->getAllMainStates(mainstates, current_servername, service_name);
			if (status != Smart::SMART_OK)
			{
				std::cout <<current_servername <<" failed to respond, Error: " << Smart::StatusCodeConversion(s_c) <<std::endl;
			} else
			{
				for (std::string tmp_state : mainstates)
				{
					current_component.add_state(tmp_state);

				}
			}
		}


		if(!(commobj1_name.compare("CommParameter::CommParameterRequest"))&&
		   !(commobj2_name.compare("CommParameter::CommParameterResponse"))
		)
		{
			current_component.set_has_parammeter_slave(true);
		}
	}

	sort_components();
	endResetModel();
	return true;
}

int ComponentListModel::size()
{
	return mComponents.size();
}

bool ComponentListModel::send_parameter_request(QString comp, QString parameterString)
{
	SmartACE::CommParameterRequest parameterRequest;
	SmartACE::CommParameterResponse parameterResponse;
	std::string portname("param");
	Smart::StatusCode status;
	parameterRequest = lispParamToParameterRequest(parameterString.toStdString());

	std::cout << "comp : " << comp.toStdString()
			  << " parameterString : " << parameterString.toStdString()
			  << "parameterRequest : "<<parameterRequest
			  << "portname : "<<portname <<std::endl;
	std::cout << parameterRequest << std::endl;
	status = mparamMaster->sendParameterWait(parameterRequest, parameterResponse, comp.toStdString(), portname);
	std::cout << "\n\n>>> Reply: " << Smart::StatusCodeConversion(status) << std::endl;
	return true;

}

#ifndef LISP_SEPARATOR
#define LISP_SEPARATOR " ()\"\n"
#define LISP_STRING    1000
#endif
SmartACE::CommParameterRequest ComponentListModel::lispParamToParameterRequest(std::string lispString) {
	SmartACE::CommParameterRequest parameterRequest;

	char *param1  = (char *)NULL;
	char *input  = (char *)NULL;
	input = strdup(lispString.c_str());

	//find tag, the first element in parameter string
	do {
		param1 = strsep(&input,LISP_SEPARATOR);
	} while ((param1 != NULL) && (strlen(param1)==0));

	if(param1 == NULL)
		return parameterRequest;

	std::cout << "tag: " << param1 << std::endl;
	parameterRequest.setTag(param1);

	//find all other parameters
	int index = 1;
	while(input != NULL){
		//find next parameter value
		do {
			param1 = strsep(&input,LISP_SEPARATOR);
		} while ((param1 != NULL) && (strlen(param1)==0));

		if(param1 != NULL) {
			char indexStr[255];
			sprintf(indexStr,"%d",index);
			std::cout << "[" << indexStr << "] = " << param1 << std::endl;
			parameterRequest.setString(indexStr, param1);
			index++;
		}
	}

	std::cout << "lispParamToParameterRequest(" << parameterRequest << ")" << parameterRequest << std::endl;

	return parameterRequest;
}


uint8_t ComponentListModel::get_component_index(std::vector<Component>& comps, std::string& component_name)
{
	for(int i =0; i< comps.size(); ++i)
	{
		Component current_comp = comps.at(i);
		if(!current_comp.get_name().compare(component_name))
		{
			return i;
		}
	}

	Component current_comp(component_name);
	comps.push_back(current_comp);
	return comps.size()- 1;
}

void ComponentListModel::sort_components()
{
	std::sort(mComponents.begin(), mComponents.end(),
			[](Component& comp1, Component& comp2){

		if(comp1.get_stateList().size() != comp2.get_stateList().size())
		return comp1.get_stateList().size() > comp2.get_stateList().size(); // sort by num of states, more states first
		else
		return (comp1.get_name().compare(comp2.get_name()))<0; // sort by name
	}
	);
}
