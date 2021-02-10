/*
 * ComponentListModel.hh
 *
 *  Created on: Nov 19, 2019
 *      Author: shaikv3
 */

#ifndef SMARTSOFT_SRC_COMPONENTLISTMODEL_HH_
#define SMARTSOFT_SRC_COMPONENTLISTMODEL_HH_

#include <Component.hh>
#include <QObject>
#include <QAbstractListModel>
#include <QHash>
#include <QString>
#include "aceSmartSoft.hh"
#include <string.h>

class ComponentListModel : public QAbstractListModel{
	Q_OBJECT
public:
	ComponentListModel(SmartACE::StateMaster *stateMaster, SmartACE::ParameterMaster *paramMaster,QObject* parent = 0);
	virtual ~ComponentListModel();

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	bool setData(const QModelIndex& index, const QVariant& value, int role) override;
	Q_INVOKABLE bool removeRows(int row, int count, const QModelIndex& parent = QModelIndex()) override;
	QHash<int, QByteArray> roleNames() const override;
	Q_INVOKABLE bool set_state(QString comp, QString state);
	Q_INVOKABLE bool send_parameter_request(QString comp, QString parameterString);
	SmartACE::CommParameterRequest lispParamToParameterRequest(std::string lispString);
	uint8_t get_component_index(std::vector<Component>& comps, std::string& component_name);

	std::vector<Component> mComponents;
	bool isIndexValid(const QModelIndex& index) const;
	Q_INVOKABLE bool refresh();
	Q_INVOKABLE int size();
	void sort_components();

	enum Roles{
		ComponentNameRole= Qt::UserRole + 1,
		ComponentCurrentStateRole,
		ComponentStateListRole,
		ComponentHasParameterSlave
	};
	SmartACE::StateMaster *mstateMaster;
	SmartACE::ParameterMaster *mparamMaster;
};

#endif /* SMARTSOFT_SRC_COMPONENTLISTMODEL_HH_ */
