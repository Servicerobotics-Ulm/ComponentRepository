/*
 * VisualLocalizationMapVisualization.hh
 *
 *  Created on: Jan 20, 2020
 *      Author: shaikv3
 */

#ifndef SMARTSOFT_SRC_VISUALIZATION_VISUALMARKERMAP_HH_
#define SMARTSOFT_SRC_VISUALIZATION_VISUALMARKERMAP_HH_

#include "AbstractVisualization.hh"
#include "CommLocalizationObjects/CommVisualLocalizationFeatureMap.hh"

class VisualLocalizationMapVisualization: public AbstractVisualization {
private:
public:
	VisualLocalizationMapVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~VisualLocalizationMapVisualization();
	void displayMap(const CommLocalizationObjects::CommVisualLocalizationFeatureMap& map);
	void clear();
};

#endif /* SMARTSOFT_SRC_VISUALIZATION_VISUALMARKERMAP_HH_ */
