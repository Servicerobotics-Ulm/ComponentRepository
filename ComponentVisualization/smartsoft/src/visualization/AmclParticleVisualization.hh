/*
 * AmclParticleVisualization.hh
 *
 *  Created on: Jan 20, 2020
 *      Author: shaikv3
 */

#ifndef SMARTSOFT_SRC_VISUALIZATION_AMCLPARTICLEVISUALIZATION_HH_
#define SMARTSOFT_SRC_VISUALIZATION_AMCLPARTICLEVISUALIZATION_HH_

#include "AbstractVisualization.hh"
#include "CommLocalizationObjects/CommAmclVisualizationInfo.hh"

class AmclParticleVisualization: public AbstractVisualization {
private:
	size_t max_hypotheses;
public:
	AmclParticleVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~AmclParticleVisualization();
	void displayAmclInfo(const CommLocalizationObjects::CommAmclVisualizationInfo& pf_info);
	void clear();
};

#endif /* SMARTSOFT_SRC_VISUALIZATION_AMCLPARTICLEVISUALIZATION_HH_ */
