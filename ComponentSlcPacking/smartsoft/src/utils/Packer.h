/*
 * Packer.h
 *
 *  Created on: Jul 27, 2018
 *      Author: rollenhagen
 */

#ifndef PACKER_H_
#define PACKER_H_

#include "Container.h"
#include "Layer.hh"

#include "CommObjectRecognitionObjects/CommObjectRecognitionObjectProperties.hh"

class Packer {

private:
	Container slc;
	std::vector<Layer> layers;
	float gridSizeInM;

public:
	Packer();
	virtual ~Packer();

	const std::vector<Layer>& getLayers() const {
		return layers;
	}

	void setLayers(const std::vector<Layer>& layers) {
		this->layers = layers;
	}

	const Container& getSlc() const {
		return slc;
	}

	void setSlc(const Container& slc) {
		this->slc = slc;
	}

	float getGridSizeInM() const {
		return gridSizeInM;
	}

	void setGridSizeInM(float gridSizeInM) {
		this->gridSizeInM = gridSizeInM;
	}

	void initLayers();
	mrpt::poses::CPose3D findFreeSpace(float xInM, float yInM  );
	mrpt::poses::CPose3D findFreeSpaceForBox(CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties box);


};

#endif /* PACKER_H_ */
