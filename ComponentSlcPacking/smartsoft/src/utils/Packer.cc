/*
 * Packer.cc
 *
 *  Created on: Jul 27, 2018
 *      Author: rollenhagen
 */

#include "Packer.h"

#include "CommBasicObjects/CommPose3d.hh"
#include "CommBasicObjects/CommPosition3d.hh"
#include "mrpt/math/include/mrpt/math/TPlane.h"
#include "mrpt/math/include/mrpt/math/TPoint3D.h"
#include "mrpt/poses/include/mrpt/poses/CPoint3D.h"
#include "mrpt/poses/include/mrpt/poses/CPose3D.h"

Packer::Packer() {
	// TODO Auto-generated constructor stub

}

Packer::~Packer() {
	// TODO Auto-generated destructor stub
}

void Packer::initLayers() {
	int gridSizeX = slc.getDimensions()[0] / gridSizeInM;
	int gridSizeY = slc.getDimensions()[1] / gridSizeInM;

	std::cout << "[Packer] grid size x,y: " << gridSizeX << ", " << gridSizeY << std::endl;

	mrpt::poses::CPose3D cPose = mrpt::poses::CPose3D(slc.getPose());
	std::vector< std::vector<bool> > grid(gridSizeX, std::vector<bool>(gridSizeY, false));

	//build frame to avoid robot collision with slc
	float frameSizeXInM = 0.02; // TODO should not hard code!!!
	float frameSizeYInM = 0.04; // TODO should not hard code!!!
	int frameSizeXGrid = frameSizeXInM / gridSizeInM;
	int frameSizeYGrid = frameSizeYInM / gridSizeInM;


	for(int line = 0; line < grid.size(); line++){
		if(line <= frameSizeXGrid || line >= grid.size() - frameSizeXGrid){
			for(int col = 0; col < grid[line].size(); col++){
				grid[line][col] = true;
			}
		}else{
			for(int col = 0; col <= frameSizeYGrid; col++){
				grid[line][col] = true;
			}
			for(int col = grid[line].size()-frameSizeYGrid; col < grid[line].size(); col++){
				grid[line][col] = true;
			}
		}
	}

	for(int i = this->layers.size()-1 ; i >= 0; i--){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud = layers[i].getPointCloud();

		for(int j = 0; j < point_cloud->size(); j++){
			mrpt::poses::CPose3D p(point_cloud->points[j].x, point_cloud->points[j].y, point_cloud->points[j].z, 0,0,0);
			mrpt::poses::CPose3D transformedPoint = p - cPose;

			int pointGridPositionX = transformedPoint.x() / gridSizeInM;
			int pointGridPositionY = transformedPoint.y() / gridSizeInM;

			if(pointGridPositionX >= gridSizeX || pointGridPositionY >= gridSizeY || pointGridPositionX < 0 || pointGridPositionY < 0){
				continue;
			}
			grid[pointGridPositionX][pointGridPositionY] = true;
		}
		layers[i].setGrid(grid);
	}


	//DEBUG code: print grid values
	for (int l = layers.size()-1; l >= 0; l--){
		std::cout << "layer ID: " << l << std::endl;

		std::vector< std::vector<bool> > tmp_grid = layers[l].getGrid();
		for(int line = 0; line < tmp_grid.size(); line++){
			for(int col = 0; col < tmp_grid[line].size(); col++){
				std::cout << tmp_grid[line][col] << "|" << std::flush;
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
}

mrpt::poses::CPose3D Packer::findFreeSpace(float xInM, float yInM ) {
	int searchedXDiscrete = (xInM / gridSizeInM) + 8; //adding 3 grid boxes as buffer
	int searchedYDiscrete = (yInM / gridSizeInM) + 8; //adding 3 grid boxes as buffer

	std::cout << "[Packer] searching free space dimensions metric (x,y): " << xInM << ", " << yInM << std::endl;
	std::cout << "[Packer] searching free space dimensions discrete (x,y): " << searchedXDiscrete << ", " << searchedYDiscrete << std::endl;

	for(int layerIdx = 1; layerIdx < layers.size()-1; layerIdx++){
		std::vector< std::vector<bool> > tmpGrid = layers[layerIdx].getGrid();

		int ySpacesFound = 0;

		for(int gridIdxX = 0; gridIdxX < tmpGrid.size(); gridIdxX++){
			for(int gridIdxY = 0; gridIdxY < tmpGrid[gridIdxX].size(); gridIdxY++){
				if(tmpGrid[gridIdxX][gridIdxY] == true){
					ySpacesFound = 0;
					continue;
				}

				ySpacesFound++;

				if(ySpacesFound < searchedYDiscrete){
					continue;
				}

				bool spaceOccupied = true;
				for(int x = gridIdxX + 1; x < gridIdxX + searchedXDiscrete; x++){
					for(int y = gridIdxY - searchedYDiscrete; y <= gridIdxY; y++){
						spaceOccupied = tmpGrid[x][y];
						if(spaceOccupied){break;}
					}
					if(spaceOccupied){break;}
				}
				if(spaceOccupied){continue;}

//				mittelpunkt berechnen
//				diskreten mittelpunkt in xy umrechnen
//				calc z by layer index
//				rückgabe x,y,z

				int midPointX = gridIdxX + (searchedXDiscrete / 2) + 1; //adding 1 because index starts with 0
				int midPointY = gridIdxY - (searchedYDiscrete / 2) + 1; //adding 1 because index starts with 0

				float midPointXInM = midPointX * gridSizeInM;
				float midPointYInM = midPointY * gridSizeInM;

				mrpt::poses::CPose3D containerPose =  mrpt::poses::CPose3D(slc.getPose());
				float midPointZInM = layers[layerIdx - 1].getPlane().distance(mrpt::math::TPoint3D(containerPose.x(), containerPose.y(), containerPose.z()));
				midPointZInM = std::abs(midPointZInM) * (-1);


				mrpt::poses::CPose3D midpoint(midPointXInM, midPointYInM, midPointZInM, 0, 1.57, 0);
				mrpt::poses::CPose3D transformedMidPoint = containerPose + midpoint;

				std::cout << "[Packer] distance used layer and top: " << midPointZInM << std::endl;

				std::cout << "[Packer] found free space at layer " << layerIdx << "; grid x,y: " << midPointX << ", " << midPointY << std::endl;
				std::cout << "[Packer] found free space at layer " << layerIdx << "; x,y: " << midPointXInM << ", " << midPointYInM << std::endl;
				std::cout << "[Packer] found free space in robot frame: " << transformedMidPoint << std::endl;


				return transformedMidPoint;
			}
		}
	}

	return mrpt::poses::CPose3D(0,0,0,0,0,0);
}


mrpt::poses::CPose3D Packer::findFreeSpaceForBox(CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties box) {
	CommBasicObjects::CommPose3d centerPose = box.getPose();
	CommBasicObjects::CommPose3d surfacePose = box.getObjectSurfacePosesCopy()[0];

	mrpt::poses::CPose3D centerPoseMrpt(centerPose.getPosition().getX(), centerPose.getPosition().getY(), centerPose.getPosition().getZ(), centerPose.get_azimuth(), centerPose.get_elevation(), centerPose.get_roll());
	mrpt::poses::CPose3D surfacePoseMrpt(surfacePose.getPosition().getX(), surfacePose.getPosition().getY(), surfacePose.getPosition().getZ(), surfacePose.get_azimuth(), surfacePose.get_elevation(), surfacePose.get_roll());

	mrpt::poses::CPose3D transformationPose = centerPoseMrpt - surfacePoseMrpt;

	CommBasicObjects::CommPosition3d dimensions = box.getDimension();
	double longestDim = dimensions.getZ();
	double mediumDim = dimensions.getX();
	double shortestDim = dimensions.getY();

	double longerSerachSide, shorterSerachSide, objHeight;

	if (transformationPose.yaw() >= 1.5 && transformationPose.yaw() < 1.65){
		std::cout << "longest and shortest side" << std::endl;
		//if visible sides are the longest and the shortest
		longerSerachSide = longestDim;
		shorterSerachSide = shortestDim;
		objHeight = mediumDim;
	}else if (transformationPose.pitch() >= 1.5 && transformationPose.pitch() < 1.65){
		std::cout << "medium and shortest side" << std::endl;
		//if visible sides are the medium and the shortest sides
		longerSerachSide = mediumDim;
		shorterSerachSide = shortestDim;
		objHeight = longestDim;
	}else {
		std::cout << "longest and medium side" << std::endl;
		//if visible sides are the longest and medium sides
		longerSerachSide = longestDim;
		shorterSerachSide = mediumDim;
		objHeight = shortestDim;
	}

//	longerSerachSide = longerSerachSide + longerSerachSide * 0.15;
//	shorterSerachSide = shorterSerachSide + shorterSerachSide * 0.15;
	mrpt::poses::CPose3D freeSpaceSurfacePose = findFreeSpace(longerSerachSide, shorterSerachSide);

	/*
	 * Transform the pose to:
	 * 		x is going into the plane
	 * 		y is parallel to the short side
	 * 		z is parallel to the long side
	 * 	Because of convention
	*/

	std::cout << "---> objHeight: " << objHeight << std::endl;
	std::cout << "---> freeSpaceSurfacePose: " << freeSpaceSurfacePose << std::endl;
	mrpt::poses::CPose3D tcpPose = freeSpaceSurfacePose + mrpt::poses::CPose3D((-1) * (objHeight + 0.007), 0 , 0, 0, 0, 0); //todo: not hard code,  depth of bin from parameter

	return tcpPose;
	//return freeSpaceSurfacePose;

}

