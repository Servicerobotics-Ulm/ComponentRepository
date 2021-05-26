// --------------------------------------------------------------------------
//
//  Copyright (C) 2017 Matthias Rollenhagen
//
//      rollenhagen@hs-ulm.de
//		schlegel@hs-ulm.de
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
//
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//--------------------------------------------------------------------------
#include "DetectionTask.hh"
#include "ComponentKinectBoxDetection.hh"
#include "CommBasicObjects/CommPose3d.hh"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <pcl/pcl_base.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <algorithm>
#include <utility>
#include <cmath>

#include <mrpt/system/CTicTac.h>

#include <opencv4/opencv2/imgproc/types_c.h>
#include <mrpt/math/include/mrpt/math/TLine3D.h>

using namespace cv;
using namespace std;

#define STOP while (std::cin.get() != '\n');

DetectionTask::DetectionTask(SmartACE::SmartComponent *comp) 
:	DetectionTaskCore(comp)
{

	_x_detection_distance_min = COMP->getGlobalState().getGeneral().getX_detection_distance_min() / 1000;
	_x_detection_distance_max = COMP->getGlobalState().getGeneral().getX_detection_distance_max() / 1000;
	_y_detection_distance_min = COMP->getGlobalState().getGeneral().getY_detection_distance_min() / 1000;
	_y_detection_distance_max = COMP->getGlobalState().getGeneral().getY_detection_distance_max() / 1000;
	_z_detection_distance_min = COMP->getGlobalState().getGeneral().getZ_detection_distance_min() / 1000;
	_z_detection_distance_max = COMP->getGlobalState().getGeneral().getZ_detection_distance_max() / 1000;

	_send_obstacle_mesh = COMP->getGlobalState().getGeneral().getSend_obstacle_mesh();

	_use_hsv = COMP->getGlobalState().getGeneral().getUse_hsv();

	// Values for IBU Fiebersaft as default
	COMP->searched_obj_type = "RATIOPHARM-IBU";

	_length_deviation_factor = 0.1;
	_max_box_cosine = 0.2;

	_pointManipulator = PointManipulation();

}
DetectionTask::~DetectionTask() 
{
	std::cout << "destructor DetectionTask\n";
}



int DetectionTask::on_entry()
{
	setEnvId();
	// TODO: Only for testing
	return 0;
}
int DetectionTask::on_execute()
{
	COMP->start_recognition.acquire();
	cout << "[DetectionTask] Starting box detection..." << endl;
	cout << "[DetectionTask] Box distance x from " << _x_detection_distance_min << " m to " << _x_detection_distance_max << " m"<< endl;
	cout << "[DetectionTask] Box distance y from " << _y_detection_distance_min << " m to " << _y_detection_distance_max << " m"<< endl;
	cout << "[DetectionTask] Box distance z from " << _z_detection_distance_min << " m to " << _z_detection_distance_max << " m"<< endl;

	mrpt::system::CTicTac global_stopwatch;
	global_stopwatch.Tic();

	CommBasicObjects::CommVoid v;
	DomainVision::CommRGBDImage rgbd_image;

	// Get new image by query
	Smart::StatusCode status = COMP->kinectQueryClient->query(v, rgbd_image);

	if (status != Smart::SMART_OK) {
		std::cerr << "[DetectionTask] Error in KinectServer communication: " << Smart::StatusCodeConversion(status) << std::endl;
		errorQuit();
		return 0;
	}

	if (!rgbd_image.getIs_valid()) {
		std::cerr << "[DetectionTask] Error: Data is not valid!" << std::endl;
		errorQuit();
		return 0;
	}

	DomainVision::CommVideoImage current_video_image;
	DomainVision::CommDepthImage current_depth_image;

	current_video_image = rgbd_image.getColor_image();
	current_depth_image = rgbd_image.getDepth_image();

	_base_pose = rgbd_image.getBase_state().get_base_position().getPose3D();
	std::cout << "[DetectionTask] Current Base Pose: " << _base_pose << std::endl;

	if(setSearchProperties()){
		std::cout << "[DetectionTask] Search object properties were set to:" << std::endl;
		std::cout << "      type                        : " << _searched_obj.object_type << std::endl;
		std::cout << "      side length                 : " << _searched_obj.sides[0] << " x " << _searched_obj.sides[1] << " x " <<  _searched_obj.sides[2] << std::endl;
		std::cout << "      shelf level (in robot frame): " << _searched_obj.shelf_level_height[0] << " - " << _searched_obj.shelf_level_height[1] << std::endl;
	}else {
		std::cerr << "[DetectionTask] Error: Unknown object type!" << std::endl;
		errorQuit();
		return 0;
	}

	_deleted_boxes.clear();

	_pointManipulator.setMembers(rgbd_image.getSensor_pose(),
			                     current_depth_image.getIntrinsic_mCopy(), current_video_image.getIntrinsic_mCopy(),
								 current_depth_image.getExtrinsic_mCopy(), &current_depth_image, &current_video_image);
	std::cout << "====> Sensor Pose: \n"<< rgbd_image.getSensor_pose() << std::endl;

	vHelper.setPointManipulator(_pointManipulator);

	cv::Mat rgb_matrix = cv::Mat((int)current_video_image.getParameter().height, (int)current_video_image.getParameter().width, CV_8UC3, const_cast< unsigned char*>(current_video_image.get_data()));

	COMP->point_cloud = _pointManipulator.createColoredPointCloud(&current_video_image, false);

	vector<Box> boxes;
	mrpt::system::CTicTac stopwatch;
	float tmp_length_deviation_factor = _length_deviation_factor;
	float tmp_max_box_cosine = _max_box_cosine;

	//try detection with increasing deviation for higher finding probability
	while(tmp_length_deviation_factor <= 0.4) {
		_deleted_boxes.clear();
		std::cout << "deviation factor: " << tmp_length_deviation_factor << std::endl;
		std::cout << "max cosine: " << tmp_max_box_cosine << std::endl;

		//DETECTION
		stopwatch.Tic();
		findRectangles(rgb_matrix, boxes, tmp_length_deviation_factor + 0.2 , tmp_max_box_cosine);
		double detection_time = stopwatch.Tac();
		cout << "[DetectionTask] Detection time: " << detection_time << "s" << endl;
		cout << "[DetectionTask] Detected " << boxes.size() << " candidates." << endl;

		//debug image output show detected rectangles
		DomainVision::CommVideoImage tmp_current_video_image = rgbd_image.getColor_image();
		vHelper.show_box_image(&tmp_current_video_image, boxes);
		//STOP;

		stopwatch.Tic();
		if(_send_obstacle_mesh) {
			createPointClouds(&current_video_image, boxes, COMP->point_cloud.makeShared());
		} else {
			createPointClouds(&current_video_image, boxes);
		}
		double create_pc_time = stopwatch.Tac();
		cout << "[DetectionTask] Point cloud creation time: " << create_pc_time << "s" << endl;

		int box_counter2 = boxes.size();
		eraseEmptyRectangles(boxes);
		cout << "[DetectionTask] eraseEmptyRectangles deleted " << box_counter2 - boxes.size() << " candidates" << endl;

		int box_counter3 = boxes.size();
		eraseFarRectangles(boxes);
		cout << "[DetectionTask] eraseFarRectangles deleted " << box_counter3 - boxes.size() << " candidates" << endl;

		//TODO test place delete from here
		cout << "[DetectionTask] Calculating box poses..." << endl;
		stopwatch.Tic();
		calcRectPose(boxes);
		double pose_calc_time = stopwatch.Tac();
		cout << "[DetectionTask] Box poses calculation time: " << pose_calc_time << "s" << endl;

		int box_counter4 = boxes.size();
		//check box side length in point cloud. Deviation factor is increased, because of bad measurement
		//eraseWrongLenthRecangles(boxes, tmp_length_deviation_factor + 0.1);
		eraseWrongLenthRecangles(boxes, tmp_length_deviation_factor);
		cout << "[DetectionTask] eraseWrongLenthRecangles deleted " << box_counter4 - boxes.size() << " candidates" << endl;

		//debug image output show detected rectangles
		DomainVision::CommVideoImage tmp_current_video_image2 = rgbd_image.getColor_image();
		vHelper.show_box_image(&tmp_current_video_image2, boxes);
		//STOP;

		int box_counter1 = boxes.size();
		eraseDuplicates(boxes);
		cout << "[DetectionTask] eraseDuplicates deleted " << box_counter1 - boxes.size() << " candidates" << endl;

		int box_counter5 = boxes.size();
		eraseNanPoses(boxes);
		cout << "[DetectionTask] eraseFarRectangles deleted " << box_counter5 - boxes.size() << " candidates" << endl;

		int box_counter6 = boxes.size();
		// TODO: Uncomment and use corresponding range filter parameters
		eraseWrongShelfLevelBoxes(boxes);
		cout << "[DetectionTask] eraseWrongShelfLevelBoxes deleted " << box_counter6 - boxes.size() << " candidates" << endl;


		cout << "[DetectionTask] Identified " << boxes.size() << " boxes." << endl;


		setEnvId();

		// Go on when at least one box was detected.
		// When no boxes were found the deviation is increased and the detection
		// will be run again for a higher probability of detecting boxes.
		if (boxes.size() > 0) {
			break;
		} else {
			cout << "[DetectionTask] Increasing search aspect ratio and angle value for higher detection probability." << endl;
			tmp_length_deviation_factor += 0.1;
			tmp_max_box_cosine += 0.05;
		}
	}


	if (boxes.size() <= 0) {
		cout << "[DetectionTask] Finished with 0 findings." << endl;
		vector<Box> boxes;
		setDetectedObjects(boxes);

		//sending obstacle mesh even if no boxes were found
		if(_send_obstacle_mesh) {
			PclPointCloudPtr obstacles_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
			pcl::copyPointCloud(COMP->point_cloud, *obstacles_cloud_ptr);
			stopwatch.Tic();
			setObstacleObject(obstacles_cloud_ptr);
			double mesh_calc_time = stopwatch.Tac();
			cout << "[DetectionTask] Mesh calculation time: " << mesh_calc_time << "s" << endl;
		}

		sendFinishedEvent();
		//errorQuit();
		return 0;
	}

	int box_counter5 = boxes.size();
	eraseNanPoses(boxes);
	cout << "[DetectionTask] eraseFarRectangles deleted " << box_counter5 - boxes.size() << " candidates" << endl;

	DomainVision::CommVideoImage tmp_current_video_image = rgbd_image.getColor_image();
	vHelper.show_box_image(&tmp_current_video_image, boxes);
	//STOP;
	int box_counter6 = boxes.size();
	eraseWrongShelfLevelBoxes(boxes);
	cout << "[DetectionTask] eraseWrongShelfLevelBoxes deleted " << box_counter6 - boxes.size() << " candidates" << endl;


	setObjectIds(boxes);
	rearrangePose(boxes);
	setDetectedObjects(boxes);

	if(_send_obstacle_mesh) {
		PclPointCloudPtr obstacles_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
		obstacles_cloud_ptr->clear();
		pcl::copyPointCloud(COMP->point_cloud, *obstacles_cloud_ptr);
		for(int b = 0; b < boxes.size(); b++) {
			for (uint k = 0; k < boxes[b].getPointCloudIndices().size(); k++) {
				// set points, which are part of a detected object, to NaN in obstacle cloud
				obstacles_cloud_ptr->points[boxes[b].getPointCloudIndices()[k]].x = std::numeric_limits<float>::quiet_NaN();
				obstacles_cloud_ptr->points[boxes[b].getPointCloudIndices()[k]].y = std::numeric_limits<float>::quiet_NaN();
				obstacles_cloud_ptr->points[boxes[b].getPointCloudIndices()[k]].z = std::numeric_limits<float>::quiet_NaN();
			}
		}

		stopwatch.Tic();
		setObstacleObject(obstacles_cloud_ptr);
		double mesh_calc_time = stopwatch.Tac();
		cout << "[DetectionTask] Mesh calculation time: " << mesh_calc_time << "s" << endl;
	}
	sendFinishedEvent();

	PclPointCloudPtr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	//show Obstacle mesh cloud instead of complete colored obstacle cloud
	output_cloud_ptr = COMP->concreteObjects.back().getObstacleHullPointCloud().getPclPointCloudPtr();

	//show model cloud instead of complete colored obstacle cloud
	//output_cloud_ptr = PclPointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>(COMP->point_cloud));

	double global_detection_time = global_stopwatch.Tac();
	cout << "[DetectionTask] Complete detection time: " << global_detection_time << "s" << endl;

	if (_DO_TEST) {
		for (std::list<ConcreteObject>::const_iterator iter = COMP->concreteObjects.begin(); iter != COMP->concreteObjects.end(); iter++) {
			cout << "  ID:" << iter->getId() << endl;
			cout << "  Surface Pose:" << iter->getSurfacePose() << endl;
			cout << "  Object center Pose:" << iter->getPose() << endl;
			cout << "  ---------" << endl;
		}
		vHelper.run_window_test(&rgbd_image, boxes, output_cloud_ptr);
	}

	//delete &_pointManipulator;
	_deleted_boxes.clear();
	boxes.clear();
	return 0;
}
int DetectionTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
void DetectionTask::setObjectIds(vector<Box>& boxes) {
	for (int i = 0; i < boxes.size(); i++) {
		boxes[i].setId(COMP->objectIdCounter++);
	}
}

/*
* Sends a finished event with empty object list
*/
void DetectionTask::errorQuit() {
	vector<Box> boxes;
	setDetectedObjects(boxes);
	sendFinishedEvent();
}

/*
 * Defines the properties, which are needed for the detection
 * Before detection (trigger recognize) the trigger addobject needs
 * to be executed to set the searched object type. Object properties
 * should requested from a database.
 * If object type is unknown false is returned, otherwise true.
 */
bool DetectionTask::setSearchProperties() {
	std::string obj_type_upper = COMP->searched_obj_type;
	std::transform(COMP->searched_obj_type.begin(), COMP->searched_obj_type.end(), obj_type_upper.begin(), ::toupper);
	cout << "[DetectionTask] Set object type " << obj_type_upper << endl;

	bool obj_found = false;

	if (obj_type_upper.compare("RATIOPHARM-IBU") == 0) {
		cout << "[DetectionTask] Setting search properties for RATIOPHARM-IBU" << endl;
		// Values for IBU Fiebersaft
		_searched_obj.sides[0] = 0.135;
		_searched_obj.sides[1] = 0.062;
		_searched_obj.sides[2] = 0.05;
		obj_found = true;
	} else if (obj_type_upper.compare("RATIOPHARM-ASS") == 0) {
		cout << "[DetectionTask] Setting search properties for RATIOPHARM-ASS" << endl;
		// Values for ASS+C
		_searched_obj.sides[0] = 0.08;
		_searched_obj.sides[1] = 0.074;
		_searched_obj.sides[2] = 0.038;
		obj_found = true;
	} else if (obj_type_upper.compare("CORN-FLAKES") == 0) {
		cout << "[DetectionTask] Setting search properties for CORN-FLAKES" << endl;
		// Values for CORN-FLAKES
		_searched_obj.sides[0] = 0.107;
		_searched_obj.sides[1] = 0.074;
		_searched_obj.sides[2] = 0.042;
		obj_found = true;
	} else if (obj_type_upper.compare("WHITE-BOX-CUBE") == 0) {
		cout << "[DetectionTask] Setting search properties for KNOPPERS" << endl;
		// Values for KNOPPERS
		_searched_obj.sides[0] = 0.065;
		_searched_obj.sides[1] = 0.065;
		_searched_obj.sides[2] = 0.065;
		obj_found = true;
	} else if (obj_type_upper.compare("WHITE-BOX-RECT") == 0) {
		cout << "[DetectionTask] Setting search properties for UELTJE" << endl;
		// Values for UELTJE
		_searched_obj.sides[0] = 0.09;
		_searched_obj.sides[1] = 0.08;
		_searched_obj.sides[2] = 0.04;

		obj_found = true;
	}

	if(obj_found){
		_searched_obj.shelf_level_height[0] = COMP->getGlobalState().getSETSEARCHHEIGHT().getMinM() - _base_pose.get_z(1);
		_searched_obj.shelf_level_height[1] = COMP->getGlobalState().getSETSEARCHHEIGHT().getMaxM() - _base_pose.get_z(1);
		_searched_obj.object_type = obj_type_upper;
	}

	//returns false, if no match (object type is unknown)
	return obj_found;
}


/*
 * Inserts the detected objects into the global list,
 * for publishing them via queries.
 */
void DetectionTask::setDetectedObjects(std::vector<Box>& boxes) {

	COMP->concreteObjects.clear();
	//COMP->environmentIdCounter++;

	for (int i = 0; i < boxes.size(); i++) {
		ConcreteObject box;
		//CommBasicObjects::CommPosition3d box_dimension((const double)_searched_obj.sides[1], (const double)_searched_obj.sides[2], (const double)_searched_obj.sides[0]);
		CommBasicObjects::CommPosition3d box_dimension;
		box_dimension.setX(_searched_obj.sides[1]);
		box_dimension.setY(_searched_obj.sides[2]);
		box_dimension.setZ(_searched_obj.sides[0]);

		box.setId(boxes[i].getId());


		cout << "[DetectionTask] visible sites:" << endl;
		cout << "    side ID: " <<  boxes[i].getVisibleSides()[0] << " , length: " << _searched_obj.sides[boxes[i].getVisibleSides()[0]] << endl;
		cout << "    side ID: " <<  boxes[i].getVisibleSides()[1] << " , length: " << _searched_obj.sides[boxes[i].getVisibleSides()[1]] << endl;


		if (boxes[i].getVisibleSides()[0] == 0 && boxes[i].getVisibleSides()[1] == 1){
			cout << "longest and medium side" << endl;
			//if visible sides are the longest and medium sides
			box.setPose(boxes[i].getSurfacePose() + mrpt::poses::CPose3D(_searched_obj.sides[1] / 2, 0, 0, 0, 0, 0));
		}else if (boxes[i].getVisibleSides()[0] == 0 && boxes[i].getVisibleSides()[1] == 2){
			cout << "longest and shortest side" << endl;
			//if visible sides are the longest and the shortest
			box.setPose(boxes[i].getSurfacePose() + mrpt::poses::CPose3D(_searched_obj.sides[2] / 2, 0, 0, 1.57, 0, 0));
		}else{
			cout << "medium and shortest side" << endl;
			//if visible sides are the medium and the shortest sides
			box.setPose(boxes[i].getSurfacePose() + mrpt::poses::CPose3D(_searched_obj.sides[0] / 2, 0, 0, 0, 1.57, 0));
		}

		//box.setPose(boxes[i].getObjCenterPose());
		box.setSurfacePose(boxes[i].getSurfacePose());
		box.setObjectClass(_searched_obj.object_type);

		//setting dimensions is not necessary, since object
		//size is taken from openrave DB.
		box.setDimension(box_dimension);
		COMP->concreteObjects.push_back(box);
	}
}

/*
 * Sets the whole cloud without the detected objects as mesh to the concrete objects list.
 * This will be used for avoiding obstacles during path planning.
 */
void DetectionTask::setObstacleObject(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_cloud) {

	///////////////////////////////////////////
	// filter cloud in x-direction to shrink size for faster calculations and get rid of NaN values

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_cloud_filtered_x(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	if (COMP->getGlobalState().getGeneral().getDo_x_filtering()) {
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (obstacles_cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (_x_detection_distance_min, _x_detection_distance_max);
		cout << "Distance filter x cloud..." << std::flush;
		pass.filter(*obstacles_cloud_filtered_x);
		cout << " Done!" << endl;
		cout << "Cloud size: " << obstacles_cloud_filtered_x->size() <<endl;
	} else {
		obstacles_cloud_filtered_x = obstacles_cloud;
	}

	///////////////////////////////////////////
	// filter cloud in z-direction to shrink size for faster calculations and get rid of NaN values
	// and deleting the floor

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_cloud_filtered_z(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	if (COMP->getGlobalState().getGeneral().getDo_z_filtering()) {
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (obstacles_cloud_filtered_x);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (_z_detection_distance_min, _z_detection_distance_max);
		cout << "Distance filter z cloud..." << std::flush;
		pass.filter(*obstacles_cloud_filtered_z);
		cout << " Done!" << endl;
		cout << "Cloud size: " << obstacles_cloud_filtered_z->size() <<endl;
	} else {
		obstacles_cloud_filtered_z = obstacles_cloud_filtered_x;
	}

	//////////////////////////////////////////////////
	// VoxelGrid filter to reduce the number of points. Additionally filter cloud in y-direction

	PclPointCloudPtr downsampled_cloud(new PclPointCloud);
	pcl::VoxelGrid<PointT> voxel_grid;
	// define filter leaf size 0,5cm
	float leaf_size = 0.005;

	//voxel_grid.setInputCloud(obstacles_cloud_filtered);
	voxel_grid.setInputCloud(obstacles_cloud_filtered_z);
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	if (COMP->getGlobalState().getGeneral().getDo_y_filtering()) {
		voxel_grid.setFilterFieldName("y");
		voxel_grid.setFilterLimits(_y_detection_distance_min, _y_detection_distance_max);
	}
	cout << "Voxel filter cloud..." << std::flush;
	voxel_grid.filter(*downsampled_cloud);
	cout << " Done!" << endl;
	cout << "Cloud size: " << downsampled_cloud->size() << endl;

	///////////////////////////////////////////
	// Create mesh (code from mapper.cc:915)

	pcl::ConcaveHull<pcl::PointXYZRGB> hr;
	hr.setAlpha(0.01);
	hr.setInputCloud(downsampled_cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>());
	vector<pcl::Vertices> vertices;
	//calculate mesh from point cloud
	cout << "Create mesh cloud..." << std::flush;
	hr.reconstruct(*hull_cloud, vertices);
	cout << " Done!" << endl;
	cout << "Mesh cloud size: " << hull_cloud->size() << endl;

	///////////////////////////////////////////
	// Convert mesh into transmission format

	PointCloud cloud_out;
	cloud_out.setPclPointCloud(hull_cloud);

	vector<vector<uint32_t> > vertices_out;
	vertices_out.resize(vertices.size());

	uint32_t vertices_size = vertices[0].vertices.size();
	for (size_t i = 0; i < vertices.size(); ++i) {
		vertices_out[i].resize(vertices_size);
		vertices_out[i][0] = vertices[i].vertices[0];
		vertices_out[i][1] = vertices[i].vertices[1];
		vertices_out[i][2] = vertices[i].vertices[2];
	}

	///////////////////////////////////////////
	// Prepare transmission object

	ConcreteObject obstacle_hull;
	CPose3D zero_pose;
	TPoint3D zero_point;

	obstacle_hull.setMinPoint(zero_point);
	obstacle_hull.setMaxPoint(zero_point);
	obstacle_hull.setPose(zero_pose);

	obstacle_hull.setId(COMP->objectIdCounter++);
	obstacle_hull.setObjectClass("OBSTACLE-HULL");

	ObjectBeliefs belief_hull;
	belief_hull.setBelief("OBSTACLE-HULL", 1.0);
	obstacle_hull.setBeliefs(belief_hull);

	obstacle_hull.setObstacleHullPointCloud(cloud_out);
	obstacle_hull.setObstacleHullvertices(vertices_out);

	COMP->concreteObjects.push_back(obstacle_hull);
}

void DetectionTask::setEnvId() {
	//TODO parameters are missing
//	std::cout << "setting env id: " << COMP->getGlobalState().getCommObjectRecognitionObjects().getObjectRecognitionParameter().getSETENVID().getId() << std::endl;
//	COMP->environmentIdCounter = COMP->getGlobalState().getCommObjectRecognitionObjects().getObjectRecognitionParameter().getSETENVID().getId();
COMP->environmentIdCounter =0;
//	std::cout << "setting obj id: " << COMP->getGlobalState().getCommObjectRecognitionObjects().getObjectRecognitionParameter().getSETENVID().getId() << std::endl;
//	COMP->objectIdCounter = COMP->getGlobalState().getCommObjectRecognitionObjects().getObjectRecognitionParameter().getSETOBJECTID().getId();
COMP->objectIdCounter = 0;
}



/*
 * Prepares and sets an event for signaling the completion of detection
 */
void DetectionTask::sendFinishedEvent() {
	CommObjectRecognitionObjects::CommObjectRecognitionEventState event;

	event.set_environment_id(COMP->environmentIdCounter);
	event.set_object_id_size(COMP->concreteObjects.size());

	uint32_t index = 0;
	for (std::list<ConcreteObject>::const_iterator iter =
			COMP->concreteObjects.begin(); iter != COMP->concreteObjects.end(); iter++) {
				event.set_object_id(index, iter->getId());
				index++;
			}

	COMP->objectEventServer->put(event);
	std::cout << "[DetectionTask]" << " Sent " << event.get_object_id_size()
			<< " object IDs by event." << endl;
}

/*
 * Calculates aspect ratio of a rectangle according to four points
 */
double DetectionTask::calcRectRatio(std::vector<Point> points) {
	double delta_x = points.at(0).x - points.at(1).x;
	double delta_y = points.at(0).y - points.at(1).y;
	double dist_a = sqrt(delta_x * delta_x + delta_y * delta_y);

	delta_x = points.at(1).x - points.at(2).x;
	delta_y = points.at(1).y - points.at(2).y;
	double dist_b = sqrt(delta_x * delta_x + delta_y * delta_y);

	delta_x = points.at(2).x - points.at(3).x;
	delta_y = points.at(2).y - points.at(3).y;
	double dist_c = sqrt(delta_x * delta_x + delta_y * delta_y);

	delta_x = points.at(3).x - points.at(0).x;
	delta_y = points.at(3).y - points.at(0).y;
	double dist_d = sqrt(delta_x * delta_x + delta_y * delta_y);

	double shortest_side = MIN(dist_a, dist_b);
	shortest_side = MIN(dist_c, shortest_side);
	shortest_side = MIN(dist_d, shortest_side);

	double longest_side = MAX(dist_a, dist_b);
	longest_side = MAX(dist_c, longest_side);
	longest_side = MAX(dist_d, longest_side);

	return longest_side / shortest_side;
}

/*
 * Finds a cosine of angle between vectors
 * from pt0->pt1 and from pt0->pt2
 */
double DetectionTask::angle(Point pt1, Point pt2, Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2)
			/ sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}


/*
 * This function sorts the boxes according to its grippability (high IDs are grippable)
 * The function expects the usage of a vacuum gripper with a diameter i.e. 10 cm and checks,
 * if the box is close to a wall. Since the gripper can not reach it then.
 * A point is defined some cm above the midpoint of the boxes surface pose.
 * Around this point all points with a defined radius (dependet on gripper) are found by a KD-tree
 * The boxes surface plane is defined some cm above the real one and the rectangular distance from
 * the point to the plane is calculated. If this distance is small (i.e. 5 mm) it is likely, that
 * there is an object / wall close to the box.
 */
void DetectionTask::sortBoxesForVacuumGripper(vector<Box>& boxes) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = COMP->point_cloud.makeShared();
	std::vector< std::vector<int> > numberOfCloseNeighours;

	for (int i = 0; i < boxes.size(); i++) {
		pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
		kdtree.setInputCloud (cloud);


		pcl::PointXYZRGB searchPoint(0, 0, 0);

		//defining a pose 3 cm above the box surface
		mrpt::poses::CPose3D planePose(boxes[i].getSurfacePose() + mrpt::poses::CPose3D(-0.03, 0.0, 0.0, 0.0, 0.0, 0.0));

		searchPoint.x = planePose.x();
		searchPoint.y = planePose.y();
		searchPoint.z = planePose.z();


		//defining a plane by 3 points of the pose (y and z axis are lieing in the plane)
		mrpt::math::TPoint3D planePoint1(planePose.x(), planePose.y(), planePose.z());

		mrpt::poses::CPose3D tmpPlanePose1(planePose + mrpt::poses::CPose3D(0.0, 0.3, 0.0, 0.0, 0.0, 0.0));
		mrpt::math::TPoint3D planePoint2(tmpPlanePose1.x(), tmpPlanePose1.y(), tmpPlanePose1.z());

		mrpt::poses::CPose3D tmpPlanePose2(planePose + mrpt::poses::CPose3D(0.0, 0.0, 0.13, 0.0, 0.0, 0.0));
		mrpt::math::TPoint3D planePoint3(tmpPlanePose2.x(), tmpPlanePose2.y(), tmpPlanePose2.z());

		mrpt::math::TPlane plane(planePoint1, planePoint2, planePoint3);

		// Neighbors within radius search
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		//find points in radius of 5 cm
		double radius = 0.05;

		long numberOfNeighours = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

		numberOfCloseNeighours.push_back(std::vector<int>());
		//setting box id
		numberOfCloseNeighours[i].push_back(boxes[i].getId());
		numberOfCloseNeighours[i].push_back(0);

		if ( numberOfNeighours > 0) {
			for (size_t p = 0; p < pointIdxRadiusSearch.size(); ++p){
				//std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x	<< " " << cloud->points[pointIdxRadiusSearch[i]].y << " " << cloud->points[pointIdxRadiusSearch[i]].z << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
				double planeDistance = plane.distance(mrpt::math::TPoint3D(cloud->points[pointIdxRadiusSearch[p]].x, cloud->points[pointIdxRadiusSearch[p]].y, cloud->points[pointIdxRadiusSearch[p]].z));
				//std::cout << "plane dist: " << planeDistance << std::endl;
				if(planeDistance < 0.005){
					//std::cout << "    " << cloud->points[pointIdxRadiusSearch[p]].x	<< " " << cloud->points[pointIdxRadiusSearch[p]].y << " " << cloud->points[pointIdxRadiusSearch[p]].z << " (squared distance: " << pointRadiusSquaredDistance[p] << ")" << std::endl;
					numberOfCloseNeighours[i][1]++;
				}

			}
		}

		std::cout << "----------------------------" << std::endl;
		std::cout << "Neighbors within radius search at (" << searchPoint.x	<< " " << searchPoint.y << " " << searchPoint.z	<< ") with radius=" << radius << std::endl;
		std::cout << "Surface pose: " << boxes[i].getSurfacePose() << std::endl;

		std::cout << "Number of neighbours: " << numberOfNeighours << std::endl;
		std::cout << "Number of close neighbours: " << numberOfCloseNeighours[i][1] << std::endl;
		std::cout << "----------------------------" << std::endl;
	}

	std::sort(numberOfCloseNeighours.begin(), numberOfCloseNeighours.end(),
	          [](const std::vector<int>& a, const std::vector<int>& b) {
					return a[1] > b[1];
				});



	std::cout << "===========================" << std::endl;
	std::cout << "Inverted Grasp Order (grasp last first):" << std::endl;
	for (int i = 0; i < boxes.size(); i++) {
		std::cout << "   box ID: " <<  numberOfCloseNeighours[i][0] << ", close neighbours: " << numberOfCloseNeighours[i][1] << std::endl;
	}
	std::cout << "===========================" << std::endl;

}


/*
 * Apply RANSAC plane fitting on every rectangle point cloud to erase outlaying points,
 * and to calculate the slope (roll, yaw angle) of the box, for later gripping.
 * The calculated slopes are combined with the center of the box to a pose
 * and added to the corresponding object.
 */
void DetectionTask::calcRectPose(vector<Box>& boxes) {

	for (uint i = 0; i < boxes.size(); i++) {
		std::vector<int> inliers;
		mrpt::poses::CPose3D pose;
		TPose3D tmp;
		bool failed = false;

		//////////////////////////////////////////
		// Use normals for plane fitting.
		// Not used, because of performace issues.

//		pcl::PointCloud<pcl::Normal>::Ptr normals_out(new pcl::PointCloud<pcl::Normal>);
//		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
//		//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//		//norm_est.setSearchMethod(tree);
//		norm_est.setRadiusSearch (0.02);
//		norm_est.setInputCloud (boxes[i].point_cloud);
//		norm_est.setSearchSurface (boxes[i].point_cloud);
//		norm_est.compute (*	normals_out);
//		pcl::SampleConsensusModelNormalPlane<pcl::PointXYZRGB, pcl::Normal>::Ptr model(new pcl::SampleConsensusModelNormalPlane<pcl::PointXYZRGB, pcl::Normal> (boxes[i].point_cloud, normals_out));
//		model->setInputNormals(normals_out);
//		// Set the normal angular distance weight.
//		model->setNormalDistanceWeight(0.5f);
//		// Create the RANSAC object
//		pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model, 1.0);
//
//		cout << ">> Computing model..." << endl;
//		bool result = sac.computeModel ();
//		cout << ">> Finished computing model." << endl;
//		sac.getInliers(inliers);
//
//		Eigen::VectorXf coeff;
//		sac.getModelCoefficients(coeff);
//
//		mrpt::math::TPlane plane((double)coeff[0], (double)coeff[1], (double)coeff[2], (double)coeff[3]);
//
//		plane.getAsPose3D(pose);
//
//		//set pose position to rect center
//		Point center = boxes.at(i).getRectCenter();
//		float x_in_m, y_in_m, z_in_m;
//		calcPointXYZ(kinect_image, center.y, center.x, x_in_m, y_in_m, z_in_m);
//		pose.x(x_in_m);
//		pose.y(y_in_m);
//		pose.z(z_in_m);
//
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//		pcl::copyPointCloud<pcl::PointXYZRGB>(*boxes[i].point_cloud, inliers, *final_cloud);
//		pcl::copyPointCloud<pcl::PointXYZRGB>(*final_cloud, inliers, *boxes[i].point_cloud);

		// Doing ransac plane fitting several times, to get rid of outliers.
		// Several times is necessary, because points are tested randomly.
		// Tests showed, that after 5 times ransac typically no more outliers are found.
		for (int k = 0; k < 5; k++) {
			//std::cout << "Point cloud size box no. " << i << ": " << boxes[i].getPointCloud()->size() << std::endl;

			if (boxes[i].getPointCloud()->size() < 50) {
				_deleted_boxes.resize(_deleted_boxes.size() + 1);
				std::cout << "Deleting box, because point cloud is too small" << std::endl;
				std::move(boxes.begin() + i , boxes.begin() + i + 1, _deleted_boxes.end() - 1);

				boxes.erase(boxes.begin() + i);
				i--;
				failed = true;
				break;
			}


			try {

			// created RandomSampleConsensus object and compute the appropriated model
			pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p(
					new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(
							boxes[i].getPointCloud()));
			pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_p);
			ransac.setDistanceThreshold(.01);
			//ransac.setMaxIterations(1000);
			ransac.computeModel();
			ransac.getInliers(inliers);

			Eigen::VectorXf coeff;
			ransac.getModelCoefficients(coeff);

			mrpt::math::TPlane plane((double) coeff[0], (double) coeff[1],
					(double) coeff[2], (double) coeff[3]);
					
			// Timo - 20.04
			//---------------
			// new:	
			//TPose3D tmp = pose.asTPose();
			plane.getAsPose3D(tmp);
			std::cout << "Box Plane pose : "<< tmp.x << ", "<< tmp.y << ", "<< tmp.z<<", "
										<< tmp.yaw << ", "<< tmp.pitch << ", "<< tmp.roll<< std::endl;
			// old:
			// plane.getAsPose3D(pose);
			//---------------

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(
					new pcl::PointCloud<pcl::PointXYZRGB>());
			pcl::copyPointCloud<pcl::PointXYZRGB>(*boxes[i].getPointCloud(),
					inliers, *final_cloud);
			pcl::copyPointCloud<pcl::PointXYZRGB>(*final_cloud, inliers,
					*boxes[i].getPointCloud());

			} catch (...) {
				_deleted_boxes.resize(_deleted_boxes.size() + 1);
				std::cout << "Deleting box, because point cloud is too small" << std::endl;
				std::move(boxes.begin() + i , boxes.begin() + i + 1, _deleted_boxes.end() - 1);

				boxes.erase(boxes.begin() + i);
				i--;
				failed = true;
				break;
			}
		}

		if(failed == false){
			//set pose position to rect center
			Point center = boxes[i].getRectCenter();
			float x_in_m, y_in_m, z_in_m;
			bool found_valid_xyz = false;

			//sometimes there is no x,y,z value for a specific RGB point available
			//increase / decrease row and col values to find a valide point cloud value
			for(int i = 0; i < 5; i++){
				uint32_t r = center.y + i, c = center.x + i;
				uint32_t r_neg = center.y - i, c_neg = center.x - i;

				_pointManipulator.pixelToXyz(r, c, x_in_m, y_in_m, z_in_m, true);
				if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
					found_valid_xyz = true;
					break;
				}

				_pointManipulator.pixelToXyz(r_neg, c_neg, x_in_m, y_in_m, z_in_m, true);
				if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
					found_valid_xyz = true;
					break;
				}
			}

			if(!found_valid_xyz){
				continue;
			}

			pose.x(x_in_m);
			pose.y(y_in_m);
			pose.z(z_in_m);
			pose.setYawPitchRoll(tmp.yaw, tmp.pitch, tmp.roll);

			boxes[i].setSurfacePose(pose);
		    std::cout << " plane cpose : " <<pose.asString() << std::endl;
		}
	}
}


/*
 * Checks whether the detected boxes are between the min ande max values
 * of the corresponding shelf level. If not they get deleted
 */
void DetectionTask::eraseWrongShelfLevelBoxes(vector<Box>& boxes) {
//float shelfHeightMin = _searched_obj.shelf_level_height[0];
//	float shelfHeightMax = _searched_obj.shelf_level_height[1];
	float shelfHeightMin = 0.0;
	float shelfHeightMax = 2.0;

	std::cout << "shelf height: " << shelfHeightMin << " - " << shelfHeightMax << std::endl;
	for (int i = 0; i < boxes.size(); i++) {

//Nayab		std::cout << boxes[i].getSurfacePose().z() << std::endl;
//Nayab		STOP

		if (boxes[i].getSurfacePose().z() < shelfHeightMin || boxes[i].getSurfacePose().z() > shelfHeightMax ) {
			std::cout << "box surfacepose: " <<  boxes[i].getSurfacePose()  << std::endl;
			_deleted_boxes.resize(_deleted_boxes.size() + 1);
			std::cout << "--> Deleting box point 0: " << boxes[i].getPoints()[0] << std::endl;
			std::move(boxes.begin() + i , boxes.begin() + i + 1, _deleted_boxes.end() - 1);

			boxes.erase(boxes.begin() + i);
			i--;

		}
	}
}

/*
 * Checks whether the real side lengths (in 3D) of the detected objects are
 * equal to the searched lengths, because the detection is done in 2D. If the
 * side lengths are not equal, the corresponding rectangle is deleted from the vector.
 */
void DetectionTask::eraseWrongLenthRecangles(vector<Box>& boxes,
		float deviation_factor) {


	// create min and max values of the side lengths for preventing wrong decisions
	// due to measurement errors.
	vector<float> max_deviations;
	max_deviations.push_back(_searched_obj.sides[0] * deviation_factor);
	max_deviations.push_back(_searched_obj.sides[1] * deviation_factor);
	max_deviations.push_back(_searched_obj.sides[2] * deviation_factor);

	for (int i = 0; i < boxes.size(); i++) {

////////////////////////////////////////
// old rectangel side length calculation

//		//vector<mrpt::math::TPoint3D> points3d;
//		vector<mrpt::poses::CPoint3D> points3d;
//		float x_in_m, y_in_m, z_in_m;
//
//		// create 3D points of the rectangle corners
//		for (int k = 0; k < 4; k++) {
//			//sometimes there is no x,y,z value for a specific RGB point available
//			//increase / decrease row and col values to find a valide point cloud value
//			bool found_valid_xyz = false;
//			for(int s = 0; s < 4; s++){
//				uint32_t r, c;
//				if(boxes[i].getPoints()[k].x < boxes[i].getRectCenter().x){
//					c = boxes[i].getPoints()[k].x + s;
//				}else {
//					c = boxes[i].getPoints()[k].x - s;
//				}
//
//				if(boxes[i].getPoints()[k].y < boxes[i].getRectCenter().y){
//					r = boxes[i].getPoints()[k].y + s;
//				}else {
//					r = boxes[i].getPoints()[k].y - s;
//				}
//
//				_pointManipulator.pixelToXyz(r, c, x_in_m, y_in_m, z_in_m, true);
//				if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
//					found_valid_xyz = true;
//					break;
//				}
//			}
//			points3d.push_back(mrpt::math::TPoint3D(x_in_m, y_in_m, z_in_m));
//		}
//
//		float rect_side_length1 = points3d[0].distance3DTo( points3d[1].x(), points3d[1].y(), points3d[1].z());
//		float rect_side_length2 = points3d[0].distance3DTo( points3d[3].x(), points3d[3].y(), points3d[3].z());;

////////////////////////////////////////



////////////////////////////////////////
// find surface cloud corner points
// points with the highes distance to the rect center are the corner points

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud = boxes[i].getPointCloud();

		/////////////////////////////
		mrpt::math::TPoint3D box_midpoint(boxes[i].getSurfacePose().x(), boxes[i].getSurfacePose().y(), boxes[i].getSurfacePose().z());

		//sort points by distance to surface midpoint
		std::sort(box_cloud->points.begin(), box_cloud->points.end(),
				[&box_midpoint](pcl::PointXYZRGB a, pcl::PointXYZRGB b) {
			mrpt::math::TPoint3D tmp_point_a(a.x, a.y, a.z);
			mrpt::math::TPoint3D tmp_point_b(b.x, b.y, b.z);
			double dist_a = box_midpoint.distanceTo(tmp_point_a);
			double dist_b = box_midpoint.distanceTo(tmp_point_b);

			if(tmp_point_a.x == 0.0 || tmp_point_a.y == 0.0 || tmp_point_a.z == 0.0 ){return false;}
			if(tmp_point_b.x == 0.0 || tmp_point_b.y == 0.0 || tmp_point_b.z == 0.0 ){return true;}

			return dist_a > dist_b;
		});

		//the point with highest distance to mid point is probably a corner point
		vector<int> corner_point_idxs{0};
		//corner points must have min distance of the shortest side of the object
		double min_corner_dist = _searched_obj.sides[2] * 0.7;

		for(int j = 0; j<box_cloud->size(); j++){
			mrpt::math::TPoint3D tmp_point(box_cloud->points[j].x, box_cloud->points[j].y, box_cloud->points[j].z);
			bool too_close = false;

			for(int k = 0; k<corner_point_idxs.size(); k++){
				mrpt::math::TPoint3D tmp_corner_point(box_cloud->points[corner_point_idxs[k]].x, box_cloud->points[corner_point_idxs[k]].y, box_cloud->points[corner_point_idxs[k]].z) ;
				double tmp_dist = tmp_point.distanceTo(tmp_corner_point);

				//if current point is close to the last one it is probably
				//in the same corner, so try the next one
				if(tmp_dist < min_corner_dist){
					too_close = true;
					break;
				}
			}

			if(!too_close){
				corner_point_idxs.push_back(j);
				if (corner_point_idxs.size() == 4){
					j = box_cloud->size();
				}
			}

		}

		std::vector<mrpt::poses::CPoint3D> corner_points;

		for(int c = 0; c<corner_point_idxs.size(); c++ ){
			box_cloud->points[corner_point_idxs[c]].r = 250;
			box_cloud->points[corner_point_idxs[c]].g = 0;
			box_cloud->points[corner_point_idxs[c]].b = 0;
			cout << "Corner point: " << corner_point_idxs[c] << " , " << box_cloud->points[corner_point_idxs[c]] << endl;
			corner_points.push_back(mrpt::poses::CPoint3D(box_cloud->points[corner_point_idxs[c]].x, box_cloud->points[corner_point_idxs[c]].y, box_cloud->points[corner_point_idxs[c]].z));
		}

////////////////////////////////////////////////////

///////////////////////////////////////////////////
//calculate the side lengthes

		std::vector<double> point_dists;
		//calculate the distances from each corner point to each other
		//the longest two are the diagonal lines of the rectangel
		point_dists.push_back(corner_points[0].distance3DTo( corner_points[1].x(), corner_points[1].y(), corner_points[1].z()));
		point_dists.push_back(corner_points[0].distance3DTo( corner_points[2].x(), corner_points[2].y(), corner_points[2].z()));
		point_dists.push_back(corner_points[0].distance3DTo( corner_points[3].x(), corner_points[3].y(), corner_points[3].z()));
		point_dists.push_back(corner_points[1].distance3DTo( corner_points[2].x(), corner_points[2].y(), corner_points[2].z()));
		point_dists.push_back(corner_points[1].distance3DTo( corner_points[3].x(), corner_points[3].y(), corner_points[3].z()));
		point_dists.push_back(corner_points[2].distance3DTo( corner_points[3].x(), corner_points[3].y(), corner_points[3].z()));

		std::vector<double> point_dists_bak(point_dists);

//		for (int d = 0 ; d < point_dists.size(); d++){
//			std::cout << "==> point dist (unsorted): " << point_dists[d] << std::endl;
//		}

		//sort the distances (smallest ist index 0)
		std::sort(point_dists.begin(), point_dists.end());


		float rect_side_length1 = point_dists[1]; //longer of the short sides because edges of the surface cloud are cut
		float rect_side_length2 = point_dists[3]; //longer of the long sides because edges of the surface cloud are cut

		std::vector<double>::iterator short_side_it = std::find(point_dists_bak.begin(), point_dists_bak.end(), point_dists[1]);
		int short_side_index = std::distance(point_dists_bak.begin(), short_side_it);

		std::vector<double>::iterator long_side_it = std::find(point_dists_bak.begin(), point_dists_bak.end(), point_dists[3]);
		int long_side_index = std::distance(point_dists_bak.begin(), long_side_it);

		std::cout << "==> short side index:" << short_side_index << std::endl;
		std::cout << "==> long side index:" << long_side_index << std::endl;

		mrpt::poses::CPoint3D midpoint_short_side;

		if(short_side_index == 0){
			midpoint_short_side.x((corner_points[0].x() + corner_points[1].x()) / 2);
			midpoint_short_side.y((corner_points[0].y() + corner_points[1].y()) / 2);
			midpoint_short_side.z((corner_points[0].z() + corner_points[1].z()) / 2);
		}else if(short_side_index == 1){
			midpoint_short_side.x((corner_points[0].x() + corner_points[2].x()) / 2);
			midpoint_short_side.y((corner_points[0].y() + corner_points[2].y()) / 2);
			midpoint_short_side.z((corner_points[0].z() + corner_points[2].z()) / 2);
		}else if(short_side_index == 2){
			midpoint_short_side.x((corner_points[0].x() + corner_points[3].x()) / 2);
			midpoint_short_side.y((corner_points[0].y() + corner_points[3].y()) / 2);
			midpoint_short_side.z((corner_points[0].z() + corner_points[3].z()) / 2);
		}else if(short_side_index == 3){
			midpoint_short_side.x((corner_points[1].x() + corner_points[2].x()) / 2);
			midpoint_short_side.y((corner_points[1].y() + corner_points[2].y()) / 2);
			midpoint_short_side.z((corner_points[1].z() + corner_points[2].z()) / 2);
		}else if(short_side_index == 4){
			midpoint_short_side.x((corner_points[1].x() + corner_points[3].x()) / 2);
			midpoint_short_side.y((corner_points[1].y() + corner_points[3].y()) / 2);
			midpoint_short_side.z((corner_points[1].z() + corner_points[3].z()) / 2);
		}else if(short_side_index == 5){
			midpoint_short_side.x((corner_points[2].x() + corner_points[3].x()) / 2);
			midpoint_short_side.y((corner_points[2].y() + corner_points[3].y()) / 2);
			midpoint_short_side.z((corner_points[2].z() + corner_points[3].z()) / 2);
		}

//		pcl::PointXYZRGB midpoint_pcl(0, 255, 0);
//		midpoint_pcl.x = midpoint_short_side.x();
//		midpoint_pcl.y = midpoint_short_side.y();
//		midpoint_pcl.z = midpoint_short_side.z();
//
//		std::cout << "==> midpoint pcl:" << midpoint_pcl << std::endl;
//
//
//		boxes[i].getPointCloud()->push_back(midpoint_pcl);

///////////////////////////////////////////////////


		std::cout << "-----------------------------------" << std::endl;
		std::cout << "measured sites: " << std::endl;
		std::cout << "    " << rect_side_length1 << " , " << rect_side_length2 << std::endl;


		int fitting_sides = 0;
		vector<int> visible_side_ids;
		vector<float> min_deltas;
		vector< vector<float> > real_deltas;
		real_deltas.resize(2);

		for (int k = 0; k < 3; k++) {
			float real_delta1;
			float real_delta2;

			real_delta1 = abs(rect_side_length1 - _searched_obj.sides[k]);
			real_delta2 = abs(rect_side_length2 - _searched_obj.sides[k]);

			if(real_delta1 > max_deviations[k]){
				real_delta1 = 10000;
			}
			if(real_delta2 > max_deviations[k]){
				real_delta2 = 10000;
			}

			real_deltas[0].push_back(real_delta1);
			real_deltas[1].push_back(real_delta2);

		}


		cout << "deviation factor: " << deviation_factor << endl;
		cout << "searched sides: " << _searched_obj.sides[0] << ", " << _searched_obj.sides[1] << ", " << _searched_obj.sides[2] << endl;
		cout << "max deviations: " << max_deviations[0] << ", " << max_deviations[1] << ", " << max_deviations[2] << endl;

		cout << "real deltas" << endl;
		for (int j = 0; j < 2; j++) {
			cout << "" << endl;
			for (int k = 0; k < 3; k++) {
				cout << "  " << real_deltas[j][k];
			}
		}
		cout << "" << endl;


		float smallest_delta = 100000;
		int visible_side_id1 = -1, visible_side_id2 = -1;
		vector< vector<float> > summed_deltas;
		summed_deltas.resize(3);

		for (int j = 0; j < 3; j++) {
			cout << "" << endl;
			for (int k = 0; k < 3; k++) {
				if(j == k ){
					summed_deltas[j].push_back(1000);
					cout << "  " << summed_deltas[j][summed_deltas[j].size() - 1];

					continue;
				}

				summed_deltas[j].push_back(real_deltas[0][j] + real_deltas[1][k]);

				cout << "  " << summed_deltas[j][summed_deltas[j].size() - 1];

				if(smallest_delta > summed_deltas[j][k]){
					smallest_delta = summed_deltas[j][k];
					visible_side_id1 = j;
					visible_side_id2 = k;
				}
			}
		}
		cout << "" << endl;



		if (visible_side_id1 < 0 || visible_side_id2 < 0 || smallest_delta > 1.0) {
			_deleted_boxes.resize(_deleted_boxes.size() + 1);
			std::cout << "--> Deleting box point 0: " << boxes[i].getPoints()[0]<< std::endl;
			std::move(boxes.begin() + i, boxes.begin() + i + 1,	_deleted_boxes.end() - 1);

			boxes.erase(boxes.begin() + i);
			i--;
			std::cout << "Deleted Box point 0: " << _deleted_boxes[_deleted_boxes.size() - 1].getPoints()[0] << std::endl;
			continue;
		}



		//for finding orientation of the box, the visible sides are searched

		boxes[i].setVisibleSides(visible_side_id1, visible_side_id2);

		cout << "set visible sites:" << endl;
		cout << "    side ID: " <<  boxes[i].getVisibleSides()[0] << " , length: " << _searched_obj.sides[boxes[i].getVisibleSides()[0]] << endl;
		cout << "    side ID: " <<  boxes[i].getVisibleSides()[1] << " , length: " << _searched_obj.sides[boxes[i].getVisibleSides()[1]] << endl;


	}

}


/*
 * Checks whether the real side lengths (in 3D) of the detected objects are
 * equal to the searched lengths, because the detection is done in 2D. If the
 * side lengths are not equal, the corresponding rectangle is deleted from the vector.
 */
/*
void DetectionTask::eraseWrongLenthRecangles_original(vector<Box>& boxes,
		float deviation_factor) {
	// create min and max values of the side lengths for preventing wrong decisions
	// due to measurement errors.
	vector<vector<float> > sides;
	sides.resize(3);
	sides[0].push_back(
			_searched_obj.sides[0] + _searched_obj.sides[0] * deviation_factor);
	sides[0].push_back(
			_searched_obj.sides[0] - _searched_obj.sides[0] * deviation_factor);
	sides[1].push_back(
			_searched_obj.sides[1] + _searched_obj.sides[1] * deviation_factor);
	sides[1].push_back(
			_searched_obj.sides[1] - _searched_obj.sides[1] * deviation_factor);
	sides[2].push_back(
			_searched_obj.sides[2] + _searched_obj.sides[2] * deviation_factor);
	sides[2].push_back(
			_searched_obj.sides[2] - _searched_obj.sides[2] * deviation_factor);


	for (int i = 0; i < boxes.size(); i++) {
		vector<mrpt::math::TPoint3D> points3d;
		float x_in_m, y_in_m, z_in_m;

		// create 3D points of the rectangle corners
		for (int k = 0; k < 3; k++) {
			uint32_t r = boxes[i].getPoints()[k].y, c =	boxes[i].getPoints()[k].x;
			_pointManipulator.pixelToXyz(r, c, x_in_m, y_in_m, z_in_m, true);
			points3d.push_back(mrpt::math::TPoint3D(x_in_m, y_in_m, z_in_m));
		}

		int dx, dy;
		// calculate length of plane's top side by Pythagoras' theorem
		dx = boxes[i].getPoints()[0].x -boxes[i].getPoints()[1].x;
		dy = fabs(boxes[i].getPoints()[0].y - boxes[i].getPoints()[1].y);
		float length_top_side = sqrt(pow(dx, 2) + pow(dy, 2));

		// calculate length of plane's left side by Pythagoras' theorem
		dx = boxes[i].getPoints()[0].x - boxes[i].getPoints()[3].x;
		dy = fabs(boxes[i].getPoints()[0].y - boxes[i].getPoints()[3].y);
		float length_left_side = sqrt(pow(dx, 2) + pow(dy, 2));

		float rect_side_length1 = length_top_side / 1000;
		float rect_side_length2 = length_left_side / 1000;

		// calculate the side lengths of the rectangle
		//float rect_side_length1 = points3d[0].distanceTo(points3d[1]);
		//float rect_side_length2 = points3d[1].distanceTo(points3d[2]);

		std::cout << "-----------------------------------" << std::endl;
		std::cout << "measured sites: " << std::endl;
		std::cout << "    " << rect_side_length1 << " , " << rect_side_length2 << std::endl;


		int fitting_sides = 0;
		vector<int> visible_side_ids;
		vector<float> min_deltas;

		for (int k = 0; k < 3; k++) {
			float min_delta = -1;

			if (rect_side_length1 < sides[k][0]	&& rect_side_length1 > sides[k][1]) {
				fitting_sides++;
				min_delta = min(sides[k][0] - rect_side_length1, rect_side_length1 - sides[k][1]);
			}
			if (rect_side_length2 < sides[k][0] && rect_side_length2 > sides[k][1]) {
				float min_delta_side2 = min(sides[k][0] - rect_side_length2, rect_side_length2 - sides[k][1]);

				if(fitting_sides > k){
					min_delta = min(min_delta_side2, min_delta);
				}else {
					fitting_sides++;
					min_delta = min_delta_side2;
				}
			}

			if(min_delta > 0.0){
				min_deltas.push_back(min_delta);
				visible_side_ids.push_back(k);
			}

		}


		//for finding orientation of the box, the visible sides are searched
		if (visible_side_ids.size() < 2) {
			_deleted_boxes.resize(_deleted_boxes.size() + 1);
			std::cout << "--> Deleting box point 0: " << boxes[i].getPoints()[0]<< std::endl;
			std::move(boxes.begin() + i, boxes.begin() + i + 1,	_deleted_boxes.end() - 1);

			boxes.erase(boxes.begin() + i);
			i--;
			std::cout << "Deleted Box point 0: " << _deleted_boxes[_deleted_boxes.size() - 1].getPoints()[0] << std::endl;
			continue;
		}else if (visible_side_ids.size() == 3) {
			if (min_deltas[0] < min_deltas[2] && min_deltas[1] < min_deltas[2]) {
				boxes[i].setVisibleSides(visible_side_ids[0], visible_side_ids[1]);
			} else if (min_deltas[0] < min_deltas[1] && min_deltas[2] < min_deltas[1]) {
				boxes[i].setVisibleSides(visible_side_ids[0], visible_side_ids[2]);
			} else {
				boxes[i].setVisibleSides(visible_side_ids[1],visible_side_ids[2]);
			}

		}else{
			boxes[i].setVisibleSides(visible_side_ids[0], visible_side_ids[1]);
		}


		std::cout << "visible sites: " << std::endl;
		for(int a = 0; a < visible_side_ids.size(); a++){
			std::cout << "    " << visible_side_ids[a] << " , min delta: " << min_deltas[a] << std::endl;
		}
		cout << "set visible sites:" << endl;
		cout << "    side ID: " <<  boxes[i].getVisibleSides()[0] << " , length: " << _searched_obj.sides[boxes[i].getVisibleSides()[0]] << endl;
		cout << "    side ID: " <<  boxes[i].getVisibleSides()[1] << " , length: " << _searched_obj.sides[boxes[i].getVisibleSides()[1]] << endl;


	}

}
*/

/*
 * Checks whether the point cloud contains enough objects. Too few points
 * indicate measuring errors in the rectangle area. If cloud is empty or
 * too small, the corresponding rectangle is deleted.
*/
void DetectionTask::eraseNanPoses(vector<Box>& boxes) {
	for (int i = 0; i < boxes.size(); i++) {
		if (isnan(boxes[i].getSurfacePose().x())
				|| isnan(boxes[i].getSurfacePose().y())
				|| isnan(boxes[i].getSurfacePose().z())
				|| isnan(boxes[i].getSurfacePose().yaw())
				|| isnan(boxes[i].getSurfacePose().pitch())
				|| isnan(boxes[i].getSurfacePose().roll())) {

			_deleted_boxes.resize(_deleted_boxes.size() + 1);
			std::cout << "--> Deleting box point 0: " << boxes[i].getPoints()[0] << std::endl;
			std::move(boxes.begin() + i , boxes.begin() + i + 1, _deleted_boxes.end() - 1);

			boxes.erase(boxes.begin() + i);
			i--;
		}
	}
}

/*
 * Checks whether the point cloud contains enough objects. Too few points
 * indicate measuring errors in the rectangle area. If cloud is empty or
 * too small, the corresponding rectangle is deleted.
 */
void DetectionTask::eraseEmptyRectangles(vector<Box>& boxes) {
	for (int i = 0; i < boxes.size(); i++) {
		if (boxes[i].getPointCloud()->size() < 50) {

			_deleted_boxes.resize(_deleted_boxes.size() + 1);
			std::cout << "--> Deleting box point 0: " << boxes[i].getPoints()[0] << "cloud size: " << boxes[i].getPointCloud()->size() << std::endl;
			std::move(boxes.begin() + i , boxes.begin() + i + 1, _deleted_boxes.end() - 1);

			boxes.erase(boxes.begin() + i);
			i--;
		}
	}
}

/*
 * Checks for distances in rectangle cloud object (x direction).
 * If box is considered to be too far away (out of manipulator range), it is deleted.
 * This function can cause trouble and will do wrong findings, when the sensor pose
 * angles are different than 0. In this case the function needs to be changed to
 * manual distance check. This approach is more robust, but due to performance reasons,
 * it is not used at the moment.
 */
void DetectionTask::eraseFarRectangles(vector<Box>& boxes) {

	for (int i = 0; i < boxes.size(); i++) {
		std::vector<int, std::allocator<int> > indices_x;
		pcl::PassThrough<pcl::PointXYZRGB> ptfilter(true); // Initializing with true will allow to extract the removed indices
		ptfilter.setInputCloud(boxes[i].getPointCloud());
		ptfilter.setFilterFieldName("x");
		ptfilter.setFilterLimits(_x_detection_distance_min,
				_x_detection_distance_max);
		ptfilter.filter(indices_x);

		// indexes all points of cloud that have x between the two values
		pcl::IndicesConstPtr indices_rem = ptfilter.getRemovedIndices();

		// if too many points (50%) are not inside the distance range,
		// the whole box is expected to be not inside the correct distance
		double needed_inliers = boxes[i].getPointCloud()->points.size() * 0.5;
		if (indices_x.size() < needed_inliers) {
			boxes.erase(boxes.begin() + i);
			i--;
		} else {
			std::vector<int, std::allocator<int> > indices_y;
			pcl::PassThrough<pcl::PointXYZRGB> ptfilter(true); // Initializing with true will allow to extract the removed indices
			ptfilter.setInputCloud(boxes[i].getPointCloud());
			ptfilter.setFilterFieldName("y");
			ptfilter.setFilterLimits(_y_detection_distance_min,
					_y_detection_distance_max);
			ptfilter.filter(indices_y);

			// indexes all points of cloud that have x between the two values
			pcl::IndicesConstPtr indices_rem_y = ptfilter.getRemovedIndices();

			// if too many points (50%) are not inside the distance range,
			// the whole box is expected to be not inside the correct distance
			if (indices_y.size() < needed_inliers) {
				boxes.erase(boxes.begin() + i);
				i--;
			}

		}
	}
}

/*
 * Creates point cloud for every single rectangle
 * and adds it to the corresponding rectangle object.
 * Points which are identified as being not part of a box
 * are deleted from the obstacles_cloud object.
 */
void DetectionTask::createPointClouds(DomainVision::CommVideoImage* color_image, vector<Box>& boxes, PclPointCloudPtr cloud) {


	for (uint i = 0; i < boxes.size(); i++) {
		//add only the points which are inside the area of a found rectangle
		int minX, maxX, minY, maxY;
		std::vector<uint> box_point_indices;
		boxes[i].getRectMinMaxValues(minX, maxX, minY, maxY);

		std::cout << "minX, maxX, minY, maxY: " <<minX << ", " << maxX << ", " << minY << ", " << maxY << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

		for (uint y = minY; y <= maxY; y++) {
			for (uint x = minX; x <= maxX; x++) {
				unsigned long cloud_index = y * color_image->get_width() + x;

				//check if point (x/y) is inside the rectangle,
				//if is inside add point.
				if (boxes[i].isPointInRect(x, y)) {
				//if (true) {
					pcl::PointXYZRGB p(cloud->points[cloud_index].r,
							cloud->points[cloud_index].g,
							cloud->points[cloud_index].b);
					p.x = cloud->points[cloud_index].x;
					p.y = cloud->points[cloud_index].y;
					p.z = cloud->points[cloud_index].z;

					// check if measured distance is NaN or inf
					if (isinf(p.x) || isinf(p.y) || isinf(p.z) || p.x != p.x
							|| p.y != p.y || p.z != p.z) {
						continue;
					}

					box_cloud->push_back(p);
				}

				// set point, which is part of a detected object, to NaN in obstacle cloud
				//obstacles_cloud->points[cloud_index].x = std::numeric_limits<float>::quiet_NaN();
				//obstacles_cloud->points[cloud_index].y = std::numeric_limits<float>::quiet_NaN();
				//obstacles_cloud->points[cloud_index].z = std::numeric_limits<float>::quiet_NaN();
				box_point_indices.push_back(cloud_index);
			}
		}

		boxes[i].setPointCloud(box_cloud);
		boxes[i].setPointCloudIndices(box_point_indices);
	}
}

/*
 * Creates point cloud for every single rectangle
 * and adds it to the corresponding rectangle object.
 */
void DetectionTask::createPointClouds(DomainVision::CommVideoImage* color_image, vector<Box>& boxes) {
	const uint8_t* imageData = color_image->get_data();

	float r, g, b;
	float x_in_m, y_in_m, z_in_m;

	for (uint i = 0; i < boxes.size(); i++) {
		//add only the points which are inside the area of a found rectangle
		int minX, maxX, minY, maxY;
		boxes[i].getRectMinMaxValues(minX, maxX, minY, maxY);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud(
				new pcl::PointCloud<pcl::PointXYZRGB>());

		for (uint y = minY; y <= maxY; y++) {
			for (uint x = minX; x <= maxX; x++) {

				//check if point (x/y) is inside the rectangle,
				//if NOT inside ignore point.
				if (!boxes[i].isPointInRect(x, y)) {
					continue;
				}
				//_pointManipulator.pixelToXyz(y, x, x_in_m, y_in_m, z_in_m);

				//TODO: This loop should be done more elegant, appears in several places
				for(int s = 0; s < 2; s++){
					uint tmp_x1 = x + s;
					uint tmp_y1 = y;
					uint tmp_x2 = x + s;
					uint tmp_y2 = y - s;
					uint tmp_x3 = x;
					uint tmp_y3 = y - s;
					uint tmp_x4 = x - s;
					uint tmp_y4 = y - s;
					uint tmp_x5 = x - s;
					uint tmp_y5 = y;
					uint tmp_x6 = x - s;
					uint tmp_y6 = y - s;
					uint tmp_x7 = x;
					uint tmp_y7 = y + s;
					uint tmp_x8 = x + s;
					uint tmp_y8 = y + s;

					_pointManipulator.pixelToXyz(tmp_y1, tmp_x1, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
					_pointManipulator.pixelToXyz(tmp_y2, tmp_x2, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
					_pointManipulator.pixelToXyz(tmp_y3, tmp_x3, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
					_pointManipulator.pixelToXyz(tmp_y4, tmp_x4, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
					_pointManipulator.pixelToXyz(tmp_y5, tmp_x5, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
					_pointManipulator.pixelToXyz(tmp_y6, tmp_x6, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
					_pointManipulator.pixelToXyz(tmp_y7, tmp_x7, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
					_pointManipulator.pixelToXyz(tmp_y8, tmp_x8, x_in_m, y_in_m, z_in_m, true);
					if(x_in_m != 0 && y_in_m != 0 && z_in_m != 0){
						break;
					}
				}


				// check if measured distance is NaN
				// if NaN or inf values are in point cloud it will not be shown
				// To get the mapping form RGB pixel (row / column) to
				// cloud object index (both having size of (1920x1080)) the invalid values are set to max values
				if (isinf(x_in_m) || isinf(z_in_m) || isinf(y_in_m)
						|| x_in_m != x_in_m || y_in_m != y_in_m
						|| z_in_m != z_in_m) {
					continue;
				}

				//extract RGB out of float and cast into bytes
				const uint8_t* pixel = (imageData
						+ (y - 1) * 3 * color_image->get_width() + x * 3);
				uint8_t r = pixel[0];
				uint8_t g = pixel[1];
				uint8_t b = pixel[2];

				pcl::PointXYZRGB p(r, g, b);
				p.x = x_in_m;
				p.y = y_in_m;
				p.z = z_in_m;

				box_cloud->push_back(p);
			}
		}

		boxes[i].setPointCloud(box_cloud);
	}
}

void DetectionTask::findContainer(const Mat& image, vector<Box>& boxes) {
	boxes.clear();

	int thresh = 20;
	int N = 10;
	RNG rng(12345);

	Mat pyr, timg, gray0(Size(image.cols, image.rows), CV_8U), gray;

	// down-scale and upscale the image to filter out the noise
	pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
	pyrUp(pyr, timg, image.size());

	vector<vector<Point> > contours;

	int ch[] = { 0, 0 };

	mixChannels(&timg, 1, &gray0, 1, ch, 1);

	// try several threshold levels
	for (int l = 0; l < N; l++) {
		Canny(gray0, gray, thresh * (l / 2), thresh * l, 5);
		dilate(gray, gray, Mat(), Point(-1, -1));

		findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

		vector<Point> approx;

		// test each contour
		for (size_t i = 0; i < contours.size(); i++) {
			approxPolyDP(Mat(contours[i]), approx,
					arcLength(Mat(contours[i]), true) * 0.02, true);

			double contour_area = fabs(contourArea(Mat(approx)));

			//if (approx.size() == 4 && contour_area > 1500 && contour_area < (image.cols * image.rows) / 5 && isContourConvex(Mat(approx))) {
			if (contour_area > 1500
					&& contour_area < (image.cols * image.rows) / 2) {
				double maxCosine = 0;

				for (int j = 2; j < 5; j++) {
					// find the maximum cosine of the angle between joint edges
					double cosine = fabs(
							angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}

				if (maxCosine < 0.3) {
					Scalar color = Scalar(rng.uniform(0, 255),
							rng.uniform(0, 255), rng.uniform(0, 255));
					drawContours(timg, contours, i, color, -1);	//, 2, 8, hierarchy, 0, Point() );
				}
			}
		}
	}

	///////////
	// now find drawn contours as rectangles with gaussian blur + difference
	// --> contour segmentations get lost

	N = 5;

	mixChannels(&timg, 1, &gray0, 1, ch, 1);
	Mat previous = gray0;

	// try several threshold levels
	for (int l = 0; l < N; l++) {

		/////////////////////////
		// Gaussian + Difference

		Mat gaussian;
		GaussianBlur(previous, gaussian, Size(3, 3), 20);
		Mat gray = previous - gaussian;
		previous = gaussian;

		findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

		vector<Point> approx;

		// test each contour
		for (size_t i = 0; i < contours.size(); i++) {
			approxPolyDP(Mat(contours[i]), approx,
					arcLength(Mat(contours[i]), true) * 0.02, true);

			double contour_area = fabs(contourArea(Mat(approx)));

			if (approx.size() == 4 && contour_area > 1500
					&& contour_area < (image.cols * image.rows) / 5) {
				//if ( contour_area > 1500 && contour_area < (image.cols * image.rows) / 2 ) {
				double maxCosine = 0;

				for (int j = 2; j < 5; j++) {
					// find the maximum cosine of the angle between joint edges
					double cosine = fabs(
							angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}

				if (maxCosine < 0.3) {
					boxes.push_back(Box());
					boxes.back().setPoints(approx);

				}
			}
		}
	}

	cout << ">>> Container candidates found: " << boxes.size() << endl;
}

/*
 * Returns sequence of rectangles detected in the image.
 * Basic idea of detecting rectangles from https://github.com/npinto/opencv/blob/master/samples/ocl/squares.cpp
 */
void DetectionTask::findRectangles(const Mat& image_rgb, vector<Box>& boxes, float ratio_deviation_factor, float max_box_cosine) {
	boxes.clear();

	Mat pyr, timg, gray0(image_rgb.size(), CV_8U), gray;
	Mat image;

	if (_use_hsv) {
		cvtColor(image_rgb, image, CV_RGB2HSV);
	} else {
		image = image_rgb;
	}

	// down-scale and upscale the image to filter out the noise
	pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
	pyrUp(pyr, timg, image.size());

	vector<vector<Point> > contours;

	// find rectangles in every color channel of the image
	for (int c = 0; c < 3; c++) {
		int ch[] = { c, 0 };

		// get grey image with one channel (red = 0, green = 1, blue = 2)
		mixChannels(&timg, 1, &gray0, 1, ch, 1);

		// apply different threshold levels
		for (int l = 0; l < N; l++) {
			// use Canny instead of zero threshold level
			if (l == 0) {
				Canny(gray0, gray, 0, thresh, 5);
//				vHelper.show_rgb_contour_image(gray);
//				std::cout << "WAIT Key press" << std::endl;
//				STOP;
				// remove potential holes between segments
				dilate(gray, gray, Mat(), Point(-1, -1));
			} else {
				// thresholding, if l != 0
				gray = gray0 >= (l + 1) * 255 / N;
			}

			//debug output
//			std::cout << "N: " << N << std::endl;
//			std::cout << "thresh: " << thresh << std::endl;
//			std::cout << "sides : " << _searched_obj.sides[0] << std::endl;
//			std::cout << "sides : " << _searched_obj.sides[1] << std::endl;
//			std::cout << "sides : " << _searched_obj.sides[2] << std::endl;
//			std::cout << "l: " << l << std::endl;
//			std::cout << "channel: " << c << std::endl;

			vector<Vec4i> hierarchy;

			findContours(gray, contours, hierarchy, RETR_LIST,
					CHAIN_APPROX_SIMPLE);

//			RNG rng(12345);
//			  Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
//			  for( int i = 0; i< contours.size(); i++ )
//			     {
//			       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//			       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
//			     }
//			  vHelper.show_rgb_contour_image(drawing);
//			  std::cout << "WAIT Key press" << std::endl;
//			  STOP;

			vector<Point> approx;

			// test each contour
			for (size_t i = 0; i < contours.size(); i++) {
				// approximate contour with accuracy proportional to the contour perimeter
				approxPolyDP(Mat(contours[i]), approx,
						arcLength(Mat(contours[i]), true) * 0.02, true);

				// absolute value of an area is used because area may be
				// positive or negative - in accordance with the contour orientation
				double contour_area = fabs(contourArea(Mat(approx)));

				if (approx.size() == 4 && contour_area > 1000 && contour_area < (image.cols * image.rows) / 5 && isContourConvex(Mat(approx))) {

					double maxCosine = 0;

					for (int j = 2; j < 5; j++) {
						// find the maximum cosine of the angle between joint edges
						double cosine = fabs(
								angle(approx[j % 4], approx[j - 2],
										approx[j - 1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					// check for about 90 degree
					if (maxCosine < max_box_cosine) {
						double finding_aspect_ratio = calcRectRatio(approx);
						// Vector contains three vectors (one for each side aspect ratio, which are given from outside)
						// Each vector contains three values again:
						//   1. max aspect ratio (aspect ratio + deviation factor)
						//   2. min aspect ratio (aspect ratio - deviation factor)
						//   3. original aspect ratio
						vector<vector<double> > aspect_ratios;
						aspect_ratios.resize(3);

						double aspect_ratio_1 = _searched_obj.sides[1]
								/ _searched_obj.sides[2];
						aspect_ratios[0].push_back(
								aspect_ratio_1
										+ aspect_ratio_1
												* ratio_deviation_factor);
						aspect_ratios[0].push_back(
								aspect_ratio_1
										- aspect_ratio_1
												* ratio_deviation_factor);
						aspect_ratios[0].push_back(aspect_ratio_1);

						double aspect_ratio_2 = _searched_obj.sides[0]
								/ _searched_obj.sides[2];
						aspect_ratios[1].push_back(
								aspect_ratio_2
										+ aspect_ratio_2
												* ratio_deviation_factor);
						aspect_ratios[1].push_back(
								aspect_ratio_2
										- aspect_ratio_2
												* ratio_deviation_factor);
						aspect_ratios[1].push_back(aspect_ratio_2);

						double aspect_ratio_3 = _searched_obj.sides[0]
								/ _searched_obj.sides[1];
						aspect_ratios[2].push_back(
								aspect_ratio_3
										+ aspect_ratio_3
												* ratio_deviation_factor);
						aspect_ratios[2].push_back(
								aspect_ratio_3
										- aspect_ratio_3
												* ratio_deviation_factor);
						aspect_ratios[2].push_back(aspect_ratio_3);

//						std::cout<<"[MCL] finding_aspect_ratio: "<<finding_aspect_ratio<<std::endl;
//						std::cout<<"[MCL] aspect_ratio_1: "<<aspect_ratio_1<<std::endl;
//						std::cout<<"[MCL] aspect_ratio_2: "<<aspect_ratio_2<<std::endl;
//						std::cout<<"[MCL] aspect_ratio_3: "<<aspect_ratio_3<<std::endl;

						for (int s = 0; s < 3; s++) {
							if (finding_aspect_ratio <= aspect_ratios[s][0] && finding_aspect_ratio	>= aspect_ratios[s][1]) {
								boxes.push_back(Box());
								boxes.back().setPoints(approx);
								// the one side, which is not part of the calculation of the aspect ratio
								boxes.back().setDepth(_searched_obj.sides[s]);
								boxes.back().setModelSideLengths(_searched_obj.sides);
								boxes.back().setPointManipulator(&_pointManipulator);
								break;
							}
						}
//						boxes.push_back(Box());
//						boxes.back().setPoints(approx);
//						// the one side, which is not part of the calculation of the aspect ratio
//						//boxes.back().setDepth(_searched_obj.sides[s]);
//						boxes.back().setModelSideLengths(_searched_obj.sides);
//						boxes.back().setPointManipulator(&_pointManipulator);
					}
				}
			}
		}
	}
}

/*
 * Compares object center in 2D +- deviation to identify duplicate findings.
 * Identified duplicates are erased from the list.
 */
void DetectionTask::eraseDuplicates(vector<Box>& boxes) {
	for (uint i = 0; i < boxes.size(); i++) {
		Point rect_center = boxes.at(i).getRectCenter();

		int max_x = rect_center.x + 15, min_x = rect_center.x - 15;
		int max_y = rect_center.y + 15, min_y = rect_center.y - 15;

		for (uint j = i + 1; j < boxes.size(); j++) {
			Point cmp_rect_center = boxes.at(j).getRectCenter();
			if (cmp_rect_center.x > min_x && cmp_rect_center.x < max_x) {
				if (cmp_rect_center.y > min_y && cmp_rect_center.y < max_y) {
					boxes.erase(boxes.begin() + j);
					j--;
				}
			}
		}
	}
}



/*
 * Transforms the box pose to:
 * 		x is going into the plane
 * 		y is parallel to the short side
 * 		z is parallel to the long side
 * 	This is possible since, the plane points are sorted.
 */
void DetectionTask::rearrangePose(vector<Box>& boxes){

	for (int i = 0; i< boxes.size(); i++){
		mrpt::poses::CPose3D pose = boxes[i].getSurfacePose();
		CommBasicObjects::CommPose3d sensor_pose = _pointManipulator.getSensorPose();

		///////////////////////////////////////
		// Rotate the pose with x into the object

		// Rotate to make pose x axis to plane normal (ZYX Konvention)
		pose += mrpt::poses::CPose3D(0, 0, 0, 0, 1.57079, 0);
		// Should also be 1.57079, but then the delta velues are too small,
		// which results in inaccurate positions. Hence only 1.45.
		pose += mrpt::poses::CPose3D(0, 0, 0, 0, 0, -1.45);

		mrpt::math::TPoint3D sensor_origin(sensor_pose.getPosition().getX(), sensor_pose.getPosition().getY(), sensor_pose.getPosition().getZ());
		
		// Timo - 20.04
		//---------------
		// new:	
		mrpt::math::TPoint3D point1 = mrpt::math::TPoint3D(pose.asTPose()) + mrpt::math::TPoint3D(0.1,0,0);
		mrpt::math::TPoint3D point2 = mrpt::math::TPoint3D(pose.asTPose()) + mrpt::math::TPoint3D(-0.1,0,0);
		// old:
		// mrpt::math::TPoint3D point1 = pose + mrpt::math::TPoint3D(0.1,0,0);
		// mrpt::math::TPoint3D point2 = pose + mrpt::math::TPoint3D(-0.1,0,0);
		//---------------
		
		double dist1 = point1.distanceTo(sensor_origin);
		double dist2 = point2.distanceTo(sensor_origin);

		// check, whether the box coordinate frame is rotated correctly
		// by checking the distance to sensor frame origin.
		if(dist1 < dist2){
			// if box coordinate frame is wrong, it is rotated (180 deg)
			//pose += mrpt::poses::CPose3D(0, 0, 0, 3.14, 0, 3.14);
			pose += mrpt::poses::CPose3D(0, 0, 0, 3.14, 0, 0);
		}


		////////////////////////////////////////
		// Todo its the same code like in eraseWrongLenthRecangles --> should not be repeated/executed twice
		// find surface cloud corner points
		// points with the highes distance to the rect center are the corner points

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud = boxes[i].getPointCloud();
		mrpt::math::TPoint3D box_midpoint(boxes[i].getSurfacePose().x(), boxes[i].getSurfacePose().y(), boxes[i].getSurfacePose().z());

		//sort points by distance to surface midpoint
		std::sort(box_cloud->points.begin(), box_cloud->points.end(),
				[&box_midpoint](pcl::PointXYZRGB a, pcl::PointXYZRGB b) {
			mrpt::math::TPoint3D tmp_point_a(a.x, a.y, a.z);
			mrpt::math::TPoint3D tmp_point_b(b.x, b.y, b.z);
			double dist_a = box_midpoint.distanceTo(tmp_point_a);
			double dist_b = box_midpoint.distanceTo(tmp_point_b);

			if(tmp_point_a.x == 0.0 || tmp_point_a.y == 0.0 || tmp_point_a.z == 0.0 ){return false;}
			if(tmp_point_b.x == 0.0 || tmp_point_b.y == 0.0 || tmp_point_b.z == 0.0 ){return true;}

			return dist_a > dist_b;
		});

		//the point with highest distance to mid point is probably a corner point
		vector<int> corner_point_idxs{0};
		//corner points must have min distance of the shortest side of the object
		double min_corner_dist = _searched_obj.sides[2] * 0.7;

		for(int j = 0; j<box_cloud->size(); j++){
			mrpt::math::TPoint3D tmp_point(box_cloud->points[j].x, box_cloud->points[j].y, box_cloud->points[j].z);
			bool too_close = false;

			for(int k = 0; k<corner_point_idxs.size(); k++){
				mrpt::math::TPoint3D tmp_corner_point(box_cloud->points[corner_point_idxs[k]].x, box_cloud->points[corner_point_idxs[k]].y, box_cloud->points[corner_point_idxs[k]].z) ;
				double tmp_dist = tmp_point.distanceTo(tmp_corner_point);

				//if current point is close to the last one it is probably
				//in the same corner, so try the next one
				if(tmp_dist < min_corner_dist){
					too_close = true;
					break;
				}
			}

			if(!too_close){
				corner_point_idxs.push_back(j);
				if (corner_point_idxs.size() == 4){
					j = box_cloud->size();
				}
			}

		}

		std::vector<mrpt::poses::CPoint3D> corner_points;

		for(int c = 0; c<corner_point_idxs.size(); c++ ){
			box_cloud->points[corner_point_idxs[c]].r = 250;
			box_cloud->points[corner_point_idxs[c]].g = 0;
			box_cloud->points[corner_point_idxs[c]].b = 0;
			cout << "Corner point: " << corner_point_idxs[c] << " , " << box_cloud->points[corner_point_idxs[c]] << endl;
			corner_points.push_back(mrpt::poses::CPoint3D(box_cloud->points[corner_point_idxs[c]].x, box_cloud->points[corner_point_idxs[c]].y, box_cloud->points[corner_point_idxs[c]].z));
		}

		////////////////////////////////////////////////////

		///////////////////////////////////////////////////
		//calculate the side lengthes

		std::vector<double> point_dists;
		//calculate the distances from each corner point to each other
		//the longest two are the diagonal lines of the rectangel
		point_dists.push_back(corner_points[0].distance3DTo( corner_points[1].x(), corner_points[1].y(), corner_points[1].z()));
		point_dists.push_back(corner_points[0].distance3DTo( corner_points[2].x(), corner_points[2].y(), corner_points[2].z()));
		point_dists.push_back(corner_points[0].distance3DTo( corner_points[3].x(), corner_points[3].y(), corner_points[3].z()));
		point_dists.push_back(corner_points[1].distance3DTo( corner_points[2].x(), corner_points[2].y(), corner_points[2].z()));
		point_dists.push_back(corner_points[1].distance3DTo( corner_points[3].x(), corner_points[3].y(), corner_points[3].z()));
		point_dists.push_back(corner_points[2].distance3DTo( corner_points[3].x(), corner_points[3].y(), corner_points[3].z()));

		std::vector<double> point_dists_bak(point_dists);

		for (int d = 0 ; d < point_dists.size(); d++){
			std::cout << "==> point dist (unsorted): " << point_dists[d] << std::endl;
		}

		//sort the distances (smallest ist index 0)
		std::sort(point_dists.begin(), point_dists.end());


		float rect_side_length1 = point_dists[1]; //longer of the short sides because edges of the surface cloud are cut
		float rect_side_length2 = point_dists[3]; //longer of the long sides because edges of the surface cloud are cut

		std::vector<double>::iterator short_side_it = std::find(point_dists_bak.begin(), point_dists_bak.end(), point_dists[1]);
		int short_side_index = std::distance(point_dists_bak.begin(), short_side_it);

		std::vector<double>::iterator long_side_it = std::find(point_dists_bak.begin(), point_dists_bak.end(), point_dists[3]);
		int long_side_index = std::distance(point_dists_bak.begin(), long_side_it);

		std::cout << "==> short side index:" << short_side_index << std::endl;
		std::cout << "==> long side index:" << long_side_index << std::endl;

		mrpt::poses::CPoint3D z_target_3d;

		if(short_side_index == 0){
			z_target_3d.x((corner_points[0].x() + corner_points[1].x()) / 2);
			z_target_3d.y((corner_points[0].y() + corner_points[1].y()) / 2);
			z_target_3d.z((corner_points[0].z() + corner_points[1].z()) / 2);
		}else if(short_side_index == 1){
			z_target_3d.x((corner_points[0].x() + corner_points[2].x()) / 2);
			z_target_3d.y((corner_points[0].y() + corner_points[2].y()) / 2);
			z_target_3d.z((corner_points[0].z() + corner_points[2].z()) / 2);
		}else if(short_side_index == 2){
			z_target_3d.x((corner_points[0].x() + corner_points[3].x()) / 2);
			z_target_3d.y((corner_points[0].y() + corner_points[3].y()) / 2);
			z_target_3d.z((corner_points[0].z() + corner_points[3].z()) / 2);
		}else if(short_side_index == 3){
			z_target_3d.x((corner_points[1].x() + corner_points[2].x()) / 2);
			z_target_3d.y((corner_points[1].y() + corner_points[2].y()) / 2);
			z_target_3d.z((corner_points[1].z() + corner_points[2].z()) / 2);
		}else if(short_side_index == 4){
			z_target_3d.x((corner_points[1].x() + corner_points[3].x()) / 2);
			z_target_3d.y((corner_points[1].y() + corner_points[3].y()) / 2);
			z_target_3d.z((corner_points[1].z() + corner_points[3].z()) / 2);
		}else if(short_side_index == 5){
			z_target_3d.x((corner_points[2].x() + corner_points[3].x()) / 2);
			z_target_3d.y((corner_points[2].y() + corner_points[3].y()) / 2);
			z_target_3d.z((corner_points[2].z() + corner_points[3].z()) / 2);
		}

		pcl::PointXYZRGB midpoint_pcl(0, 255, 0);
		midpoint_pcl.x = z_target_3d.x();
		midpoint_pcl.y = z_target_3d.y();
		midpoint_pcl.z = z_target_3d.z();

		boxes[i].getPointCloud()->push_back(midpoint_pcl);

		///////////////////////////////////////////////////



		////////////////////////////////////
		// Define two straights:
		// 1. plane center -> pose z axis
		// 2. plane center -> mid point of smaller plane side
		// and calculate angle between them

		float init_roll;
		if (z_target_3d.x() > box_midpoint.x && z_target_3d.y() > box_midpoint.y){
			init_roll = -0.6;
		} else if (z_target_3d.x() > box_midpoint.x && z_target_3d.y() < box_midpoint.y){
			init_roll = 1;
		} else if (z_target_3d.x() < box_midpoint.x && z_target_3d.y() < box_midpoint.y){
			init_roll = 1.7;
		} else if (z_target_3d.x() < box_midpoint.x && z_target_3d.y() > box_midpoint.y){
			init_roll = -2.2;
		}

		pose.setYawPitchRoll(pose.yaw(),pose.pitch(), init_roll);

		mrpt::math::TPoint3D point_on_pose_z = mrpt::math::TPoint3D(pose.asTPose()) + mrpt::math::TPoint3D(mrpt::poses::CPose3D(mrpt::poses::CPoint3D(0.0, 0.0, 0.07)).asTPose());

		pcl::PointXYZRGB point_on_pose_z_pcl(255, 0, 0);
		point_on_pose_z_pcl.x = point_on_pose_z.x;
		point_on_pose_z_pcl.y = point_on_pose_z.y;
		point_on_pose_z_pcl.z = point_on_pose_z.z;

		boxes[i].getPointCloud()->push_back(point_on_pose_z_pcl);

		mrpt::poses::CPose3D target_z_pose(z_target_3d.x(), z_target_3d.y(), z_target_3d.z(), 0, 0, 0);
		mrpt::poses::CPose3D target_z_pose_in_box_frame = target_z_pose - pose;
		mrpt::math::TPoint3D target_z_point_in_box_frame(target_z_pose_in_box_frame.x(),target_z_pose_in_box_frame.y(),target_z_pose_in_box_frame.z());

		mrpt::math::TLine3D target_z_axis(mrpt::math::TPoint3D(0,0,0), target_z_point_in_box_frame);
		mrpt::math::TLine3D pose_z_axis(mrpt::math::TPoint3D(0,0,0), mrpt::math::TPoint3D(0.0, 0.0, 0.05));

		double angle = mrpt::math::getAngle(pose_z_axis, target_z_axis);



		// rotate pose around x axis
		pose = pose + mrpt::poses::CPose3D(0, 0, 0, 0, 0, angle);

		if (pose.roll() > 1.65){
			pose = pose + mrpt::poses::CPose3D(0, 0, 0, 0, 0, M_PI);

		} else if (pose.roll() < -1.65){
			pose = pose + mrpt::poses::CPose3D(0, 0, 0, 0, 0, -1 * M_PI);
		}

		boxes[i].setSurfacePose(pose);
	}
}
