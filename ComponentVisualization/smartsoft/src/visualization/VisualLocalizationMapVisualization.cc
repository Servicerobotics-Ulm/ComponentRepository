/*
 * AmclParticleVisualization.cc
 *
 *  Created on: Jan 20, 2020
 *      Author: shaikv3
 */

#include <visualization/VisualLocalizationMapVisualization.hh>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3D.h>
#include <cmath>

#ifdef WITH_MRPT_2_0_VERSION
#include <mrpt/opengl/CEllipsoid2D.h>
#else
#include <mrpt/math.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/utils.h>
#endif

VisualLocalizationMapVisualization::VisualLocalizationMapVisualization(CDisplayWindow3D& window3D, const std::string& identifier):
	AbstractVisualization(window3D, identifier) {

#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr particles = opengl::CPointCloudColoured::Create();
		particles->setName(identifier+"particles");
		particles->setColor(0, 1, 0, 0.4);
		particles->setPointSize(3);
		ptrScene->insert(particles);
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr particles = opengl::CPointCloudColoured::Create();
		particles->setName(identifier+"particles");
		particles->setPointSize(3);
		ptrScene->insert(particles);
	}
#endif
	window3D.unlockAccess3DScene();
}

VisualLocalizationMapVisualization::~VisualLocalizationMapVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr particles_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "particles"));
		ptrScene->removeObject(particles_obj);
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr particles_obj = (opengl::CPointCloudColouredPtr)ptrScene->getByName(identifier + "particles");
		ptrScene->removeObject(particles_obj);
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
void VisualLocalizationMapVisualization::displayMap(const CommLocalizationObjects::CommVisualLocalizationFeatureMap& map)
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
		{
			opengl::CPointCloudColoured::Ptr particles_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "particles"));

			if(map.getFeaturesSize()>0)
			{
			particles_obj->clear();
			}

			//particles with direction
			for (size_t i = 0; i < map.getFeaturesSize(); i++) {
				CommBasicObjects::CommPose3d current_particle_pose = map.getFeaturesElemAtPos(i).getPose();
				particles_obj->push_back(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0, 0, 1, 0);
			}
		}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{
			opengl::CPointCloudColouredPtr particles_obj = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "particles");

			if(map.getFeaturesSize()>0)
			{
			particles_obj->clear();
			}

			//particle
			for (size_t i = 0; i < map.getFeaturesSize(); i++) {
				CommBasicObjects::CommPose3d current_particle_pose = map.getFeaturesElemAtPos(i).getPose();
				particles_obj->push_back(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0, 0, 1, 0);
			}
		}
#endif
		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
}

void VisualLocalizationMapVisualization::clear()
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
	opengl::CPointCloudColoured::Ptr particles_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "particles"));
	particles_obj->clear();

	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
	opengl::CPointCloudColouredPtr particles_obj = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "particles");
	particles_obj->clear();

	}
#endif

	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

