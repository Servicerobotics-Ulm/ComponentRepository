/*
 * AmclParticleVisualization.cc
 *
 *  Created on: Jan 20, 2020
 *      Author: shaikv3
 */

#include <visualization/AmclParticleVisualization.hh>
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

AmclParticleVisualization::AmclParticleVisualization(CDisplayWindow3D& window3D, const std::string& identifier):
	AbstractVisualization(window3D, identifier) {

	max_hypotheses = 10;
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr particles = opengl::CPointCloudColoured::Create();
		particles->setName(identifier+"particles");
		particles->setPointSize(3);
		ptrScene->insert(particles);

		opengl::CSetOfLines::Ptr particleLines = opengl::CSetOfLines::Create();
		particleLines->setName(identifier+"particleLines");
		particleLines->setColor(0, 1, 0);
		ptrScene->insert(particleLines);

		opengl::CPointCloudColoured::Ptr hypotheses = opengl::CPointCloudColoured::Create();
		hypotheses->setName(identifier+"hypotheses");
		hypotheses->setPointSize(10);
		ptrScene->insert(hypotheses);


		size_t max_hypotheses = 10;
		//marker pose and tag id objects
		for(size_t i = 0; i< max_hypotheses; ++i)
		{
			opengl::CEllipsoid2D::Ptr covar_obj = opengl::CEllipsoid2D::Create();
			covar_obj->setName(identifier + "_covar_" + std::to_string(i));
			covar_obj->setColor(1, 0, 0);
			ptrScene->insert(covar_obj);
		}
	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr particles = opengl::CPointCloudColoured::Create();
		particles->setName(identifier+"particles");
		particles->setPointSize(3);
		ptrScene->insert(particles);

		opengl::CSetOfLinesPtr particleLines = opengl::CSetOfLines::Create();
		particleLines->setName(identifier+"particleLines");
		particleLines->setColor(0, 1, 0);
		ptrScene->insert(particleLines);

		opengl::CPointCloudColouredPtr hypotheses = opengl::CPointCloudColoured::Create();
		hypotheses->setName(identifier+"hypotheses");
		hypotheses->setPointSize(10);
		ptrScene->insert(hypotheses);


		size_t max_hypotheses = 10;
		//marker pose and tag id objects
		for(size_t i = 0; i< max_hypotheses; ++i)
		{
			opengl::CEllipsoidPtr covar_obj = opengl::CEllipsoid::Create();
			covar_obj->setName(identifier + "_covar_" + std::to_string(i));
			covar_obj->setColor(1, 0, 0);
			ptrScene->insert(covar_obj);
		}
	}
#endif
	window3D.unlockAccess3DScene();
}

AmclParticleVisualization::~AmclParticleVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr particles_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "particles"));
		ptrScene->removeObject(particles_obj);

		opengl::CSetOfLines::Ptr particleLines_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + "particleLines"));
		ptrScene->removeObject(particleLines_obj);

		opengl::CPointCloudColoured::Ptr hypotheses_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "hypotheses"));
		ptrScene->removeObject(hypotheses_obj);

		for (size_t i = 0; i <max_hypotheses; i++) {
			opengl::CEllipsoid2D::Ptr covar_obj = std::dynamic_pointer_cast<opengl::CEllipsoid2D>(ptrScene->getByName(identifier + "_covar_" + std::to_string(i)));
			ptrScene->removeObject(covar_obj);
		}

	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr particles_obj = (opengl::CPointCloudColouredPtr)ptrScene->getByName(identifier + "particles");
		ptrScene->removeObject(particles_obj);

		opengl::CSetOfLinesPtr particleLines_obj = (opengl::CSetOfLinesPtr) ptrScene->getByName(identifier + "particleLines");
		ptrScene->removeObject(particleLines_obj);

		opengl::CPointCloudColouredPtr hypotheses_obj = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "hypotheses");
		ptrScene->removeObject(hypotheses_obj);

		for (size_t i = 0; i <max_hypotheses; i++) {
			opengl::CEllipsoidPtr covar_obj = (opengl::CEllipsoidPtr)ptrScene->getByName(identifier + "_covar_" + std::to_string(i));
			ptrScene->removeObject(covar_obj);
		}

	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
void AmclParticleVisualization::displayAmclInfo(const CommLocalizationObjects::CommAmclVisualizationInfo& pf_info)
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
		{
			opengl::CPointCloudColoured::Ptr particles_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "particles"));
			opengl::CSetOfLines::Ptr particleLines_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + "particleLines"));
			opengl::CPointCloudColoured::Ptr hypotheses_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "hypotheses"));

			if(pf_info.getParticlesSize()>0)
			{
			particles_obj->clear();
			particleLines_obj->clear();
			}

			if(pf_info.getHypothesesSize()>0)
			{
			hypotheses_obj->clear();
//			for (size_t i = 0; i <max_hypotheses; i++) {
//				opengl::CEllipsoidPtr covar_obj = (opengl::CEllipsoidPtr)ptrScene->getByName(identifier + "_covar_" + std::to_string(i));
//				covar_obj->clear();
//			}
			}


			//particle
			for (size_t i = 0; i < pf_info.getParticlesSize(); i++) {
				CommBasicObjects::CommPose3d current_particle_pose = pf_info.getParticlesElemAtPos(i).getPose();
				double current_particle_weight = pf_info.getParticlesElemAtPos(i).getWeight();

				particles_obj->push_back(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0, 0, 1, 0);

				double r = std::fmod(current_particle_weight * 100000, 0.25);
				particleLines_obj->appendLine(current_particle_pose.get_x(1.0) ,current_particle_pose.get_y(1.0), 0,
						current_particle_pose.get_x(1.0) + r * cos(current_particle_pose.get_azimuth()),
						current_particle_pose.get_y(1.0) + r * sin(current_particle_pose.get_azimuth()), 0);

			}
			//Hypotheses
			size_t hype_num_to_draw = (pf_info.getHypothesesSize() > max_hypotheses) ? max_hypotheses : pf_info.getHypothesesSize();
			for (size_t i = 0; i <hype_num_to_draw; i++) {
				CommBasicObjects::CommPose3d current_particle_pose = pf_info.getHypothesesElemAtPos(i).getPose();
				double current_hypothesis_weight = pf_info.getHypothesesElemAtPos(i).getWeight();
				hypotheses_obj->push_back(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0, 1, 0, 0);

				//covariance as Ellipsoid
				opengl::CEllipsoid2D::Ptr covar_obj = std::dynamic_pointer_cast<opengl::CEllipsoid2D>(ptrScene->getByName(identifier + "_covar_" + std::to_string(i)));
				covar_obj->setLocation(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0);

				mrpt::math::CMatrixDouble22 cov2d(pf_info.getHypothesesElemAtPos(i).getCovMatrixCopy().data());
				covar_obj->setCovMatrix(cov2d);
			}


		}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{
			opengl::CPointCloudColouredPtr particles_obj = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "particles");
			opengl::CSetOfLinesPtr particleLines_obj = (opengl::CSetOfLinesPtr) ptrScene->getByName(identifier + "particleLines");
			opengl::CPointCloudColouredPtr hypotheses_obj = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "hypotheses");

			if(pf_info.getParticlesSize()>0)
			{
			particles_obj->clear();
			particleLines_obj->clear();
			}

			if(pf_info.getHypothesesSize()>0)
			{
			hypotheses_obj->clear();
//			for (size_t i = 0; i <max_hypotheses; i++) {
//				opengl::CEllipsoidPtr covar_obj = (opengl::CEllipsoidPtr)ptrScene->getByName(identifier + "_covar_" + std::to_string(i));
//				covar_obj->clear();
//			}
			}


			//particle
			for (size_t i = 0; i < pf_info.getParticlesSize(); i++) {
				CommBasicObjects::CommPose3d current_particle_pose = pf_info.getParticlesElemAtPos(i).getPose();
				double current_particle_weight = pf_info.getParticlesElemAtPos(i).getWeight();

				particles_obj->push_back(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0, 0, 1, 0);

				double r = std::fmod(current_particle_weight * 100000, 0.25);
				particleLines_obj->appendLine(current_particle_pose.get_x(1.0) ,current_particle_pose.get_y(1.0), 0,
						current_particle_pose.get_x(1.0) + r * cos(current_particle_pose.get_azimuth()),
						current_particle_pose.get_y(1.0) + r * sin(current_particle_pose.get_azimuth()), 0);

			}
			//Hypotheses
			size_t hype_num_to_draw = (pf_info.getHypothesesSize() > max_hypotheses) ? max_hypotheses : pf_info.getHypothesesSize();
			for (size_t i = 0; i <hype_num_to_draw; i++) {
				CommBasicObjects::CommPose3d current_particle_pose = pf_info.getHypothesesElemAtPos(i).getPose();
				double current_hypothesis_weight = pf_info.getHypothesesElemAtPos(i).getWeight();
				hypotheses_obj->push_back(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0, 1, 0, 0);

				//covariance as Ellipsoid
				opengl::CEllipsoidPtr covar_obj = (opengl::CEllipsoidPtr)ptrScene->getByName(identifier + "_covar_" + std::to_string(i));
				covar_obj->setLocation(current_particle_pose.get_x(1.0), current_particle_pose.get_y(1.0), 0);

				mrpt::math::CMatrixDouble22 cov2d(pf_info.getHypothesesElemAtPos(i).getCovMatrixCopy().data());
				covar_obj->setCovMatrix(cov2d);
			}


		}
#endif
		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
}

void AmclParticleVisualization::clear()
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
	{
	opengl::CPointCloudColoured::Ptr particles_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "particles"));
	particles_obj->clear();

	opengl::CSetOfLines::Ptr particleLines_obj = std::dynamic_pointer_cast<opengl::CSetOfLines>(ptrScene->getByName(identifier + "particleLines"));
	particleLines_obj->clear();

	opengl::CPointCloudColoured::Ptr hypotheses_obj = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "hypotheses"));
	hypotheses_obj->clear();

//	for (size_t i = 0; i <max_hypotheses; i++) {
//		opengl::CEllipsoidPtr covar_obj = (opengl::CEllipsoidPtr)ptrScene->getByName(identifier + "_covar_" + std::to_string(i));
//		covar_obj->clear();
//	}

	}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
	{
	opengl::CPointCloudColouredPtr particles_obj = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "particles");
	particles_obj->clear();

	opengl::CSetOfLinesPtr particleLines_obj = (opengl::CSetOfLinesPtr) ptrScene->getByName(identifier + "particleLines");
	particleLines_obj->clear();

	opengl::CPointCloudColouredPtr hypotheses_obj = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "hypotheses");
	hypotheses_obj->clear();

//	for (size_t i = 0; i <max_hypotheses; i++) {
//		opengl::CEllipsoidPtr covar_obj = (opengl::CEllipsoidPtr)ptrScene->getByName(identifier + "_covar_" + std::to_string(i));
//		covar_obj->clear();
//	}

	}
#endif

	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}

