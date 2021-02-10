//--------------------------------------------------------------------------
//
//  Copyright (C) 2017 Timo Blender
//
//      schlegel@hs-ulm.de
//
//      Service Robotics Ulm
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
//-------------------------------------------------------------------------

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>

#include <functional>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
  class PubBasePosePlugin : public ModelPlugin
  {
    public: PubBasePosePlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

	  this->node = transport::NodePtr(new transport::Node());
	  node->Init();
	  std::string topicPath = "~/";
	  std::string topicName = "/basePose";
	  topicPath = topicPath + _model->GetName();
	  topicPath = topicPath + topicName;
	  this->pub = node->Advertise<gazebo::msgs::Pose>(topicPath);

      update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&PubBasePosePlugin::Update, this));
    }

	void Update()
	{
	  gazebo::msgs::Pose msg;
	  ignition::math::Pose3d currentPose = this->model->WorldPose();
	  ignition::math::Pose3d pose (ignition::math::Vector3d(currentPose.Pos().X(), currentPose.Pos().Y(), currentPose.Pos().Z()), ignition::math::Quaterniond(currentPose.Rot().W(), currentPose.Rot().X(), currentPose.Rot().Y(), currentPose.Rot().Z()));

	  gazebo::msgs::Set(&msg, pose);
	  this->pub->Publish(msg);
	}

    private: transport::NodePtr node;

	private: gazebo::transport::PublisherPtr pub;

    private: physics::ModelPtr model;
  	
	private: gazebo::event::ConnectionPtr update_connection_;

  };

  GZ_REGISTER_MODEL_PLUGIN(PubBasePosePlugin)
}
