/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// This plugin is based on https://bitbucket.org/osrf/gazebo/src/56be256c4ba911e9ffb6fece23b0c51170f6dbdf/plugins/DiffDrivePlugin.cc?at=default&fileviewer=file-view-default
// including minor modifications (Timo Blender, 2017)

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class NewDiffDrivePlugin : public ModelPlugin
  {
    public: NewDiffDrivePlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
		this->model = _model;

    	this->leftWheelJoint = _model->GetJoint("robot::wheel_left_joint");
  		this->rightWheelJoint = _model->GetJoint("robot::wheel_right_joint");
	
  		if (!this->leftWheelJoint) {
    		std::cout << "Unable to find left wheel joint" << std::endl;
  		}
  		if (!this->rightWheelJoint) {
    		std::cout << "Unable to find right wheel joint" << std::endl;
		}
      
      	this->pid = common::PID(0.1, 0, 0);

      	this->model->GetJointController()->SetVelocityPID(
        this->leftWheelJoint->GetScopedName(), this->pid);

        this->model->GetJointController()->SetVelocityPID(
        this->rightWheelJoint->GetScopedName(), this->pid);

		// https://bitbucket.org/osrf/gazebo/src/56be256c4ba911e9ffb6fece23b0c51170f6dbdf/plugins/DiffDrivePlugin.cc?at=default&fileviewer=file-view-default
 		this->wheelSeparation = this->leftWheelJoint->Anchor(0).Distance(this->rightWheelJoint->Anchor(0));
  		physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(this->leftWheelJoint->GetChild());
		ignition::math::Box bb = parent->BoundingBox();

  		this->wheelRadius = bb.Size().Max() * 0.5;

		this->leftWheelJoint->SetVelocity(0, 0);
		this->rightWheelJoint->SetVelocity(0, 0);

      	this->node = transport::NodePtr(new transport::Node());
      	this->node->Init(this->model->GetWorld()->Name());

      	std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
	
      	this->sub = this->node->Subscribe(topicName, &NewDiffDrivePlugin::OnMsg, this);

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&NewDiffDrivePlugin::OnUpdate, this));

		this->leftWheelSpeed = 0;
		this->rightWheelSpeed = 0;
    }

    public: void setVelocity(const double &_velLeft, const double &_velRight)
    {
      // Set the joint's target velocity.
      //this->model->GetJointController()->SetVelocityTarget(this->leftWheelJoint->GetScopedName(), _velLeft);
      //this->model->GetJointController()->SetVelocityTarget(this->rightWheelJoint->GetScopedName(), _velRight);
    }

    private: void OnMsg(ConstVector3dPtr &_msg)
    {
		// _msg->x() is mm/s
		double v = _msg->x()/1000.0;
		// _msg->z() is rad/s
		double w = _msg->z()*-1;
		double velLeft = (v+w*this->wheelSeparation/2.0)/this->wheelRadius;
		double velRight = (v-w*this->wheelSeparation/2.0)/this->wheelRadius;
		this->leftWheelSpeed = v+w*this->wheelSeparation/2.0;
		this->rightWheelSpeed = v-w*this->wheelSeparation/2.0;
      	//this->setVelocity(velLeft,velRight);
    }

	private: void OnUpdate() {
		this->leftWheelJoint->SetVelocity(0, this->leftWheelSpeed/this->wheelRadius);
		this->rightWheelJoint->SetVelocity(0, this->rightWheelSpeed/this->wheelRadius);
	}

    private: transport::NodePtr node;

    private: transport::SubscriberPtr sub;

    private: physics::ModelPtr model;

    private: physics::JointPtr leftWheelJoint;
    private: physics::JointPtr rightWheelJoint;

    private: common::PID pid;

	private: double wheelSeparation;

	private: double wheelRadius;

	private: event::ConnectionPtr updateConnection;

	private: double leftWheelSpeed;
	private: double rightWheelSpeed;

  };

  GZ_REGISTER_MODEL_PLUGIN(NewDiffDrivePlugin)
}
