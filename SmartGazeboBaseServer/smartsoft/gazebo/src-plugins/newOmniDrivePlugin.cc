// TODO: GPL license
// This plugin is based on https://github.com/robocup-logistics/gazebo-rcll/tree/master/plugins/src/plugins/motor
// including minor modifications (Timo Blender, 2018)

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class NewOmniDrivePlugin : public ModelPlugin
  {
    public: NewOmniDrivePlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
		this->model = _model;

      	this->node = transport::NodePtr(new transport::Node());
      	this->node->Init(this->model->GetWorld()->Name());

      	std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
	
      	this->sub = this->node->Subscribe(topicName, &NewOmniDrivePlugin::OnMsg, this);

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&NewOmniDrivePlugin::OnUpdate, this));
    }

    private: void OnMsg(ConstVector3dPtr &_msg)
    {
		this->vx_ = _msg->x();
		this->vy_ = _msg->y();
		this->vomega_ = _msg->z();
    }

	private: void OnUpdate() {
		//Apply movement command
		float x,y;
		//float yaw = this->model->GetWorldPose().rot.GetAsEuler().z;
		float yaw = this->model->WorldPose().Rot().Euler().Z();
		//foward part
		x = cos(yaw) * vx_;
		y = sin(yaw) * vx_;
		//sideways part
		x += cos(yaw + 3.1415926f / 2) * vy_;
		y += sin(yaw + 3.1415926f / 2) * vy_;
		// Apply velocity to the model.
		this->model->SetLinearVel(ignition::math::Vector3d(x, y, 0));
		this->model->SetAngularVel(ignition::math::Vector3d(0, 0, vomega_));
	}

    private: transport::NodePtr node;

    private: transport::SubscriberPtr sub;

    private: physics::ModelPtr model;

	private: event::ConnectionPtr updateConnection;

	private: double vx_;
	private: double vy_;
	private: double vomega_;

  };

  GZ_REGISTER_MODEL_PLUGIN(NewOmniDrivePlugin)
}
