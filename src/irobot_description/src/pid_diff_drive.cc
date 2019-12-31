#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


// ros::Rate loop_rate(5);
ignition::math::Pose3d robot_pose;

namespace gazebo
{
  class PIDDiffDrive : public ModelPlugin
  {
    public: PIDDiffDrive() : ModelPlugin()
    {
      ROS_WARN("Loading PIDDiffDrive plugin");
    }
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // gazebo::math::Pose robot_pose;
      // float yaw;
      

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PIDDiffDrive::OnUpdate, this));
      ROS_WARN("PIDDiffDrive plugin loaded with %s", this->model->GetName().c_str());

      // Create transport node
      this->node = transport::NodePtr(new transport::Node());

      std::string models_state = "/gazebo/model_states";

      // Subscribe to topic
      // this->sub = this->node->Subscribe(models_state, PIDDiffDrive::OnMsg, this, false);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

      // Mark the existance
      // ROS_INFO("OnUpdate");

      // Get pose of the model
      robot_pose = this->model->WorldPose();
      // yaw = robot_pose.GetYaw()
      ignition::math::Vector3d pos = robot_pose.Pos();
      ignition::math::Quaterniond rot = robot_pose.Rot();
      float x = pos.X();
      float y = pos.Y();
      float yaw = rot.Yaw();
      // ROS_INFO("Pose of %s is %f, %f and %f", this->model->GetName().c_str(), x, y, yaw);

      // Get pose of the target

      // Compute the velocities with PID

      // Publish to cmd_vel

      // Sleep
      // loop_rate.sleep();
    }

    public: void OnMsg(physics::ModelPtr &_msg)
    {
      ROS_INFO("Please work");
    }

    // node used for transport
    private: transport::NodePtr node;

    // subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PIDDiffDrive)
}