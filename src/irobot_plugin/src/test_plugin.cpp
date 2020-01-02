#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
// #include <create_node/TurtlebotSensorState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "irobot_plugin/test_plugin.h"

using namespace gazebo;

GazeboRosCreate::GazeboRosCreate()
  : gazebo_node_(new transport::Node())
{
  this->spinner_thread_ = new boost::thread( boost::bind( &GazeboRosCreate::spin, this) );
}

GazeboRosCreate::~GazeboRosCreate()
{
  rosnode_->shutdown();
  this->spinner_thread_->join();
  delete this->spinner_thread_;
//   delete [] wheel_speed_;
  delete rosnode_;
}

void GazeboRosCreate::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->my_world_ = _parent->GetWorld();

  this->my_parent_ = _parent;
  if (!this->my_parent_)
  {
    ROS_FATAL("Gazebo_ROS_Create controller requires a Model as its parent");
    return;
  }


  this->node_namespace_ = "";
  if (_sdf->HasElement("node_namespace"))
    this->node_namespace_ = _sdf->GetElement("node_namespace")->Get<std::string>() + "/";


  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    // ros::init(argc, argv, "gazebo_turtlebot", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle( node_namespace_ );

  cmd_vel_sub_ = rosnode_->subscribe("cmd_vel", 1, &GazeboRosCreate::OnCmdVel, this );


//   sensor_state_pub_ = rosnode_->advertise<create_node::TurtlebotSensorState>("sensor_state", 1);
  odom_pub_ = rosnode_->advertise<nav_msgs::Odometry>("/odom", 1);

  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("/joint_states", 1);


  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosCreate::UpdateChild, this));
  gzdbg << "plugin model name: " << modelName << "\n";
}

void GazeboRosCreate::UpdateChild()
{
    nav_msgs::Odometry odom;
  odom_pub_.publish( odom );

  joint_state_pub_.publish( js_ );

  this->UpdateSensors();

}

void GazeboRosCreate::UpdateSensors()
{

}

void GazeboRosCreate::OnCmdVel( const geometry_msgs::TwistConstPtr &msg)
{
  last_cmd_vel_time_ = this->my_world_->SimTime();

}

void GazeboRosCreate::spin()
{
  while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosCreate);