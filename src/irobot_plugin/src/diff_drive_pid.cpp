#include "irobot_plugin/diff_drive_pid.h"

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/console.h>

using namespace gazebo;

DiffDrivePID::DiffDrivePID() : gazebo_node_(new transport::Node())
{
	this->spinner_thread_ = new boost::thread( boost::bind( &DiffDrivePID::spin, this) );
}

DiffDrivePID::~DiffDrivePID()
{
	rosnode_->shutdown();
	this->spinner_thread_->join();
	delete this->spinner_thread_;
	//   delete [] wheel_speed_;
	delete rosnode_;
}

void DiffDrivePID::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
	this->my_world_ = _parent->GetWorld();

	this->my_parent_ = _parent;
	if (!this->my_parent_)
	{
		ROS_FATAL("rosnode_Create controller requires a Model as its parent");
		return;
	}

	this->node_namespace_ = "";
	if (_sdf->HasElement("node_namespace"))
		this->node_namespace_ = _sdf->GetElement("node_namespace")->Get<std::string>() + "/";


	// if (!ros::isInitialized())
	// {
	// 	int argc = 0;
	// 	char** argv = NULL;
	// 	ros::init(argc, argv, "irobot_gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	// }
	ros::isInitialized();

	rosnode_ = new ros::NodeHandle( node_namespace_ );

	rosnode_->param<std::string> ( "velocityTopic", velocity_topic_, "/cmd_vel" );
    rosnode_->param<std::string> ( "modelStateTopic", model_state_topic_, "/gazebo/model_states" );

	// velocity_topic_ = rosnode_->param()

	model_state_subscriber_ = rosnode_->subscribe(model_state_topic_, 1, &DiffDrivePID::modelStateCallback, this );
	if ( model_state_subscriber_ )
    	ROS_INFO_NAMED("diff_drive_pid", "Subscribe to %s", model_state_topic_.c_str());

	velocity_publisher_ = rosnode_->advertise<geometry_msgs::Twist>(velocity_topic_, 1);
	if ( velocity_publisher_ )
		ROS_INFO_NAMED("diff_drive_pid", "Advertise velocity on %s ", velocity_topic_.c_str());

	// Get then name of the parent model
	std::string model_name = _sdf->GetParent()->Get<std::string>("name");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DiffDrivePID::UpdateChild, this));

	ROS_INFO("Model name: %s", model_name.c_str());
	// gzdbg << "plugin model name: " << model_name << "\n";
}

void DiffDrivePID::UpdateChild()
{

	publishVelocity();

	// Update the pose data from the simulator
	getPose();

	// The main algorithm goes here. If needed, call other function(s)
}

void DiffDrivePID::getPose()
{
    boost::mutex::scoped_lock scoped_lock ( lock );
	ROS_DEBUG("Mutex locked for getPose");

	// Get pose of the robot and target

}

void DiffDrivePID::modelStateCallback ( const gazebo_msgs::ModelStates::ConstPtr& _msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
	ROS_DEBUG("Mutex locked for modelStateCallback");
    // x_ = cmd_msg->linear.x;
    // rot_ = cmd_msg->angular.z;

	// Do something useful here
}

void DiffDrivePID::publishVelocity ()
{

    velocity_publisher_.publish ( velocity_ );
}

void DiffDrivePID::spin()
{
	while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(DiffDrivePID);