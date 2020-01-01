#include <irobot_plugin/pid_diff_drive.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{

PIDDiffDrive::PIDDiffDrive() 
{
    ROS_WARN("PID Plugin is loading");
}

// Destructor
PIDDiffDrive::~PIDDiffDrive() 
{
	FiniChild();
}

// Load the controller
void PIDDiffDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "PIDDiffDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( velocity_topic_, "velocityTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( model_state_topic_, "modelStateTopic", "gazebo/model_states" );
	gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    // gazebo_ros_->getParameterBoolean ( legacy_mode_, "legacyMode", true );

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
#else
    last_update_time_ = parent->GetWorld()->GetSimTime();
#endif

	alive_ = true;

    // ROS: Subscribe to the model_states topic to get the pose of robot and target
    ROS_INFO_NAMED("pid_diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), model_state_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(model_state_topic_, 1,
                boost::bind(&PIDDiffDrive::modelStateCallback, this, _1),
                ros::VoidPtr(), &queue_);

    model_state_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("pid_diff_drive", "%s: Subscribe to %s", gazebo_ros_->info(), model_state_topic_.c_str());

	// Publish twist msgs
	velocity_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Twist>(velocity_topic_, 1);
	ROS_INFO_NAMED("pid_diff_drive", "%s: Advertise velocity on %s ", gazebo_ros_->info(), velocity_topic_.c_str());

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &PIDDiffDrive::QueueThread, this ) );
    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &PIDDiffDrive::UpdateChild, this ) );

}


// Update the controller
void PIDDiffDrive::UpdateChild()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        // if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
		publishVelocity();

        // Update the pose data from the simulator
        getPose();

        // The main algorithm goes here. If needed, call other function(s)

        last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void PIDDiffDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void PIDDiffDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void PIDDiffDrive::getPose()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

	// Get pose of the robot and target

}

void PIDDiffDrive::modelStateCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    // x_ = cmd_msg->linear.x;
    // rot_ = cmd_msg->angular.z;

	// Do something useful here
}

void PIDDiffDrive::publishVelocity ()
{

    velocity_publisher_.publish ( velocity_ );
}

GZ_REGISTER_MODEL_PLUGIN ( PIDDiffDrive )
}