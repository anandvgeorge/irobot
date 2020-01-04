// #include <algorithm>
// #include <assert.h>

#include <irobot_plugin/diff_drive_gazebo_ros.h>

// #include <ignition/math/Angle.hh>
// #include <ignition/math/Pose3.hh>
// #include <ignition/math/Quaternion.hh>
// #include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <tf/tf.h>

#include <ros/ros.h>

namespace gazebo
{

enum {
    ROBOT,
    TARGET,
};

DiffDriveGazeboRos::DiffDriveGazeboRos() {}

// Destructor
DiffDriveGazeboRos::~DiffDriveGazeboRos() 
{
	FiniChild();
}

// Load the controller
void DiffDriveGazeboRos::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrivePID" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( velocity_topic_, "velocityTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( model_state_topic_, "modelStateTopic", "gazebo/model_states" );
    gazebo_ros_->getParameter<std::string> ( target_model_, "targetModel", "cricket_ball" );

    alive_ = true;

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO_NAMED("diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), model_state_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(model_state_topic_, 1,
                boost::bind(&DiffDriveGazeboRos::modelStateCallback, this, _1),
                ros::VoidPtr(), &queue_);

    model_state_subscriber_ = gazebo_ros_->node()->subscribe(so);

    // Velocity Publisher
    velocity_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Twist>(velocity_topic_, 1);
    if ( velocity_publisher_ )
        ROS_INFO_NAMED("diff_drive", "%s: Publish velocity over %s ", gazebo_ros_->info(), velocity_topic_.c_str());
    else
        ROS_ERROR_NAMED("diff_drive", "%s: Cannot publish data over %s", gazebo_ros_->info(), velocity_topic_.c_str());
    
    // // ROS Service client for model states
    // model_state_client_ = gazebo_ros_->node()->serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    // if ( model_state_client_ )
    //     ROS_INFO_NAMED("diff_drive", "%s: Connected to ROSserice server %s ", gazebo_ros_->info(), "gazebo/get_model_state");
    // else
    //     ROS_ERROR_NAMED("diff_drive", "%s: Cannot connect to  ROSserice server %s", gazebo_ros_->info(), "gazebo/get_model_state");


    gazebo::DiffDriveGazeboRos::model robot { _sdf->GetParent()->Get<std::string>("name") };
    gazebo::DiffDriveGazeboRos::model target { target_model_ };

    // map objects
    objects = {
        { ROBOT , robot},
        { TARGET , target}
    };
    
    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &DiffDriveGazeboRos::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &DiffDriveGazeboRos::UpdateChild, this ) );

}

void DiffDriveGazeboRos::UpdateChild()
{

    publishVelocity();

    // Update the pose data from the simulator
    getPose();

    // The main algorithm goes here. If needed, call other function(s)
    tf::Quaternion q(
        objects[ROBOT].pose.orientation.x,
        objects[ROBOT].pose.orientation.y,
        objects[ROBOT].pose.orientation.z,
        objects[ROBOT].pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

}

void DiffDriveGazeboRos::getPose()
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    ROS_INFO("Mutex locked for getPose");

    // Get pose of the robot and target
  
}

void DiffDriveGazeboRos::publishVelocity ()
{

    // velocity_publisher_.publish ( velocity_ );
}

void DiffDriveGazeboRos::modelStateCallback ( const gazebo_msgs::ModelStates::ConstPtr& _msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    ROS_INFO("Mutex locked for modelStateCallback");
    // x_ = cmd_msg->linear.x;
    // rot_ = cmd_msg->angular.z;

    // Do something useful here
    // ROS_INFO("Hope this msg gets printed %s", _msg.c_str());
}

void DiffDriveGazeboRos::Reset()
{

}

// Finalize the controller
void DiffDriveGazeboRos::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void DiffDriveGazeboRos::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

GZ_REGISTER_MODEL_PLUGIN ( DiffDriveGazeboRos )
}
