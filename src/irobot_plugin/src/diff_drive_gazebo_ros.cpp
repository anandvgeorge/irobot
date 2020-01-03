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
    gazebo_ros_->getParameter<std::string> ( target_model_, "targetModel", "cricket_ball" );

    alive_ = true;

    // // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    // ROS_INFO_NAMED("diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    // ros::SubscribeOptions so =
    //     ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
    //             boost::bind(&DiffDriveGazeboRos::cmdVelCallback, this, _1),
    //             ros::VoidPtr(), &queue_);x

    // cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);

    // Velocity Publisher
    velocity_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Twist>(velocity_topic_, 1);
    if ( velocity_publisher_ )
        ROS_INFO_NAMED("diff_drive", "%s: Publish velocity over %s ", gazebo_ros_->info(), velocity_topic_.c_str());
    else
        ROS_ERROR_NAMED("diff_drive", "%s: Cannot publish data over %s", gazebo_ros_->info(), velocity_topic_.c_str());
    
    // ROS Service client for model states
    model_state_client_ = gazebo_ros_->node()->serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    if ( model_state_client_ )
        ROS_INFO_NAMED("diff_drive", "%s: Connected to ROSserice server %s ", gazebo_ros_->info(), "gazebo/get_model_state");
    else
        ROS_ERROR_NAMED("diff_drive", "%s: Cannot connect to  ROSserice server %s", gazebo_ros_->info(), "gazebo/get_model_state");


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
    ROS_DEBUG("Mutex locked for getPose");

    // Get pose of the robot and target
    for(auto& x : objects)
    {
        ROS_DEBUG("Get pose of %s", x.second.name.c_str());

        // send request for model_state
        model_state_srv_.request.model_name = x.second.name;
        // model_state_srv_.request.relative_entity_name = x.second.name;
        if (model_state_client_.call(model_state_srv_))
        {
            ROS_DEBUG("Got data for %s", model_state_srv_.request.model_name.c_str());

            // Save pose
            // Model is accessed by 'x.second' -> mapped to 'objects'
            x.second.pose = model_state_srv_.response.pose;
        }
        else
        {
            ROS_ERROR("Failed to call service to get state of %s", model_state_srv_.request.model_name.c_str());
        }
    }
    
    // ROS_INFO("The robot is at %f, %f, heading towards %f and the ball at %f, %f", objects[ROBOT].pose.position.x, objects[ROBOT].pose.position.y, yaw, objects[TARGET].pose.position.x, objects[TARGET].pose.position.y );
}

void DiffDriveGazeboRos::publishVelocity ()
{

    // velocity_publisher_.publish ( velocity_ );
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
