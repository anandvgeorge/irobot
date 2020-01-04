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
#include <ros/console.h>

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

    robot = _sdf->GetParent()->Get<std::string>("name");

    // Set log level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
        ros::console::notifyLoggerLevelsChanged();

    gazebo_ros_->getParameter<std::string> ( velocity_topic_, "velocityTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( model_state_topic_, "modelStateTopic", "gazebo/model_states" );
    gazebo_ros_->getParameter<std::string> ( target, "targetModel", "cricket_ball" );

    alive_ = true;

    ROS_INFO_NAMED("diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), model_state_topic_.c_str());

    // Model state subscriber
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(model_state_topic_, 1,
                boost::bind(&DiffDriveGazeboRos::modelStateCallback, this, _1),
                ros::VoidPtr(), &queue_);

    model_state_subscriber_ = gazebo_ros_->node()->subscribe(so);
    if ( model_state_subscriber_ )
        ROS_INFO_NAMED("diff_drive", "%s: Subscribed to %s ", gazebo_ros_->info(), model_state_topic_.c_str());
    else
        ROS_ERROR_NAMED("diff_drive", "%s: Cannot subscribe to %s", gazebo_ros_->info(), model_state_topic_.c_str());
    
    // Velocity Publisher
    velocity_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Twist>(velocity_topic_, 1);
    if ( velocity_publisher_ )
        ROS_INFO_NAMED("diff_drive", "%s: Publish velocity over %s ", gazebo_ros_->info(), velocity_topic_.c_str());
    else
        ROS_ERROR_NAMED("diff_drive", "%s: Cannot publish data over %s", gazebo_ros_->info(), velocity_topic_.c_str());
    
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

}

void DiffDriveGazeboRos::getPose()
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    ROS_DEBUG ("::: Mutex locked for getPose :::");

    // Get pose of the robot and target
    std::vector<geometry_msgs::Pose> pose = msg_.pose;

    if (!models.empty())
    {
        ROS_DEBUG_STREAM ("====== Get pose for each model ======");
        for (int i = 0; i < models.size(); i++)
        {
            ROS_DEBUG_STREAM ("Get pose of " << models[i].name);

            // Iterator for search
            auto search = model_index.find(models[i].name);
            if (search != model_index.end())
            {
                ROS_DEBUG_STREAM ("index of " << models[i].name << " is " << search->second);
            }
            else
            {
                ROS_ERROR_STREAM ("Model index not mapped");
            }
            
            // Get pose
            models[i].pose = pose.at(search->second);

            // Calculate yaw and assign to Pose2D pos
            tf::Quaternion q(
                models[i].pose.orientation.x,
                models[i].pose.orientation.y,
                models[i].pose.orientation.z,
                models[i].pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            models[i].pos.x = models[i].pose.orientation.x;
            models[i].pos.y = models[i].pose.orientation.y;
            models[i].pos.theta = yaw;
        }
    }

    // Print pose of each model
    for (auto model_ : models)
    {
        ROS_DEBUG_STREAM ("Pose of " << model_.name << " is " << model_.pos.x << "\t" << model_.pos.y << "\t" << model_.pos.theta);
    }
  
}

void DiffDriveGazeboRos::publishVelocity ()
{

    // velocity_publisher_.publish ( velocity_ );
}

void DiffDriveGazeboRos::modelStateCallback ( const gazebo_msgs::ModelStates::ConstPtr& _msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    ROS_DEBUG ("::: Mutex locked for modelStateCallback :::");

    msg_ = *_msg;
    
    // Map model index
    if (models.empty()) // Change the logic to check for extra models from the gazebo (there will be extra spawning!)
    {
        std::vector<std::string> name = msg_.name;
        auto it = models.begin(); // iterator

        // Define the model_index map to map indexes
        for (int i=0; i<name.size(); i++)
        {
            const auto [var, success] = model_index.insert( {name[i], i} );
            if (!success)
                ROS_ERROR ("Model index mapping failed");
            
            // Add models to the models vector
            it = models.insert(it, {name[i]});
        }

        // Print models
        ROS_INFO_STREAM ("Models :");
        for (auto model_ : models)
        {
            ROS_INFO_STREAM ("\t" << model_.name );
        }

        // Print the map
        ROS_INFO_STREAM ("Model index mapping :");
        for (auto [model_, index_] : model_index)
        {
            ROS_INFO_STREAM ("\t" << model_ << " : " << index_);
        }
    } 
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
