// #include <algorithm>
// #include <assert.h>

#include <irobot_plugin/diff_drive_gazebo_ros.h>

// #include <ignition/math/Angle.hh>
// #include <ignition/math/Pose3.hh>
// #include <ignition/math/Quaternion.hh>
// #include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <tf/tf.h>
// #include <cmath>

#include <ros/ros.h>
#include <ros/console.h>

namespace gazebo
{

// enum object {
//     ROBOT,
//     TARGET,
// };

int ROBOT, TARGET;

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

    // Set the relevant models : robot and target
    auto it = relevant_models.begin();
    it = relevant_models.insert(it, robot);
    relevant_models.insert(it, target);
    ROS_INFO_NAMED("diff_drive", "%s: Relevant models are %s and %s", gazebo_ros_->info(), relevant_models[0].c_str(), relevant_models[1].c_str());

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
    // Update the pose data from the simulator
    getPose();

    if (models.empty())
    {
        ROS_WARN_STREAM ("NO ROBOT AND TARGET DEFINED");
        return;
    }
    // PID?
    // ROS_INFO_STREAM ("ROBOT: " << models[ROBOT].name << "\tTARGET: " << models[TARGET].name);

    // Constant linear velocity
    const float vel = 0.5;

    // PID coeffs
    kP = 0.5;

    geometry_msgs::Twist velocity;

    // Stop robot if it is very near the target
    const float offset = 0.3;

    if ( std::abs(models[TARGET].pos.y - models[ROBOT].pos.y) < offset and std::abs(models[TARGET].pos.x - models[ROBOT].pos.x) < offset )
    {
        ROS_DEBUG_STREAM_THROTTLE (0.1, "Robot near the target, stopping robot");
        velocity.linear.x = 0;
        velocity.angular.z = 0;
    }
    else
    {
        // find desired angle
        theta_desired = atan2( (models[TARGET].pos.y - models[ROBOT].pos.y), (models[TARGET].pos.x - models[ROBOT].pos.x) );

        // Get error
        theta_error = theta_desired - models[ROBOT].pos.theta;
        // Adjust it between (-pi, pi]
        theta_error = atan2 ( sin(theta_error), cos(theta_error));

        ROS_INFO_STREAM_THROTTLE (0.1, "ROBOT: " << models[ROBOT].pos.x << " " << models[ROBOT].pos.y << "\tTARGET: " << models[TARGET].pos.x << " " << models[TARGET].pos.y << "\n\t\t\t\t\t\tANGLE: heading:" <<  models[ROBOT].pos.theta << "\tdesired: " << theta_desired << "\terror: " << theta_error);

        // calculate the Twist with constant linear velocity
        velocity.linear.x = vel;
        velocity.angular.z = kP * theta_error;
    }
   
    publishVelocity(velocity);

}

void DiffDriveGazeboRos::getPose()
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    // ROS_DEBUG ("::: Mutex locked for getPose :::");

    // Get pose of the robot and target
    std::vector<geometry_msgs::Pose> pose = msg_.pose;

    if (!models.empty())
    {
        // ROS_DEBUG_STREAM ("====== Get pose for each model ======");
        for (int i = 0; i < models.size(); i++)
        {
            ROS_DEBUG_STREAM ("Get pose of " << models[i].name);

            // Iterator for search
            auto search = model_index.find(models[i].name);
            if (search != model_index.end())
            {
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

                models[i].pos.x = models[i].pose.position.x;
                models[i].pos.y = models[i].pose.position.y;
                models[i].pos.theta = yaw;
            }
            else
            {
                ROS_ERROR_STREAM ("Model index not mapped");
            }
        }
    }

    // Print pose of each model
    for (auto model_ : models)
    {
        ROS_DEBUG_STREAM_THROTTLE (0.1, "Pose of " << model_.name << " : " << model_.pos.x << "\t" << model_.pos.y << "\t" << model_.pos.theta);
    }
  
}

void DiffDriveGazeboRos::publishVelocity ( const geometry_msgs::Twist _vel )
{
    geometry_msgs::Twist velocity = _vel;
    ROS_DEBUG_STREAM_THROTTLE (1,"Publishing velocity " << velocity.linear.x << "\t" << velocity.angular.z);
    velocity_publisher_.publish ( velocity );
}

void DiffDriveGazeboRos::modelStateCallback ( const gazebo_msgs::ModelStates::ConstPtr& _msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    // ROS_DEBUG ("::: Mutex locked for modelStateCallback :::");

    msg_ = *_msg;
    
    // Map model index
    if (models.empty())
    {
        std::vector<std::string> name = msg_.name;
        auto it = models.begin(); // iterator

        // Define the model_index map to map indexes
        for (int i=0; i<name.size(); i++)
        {
            const auto [var, success] = model_index.insert( {name[i], i} );
            if (!success)
                ROS_ERROR ("Model index mapping failed");
            
            // Add models to the models vector if it is in relevant models
            if (std::find (relevant_models.begin(), relevant_models.end(), name[i]) != relevant_models.end())
                it = models.insert(it, {name[i]});
            else ROS_INFO_STREAM ( name[i] << " not added to the models vector");
        }

        // Print models, also correct the enum values
        ROS_INFO_STREAM ("Models :");
        for (int i=0; i<models.size(); i++)
        {
            // Assign correct enum values
            if (models[i].name == robot)
                ROBOT = i;
            else if (models[i].name == target)
                TARGET = i;
            
            ROS_INFO_STREAM ("\t" << models[i].name );
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
