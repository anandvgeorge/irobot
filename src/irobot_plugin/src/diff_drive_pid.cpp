#include "irobot_plugin/diff_drive_pid.h"

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <tf/tf.h>

#include <ros/console.h>

using namespace gazebo;

enum {
    ROBOT,
    TARGET,
};

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

    rosnode_->param<std::string> ( "velocityTopic", velocity_topic_, "cmd_vel" );
    rosnode_->param<std::string> ( "modelStateTopic", model_state_topic_, "gazebo/model_states" );

    // model_state_subscriber_ = rosnode_->subscribe(model_state_topic_, 1, &DiffDrivePID::modelStateCallback, this );
    // if ( model_state_subscriber_ )
    //     ROS_INFO("DiffDrivePID: Subscribe to %s", model_state_topic_.c_str());

    model_state_client_ = rosnode_->serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

    velocity_publisher_ = rosnode_->advertise<geometry_msgs::Twist>(velocity_topic_, 1);
    if ( velocity_publisher_ )
        ROS_INFO("DiffDrivePID: Advertise velocity on %s ", velocity_topic_.c_str());

    // Get then name of the parent model
    // robot_name = _sdf->GetParent()->Get<std::string>("name");

    // target_name = "cricket_ball";

    gazebo::DiffDrivePID::model robot { _sdf->GetParent()->Get<std::string>("name") };
    gazebo::DiffDrivePID::model target { "cricket_ball" };

    // map objects
    objects = {
        { ROBOT , robot},
        { TARGET , target}
    };

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DiffDrivePID::UpdateChild, this));

    ROS_INFO("Model name: %s", robot.name.c_str());
    // gzdbg << "plugin model name: " << robot_name << "\n";
}

void DiffDrivePID::UpdateChild()
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

void DiffDrivePID::getPose()
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

// void DiffDrivePID::modelStateCallback ( const gazebo_msgs::ModelStates::ConstPtr& _msg )
// {
//     boost::mutex::scoped_lock scoped_lock ( lock );
//     ROS_DEBUG("Mutex locked for modelStateCallback");
//     // x_ = cmd_msg->linear.x;
//     // rot_ = cmd_msg->angular.z;

//     // Do something useful here
//     // ROS_INFO("Hope this msg gets printed %s", _msg.c_str());
// }

void DiffDrivePID::publishVelocity ()
{

    velocity_publisher_.publish ( velocity_ );
}

void DiffDrivePID::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(DiffDrivePID);