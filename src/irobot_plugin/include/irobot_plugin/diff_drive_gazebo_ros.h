#ifndef DIFFDRIVE_GAZEBO_ROS_HH
#define DIFFDRIVE_GAZEBO_ROS_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <irobot_plugin/gazebo_ros_utils.h> // Added since the one with gazebo_plugins not found

// ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class DiffDriveGazeboRos : public ModelPlugin {

    public:
      DiffDriveGazeboRos();
      ~DiffDriveGazeboRos();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void getPose();
      void publishVelocity( const geometry_msgs::Twist _vel );
      void modelStateCallback( const gazebo_msgs::ModelStates::ConstPtr& _msg );

      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      // ROS STUFF
      ros::Publisher velocity_publisher_;
      ros::Subscriber model_state_subscriber_;
      ros::ServiceClient model_state_client_;
      gazebo_msgs::GetModelState model_state_srv_;
      gazebo_msgs::ModelStates msg_;

      struct model
        {
          std::string name;
          geometry_msgs::Pose pose;
          geometry_msgs::Pose2D pos;
        };

      std::vector <DiffDriveGazeboRos::model> models;
      std::map <std::string, int> model_index;
      std::vector <std::string> relevant_models;

      boost::mutex lock;

      std::string velocity_topic_;
      std::string model_state_topic_;
      std::string robot;
      std::string target;

      // PID
      float theta_desired;
      float theta_error;
      float kP;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      bool alive_;
  };

}

#endif
