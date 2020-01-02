#ifndef PID_DIFFDRIVE_PLUGIN_HH
#define PID_DIFFDRIVE_PLUGIN_HH

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

class PIDDiffDrive : public ModelPlugin {

	public:
		PIDDiffDrive();
		~PIDDiffDrive();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		void Reset();

	protected:
		virtual void UpdateChild();
		virtual void FiniChild();

	private:
		void getPose();
		void publishVelocity();

		GazeboRosPtr gazebo_ros_;
		physics::ModelPtr parent;
		event::ConnectionPtr update_connection_;

		// ROS STUFF
		// ros::Publisher velocity_publisher_;
		// ros::Subscriber model_state_subscriber_;
		// geometry_msgs::Twist velocity_;

		// boost::mutex lock;

		// std::string velocity_topic_;
		// std::string model_state_topic_;
		// bool publish_twist_;

		// Custom Callback Queue
		ros::CallbackQueue queue_;
		boost::thread callback_queue_thread_;
		void QueueThread();

		// DiffDrive stuff
		// void modelStateCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

		bool alive_;

		// Update Rate
		double update_rate_;
		double update_period_;
		common::Time last_update_time_;
};

}

#endif