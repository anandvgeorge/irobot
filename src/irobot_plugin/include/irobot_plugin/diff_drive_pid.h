#ifndef DIFF_DRIVE_PID_H
#define DIFF_DRIVE_PID_H

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
// #include <gazebo/physics/PhysicsTypes.hh>
// #include <gazebo/sensors/SensorTypes.hh>
// #include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
// #include <gazebo/common/Events.hh>

// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/TwistWithCovariance.h>
// #include <geometry_msgs/PoseWithCovariance.h>
// #include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "gazebo_msgs/ModelStates.h"

namespace gazebo
{
class DiffDrivePID : public ModelPlugin
{
	public:
		DiffDrivePID();
		virtual ~DiffDrivePID();

		virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

	protected:
		virtual void UpdateChild();

	private:
		void getPose();
		void publishVelocity();
		void modelStateCallback( const gazebo_msgs::ModelStates::ConstPtr& cmd_msg );
		
		/// Parameters
		std::string node_namespace_;
		std::string velocity_topic_;
		std::string model_state_topic_;
		std::string update_rate_;

		// ROS STUFF
		ros::NodeHandle *rosnode_;
		//ros::Service operating_mode_srv_;

		ros::Publisher velocity_publisher_;
		ros::Subscriber model_state_subscriber_;
		geometry_msgs::Twist velocity_;

		physics::WorldPtr my_world_;
		physics::ModelPtr my_parent_;

		// Simulation time of the last update
		common::Time prev_update_time_;

		void spin();
		boost::thread *spinner_thread_;
		boost::mutex lock;		

		transport::NodePtr gazebo_node_;
		transport::SubscriberPtr contact_sub_;

		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
};
}
#endif