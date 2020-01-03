#ifndef DIFF_DRIVE_PID_H
#define DIFF_DRIVE_PID_H

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>

#include <map>

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
        void modelStateCallback( const gazebo_msgs::ModelStates::ConstPtr& _msg );
        
        /// Parameters
        std::string node_namespace_;
        std::string velocity_topic_;
        std::string model_state_topic_;
        std::string update_rate_;

        // ROS STUFF
        ros::NodeHandle *rosnode_;
        //ros::Service operating_mode_srv_;

        ros::Publisher velocity_publisher_;
        // ros::Subscriber model_state_subscriber_;
        geometry_msgs::Twist velocity_;
        ros::ServiceClient model_state_client_;
        gazebo_msgs::GetModelState model_state_srv_;
        
        struct model
        {
            std::string name;
            geometry_msgs::Pose pose;
        };

        physics::WorldPtr my_world_;
        physics::ModelPtr my_parent_;

        
        // std::string robot_name;
        // std::string target_name;
        // geometry_msgs::Pose robot_pose_;
        // geometry_msgs::Pose target_pose_;

        std::map<int, gazebo::DiffDrivePID::model> objects;

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