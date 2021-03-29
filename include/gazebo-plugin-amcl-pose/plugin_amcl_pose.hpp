#ifndef GZ_MODEL_PLUGIN_AMCL_POSE_H_
#define GZ_MODEL_PLUGIN_AMCL_POSE_H_

// Gazebo plugin features
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>

// ROS basic features
#include <ros/ros.h>
#include <ros/callback_queue.h>

// Dependencies
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// STL
#include <memory>
#include <thread>
#include <string>


namespace gazebo
{
    class plugin_amcl_pose : public ModelPlugin
    {
    private:
        ros::NodeHandle amcl_pose_nh_;
        ros::CallbackQueue amcl_pose_queue_;

        ros::Publisher amcl_pose_pub_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        std::shared_ptr<std::thread> thread_ptr_;

        double rate_;

        geometry_msgs::PoseWithCovarianceStamped msg_;
        geometry_msgs::TransformStamped read_transformation_;

        bool updateAmclPose();

        std::string fixed_frame_;
        std::string robot_frame_;


    public:
        plugin_amcl_pose();
        ~plugin_amcl_pose();

        virtual void Load(
            physics::ModelPtr _model,
            sdf::ElementPtr _sdf
        ) override;
    };

} // namespace gazebo


#endif