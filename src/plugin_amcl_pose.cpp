#include "gazebo-plugin-amcl-pose/plugin_amcl_pose.hpp"


namespace gazebo
{
    plugin_amcl_pose::plugin_amcl_pose()
    :   tfListener_(tfBuffer_)
    {
    }

    plugin_amcl_pose::~plugin_amcl_pose()
    {
    }

    void plugin_amcl_pose::Load(
        physics::ModelPtr _model,
        sdf::ElementPtr _sdf
    )
    {
        // Obtain parameters from xacro file
        if(_sdf->HasElement("topicRate"))
        {
            rate_ = _sdf->GetElement("topiceRate")->Get<double>();
        }
        else
        {
            rate_ = 10.0;
        }

        if(_sdf->HasElement("fixedFrame"))
        {
            fixed_frame_ = _sdf->GetElement("fixedFrame")->Get<std::string>();
        }
        else
        {
            fixed_frame_ = "map";
        }

        if(_sdf->HasElement("robotFrame"))
        {
            fixed_frame_ = _sdf->GetElement("robotFrame")->Get<std::string>();
        }
        else
        {
            fixed_frame_ = "base_link";
        }

        // Ensure that ROS node for Gazebo has been initialized
        if(!ros::isInitialized())
        {
            int argc = 0; char ** argv = NULL;
            ros::init(argc, argv, "gazebo_robot_battery_plugin_node", ros::init_options::NoSigintHandler);
        }

        // Set callback queue
        amcl_pose_nh_.setCallbackQueue(& amcl_pose_queue_);
        amcl_pose_pub_ = amcl_pose_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, true);

        thread_ptr_ = std::make_shared<std::thread>(
            [this]()
            {
                // Set process rate
                ros::Rate r(this->rate_);
                while (this->amcl_pose_nh_.ok())
                {
                    this->updateAmclPose();

                    this->amcl_pose_pub_.publish(this->msg_);

                    this->amcl_pose_queue_.callAvailable(ros::WallDuration(0.0));

                    r.sleep();
                }
            }
        );

    }


    bool plugin_amcl_pose::updateAmclPose()
    {
        bool isok {false};

        try
        {
            read_transformation_ = tfBuffer_.lookupTransform(fixed_frame_, robot_frame_, ros::Time(0), ros::Duration(60.0));

            msg_.header.frame_id = fixed_frame_;
            msg_.header.stamp = ros::Time::now();

            msg_.pose.pose.position.x = read_transformation_.transform.translation.x;
            msg_.pose.pose.position.y = read_transformation_.transform.translation.y;
            msg_.pose.pose.position.z = read_transformation_.transform.translation.z;

            msg_.pose.pose.orientation.x = read_transformation_.transform.rotation.x;
            msg_.pose.pose.orientation.y = read_transformation_.transform.rotation.y;
            msg_.pose.pose.orientation.z = read_transformation_.transform.rotation.z;
            msg_.pose.pose.orientation.w = read_transformation_.transform.rotation.w;
        }
        catch(const tf2::TransformException & e)
        {
            ROS_ERROR_STREAM(
                ros::this_node::getName() << " " << __func__ <<
                " transform exception: " << e.what()
            );
            return isok;
        }
        catch(const tf2::TimeoutException & e)
        {
            ROS_ERROR_STREAM(
                ros::this_node::getName() << " " << __func__ <<
                " timeout exception: " << e.what()
            );
            return isok;
        }

        return isok;
    }
} // namespace gazebo