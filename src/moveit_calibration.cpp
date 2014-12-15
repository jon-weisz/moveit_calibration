#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <joint_states_settler/joint_states_settler.h>
#include <calibration_msgs/Interval.h>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

class MoveitCalibrationScene
{
    class SettledJointsSynchronizer
    {
    public:
        SettledJointsSynchronizer():
        nh_("~"), published_interval_(false)
        {
            nh_.getParam("minimum_duration",&min_duration_, 2);
            nh_.getParam("joint_tolerance", &joint_tolerance_, .005);
            nh_.getParam("max_step", &max_step_, .05);

            joint_state_sub_ = nh_.subscribe("joint_states", 1, &MoveitCalibrationScene::jointStatesCallback, this);

            interval_pub_ = nh_.advertise<calibration_msgs::Interval>("settled_interval", 1);
            pruned_pub_ = nh_.advertise<sensor_msgs::JointState>("settled_joints", 1);
            sensor_state_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_cloud");
            sensor_state_sub_ = message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,"input_cloud", 1);
            sensor_cache_sub_ =  message_filters::Cache<sensor_msgs::PointCloud2>(sub, 5);
        }


        bool configure(std::vector<std::string> & joint_name_list)
        {
            joint_states_settler::ConfigGoal goal;

            std::vector<double> joint_tolerances(joint_name_list.size(), joint_tolerance_);
            goal.joint_names = joint_name_list;
            goal.tolerances = joint_tolerances;
            goal.max_step = max_step_;
            ROS_INFO_STREAM("Configuration: " << goal);

            return settler_.configure(goal);
        }

    private:


        void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
        {

          // Add the joint state message, and get the latest processed settled interval
          calibration_msgs::Interval interval = settler_.add(msg);
          if(!published_interval_ && interval.end - interval.start > min_duration_)
          {
              std::vector<sensor_msgs::PointCloud2Ptr> sensor_interval = sensor_cache_sub_.getInterval(interval.start, interval.end);
              if(!sensor_interval.size())
                  return;

              // Publish the interval
              interval_pub_.publish(interval);

              // Build the joint state message for this subset of joints
              pruned_pub_.publish(settler_.pruneJointState(msg));

              // Output first sensor message within the interval
              sensor_state_pub_.publish(sensor_interval[0]);

              // Disable more publication
              published_interval_ = true;
           else
              published_interval_ = false;
          }
        }

        ros::NodeHandle nh_;

        bool published_interval_;
        ros::Duration min_duration_;

        joint_states_settler::JointStatesSettler settler_;
        ros::Subscriber joint_state_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sensor_state_sub_;
        message_filters::Cache<sensor_msgs::PointCloud2> sensor_cache_sub_;

        ros::Publisher sensor_state_pub_;
        ros::Publisher interval_pub_;
        ros::Publisher pruned_pub_;


        double joint_tolerance_;
        double max_step_;
    };

public:
    MoveitCalibrationScene():
    nh_('~'){

    }

    void initialize()
    {

        std::string robot_description_name("robot_description");
        bool planning_scene_ok;

        initialized_ = false;

        planning_scene_.reset(new planning_scene_monitor::PlanningSceneMonitor(
                                  robot_description_name));

        planning_scene_ok = planning_scene_;
        if(!planning_scene_ok)
        {
            ROS_ERROR_STREAM("Failed to load planning scene with robot_description");
            return;
        }

        if(!synchronizer.configure(
                    planning_scene_->getRobotModel()->getVariableNames()))
        {
            initialized_ = false;
            return;
        }

        //Make the planning scene monitor use the settled states.
        planning_scene_->stopStateMonitor();
        planning_scene_->startStateMonitor("settled_joints");


        initialized_ = true;

    }




private:
    ros::NodeHandle nh_;
    SettledJointsSynchronizer synchronizer_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_;
    bool initialized_;

};
