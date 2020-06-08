#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "google_planner.h"

PLUGINLIB_EXPORT_CLASS(google_planner::GooglePlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace google_planner {
    GooglePlanner::GooglePlanner(){

    }

    GooglePlanner::GooglePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }
    void GooglePlanner::plannerCallBack(const nav_msgs::Path::ConstPtr& pythonPlan){
        if(!car_busy){
            plan_size = sizeof(pythonPlan->poses)/sizeof(pythonPlan->poses[0]);
            for (int i = 0; i < plan_size; ++i){
                pose_.push_back(pythonPlan->poses[i]);
            }
            ROS_INFO("Got plan from google map planner, with plan size %d", plan_size);
        }
        else{
            ROS_INFO("Car is executing plan");
        }
    }
    void GooglePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle nh;
        sub_google = nh.subscribe("GoogleMapGlobalPlanner/plan", 5, &GooglePlanner::plannerCallBack, this);
        //sub_move_base_stt = nh.subscribe("move_base/status", 5, &GooglePlanner::statusCallBack, this);
        ROS_INFO("GooglePlanner:");
        ROS_INFO("Finished Initialization");
    }

    
    bool GooglePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan){
        plan.push_back(start);
        for (int i = 0; i < plan_size; ++i){
            plan.push_back(pose_[i]);
        }
        plan.push_back(goal);
        return true;
    }
};
