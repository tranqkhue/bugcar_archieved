#include <pluginlib/class_list_macros.h>
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
        if(!exec_plan_){
            exec_plan_ = true;
            plan_size = pythonPlan->poses.size();
            if(plan_size != 0){
                ROS_INFO("\tGooglePlanner:");
                ROS_INFO("\nGot plan from google map planner, with plan size %d", plan_size);
                count = 1;
                got_plan_ = true;
                for (int i = 0; i < plan_size; ++i){
                    pose_google.push_back(pythonPlan->poses[i]);
                }                
            }
            else{
                got_plan_ = false;
            }
        }
    }
    void GooglePlanner::PublishPlan(const std::vector<geometry_msgs::PoseStamped>& plan_){
        global_plan_.header.frame_id = plan_[0].header.frame_id;
        global_plan_.header.stamp = plan_[0].header.stamp;
        for (int i = 0; i < plan_.size(); ++i){
            global_plan_.poses.push_back(plan_[i]);
        }
        pub_visualization.publish(global_plan_);
        global_plan_.poses.clear();
    }
    void GooglePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle nh;
        sub_google = nh.subscribe("GoogleMapGlobalPlanner/plan", 5, &GooglePlanner::plannerCallBack, this);
        pub_visualization = nh.advertise<nav_msgs::Path>("move_base/google_planner/plan", 1);
        ROS_INFO("\tGooglePlanner:");
        ROS_INFO("Finished Initialization");
    }

    
    bool GooglePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
        
        if(count == 0){
            ROS_INFO("\tGooglePlanner:");
            ROS_INFO("Request plan from move_base");
        }
        exec_plan_ = false;
        if(plan_size != 0 || got_plan_){
            // Add waypoints to plan
            ROS_INFO("\tGooglePlanner");
            ROS_INFO("Points in original plan: %d", (int)pose_google.size());
            plan.push_back(start);
            float dist = sqrt(pow(start.pose.position.x-pose_google[0].pose.position.x,2)
                             +pow(start.pose.position.y-pose_google[0].pose.position.y,2));
            if (dist<0.5){
                pose_google.erase(pose_google.begin());
            }
            for (int i = 0; i < pose_google.size(); ++i){
                plan.push_back(pose_google[i]);
                ROS_INFO("\tGooglePlanner:");
                ROS_INFO("Adding google waypoint %d to plan",i);
            }
            plan.push_back(goal);
            ROS_INFO("Total points in plan: %d", (int)plan.size());
            // Reset buffer
            ROS_INFO("\tGooglePlanner:");
            ROS_INFO("Clearing buffer...");
            pose_google.clear();
            ROS_INFO("Buffer size: %d", (int)pose_google.size());
            // Publish plan for visualization
            PublishPlan(plan);

            // Set conditions
            got_plan_ = false;
            exec_plan_ = true;
            count = 0;

            return true;
        }
        else{
            if(count == 0){
                ROS_INFO("\tGooglePlanner:");
                ROS_INFO("No plan from Google API");
            }
            return false;
        }
        
    }
};
