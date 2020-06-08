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
        ROS_INFO("\tGooglePlanner:");

        client_google = nh.serviceClient<bugcar_google_map_global_planner::GetPathMap>("get_path_map");
        ROS_INFO("Create client to get_path_map service");

        pub_visualization = nh.advertise<nav_msgs::Path>("move_base/GooglePlanner/plan", 1);
        ROS_INFO("Advertise move_base/GooglePlanner/plan topic to visualize global plan");

        ROS_INFO("Finished Initialization");
    }

    
    bool GooglePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
        
        if(count == 0){
            ROS_INFO("\tGooglePlanner:");
            ROS_INFO("Request plan from google API...");
        }
        exec_plan_ = false;
        if(plan_size != 0 && got_plan_){
            srv_path_map.request.map_goal.goal.target_pose = goal;
            if(client_google.call(srv_path_map)){
                
                srv_path_map.response.goal_path.poses = pose_google;
                
                // Add waypoints to plan
                ROS_INFO("\tGooglePlanner");
                ROS_INFO("Points in original plan: %d", (int)pose_google.size());
                plan.push_back(start);
                float dist = sqrt(pow(start.pose.position.x-pose_google[0].pose.position.x,2)
                                +pow(start.pose.position.y-pose_google[0].pose.position.y,2));
                if (dist<0.5){
                    ROS_INFO("Point 0 in plan and start are too close to each other");
                    ROS_INFO("Remove point 0 from original plan");
                    pose_google.erase(pose_google.begin());
                }
                for (int i = 0; i < pose_google.size(); ++i){
                    plan.push_back(pose_google[i]);
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
                return false;
            }
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
