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
        global_plan_pub.poses.clear();
        global_plan_pub.header.frame_id = plan_[0].header.frame_id;
        global_plan_pub.header.stamp = plan_[0].header.stamp;
        for (int i = 0; i < plan_.size(); ++i){
            global_plan_pub.poses.push_back(plan_[i]);
        }
        pub_visualization.publish(global_plan_pub);
    }

    void GooglePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle nh;
        ROS_INFO("\tGooglePlanner:");

        client_google = nh.serviceClient<bugcar_google_map_global_planner::GetPathMap>("get_path_map");
        ROS_INFO("Create client to get_path_map service");

        pub_visualization = nh.advertise<nav_msgs::Path>("move_base/GooglePlanner/plan", 1);
        ROS_INFO("Advertise move_base/GooglePlanner/plan topic to visualize global plan");

        //ROS_INFO("Finished Initialization");
    }
    
    bool GooglePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
        
        if(count == 0){
            ROS_INFO("\tGooglePlanner:");
            ROS_INFO("Request plan from google API...");
        }
        srv_path_map.request.map_goal.goal.target_pose = goal;
        count = 1;
        if(client_google.call(srv_path_map)){

            srv_path_map.response.goal_path.poses;
            plan_size = srv_path_map.response.goal_path.poses.size();
            if(plan_size != 0){
                // Add waypoints to plan
                ROS_INFO("\tGooglePlanner");
                ROS_INFO("Points in original plan: %d", plan_size);
                
                preProcessPlan(start, goal,plan_size);
                getHeadingRad();
                PublishPlan(pose_google);
                populatePlan();

                for (int i = 0; i < pose_google.size(); ++i){
                    pose_google[i].header.frame_id = start.header.frame_id;
                    pose_google[i].header.stamp = start.header.stamp;
                    plan.push_back(pose_google[i]);
                    ROS_INFO("Adding google waypoint %d to plan",i);
                }

                ROS_INFO("Total points in plan: %d", (int)plan.size());
                // Reset buffer
                ROS_INFO("\tGooglePlanner:");
                ROS_INFO("Clearing buffer...");
                pose_google.clear();
                ROS_INFO("Buffer size: %d", (int)pose_google.size());
                // Publish plan for visualization
                //PublishPlan(plan);

                // Set conditions
                got_plan_ = false;
                exec_plan_ = true;
                count = 0;

                return true;
            }
            else{
                ROS_INFO("!!! No Plan !!!");
                return false;
            }
        }
        else{
            ROS_INFO("!!!  No Plan  !!!");
            return false;
        }        
    }

    float GooglePlanner::calculate_distance(const geometry_msgs::PoseStamped& x1,
                                            const geometry_msgs::PoseStamped& x2){

        return sqrt(pow((x1.pose.position.x-x2.pose.position.x),2)+
                    pow((x1.pose.position.y-x2.pose.position.y),2));
    
    }

    void GooglePlanner::preProcessPlan(const geometry_msgs::PoseStamped& start,
                                       const geometry_msgs::PoseStamped& goal,
                                       int plan_size_){
        pose_google.clear();
        pose_google.push_back(start);
        for (int i = 0; i < plan_size_; ++i){
            pose_google.push_back(srv_path_map.response.goal_path.poses[i]);
        }
        pose_google.push_back(goal);

        float dist;
        for (int i = 1; i < pose_google.size()-1; ++i){
            if(i==0){
                dist = calculate_distance(start,pose_google[i]);
            }
            else if(i == pose_google.size()-1){
                dist = calculate_distance(goal,pose_google[i]);
            }
            else{
                dist = calculate_distance(pose_google[i],pose_google[i+1]);
            }
            if (dist<0.5){
                pose_google.erase(pose_google.begin()+i);
                i -= 1;
            }
        }
    }

    void GooglePlanner::populatePlan(){
        std::vector<geometry_msgs::PoseStamped> dummy_plan;
        geometry_msgs::PoseStamped dummy_point;
        float dist;
        int insert_point;
        float delta_x;
        float delta_y;

        for(int i = 0; i < pose_google.size(); ++i){
            if (i!=pose_google.size()-1){
                dummy_plan.push_back(pose_google[i]);
                dist = calculate_distance(pose_google[i],pose_google[i+1]);

                ROS_INFO("Distance between waypoint %d and %d is %f",i,i+1,dist);
                insert_point = floor(dist/point_sep);
                if (insert_point>=1){
                    ROS_INFO("Adding %d points between waypoint %d and %d", insert_point,i,i+1);
                    delta_x = (pose_google[i+1].pose.position.x - pose_google[i].pose.position.x)/dist;
                    delta_y = (pose_google[i+1].pose.position.y - pose_google[i].pose.position.y)/dist;
                    for(int j = 0; j < insert_point; ++j){
                        dummy_point.pose.position.x = pose_google[i].pose.position.x + delta_x*point_sep*(j+1);
                        dummy_point.pose.position.y = pose_google[i].pose.position.y + delta_y*point_sep*(j+1);
                        
                        if(i==0){
                            dummy_point.pose.orientation.z = sin(rad_heading[i]/2);
                            dummy_point.pose.orientation.w = cos(rad_heading[i]/2);
                        }
                        else{
                            dummy_point.pose.orientation = pose_google[i].pose.orientation;
                        }

                        dummy_plan.push_back(dummy_point);
                    }
                }
            }
            else{
                dummy_plan.push_back(pose_google[i]);
            }
        } 
        if(dummy_plan.size() != 0){
            pose_google.clear();
            for (int i = 0; i < dummy_plan.size(); ++i){
                pose_google.push_back(dummy_plan[i]);
            } 
            ROS_INFO("Populate old plan -> new plan now has %d points", (int)pose_google.size()); 
        }
    }

    void GooglePlanner::getHeadingRad(){
        float dx;
        float dy;
        rad_heading.clear();    
        for(int i = 0; i < pose_google.size(); ++i){
            if(i == pose_google.size()-1){
                rad_heading.push_back(atan2(pose_google[i].pose.orientation.z,
                                        pose_google[i].pose.orientation.w) * 2);
            }
            else{
                dx = pose_google[i+1].pose.position.x - pose_google[i].pose.position.x;
                dy = pose_google[i+1].pose.position.y - pose_google[i].pose.position.y;
                rad_heading.push_back(atan2(dy,dx));
                if(i!= 0){
                    pose_google[i].pose.orientation.z = sin(rad_heading[i]/2);
                    pose_google[i].pose.orientation.w = cos(rad_heading[i]/2);
                }
            }
        }
    }
};
