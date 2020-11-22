#include <pluginlib/class_list_macros.h>
#include "google_planner.h"

PLUGINLIB_EXPORT_CLASS(google_planner::GooglePlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace google_planner {
    GooglePlanner::GooglePlanner(){

    }
    GooglePlanner::~GooglePlanner(){
    }

    GooglePlanner::GooglePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void GooglePlanner::PublishPlan(const std::vector<geometry_msgs::PoseStamped>& plan_){
        // nav_msgs::Path takes std::vector<geometry_msgs::PoseStamped> as input ???
        //pub_visualization.publish(plan_);

    }

    void GooglePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle mb_nh("~/");
        ros::NodeHandle nh("~/" + name);
        name_ = nh.getNamespace();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nh.param("interpolate_distance", interpolate_distance, (float)1.0);
        nh.param("ignore_heading", ignore_heading, true);
        nh.param("publish_plan", publish_plan, false);
        nh.param("terminate_on_local_planner_fail", fail_terminate, true);
        nh.getParam("python_service", python_service);
        nh.getParam("api_key", api_key);

        ROS_INFO_STREAM("\t " << name_ << ":");

        if(fail_terminate){
            move_base_status_sub = mb_nh.subscribe<actionlib_msgs::GoalStatusArray>("status", 1, boost::bind(&GooglePlanner::mb_status_callback,this,_1));
        }
        if(python_service.empty() || api_key.empty()){
            ROS_FATAL_STREAM("A python service or an Google API key is missing");
            ROS_FATAL_STREAM("GooglePlanner requires a seperate Python service to acquire waypoints from Google API");
            //raise error and terminate
        }
        client_google = nh.serviceClient<bugcar_google_map_global_planner::GetPathMap>(python_service);
        ROS_INFO_STREAM("Created client to " << python_service << " service");

        if(publish_plan){
            pub_visualization = nh.advertise<nav_msgs::Path>("global_plan", 1);
            ROS_INFO_STREAM("Advertised " << name_ << "/global_plan topic for visualization");
        }

        //ROS_INFO("Finished Initialization");
    }
    
    bool GooglePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
        
        if(fail_terminate){
            ros::NodeHandle nh("~/");

            //check if move_base local planner failed
            //terminate
        }

        srv_path_map.request.map_goal.goal.target_pose = goal;
        previous_goal = goal;
        std::string err = "Can not contact service";

        if(client_google.call(srv_path_map)){
            srv_path_map.response.goal_path.poses;
            plan_size = srv_path_map.response.goal_path.poses.size();
            if(plan_size != 0){
                // Add waypoints to plan
                ROS_INFO_STREAM("\t" << name_ << ":");
                ROS_INFO_STREAM("Waypoints in original plan: " << srv_path_map.response.goal_path.poses.size());

                google_poses.clear();
                google_poses = srv_path_map.response.goal_path.poses;
                google_poses.insert(google_poses.begin(), start);
                google_poses.push_back(goal);                

                interpolatePlan();

                plan.clear();
                for (int i = 0; i < interpolated_poses.size(); ++i){
                    interpolated_poses[i].header.frame_id = global_frame_;
                    interpolated_poses[i].header.stamp = start.header.stamp;
                    plan.push_back(interpolated_poses[i]);
                    ROS_INFO("Added waypoint %d [%f;%f] to plan",i, interpolated_poses[i].pose.position.x
                                                                   , interpolated_poses[i].pose.position.y);
                }
                if(publish_plan)
                    PublishPlan(interpolated_poses);

                return true;
            }
            err = "Google plan returned empty";
        }
        ROS_FATAL_STREAM("\t" << name_ << ":");
        ROS_FATAL_STREAM(err);
        return false;
    }

    float GooglePlanner::calculate_distance(const geometry_msgs::PoseStamped& x1,
                                            const geometry_msgs::PoseStamped& x2){

        return sqrt(pow((x1.pose.position.x-x2.pose.position.x),2)+
                    pow((x1.pose.position.y-x2.pose.position.y),2));
    
    }

    void GooglePlanner::interpolatePlan(){

        interpolated_poses.clear();
        geometry_msgs::PoseStamped dummy_pose;
        float delta_x;
        float delta_y;
        int poses_to_interpolate;
        float distance_between_main_poses;
        float heading = 0;

        for(int i = 0; i < google_poses.size()-1; ++i){
            interpolated_poses.push_back(google_poses[i]);
            distance_between_main_poses = calculate_distance(google_poses[i],google_poses[i+1]);
            poses_to_interpolate = floor(distance_between_main_poses/interpolate_distance);
            
            delta_x = (google_poses[i+1].pose.position.x - google_poses[i].pose.position.x)/distance_between_main_poses;
            delta_y = (google_poses[i+1].pose.position.y - google_poses[i].pose.position.y)/distance_between_main_poses;
            
            if(!ignore_heading)
                heading = atan2(delta_y, delta_x);

            for(int j = 0; j < poses_to_interpolate; ++j){
                dummy_pose.pose.position.x = interpolated_poses.back().pose.position.x + delta_x*interpolate_distance;
                dummy_pose.pose.position.y = interpolated_poses.back().pose.position.y + delta_y*interpolate_distance;
                dummy_pose.pose.orientation.w = cos(heading/2);
                dummy_pose.pose.orientation.z = sin(heading/2);

                interpolated_poses.push_back(dummy_pose);
            } 
            ROS_INFO_STREAM("Interpolated " << poses_to_interpolate << 
            " poses between pose " << i << " [" << google_poses[i].pose.position.x << ";" << google_poses[i].pose.position.y << "] " <<
            " and pose " << i+1 << " [" << google_poses[i+1].pose.position.x << ";" << google_poses[i+1].pose.position.y << "]");
        }
        interpolated_poses.push_back(google_poses.back());
    }

    void GooglePlanner::getHeadingRad(){

    }
    
    void GooglePlanner::mb_status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr mb_status){
        // get goal id and compare if new goal was made
        
    }
};
