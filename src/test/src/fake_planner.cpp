#include <pluginlib/class_list_macros.h>
#include "fake_planner.h"

PLUGINLIB_EXPORT_CLASS(fake_planner::FakePlanner, nav_core::BaseGlobalPlanner)

namespace fake_planner{
    FakePlanner::FakePlanner(){

    }

    FakePlanner::FakePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void FakePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle private_nh("~/" + name);
        pub_points = private_nh.advertise<nav_msgs::Path>("plan",1);
        ROS_INFO("\tFakePlanner"); 
        ROS_INFO("Finished Initialize");
    }

    bool FakePlanner::generateRandPlan(const geometry_msgs::PoseStamped& start,
                                       const geometry_msgs::PoseStamped& goal){

        srand(time(NULL));
        random_samples = 1 + rand()%3;
        way_point_orientation.clear();

        double a = atan2((goal.pose.position.y-start.pose.position.y)
                        ,(goal.pose.position.x-start.pose.position.x));
        float delta = (goal.pose.position.x - start.pose.position.x)/(float)(random_samples+1);

        geometry_msgs::PoseStamped waypoint;
        double a_ = 0.0;
        rand_waypoint.push_back(start);
        for(int i = 1; i <= random_samples; ++i){
            a_ = a + (rand()%2-1)*((float)(rand()%157)/100.0);
            way_point_orientation.push_back(a_);
            waypoint.pose.position.x = start.pose.position.x + delta*i;
            waypoint.pose.position.y = start.pose.position.y + tan(a_)*delta*i;

            waypoint.pose.orientation.x = 0;
            waypoint.pose.orientation.y = 0;
            waypoint.pose.orientation.z = sin(a_/2);
            waypoint.pose.orientation.w = cos(a_/2);

            rand_waypoint.push_back(waypoint);
        }
        rand_waypoint.push_back(goal);
        ROS_INFO("Generate random plan with size of %d",(int)rand_waypoint.size());
        return true;

    }

    void FakePlanner::populatePlan(float delta){

        std::vector<geometry_msgs::PoseStamped> dummy_plan;
        geometry_msgs::PoseStamped dummy_point;
        float dist;
        float delta_x;
        float delta_y;
        int insert_point;

        for(int i = 0; i < rand_waypoint.size()-1; ++i){
            dummy_plan.push_back(rand_waypoint[i]);
            delta_x = rand_waypoint[i+1].pose.position.x -rand_waypoint[i].pose.position.x;
            delta_y = rand_waypoint[i+1].pose.position.y- rand_waypoint[i].pose.position.y;
            dist = sqrt(pow(delta_x,2) + pow(delta_y,2));
            ROS_INFO("Distance between waypoint %d and %d is %f",i,i+1,dist);
            insert_point = floor(dist/delta);
            if (insert_point>=1){
                ROS_INFO("Adding %d points between waypoint %d and %d", insert_point-1,i,i+1);
                delta_x = delta_x/dist;
                delta_y = delta_y/dist;
                for(int j = 0; j < insert_point; ++j){
                    dummy_point.pose.position.x = rand_waypoint[i].pose.position.x + delta_x*delta*(j+1);
                    dummy_point.pose.position.y = rand_waypoint[i].pose.position.y + delta_y*delta*(j+1);
                    
                    dummy_point.pose.orientation.z = sin(atan2(delta_y,delta_x)/2);
                    dummy_point.pose.orientation.w = cos(atan2(delta_y,delta_x)/2);

                    dummy_plan.push_back(dummy_point);
                }
            }
            if(i == rand_waypoint.size()-2){
                dummy_plan.push_back(rand_waypoint[i+1]);
            }
        } 
        if(dummy_plan.size() != 0){
            rand_waypoint.clear();
            for (int i = 0; i < dummy_plan.size(); ++i){
                rand_waypoint.push_back(dummy_plan[i]);
            } 
            ROS_INFO("Populate old plan -> new plan now has %d points", (int)rand_waypoint.size()); 
        }
            
    }

    void FakePlanner::PublishPlan(){
        nav_msgs::Path plan_;
        plan_.header.frame_id = "map";  
        for(int i = 0; i < rand_waypoint.size(); ++i){
            plan_.poses.push_back(rand_waypoint[i]);
        }
        pub_points.publish(plan_);
    }

    bool FakePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan){

        
        generateRandPlan(start,goal);
        PublishPlan();
        populatePlan(delta);
        for (int i = 0; i < rand_waypoint.size(); ++i){
            rand_waypoint[i].header.frame_id = "map";
            plan.push_back(rand_waypoint[i]);
        }
        rand_waypoint.clear();
        return true;
    }
};