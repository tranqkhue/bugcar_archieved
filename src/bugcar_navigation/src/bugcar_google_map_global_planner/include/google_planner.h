#ifndef GOOGLE_PLANNER_CPP
#define GOOGLE_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <bugcar_google_map_global_planner/GetPathLL.h>
#include <bugcar_google_map_global_planner/GetPathMap.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <time.h>



namespace google_planner {
    static const int RUNNING_PLAN = 1;
    static const int INTERPOLATING_PLAN = 2;
    static const int REQUESTING_PLAN = 3;
    class GooglePlanner : public nav_core::BaseGlobalPlanner {
        public:
            GooglePlanner();
            ~GooglePlanner();
            GooglePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void PublishPlan(const std::vector<geometry_msgs::PoseStamped> &plan_);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan);
            
            float calculate_distance(const geometry_msgs::PoseStamped& x1,
                                     const geometry_msgs::PoseStamped& x2);
        
            void interpolatePlan();
            void getHeadingRad();
            void mb_status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr mb_status);

            nav_msgs::Path global_plan_pub;
            int plan_size = 0;
            std::string name_;
            std::string python_service;
            std::string api_key;

            std::string global_frame_;
            geometry_msgs::PoseStamped previous_goal;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tf_listener{tfBuffer};

            std::vector<geometry_msgs::PoseStamped> interpolated_poses;
            std::vector<geometry_msgs::PoseStamped> google_poses;
            ros::ServiceClient client_google;
            ros::Subscriber move_base_status_sub;
            ros::Publisher pub_visualization;

            bugcar_google_map_global_planner::GetPathMap srv_path_map;
            bool car_busy = false;
            bool exec_plan_ = false;
            bool got_plan_ = false;
            int count = 0;
            int status = 0;

            float interpolate_distance = 2.0;
            bool ignore_heading;
            bool publish_plan;
            bool fail_terminate;

            std::vector<float> rad_heading;

    };
};

#endif
