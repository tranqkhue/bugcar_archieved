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
#include <cmath>
#include <time.h>

using std::string;

#ifndef GOOGLE_PLANNER_CPP
#define GOOGLE_PLANNER_CPP

namespace google_planner {
    class GooglePlanner : public nav_core::BaseGlobalPlanner {
        public:
            GooglePlanner();
            GooglePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void PublishPlan(const std::vector<geometry_msgs::PoseStamped> &plan_);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& google_plan,
                          std::vector<geometry_msgs::PoseStamped>& plan);
            float calculate_distance(const geometry_msgs::PoseStamped& x1,
                                     const geometry_msgs::PoseStamped& x2);
            float calculate_orientation(const geometry_msgs::PoseStamped& x1,
                                        const geometry_msgs::PoseStamped& x2);
            void preProcessPlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                int plan_size_);
            void populatePlan();
            void getHeadingRad();

            std::string topic_name = "GoogleMapGlobalPlanner/plan";
            nav_msgs::Path global_plan_pub;
            int plan_size = 0;
            std::vector<geometry_msgs::PoseStamped> pose_google;
            ros::ServiceClient client_google;
            ros::Subscriber sub_move_base_stt;
            ros::Publisher pub_visualization;
            bugcar_google_map_global_planner::GetPathMap srv_path_map;
            bool car_busy = false;
            bool exec_plan_ = false;
            bool got_plan_ = false;
            int count = 0;
            float point_sep = 2.0;
            std::vector<float> rad_heading;

    };
};

#endif
