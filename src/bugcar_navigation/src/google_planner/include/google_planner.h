#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <cmath>

using std::string;

#ifndef GOOGLE_PLANNER_CPP
#define GOOGLE_PLANNER_CPP

namespace google_planner {
    class GooglePlanner : public nav_core::BaseGlobalPlanner {
        public:
            GooglePlanner();
            GooglePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void plannerCallBack(const nav_msgs::Path::ConstPtr& pythonPlan);
            void PublishPlan(const std::vector<geometry_msgs::PoseStamped> &plan_);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan);

            std::string topic_name = "GoogleMapGlobalPlanner/plan";
            nav_msgs::Path global_plan_;
            int plan_size = 0;
            std::vector<geometry_msgs::PoseStamped> pose_google;
            ros::Subscriber sub_google;
            ros::Subscriber sub_move_base_stt;
            ros::Publisher pub_visualization;
            bool car_busy = false;
            bool exec_plan_ = false;
            bool got_plan_ = false;
            int count = 0;

    };
};

#endif
