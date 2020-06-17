#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>

using std::string;

#ifndef FAKE_PLANNER_CPP
#define FAKE_PLANNER_CPP

namespace fake_planner{
    class FakePlanner : public nav_core::BaseGlobalPlanner {
        public:
            FakePlanner();
            FakePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void PublishPlan();

            bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan);

            bool generateRandPlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal);

            void populatePlan(float delta);
            int random_samples;
            ros::Publisher pub_points;
            std::vector<geometry_msgs::PoseStamped> rand_waypoint;
            std::vector<double> way_point_orientation;
            float delta = 2;     
    };
};

#endif