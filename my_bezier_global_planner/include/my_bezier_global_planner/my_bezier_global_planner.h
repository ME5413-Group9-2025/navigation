#ifndef MY_BEZIER_GLOBALS_PLANNER_H
#define MY_BEZIER_GLOBALS_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace my_bezier_global_planner {
class MyBezierGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    MyBezierGlobalPlanner();
    ~MyBezierGlobalPlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    global_planner::GlobalPlanner global_planner_;
    geometry_msgs::Point computeBezierPoint(double t,
                                            const geometry_msgs::Point& p0,
                                            const geometry_msgs::Point& p1,
                                            const geometry_msgs::Point& p2,
                                            const geometry_msgs::Point& p3);
    std::vector<geometry_msgs::PoseStamped> generateBezierPath(const geometry_msgs::Point& p0,
                                                               const geometry_msgs::Point& p1,
                                                               const geometry_msgs::Point& p2,
                                                               const geometry_msgs::Point& p3);
};
}  // namespace my_bezier_global_planner

#endif