/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

// compute bezier curve
geometry_msgs::PoseStamped calculateBezierPoint(double t,
    const geometry_msgs::PoseStamped& p0,
    const geometry_msgs::PoseStamped& p1,
    const geometry_msgs::PoseStamped& p2,
    const geometry_msgs::PoseStamped& p3) {
    geometry_msgs::PoseStamped result;
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;

    // B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
    result.pose.position.x = mt3 * p0.pose.position.x +
                             3 * mt2 * t * p1.pose.position.x +
                             3 * mt * t2 * p2.pose.position.x +
                             t3 * p3.pose.position.x;
    result.pose.position.y = mt3 * p0.pose.position.y +
                             3 * mt2 * t * p1.pose.position.y +
                             3 * mt * t2 * p2.pose.position.y +
                             t3 * p3.pose.position.y;
    result.pose.position.z = 0.0; // 2d


    result.pose.orientation = p0.pose.orientation;
    result.header.frame_id = p0.header.frame_id;
    result.header.stamp = p0.header.stamp;

    return result;
}

float GlobalPlanner::fac(int x) {
    float f = 1;
    for (int i = 2; i <= x; i++) {
        f *= i;
    }
    return f;
}


void GlobalPlanner::smoothPathWithBezier(std::vector<geometry_msgs::PoseStamped>& path, int span_bezier, float dt) {
    if (path.size() < 6) {
        ROS_WARN("Path has fewer than 6 points, skipping Bezier smoothing.");
        return;
    }

    std::vector<geometry_msgs::PoseStamped> path_bezier;
    path_bezier.reserve(path.size() * 10);

    int count_bezier = 0;
    const int n = path.size() - 1;
    const int length_bezier = 5;

    while (count_bezier < n - 10 * span_bezier) {
        for (float t = 0.0; t <= 1.0; t += dt) {
            geometry_msgs::PoseStamped point_bezier;
            point_bezier.header.frame_id = frame_id_;
            point_bezier.header.stamp = path[count_bezier].header.stamp;
            point_bezier.pose.position.x = 0.0;
            point_bezier.pose.position.y = 0.0;
            point_bezier.pose.position.z = 0.0;
            point_bezier.pose.orientation = path[count_bezier].pose.orientation;


            for (int i = 0; i <= length_bezier; i++) {
                double k = fac(length_bezier) / (fac(i) * fac(length_bezier - i)) *
                           pow(t, i) * pow(1 - t, length_bezier - i);
                int idx = count_bezier + i * span_bezier;
                point_bezier.pose.position.x += k * path[idx].pose.position.x;
                point_bezier.pose.position.y += k * path[idx].pose.position.y;
            }
            path_bezier.push_back(point_bezier);
        }
        count_bezier += length_bezier * span_bezier;
    }

    while (count_bezier < n) {
        count_bezier++;
        path_bezier.push_back(path[count_bezier]);
    }

    path.clear();
    path = std::move(path_bezier);
}


void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true),
        p_calc_(NULL), planner_(NULL), path_maker_(NULL), orientation_filter_(NULL),
        potential_array_(NULL) {
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        GlobalPlanner() {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        raw_plan_pub_ = private_nh.advertise<nav_msgs::Path>("raw_plan", 1);
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb =
                [this](auto& config, auto level){ reconfigureCB(config, level); };
        dsrv_->setCallback(cb);

        //SG smoother parameters
        sg_smoother_.initialize(private_nh, name + "/sg_smoother");
        private_nh.param("use_sg_smoothing", use_sg_smoothing_, true);
        private_nh.param("sg_max_time", sg_max_time_, 1.0);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void GlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);

            // save raw plan
            std::vector<geometry_msgs::PoseStamped> raw_plan = plan;
            publishPlan(raw_plan, true);

            // convert it to support SG smooth
            use_sg_smoothing_ = true;
            if (use_sg_smoothing_) {
                nav_msgs::Path path_msg;
                path_msg.header = plan[0].header;
                path_msg.poses = plan;
                if (!sg_smoother_.smooth(path_msg, sg_max_time_)) {
                    ROS_WARN("SG smoothing failed!");
                }
                plan = path_msg.poses;
            }


            // bezier smooth
            smoothPathWithBezier(plan, 5, 0.1);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR_THROTTLE(5.0, "Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);


    //publish the plan for visualization purposes
    publishPlan(plan, false);
    delete[] potential_array_;
    return !plan.empty();
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, bool is_raw) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

//    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }
//
//    plan_pub_.publish(gui_path);
//    gui_path.poses = path;

    if (is_raw) {
        raw_plan_pub_.publish(gui_path);
        ROS_INFO("Published raw plan with %zu poses", path.size());
    } else {
        plan_pub_.publish(gui_path);
        ROS_INFO("Published final plan with %zu poses", path.size());
    }
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void GlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else {
            if (fabs(max) < DBL_EPSILON) {
                grid.data[i] = -1;
            } else {
                grid.data[i] = potential_array_[i] * publish_scale_ / max;
            }
        }
    }
    potential_pub_.publish(grid);
}

} //end namespace global_planner
