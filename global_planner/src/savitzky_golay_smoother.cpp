#include <global_planner/savitzky_golay_smoother.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <nav_core/base_global_planner.h>
#include <cmath>

namespace nav2_smoother {

SavitzkyGolaySmoother::SavitzkyGolaySmoother()
    : do_refinement_(true), refinement_num_(2) {}

void SavitzkyGolaySmoother::initialize(ros::NodeHandle& nh, const std::string& name) {
    nh_ = nh;
    nh_.param(name + "/do_refinement", do_refinement_, true);
    nh_.param(name + "/refinement_num", refinement_num_, 2);
    ROS_INFO("SavitzkyGolaySmoother initialized with do_refinement=%d, refinement_num=%d",
             do_refinement_, refinement_num_);
}

bool SavitzkyGolaySmoother::smooth(nav_msgs::Path& path, double max_time) {
    ROS_INFO("Starting SG smoothing with max_time: %.2f seconds", max_time);
    ros::Time start = ros::Time::now();
    double time_remaining = max_time;

    bool success = true, reversing_segment;
    nav_msgs::Path curr_path_segment;
    curr_path_segment.header = path.header;

    std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);
    ROS_INFO("Found %zu path segments to smooth (SG filter)", path_segments.size());

    for (unsigned int i = 0; i < path_segments.size(); i++) {
        if (path_segments[i].end - path_segments[i].start > 9) {
            curr_path_segment.poses.clear();
            std::copy(
                path.poses.begin() + path_segments[i].start,
                path.poses.begin() + path_segments[i].end + 1,
                std::back_inserter(curr_path_segment.poses));

            time_remaining = max_time - (ros::Time::now() - start).toSec();
            if (time_remaining <= 0.0) {
                ROS_WARN("Smoothing time exceeded allowed duration of %.2f seconds.", max_time);
                return false;
            }

            success = success && smoothImpl(curr_path_segment, reversing_segment);

            std::copy(
                curr_path_segment.poses.begin(),
                curr_path_segment.poses.end(),
                path.poses.begin() + path_segments[i].start);
        }
    }

    return success;
}

bool SavitzkyGolaySmoother::smoothImpl(nav_msgs::Path& path, bool& reversing_segment) {
    const unsigned int path_size = path.poses.size();

    const std::array<double, 7> filter = {
        -2.0 / 21.0, 3.0 / 21.0, 6.0 / 21.0, 7.0 / 21.0,
        6.0 / 21.0, 3.0 / 21.0, -2.0 / 21.0
    };

    auto applyFilter = [&](const std::vector<geometry_msgs::Point>& data)
        -> geometry_msgs::Point {
        geometry_msgs::Point val;
        for (unsigned int i = 0; i < filter.size(); i++) {
            val.x += filter[i] * data[i].x;
            val.y += filter[i] * data[i].y;
        }
        return val;
    };

    auto applyFilterOverAxes = [&](std::vector<geometry_msgs::PoseStamped>& plan_pts,
                                   const std::vector<geometry_msgs::PoseStamped>& init_plan_pts) {
        auto pt_m3 = init_plan_pts[0].pose.position;
        auto pt_m2 = init_plan_pts[0].pose.position;
        auto pt_m1 = init_plan_pts[0].pose.position;
        auto pt = init_plan_pts[1].pose.position;
        auto pt_p1 = init_plan_pts[2].pose.position;
        auto pt_p2 = init_plan_pts[3].pose.position;
        auto pt_p3 = init_plan_pts[4].pose.position;

        for (unsigned int idx = 1; idx < path_size - 1; idx++) {
            plan_pts[idx].pose.position = applyFilter({pt_m3, pt_m2, pt_m1, pt, pt_p1, pt_p2, pt_p3});
            pt_m3 = pt_m2;
            pt_m2 = pt_m1;
            pt_m1 = pt;
            pt = pt_p1;
            pt_p1 = pt_p2;
            pt_p2 = pt_p3;

            if (idx + 4 < path_size - 1) {
                pt_p3 = init_plan_pts[idx + 4].pose.position;
            } else {
                pt_p3 = init_plan_pts[path_size - 1].pose.position;
            }
        }
    };

    const auto initial_path_poses = path.poses;
    applyFilterOverAxes(path.poses, initial_path_poses);

    if (do_refinement_) {
        for (int i = 0; i < refinement_num_; i++) {
            const auto refined_initial_path_poses = path.poses;
            applyFilterOverAxes(path.poses, refined_initial_path_poses);
        }
    }

    updateApproximatePathOrientations(path, reversing_segment);
    return true;
}

void SavitzkyGolaySmoother::updateApproximatePathOrientations(nav_msgs::Path& path, bool /*reversing_segment*/) {
    for (size_t i = 1; i < path.poses.size(); i++) {
        double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        double yaw = std::atan2(dy, dx);
        path.poses[i-1].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
    path.poses.back().pose.orientation = path.poses[path.poses.size()-2].pose.orientation;
}

std::vector<PathSegment> SavitzkyGolaySmoother::findDirectionalPathSegments(const nav_msgs::Path& path) {
    std::vector<PathSegment> segments;
    PathSegment segment;
    segment.start = 0;
    segment.end = path.poses.size() - 1;
    segments.push_back(segment);
    return segments;
}

}  // namespace nav2_smoother

// use ros1 plugin
//PLUGINLIB_EXPORT_CLASS(nav2_smoother::SavitzkyGolaySmoother, nav_core::BaseGlobalPlanner)