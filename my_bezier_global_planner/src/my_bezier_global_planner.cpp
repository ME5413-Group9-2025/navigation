#include <my_bezier_global_planner/my_bezier_global_planner.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>

namespace my_bezier_global_planner {
MyBezierGlobalPlanner::MyBezierGlobalPlanner() : costmap_ros_(nullptr) {}

MyBezierGlobalPlanner::~MyBezierGlobalPlanner() {}

void MyBezierGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!costmap_ros_) {
        costmap_ros_ = costmap_ros;
        global_planner_.initialize(name, costmap_ros_);
        ROS_INFO("Initialized MyBezierGlobalPlanner");
    }
}

bool MyBezierGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                     const geometry_msgs::PoseStamped& goal,
                                     std::vector<geometry_msgs::PoseStamped>& plan) {
    std::vector<geometry_msgs::PoseStamped> rough_path;
    if (!global_planner_.makePlan(start, goal, rough_path)) {
        ROS_ERROR("Failed to generate rough path using GlobalPlanner");
        return false;
    }

    if (rough_path.size() < 2) {
        ROS_ERROR("Rough path has less than 2 points");
        return false;
    }

    // Extract control points
    geometry_msgs::Point p0 = rough_path[0].pose.position;
    geometry_msgs::Point p3 = rough_path.back().pose.position;
    geometry_msgs::Point p1, p2;

    if (rough_path.size() >= 3) {
        p1 = rough_path[1].pose.position;
    } else {
        p1.x = p0.x + 0.33 * (p3.x - p0.x);
        p1.y = p0.y + 0.33 * (p3.y - p0.y);
        p1.z = p0.z + 0.33 * (p3.z - p0.z);
    }

    if (rough_path.size() >= 4) {
        p2 = rough_path[rough_path.size() - 2].pose.position;
    } else {
        p2.x = p3.x + 0.33 * (p0.x - p3.x);
        p2.y = p3.y + 0.33 * (p0.y - p3.y);
        p2.z = p3.z + 0.33 * (p0.z - p3.z);
    }

    // Generate Bézier curve path
    std::vector<geometry_msgs::PoseStamped> bezier_path = generateBezierPath(p0, p1, p2, p3);

    // Set orientations
    plan = bezier_path;
    for (size_t i = 0; i < plan.size() - 1; ++i) {
        geometry_msgs::Point current = plan[i].pose.position;
        geometry_msgs::Point next = plan[i + 1].pose.position;
        double dx = next.x - current.x;
        double dy = next.y - current.y;
        double yaw = atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);  // Roll, Pitch, Yaw
        plan[i].pose.orientation = tf2::toMsg(q);
    }

    return true;
}

geometry_msgs::Point MyBezierGlobalPlanner::computeBezierPoint(double t,
                                                               const geometry_msgs::Point& p0,
                                                               const geometry_msgs::Point& p1,
                                                               const geometry_msgs::Point& p2,
                                                               const geometry_msgs::Point& p3) {
    double u = 1.0 - t;
    double tt = t * t;
    double uu = u * u;
    double uuu = uu * u;
    double ttt = tt * t;

    geometry_msgs::Point point;
    point.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
    point.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
    point.z = uuu * p0.z + 3 * uu * t * p1.z + 3 * u * tt * p2.z + ttt * p3.z;
    return point;
}

std::vector<geometry_msgs::PoseStamped> MyBezierGlobalPlanner::generateBezierPath(
    const geometry_msgs::Point& p0,
    const geometry_msgs::Point& p1,
    const geometry_msgs::Point& p2,
    const geometry_msgs::Point& p3) {
    std::vector<geometry_msgs::PoseStamped> path;
    for (double t = 0.0; t <= 1.0; t += 0.01) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position = computeBezierPoint(t, p0, p1, p2, p3);
        pose.pose.orientation.w = 1.0;  // Initial orientation
        path.push_back(pose);
    }
    return path;
}
}  // namespace my_bezier_global_planner

PLUGINLIB_EXPORT_CLASS(my_bezier_global_planner::MyBezierGlobalPlanner, nav_core::BaseGlobalPlanner)