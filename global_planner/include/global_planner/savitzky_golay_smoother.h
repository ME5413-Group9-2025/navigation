#ifndef SAVITZKY_GOLAY_SMOOTHER_H
#define SAVITZKY_GOLAY_SMOOTHER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <array>

namespace nav2_smoother {

struct PathSegment {
    unsigned int start;
    unsigned int end;
};

class SavitzkyGolaySmoother {
public:
    SavitzkyGolaySmoother();
    void initialize(ros::NodeHandle& nh, const std::string& name);
    bool smooth(nav_msgs::Path& path, double max_time);

private:
    bool smoothImpl(nav_msgs::Path& path, bool& reversing_segment);
    void updateApproximatePathOrientations(nav_msgs::Path& path, bool reversing_segment);
    std::vector<PathSegment> findDirectionalPathSegments(const nav_msgs::Path& path);

    ros::NodeHandle nh_;
    bool do_refinement_;
    int refinement_num_;
};


}  // namespace nav2_smoother

#endif  // SAVITZKY_GOLAY_SMOOTHER_H