#ifndef KINFU_OBSTACLEFORCEFEEDBACK_H
#define KINFU_OBSTACLEFORCEFEEDBACK_H


#include <geometry_msgs/Pose.h>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>

class ObstacleForceFeedback
{
public:
    ObstacleForceFeedback(pcl::gpu::kinfuLS::KinfuTracker* kinfu_tracker);
    ~ObstacleForceFeedback();

    geometry_msgs::Pose calculateObstacleForce();

private:
    pcl::gpu::kinfuLS::KinfuTracker* kinfu_tracker_;
};


#endif //KINFU_OBSTACLEFORCEFEEDBACK_H
