#include "ObstacleForceFeedback.h"


using pcl::gpu::kinfuLS::KinfuTracker;


ObstacleForceFeedback::ObstacleForceFeedback(KinfuTracker* kinfu_tracker) :
        kinfu_tracker_(kinfu_tracker)
{}


ObstacleForceFeedback::~ObstacleForceFeedback()
{
    delete kinfu_tracker_;
}


geometry_msgs::Pose ObstacleForceFeedback::calculateObstacleForce()
{
    //









    geometry_msgs::Pose pose_out;
    pose_out.position.x = 1;
    pose_out.position.y = 2;
    pose_out.position.z = 3;

    pose_out.orientation.w = 1;
    pose_out.orientation.x = 0;
    pose_out.orientation.y = 0;
    pose_out.orientation.z = 0;

    return pose_out;
}