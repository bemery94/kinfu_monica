#ifndef KINFU_OBSTACLEFORCEFEEDBACK_H
#define KINFU_OBSTACLEFORCEFEEDBACK_H

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>


class ObstacleForceFeedback
{
public:
    ObstacleForceFeedback(pcl::gpu::kinfuLS::KinfuTracker* kinfu_tracker,
                          ros::NodeHandle n);
    ~ObstacleForceFeedback();

    geometry_msgs::Pose calculateObstacleForce();

private:
    Eigen::Vector3i convWorldCoordToVoxelInd(float x_world, float y_world, float z_world);
    int convVoxelIndToArrayInd(Eigen::Vector3i voxel_indices);

    pcl::gpu::kinfuLS::KinfuTracker* kinfu_tracker_;

    // Size of each voxel in metres
    float voxel_size_metres_;

    // Length/width/height of entire volume in metres and voxels
    Eigen::Vector3f volume_size_metres_;
    Eigen::Vector3i volume_size_voxels_;

    ros::NodeHandle n_;
    ros::Publisher cloud_pub_;

};


#endif //KINFU_OBSTACLEFORCEFEEDBACK_H
