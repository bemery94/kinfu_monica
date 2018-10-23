#include "ObstacleForceFeedback.h"


using pcl::gpu::kinfuLS::KinfuTracker;
using pcl::gpu::kinfuLS::TsdfVolume;


ObstacleForceFeedback::ObstacleForceFeedback(KinfuTracker* kinfu_tracker, ros::NodeHandle n) :
        kinfu_tracker_(kinfu_tracker), n_(n)
{
    voxel_size_metres_ = kinfu_tracker_->getVoxelSize();
    volume_size_metres_ = kinfu_tracker->volume().getSize();
    volume_size_voxels_ = kinfu_tracker->volume().getResolution();

    cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/test_cloud", 10);
}


ObstacleForceFeedback::~ObstacleForceFeedback()
{
    delete kinfu_tracker_;
}


Eigen::Vector3i ObstacleForceFeedback::convWorldCoordToVoxelInd(float x_world, float y_world,
                                                                float z_world)
{
    Eigen::Vector3i voxel_indices;

    // The world distance is measured from the centre of the robot while the robot is in the middle
    // of the voxel grid.
    x_world += volume_size_metres_[0] / 2;
    y_world += volume_size_metres_[1] / 2;
    z_world += volume_size_metres_[2] / 2;

    // If the world value in metres is on the edge of the volume, we need to subtract 1 from the
    // calculated index so that it isn't outside the range of the volume. E.g. x_world == 3m, then
    // the voxel index will be 3 / 0.5 = 6 whereas the index should be 5.
    voxel_indices[0] = x_world == volume_size_metres_[0] ?
                       (x_world / voxel_size_metres_) - 1 : x_world / voxel_size_metres_;

    voxel_indices[1] = y_world == volume_size_metres_[1] ?
                       (y_world / voxel_size_metres_) - 1 : y_world / voxel_size_metres_;

    voxel_indices[2] = z_world == volume_size_metres_[2] ?
                       (z_world / voxel_size_metres_) - 1 : z_world / voxel_size_metres_;

    return voxel_indices;
}


int ObstacleForceFeedback::convVoxelIndToArrayInd(Eigen::Vector3i voxel_indices)
{
    return volume_size_voxels_[0] * volume_size_voxels_[1] * voxel_indices[2]
         + volume_size_voxels_[0] * voxel_indices[1]
         + voxel_indices[0];
}



geometry_msgs::Pose ObstacleForceFeedback::calculateObstacleForce()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr test = kinfu_tracker_->extractWorld();

//    for(pcl::PointCloud<pcl::PointXYZI>::iterator it = test->points.begin(); it != test->points.end(); it++)
//    {
//        std::cout << it->x << ", " << it->y << ", " << it->z << std::endl;
//        std::cout << "intensity = " << it->intensity << std::endl;
//
//        std::cout << std::endl;
//    }


    test->header.frame_id = "world";
    cloud_pub_.publish(*test);

    ros::Rate r(1);
    r.sleep();
//    pcl::toRosMsg(ros_cloud, *test);


//    if (kinfu_tracker_->hasShifted())
//    {
//        ROS_ERROR_STREAM("Kinfu has shifted. Returned volume will be offset incorrectly!");
//    }
//    else
//    {
//        const TsdfVolume tsdf_volume = kinfu_tracker_->volume();
////    TsdfVolume::Ptr tsdf_volume = kinfu_tracker_->tsdf_volume_;
//
//        std::vector<float> tsdf_buffer;
//        tsdf_volume.downloadTsdf(tsdf_buffer);
//
//        ROS_INFO_STREAM("\n\nArray size = " << tsdf_buffer.size());
////    ROS_INFO_STREAM("TSDF info: " << tsdf_volume);
//
//        std::cout << "TSDF resolution: " << tsdf_volume.getResolution() << std::endl;
//        std::cout << "TSDF voxel size: " << tsdf_volume.getVoxelSize() << std::endl;
//        std::cout << "TSDF volume size: " << tsdf_volume.getSize() << std::endl;
//        std::cout << "TSDF SHIFTED: " << kinfu_tracker_->hasShifted() << std::endl;
//
//        std::vector<float> x_vec;
//        std::vector<float> y_vec;
//        std::vector<float> z_vec;
//
//        x_vec.push_back(1.5);
//        x_vec.push_back(-1.5);
//        x_vec.push_back(0);
//        x_vec.push_back(0);
//        x_vec.push_back(0);
//        x_vec.push_back(0);
//        x_vec.push_back(0.75);
//        x_vec.push_back(-0.75);
//
//        y_vec.push_back(0);
//        y_vec.push_back(0);
//        y_vec.push_back(1.5);
//        y_vec.push_back(-1.5);
//        y_vec.push_back(-0.75);
//        y_vec.push_back(+0.75);
//        y_vec.push_back(0);
//        y_vec.push_back(0);
//
//        z_vec.push_back(-1.5);
//        z_vec.push_back(-1.5);
//        z_vec.push_back(-1.5);
//        z_vec.push_back(-1.5);
//        z_vec.push_back(-1.5);
//        z_vec.push_back(-1.5);
//        z_vec.push_back(-1.5);
//        z_vec.push_back(-1.5);
//
//
//        for (int i=0; i<x_vec.size(); ++i)
//        {
//            float x = x_vec[i];
//            float y = y_vec[i];
//            float z = z_vec[i];
//
//            Eigen::Vector3i voxel_indices = convWorldCoordToVoxelInd(x,y,z);
//
//            int index = convVoxelIndToArrayInd(voxel_indices);
//
//            std::cout << "index = " << index << std::endl;
//
//            float test_point = tsdf_buffer[index];
//
//            std::cout << "test_point(m): x = " << x << ",y = " << y << ", z = " << z << " : " << test_point << std::endl;
//            std::cout << "test_point(vox): x = " << voxel_indices[0] << ",y = " << voxel_indices[1] << ", z = " << voxel_indices[2] << " : " << test_point << std::endl;
//            std::cout << std::endl;
//        }
//
//        for (int i=0; i<volume_size_voxels_[2]; ++i)
//        {
//            for (int j=0; j<volume_size_voxels_[1]; ++j)
//            {
//                for (int k=0; k<volume_size_voxels_[0]; ++k)
//                {
//                    Eigen::Vector3i ind;
//                    ind[0] = k;
//                    ind[1] = j;
//                    ind[2] = i;
//
//                    int ind_array = convVoxelIndToArrayInd(ind);
//
////                    if (tsdf_buffer[i] != 0)
////                    {
//                        std::cout << "x = " << k << std::endl;
//                        std::cout << "y = " << j << std::endl;
//                        std::cout << "z = " << i << std::endl;
//                        std::cout << "tsdf = " << tsdf_buffer[ind_array] << std::endl;
////                    }
//                }
//            }
//        }
//
//    }

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