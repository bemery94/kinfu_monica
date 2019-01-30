#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

/*
 * This node publishes a new branch in the TF tree to allow using the UR5 pose
 * of the end effector for kinfu.
 * 
 * It publishes a fixed "kinfu_reference" frame which is the initial pose of the 
 * camera wrt. the world. It then publishes the updated camera pose with respect
 * to this frame as the UR5 moves.
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher_node");

    ros::NodeHandle node;

    tf::TransformListener listener;
    tf::TransformBroadcaster transform_br;

    bool publish_camera_pose;
    node.getParam("/tf_publisher_node/publish_camera_pose", publish_camera_pose);

    ros::Rate rate(50.0);
    bool tf_received = false;
    
    tf::Transform T_camera_to_ee;
    tf::Vector3 t_camera_to_ee(0.0, 0.0, 0.05);

    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    tf::Matrix3x3 R_camera_to_ee;
    R_camera_to_ee.setRPY(roll, pitch, yaw);

    T_camera_to_ee.setOrigin(t_camera_to_ee);
    T_camera_to_ee.setBasis(R_camera_to_ee);
    
    tf::StampedTransform T_ee_to_world_initial;
    tf::Transform T_kinfu_ref_to_world;
    while (!tf_received && ros::ok())
    {
        try
        {
            listener.lookupTransform("/world", "/tool0",
                                     ros::Time(0), T_ee_to_world_initial);

            T_kinfu_ref_to_world = T_ee_to_world_initial * T_camera_to_ee;
            tf_received = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        rate.sleep();
    }

    tf::StampedTransform T_ee_to_world;
    tf::StampedTransform T_cam_optical_to_cam;
    tf::Transform T_cam_to_world;
    tf::Transform T_cam_to_kinfu_ref;

    while (ros::ok())
    {
        tf::StampedTransform current_transform;
        if (publish_camera_pose)
        {
            try
            {
                listener.lookupTransform("/world", "/tool0",
                                        ros::Time(0), T_ee_to_world);
                listener.lookupTransform("/camera_link", "/camera_depth_optical_frame",
                                        ros::Time(0), T_cam_optical_to_cam);
                
                T_cam_to_world = T_ee_to_world * T_camera_to_ee;
                T_cam_to_kinfu_ref = T_kinfu_ref_to_world.inverse() * T_cam_to_world * T_cam_optical_to_cam.inverse();
                
                transform_br.sendTransform(tf::StampedTransform(T_cam_to_kinfu_ref, ros::Time::now(), 
                                                                "kinfu_reference", "/camera_link"));
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }
        transform_br.sendTransform(tf::StampedTransform(T_kinfu_ref_to_world, ros::Time::now(), 
                                                        "world", "kinfu_reference"));
        rate.sleep();
    }
    return 0;
}
