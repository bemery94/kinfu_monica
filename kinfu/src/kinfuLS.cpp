/*
* Software License Agreement (BSD License)
*
* Point Cloud Library (PCL) - www.pointclouds.org
* Copyright (c) 2011, Willow Garage, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/
/*
* Based on the KinfuLS ROS wrapper by Michael Korn <michael.korn(at)uni-due.de>
* http://fsstud.is.uni-due.de/svn/ros/is/kinfu/
*/
/*
* Modified by Riccardo Monica 
*   RIMLab, Department of Information Engineering, University of Parma, Italy
*   http://www.rimlab.ce.unipr.it/
* 2013-2015
*/

// STL
#include <iostream>
#include <vector>
#include <list>
#include <deque>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// PCL
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

// ROS
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/fill_image.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
#include <ros/spinner.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Custom
#include "parameters.h"
#include "worlddownloadmanager.h"
#include "commandsubscriber.h"
#include "weightcubelistener.h"
#include "incompletepointslistener.h"
#include "ObstacleForceFeedback.h"

// ROS custom messages
#include <kinfu_msgs/KinfuTsdfRequest.h>

typedef pcl::ScopeTime ScopeTimeT;

using pcl::gpu::kinfuLS::KinfuTracker;
using pcl::gpu::PtrStepSz;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

typedef unsigned int uint;
typedef short unsigned int suint;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public pcl::StopWatch
{
    enum
    {
        EACH = 33
    };

    SampledScopeTime(int &time_ms) : time_ms_(time_ms)
    {
    }

    ~SampledScopeTime()
    {
        static int i_ = 0;
        time_ms_ += getTime();
        if (i_ % EACH == 0 && i_)
        {
            ROS_INFO("Avg frame time = %.2f ms (%.2f fps)", float(time_ms_) / EACH,
                     float(1000.f * EACH / time_ms_));
            time_ms_ = 0;
        }
        ++i_;
    }

private:
    int &time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PosePublisher
{
public:
    PosePublisher(ros::NodeHandle &nhandle)
    {
        m_reverse_initial_transformation = Eigen::Affine3f::Identity();
        nhandle.param<std::string>(PARAM_NAME_TF_REFERENCE_FRAME, m_first_frame_name,
                                   PARAM_DEFAULT_TF_REFERENCE_FRAME);
        nhandle.param<std::string>(PARAM_NAME_TF_CURRENT_FRAME, m_current_frame_name,
                                   PARAM_DEFAULT_TF_CURRENT_FRAME);
        ROS_INFO_STREAM("[PosePublisher] reference frame = " << m_first_frame_name);
        ROS_INFO_STREAM("[PosePublisher] current frame = " << m_current_frame_name);
    }

    void publishPose(KinfuTracker &kinfu)
    {
        ROS_DEBUG_STREAM("[PosePublisher] Inside publishPose");
        Eigen::Affine3f original_coords = m_reverse_initial_transformation * kinfu.getCameraPose();

        // after this, the z axis is the sensor axis and points forward
        // the x axis is horizontal (points right) and the y axis is vertical (points downward)
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats = original_coords.linear();
        Eigen::Vector3f teVecs = original_coords.translation();

        tf::Transform transform(
                tf::Matrix3x3(erreMats(0, 0), erreMats(0, 1), erreMats(0, 2),
                              erreMats(1, 0), erreMats(1, 1), erreMats(1, 2),
                              erreMats(2, 0), erreMats(2, 1), erreMats(2, 2)),
                tf::Vector3(teVecs[0], teVecs[1], teVecs[2])
        );

        m_transform = tf::StampedTransform(transform, ros::Time::now(), m_first_frame_name,
                                           m_current_frame_name);
        m_tf_broadcaster.sendTransform(m_transform);
    }

    void setReverseInitialTransformation(Eigen::Affine3f it)
    {
        m_reverse_initial_transformation = it;
    }

    std::string getFirstFrameName() const
    {
        return m_first_frame_name;
    }

    std::string getCurrentFrameName() const
    {
        return m_current_frame_name;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    // initial transformation applied, already inverted
    Eigen::Affine3f m_reverse_initial_transformation;

    // for TF frames
    std::string m_first_frame_name;
    std::string m_current_frame_name;

    tf::TransformBroadcaster m_tf_broadcaster;

    tf::StampedTransform m_transform;

    // forbid this constructor
    PosePublisher()
    {
    }
};

struct ImagePublisher
{
    ImagePublisher(ros::NodeHandle &nhandle)
    {
        image_transport::ImageTransport it(nhandle);
        nhandle.param<std::string>(PARAM_NAME_CURRENT_VIEW_TOPIC, m_current_view_topic,
                                   PARAM_DEFAULT_CURRENT_VIEW_TOPIC);
        m_view_publisher = it.advertise(m_current_view_topic, 10);
    }

    void
    publishScene(KinfuTracker &kinfu, const sensor_msgs::ImageConstPtr &depth)
    {
        kinfu.getImage(view_device_);

        int cols;
        view_device_.download(view_host_, cols);

        //convert image to sensor message
        m_msg = sensor_msgs::ImagePtr(new sensor_msgs::Image);
        sensor_msgs::fillImage((*m_msg), "rgb8", view_device_.rows(), view_device_.cols(),
                               view_device_.cols() * 3, &view_host_[0]);

        m_msg->header.frame_id = depth->header.frame_id;
        m_view_publisher.publish(m_msg);
    }

private:
    bool paint_image_;
    bool accumulate_views_;

    KinfuTracker::View view_device_;
    KinfuTracker::View colors_device_;
    std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;

    std::string m_current_view_topic;

    //KinfuTracker::DepthMap generated_depth_;
    image_transport::Publisher m_view_publisher;

    sensor_msgs::ImagePtr m_msg;

    // forbid this constructor
    ImagePublisher()
    {
    }
};

class ICPIsLostPublisher
{
public:
    ICPIsLostPublisher(ros::NodeHandle &nhandle)
    {
        nhandle.param<std::string>(PARAM_NAME_ICP_LOST_TOPIC, m_icp_lost_topic_name,
                                   PARAM_DEFAULT_ICP_LOST_TOPIC);
        m_icpIsLostPub = nhandle.advertise<std_msgs::Empty>(m_icp_lost_topic_name, 2);
    }

    void publishIcpIsLost()
    {
        m_icpIsLostPub.publish(std_msgs::Empty());
    }

private:
    ros::Publisher m_icpIsLostPub;
    std::string m_icp_lost_topic_name;
};

class ResetSubscriber
{
public:
    ResetSubscriber(ros::NodeHandle &nhandle, boost::mutex &shared_mutex,
                    boost::condition_variable &cond) :
            m_shared_mutex(shared_mutex), m_cond(cond)
    {
        nhandle.param<std::string>(PARAM_NAME_IN_RESET_TOPIC, m_req_topic_name,
                                   PARAM_DEFAULT_IN_RESET_TOPIC);
        m_subReq = nhandle.subscribe(m_req_topic_name, 1, &ResetSubscriber::resetCallback, this);

        m_reset_required = false;
    }

    void resetCallback(const std_msgs::Empty & /*msg*/)
    {
        boost::mutex::scoped_lock lock(m_shared_mutex);
        m_reset_required = true;
        m_cond.notify_one();
    }

    // WARNING: lock the shared mutex before calling this
    bool isResetRequired()
    {
        return m_reset_required;
    }

    // WARNING: lock the shared mutex before calling this
    void clearResetRequired()
    {
        m_reset_required = false;
    }

private:
    ros::Subscriber m_subReq;
    std::string m_req_topic_name;
    bool m_reset_required;

    boost::mutex &m_shared_mutex;
    boost::condition_variable &m_cond;
};

class ImageSubscriber
{
public:
    ImageSubscriber(ros::NodeHandle &nhandle, boost::mutex &shared_mutex,
                    boost::condition_variable &cond) :
            m_shared_mutex(shared_mutex), m_cond(cond), m_nh(nhandle)
    {
        std::string prefix_topic;
        m_nh.param<std::string>(PARAM_NAME_PREFIX_TOPIC, prefix_topic, PARAM_DEFAULT_PREFIX_TOPIC);

        std::string depth_image_topic;
        m_nh.param<std::string>(PARAM_NAME_DEPTH_IMAGE_TOPIC, depth_image_topic,
                                prefix_topic + PARAM_DEFAULT_DEPTH_IMAGE_TOPIC);

        std::string camera_info_topic;
        m_nh.param<std::string>(PARAM_NAME_CAMERA_INFO_TOPIC, camera_info_topic,
                                prefix_topic + PARAM_DEFAULT_CAMERA_INFO_TOPIC);

        std::string image_topic;
        m_nh.param<std::string>(PARAM_NAME_IMAGE_TOPIC, image_topic,
                                prefix_topic + PARAM_NAME_IMAGE_TOPIC);

        ROS_INFO_STREAM("depth image topic = " << depth_image_topic);
        ROS_INFO_STREAM("camera image topic = " << image_topic);
        ROS_INFO_STREAM("camera info topic = " << camera_info_topic);

        bool enable_texture_extraction = PARAM_DEFAULT_EXTRACT_TEXTURES;
        m_nh.getParam(PARAM_NAME_EXTRACT_TEXTURES, enable_texture_extraction);
        m_nh.getParam(PARAM_SNAME_EXTRACT_TEXTURES, enable_texture_extraction);

        if (enable_texture_extraction)
        {
            m_depth_sub.reset(new message_filters::Subscriber<Image>(m_nh, depth_image_topic, 2));
            m_info_sub.reset(
                    new message_filters::Subscriber<CameraInfo>(m_nh, camera_info_topic, 2));
            m_rgb_sub.reset(new message_filters::Subscriber<Image>(m_nh, image_topic, 2));

            //the depth and the rgb cameras are not hardware synchronized
            //hence the depth and rgb images normally do not have the EXACT timestamp
            //so use approximate time policy for synchronization
            m_texture_sync.reset(
                    new message_filters::Synchronizer<DRGBSync>(DRGBSync(500), *m_depth_sub,
                                                                *m_info_sub, *m_rgb_sub));
            m_texture_sync->registerCallback(
                    boost::bind(&ImageSubscriber::imageCallback, this, _1, _2, _3));
            ROS_INFO("Running KinFu with texture extraction");
        } else
        {
            m_depth_sub.reset(
                    new message_filters::Subscriber<sensor_msgs::Image>(m_nh, depth_image_topic,
                                                                        1));
            m_info_sub.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(m_nh,
                                                                                      camera_info_topic,
                                                                                      1));

            m_depth_only_sync.reset(
                    new message_filters::Synchronizer<DISync>(DISync(500), *m_depth_sub,
                                                              *m_info_sub));
            m_depth_only_sync->registerCallback(
                    boost::bind(&ImageSubscriber::imageCallback, this, _1, _2,
                                sensor_msgs::ImageConstPtr()));

            ROS_INFO("Running KinFu without texture extraction");
        }

        m_has_image = false;
    }

    sensor_msgs::CameraInfoConstPtr m_prev_info;

    ~ImageSubscriber()
    {

    }

    void imageCallback(const sensor_msgs::ImageConstPtr &depth,
                       const sensor_msgs::CameraInfoConstPtr &cameraInfo,
                       const sensor_msgs::ImageConstPtr &rgb = sensor_msgs::ImageConstPtr())
    {
        boost::mutex::scoped_lock lock(m_shared_mutex);
        m_depth = depth;
        m_cameraInfo = cameraInfo;
        m_rgb = rgb;

        m_has_image = true;
        m_cond.notify_all();
    }

    // WARNING: lock the shared mutex before calling this
    bool hasImage()
    {
        return m_has_image;
    }

    // WARNING: lock the shared mutex before calling this
    void getImage(sensor_msgs::ImageConstPtr &depth, sensor_msgs::CameraInfoConstPtr &cameraInfo,
                  sensor_msgs::ImageConstPtr &rgb)
    {
        if (!m_has_image)
            return;

        depth = m_depth;
        cameraInfo = m_cameraInfo;
        rgb = m_rgb;
    }

    // WARNING: lock the shared mutex before calling this
    void clearImage()
    {
        m_has_image = false;
    }

private:

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            sensor_msgs::CameraInfo, sensor_msgs::Image> DRGBSync;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> DISync;
    boost::shared_ptr<message_filters::Synchronizer<DRGBSync> > m_texture_sync;
    boost::shared_ptr<message_filters::Synchronizer<DISync> > m_depth_only_sync;

    boost::shared_ptr<message_filters::Subscriber<Image> > m_rgb_sub;
    boost::shared_ptr<message_filters::Subscriber<Image> > m_depth_sub;
    boost::shared_ptr<message_filters::Subscriber<CameraInfo> > m_info_sub;

    sensor_msgs::ImageConstPtr m_rgb;
    sensor_msgs::ImageConstPtr m_depth;
    sensor_msgs::CameraInfoConstPtr m_cameraInfo;

    boost::mutex &m_shared_mutex;
    boost::condition_variable &m_cond;

    ros::NodeHandle &m_nh;

    bool m_has_image;
};

struct KinFuLSApp
{
    enum
    {
        PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8
    };

    KinFuLSApp(float vsz, const int vr, float shiftDistance, ros::NodeHandle &nodeHandle,
               uint depth_height, uint depth_width) :
            scan_(false), scan_mesh_(false), scan_volume_(false), independent_camera_(false),
            registration_(false), integrate_colors_(false), pcd_source_(false), focal_length_(-1.f),
            m_reset_subscriber(nodeHandle, m_mutex, m_cond),
            m_image_subscriber(nodeHandle, m_mutex, m_cond),
            m_command_subscriber(nodeHandle, m_tf_listener, m_mutex, m_cond),
            m_image_publisher(nodeHandle), m_pose_publisher(nodeHandle),
            m_icp_is_lost_publisher(nodeHandle),
            m_world_download_manager(nodeHandle, m_mutex, m_cond),
            time_ms_(0), nh(nodeHandle)
    {
        //Init Kinfu Tracker
        Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(vsz/*meters*/);
        Eigen::Vector3i volume_resolution = Eigen::Vector3i::Constant(vr);

        ROS_INFO("--- CURRENT SETTINGS ---\n");
        ROS_INFO("Volume size is set to %.2f meters\n", vsz);
        ROS_INFO("Volume resolution is set to %d voxels\n", vr);
        ROS_INFO(
                "Volume will shift when the camera target point is farther than %.2f meters from the volume center\n",
                shiftDistance);
        ROS_INFO("The target point is located at [0, 0, %.2f] in camera coordinates\n", 0.6 * vsz);
        ROS_INFO("------------------------\n");

        // warning message if shifting distance is abnormally big compared to volume size
        if (shiftDistance > 2.5 * vsz)
            ROS_WARN(
                    "WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).\nYou can modify it using --shifting_distance.\n",
                    shiftDistance, vsz);

        kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, volume_resolution, shiftDistance,
                                                     depth_height, depth_width);
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
        Eigen::Vector3f t = volume_size * 0.5f - Eigen::Vector3f(0, 0, volume_size(2) / 2 * 1.2f);

        Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

        m_pose_publisher.setReverseInitialTransformation(pose.inverse());
        m_world_download_manager.setReverseInitialTransformation(pose.inverse());
        m_command_subscriber.setInitialTransformation(pose);
        m_world_download_manager.setReferenceFrameName(m_pose_publisher.getFirstFrameName());

        // Setup publisher for force feedback topic
        std::string force_feedback_topic;
        nodeHandle.param<std::string>(PARAM_NAME_FORCE_FEEDBACK_TOPIC, force_feedback_topic,
                                      PARAM_DEFAULT_FORCE_FEEDBACK_TOPIC);
        m_obstacle_force_publisher = nodeHandle.advertise<geometry_msgs::Pose>(force_feedback_topic,
                                                                               10);

        kinfu_->setInitialCameraPose(pose);
        kinfu_->volume().setTsdfTruncDist(0.030f/*meters*/);
        kinfu_->setIcpCorespFilteringParams(0.1f/*meters*/, sin(pcl::deg2rad(20.f)));
        //kinfu_->setDepthTruncationForICP(3.f/*meters*/);
        kinfu_->setCameraMovementThreshold(0.001f);

        m_obstacle_force_feedback = new ObstacleForceFeedback(kinfu_);

        //Init KinFuLSApp
        tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        frame_counter_ = 0;
        enable_texture_extraction_ = false;
        snapshot_rate_ = 45;

        m_depth_height = depth_height;
        m_depth_width = depth_width;

        m_request_termination = false;
    }

    ~KinFuLSApp()
    {
        delete m_obstacle_force_feedback;
    }

    // this contains the main cycle for the kinfu thread
    void run()
    {
        boost::mutex::scoped_lock main_lock(m_mutex);

        ROS_INFO_STREAM("Inside run\n\n");

        while (!m_request_termination)
        {
            bool reset;
            std::string reset_command_id;
            bool hasimage;
            bool hasrequests;
            bool isrunning;
            bool istriggered;

            KinfuTracker::THint pose_hint;
            CommandSubscriber::Sphere::Ptr clear_sphere;
            CommandSubscriber::BBox::Ptr clear_bbox;

            while (!m_request_termination &&
                   !(m_reset_subscriber.isResetRequired()) &&
                   !(m_image_subscriber.hasImage()) &&
                   !(m_world_download_manager.hasRequests()) &&
                   !(m_command_subscriber.hasClearSphere()) &&
                   !(m_command_subscriber.hasClearBBox()) &&
                   !(m_command_subscriber.isTriggered()))
                m_cond.wait(main_lock);

            hasimage = m_image_subscriber.hasImage();
            hasrequests = m_world_download_manager.hasRequests();

            std::string istriggered_command_id;
            if ((istriggered = m_command_subscriber.isTriggered()))
            {
                istriggered_command_id = m_command_subscriber.getIsTriggeredCommandId();
                m_command_subscriber.clearTriggered();
            }

            reset = false;
            if (m_reset_subscriber.isResetRequired())
            {
                reset = true;
                m_reset_subscriber.clearResetRequired();
            }

            isrunning = m_command_subscriber.isRunning();

            // command execution is allowed only when a new image is available
            if (hasimage)
            {
                if (m_command_subscriber.isResetRequired())
                {
                    reset = true;
                    reset_command_id = m_command_subscriber.getResetCommandId();
                    m_command_subscriber.clearResetRequired();
                }

                if (m_command_subscriber.hasHint())
                {
                    pose_hint.type = m_command_subscriber.hasForcedHint() ?
                                     KinfuTracker::THint::HINT_TYPE_FORCED
                                                                          : KinfuTracker::THint::HINT_TYPE_HINT;
                    pose_hint.transform = m_command_subscriber.getHintTransform();
                    m_command_subscriber.clearHint();
                }

                pose_hint.ignore_minimum_movement = !m_command_subscriber.isEnabledMinimumMovement();
            }

            sensor_msgs::ImageConstPtr depth;
            sensor_msgs::CameraInfoConstPtr cameraInfo;
            sensor_msgs::ImageConstPtr rgb;

            if (hasimage)
            {
                m_image_subscriber.getImage(depth, cameraInfo, rgb);
                m_image_subscriber.clearImage();
            }

            clear_sphere = m_command_subscriber.getClearSphere();
            m_command_subscriber.clearClearSphere();

            clear_bbox = m_command_subscriber.getClearBBox();
            m_command_subscriber.clearClearBBox();

            // The information from ROS was copied, this may run along the main thread.
            main_lock.unlock();

            if (reset)
            {
                kinfu_->reset();
                m_command_subscriber.ack(reset_command_id, true);
                ROS_INFO("KinFu was reset.");
            }

            ROS_DEBUG_STREAM("hasimage run" << hasimage);
            ROS_DEBUG_STREAM("isrunning run" << isrunning);
            ROS_DEBUG_STREAM("istriggered run" << istriggered);


            if (hasimage && (isrunning || istriggered))
            {
                ROS_DEBUG_STREAM("[run] Before execute()");
                execute(depth, cameraInfo, rgb, pose_hint);
                if (istriggered)
                    m_command_subscriber.ack(istriggered_command_id, true);
            }

            // Calculate the obstacle force at the end effector based off the proximity of obstacles.
            geometry_msgs::Pose obstacle_force = m_obstacle_force_feedback->calculateObstacleForce();
            m_obstacle_force_publisher.publish(obstacle_force);

            if ((clear_sphere || clear_bbox || hasrequests) && !kinfu_->isShiftComplete())
            {
                ROS_INFO("kinfu: shift incomplete but requests pending, waiting for it...");
                ros::Rate rate(10);
                while (!kinfu_->isShiftComplete())
                {
                    kinfu_->updateShift();
                    rate.sleep();
                }
                ROS_INFO("kinfu: shift is now complete, can continue.");
            }

            if (clear_sphere)
            {
                kinfu_->clearSphere(clear_sphere->center, clear_sphere->radius,
                                    clear_sphere->set_to_empty);
                ROS_INFO("KinFu cleared sphere.");
                m_command_subscriber.ack(clear_sphere->command_id, true);
            }

            if (clear_bbox)
            {
                kinfu_->clearBBox(clear_bbox->min, clear_bbox->max, clear_bbox->set_to_empty);
                ROS_INFO("KinFu cleared bbox.");
                m_command_subscriber.ack(clear_bbox->command_id, true);
            }

            if (hasrequests)
                m_world_download_manager.respond(kinfu_);

            // lock the mutex again, it will be unlocked by the condition variable at the beginning of the loop
            main_lock.lock();
        }
    }

    suint *convertToShortIntArray(const sensor_msgs::ImageConstPtr &depth, size_t &step,
                                  std::vector<suint> &data)
    {
        const size_t height = depth->height;
        const size_t width = depth->width;
        const std::string encoding = depth->encoding;
        const bool is_float32 = encoding == "32FC1";
        const bool is_uint16 = encoding == "16UC1";
        const bool is_bigendian = depth->is_bigendian;

        suint *data_ptr;

        if (!is_float32 && !is_uint16)
            ROS_ERROR("kinfu: depth encoding not supported: %s", encoding.c_str());

        if (is_uint16 && !is_bigendian)
        {
            data_ptr = (suint *) (depth->data.data());
            step = depth->step;
        } else
        {
            data.assign(height * width, 0);

            for (uint y = 0; y < height; y++)
                for (uint x = 0; x < width; x++)
                {
                    if (is_float32)
                    {
                        float f;
                        std::memcpy(&f, &(depth->data[depth->step * y + x * 4]), 4);
                        data[y * width + x] = suint(f * 1000.0 + 0.5);
                    }
                    if (is_uint16 && is_bigendian)
                    {
                        data[y * width + x] =
                                suint(depth->data[depth->step * y + x * 2]) * suint(256) +
                                suint(depth->data[depth->step * y + x * 2 + 1]);
                    }
                }
            step = width * 2;
            data_ptr = data.data();
        }

        return data_ptr;
    }

    //callback function, called with every new depth topic message
    void execute(const sensor_msgs::ImageConstPtr &depth,
                 const sensor_msgs::CameraInfoConstPtr &cameraInfo,
                 const sensor_msgs::ImageConstPtr &rgb, const KinfuTracker::THint &pose_hint)
    {
        ROS_DEBUG_STREAM("[execute] In execute()");

        frame_counter_++;

        if (kinfu_->icpIsLost())
        {
            m_icp_is_lost_publisher.publishIcpIsLost();
            if (pose_hint.type == KinfuTracker::THint::HINT_TYPE_NONE)
                return;
        }

        if (depth->height != m_depth_height || depth->width != m_depth_width)
        {
            ROS_ERROR(
                    "kinfu: received %dx%d depth image, but %dx%d is expected (check parameters %s and %s).",
                    int(depth->width), int(depth->height), int(m_depth_width), int(m_depth_height),
                    PARAM_NAME_DEPTH_WIDTH, PARAM_NAME_DEPTH_HEIGHT);
            return;
        }

        // convert depth to short uint little-endian format
        std::vector<suint> depth_storage;
        size_t depth_step;
        suint *depth_ptr = convertToShortIntArray(depth, depth_step, depth_storage);

        depth_device_.upload(depth_ptr, depth_step, depth->height, depth->width);
        // if (integrate_colors_)
        //    image_view_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);


        /*
         *      [fx  0 cx]
         * K = 	[ 0 fy cy]
         * 		[ 0  0  1]
         */
        (*kinfu_).setDepthIntrinsics(cameraInfo->K[0], cameraInfo->K[4],
                                     cameraInfo->K[2], cameraInfo->K[5]);


        float focal_length = (cameraInfo->K[0] + cameraInfo->K[4]) / 2;
        screenshot_manager_.setCameraIntrinsics(focal_length, cameraInfo->height,
                                                cameraInfo->width);


        SampledScopeTime fps(time_ms_);
        (*kinfu_)(depth_device_, pose_hint);
        if (kinfu_->isFinished())
            nh.shutdown();

        ROS_DEBUG_STREAM("[execute] Before publish");

        m_image_publisher.publishScene(*kinfu_, depth);
        m_pose_publisher.publishPose(*kinfu_);

        //image_view_.publishGeneratedDepth(*kinfu_);
/*
    //save snapshots
    if (enable_texture_extraction_)
    {
      if (frame_counter_ % snapshot_rate_ == 0)
      {
        //convert sensor_msgs::Image to pcl::gpu::PixelRGB
        unsigned pixelCount = rgb->height * rgb->width;
        pcl::gpu::kinfuLS::PixelRGB * pixelRgbs = new pcl::gpu::kinfuLS::PixelRGB[pixelCount];
        for (unsigned i = 0; i < pixelCount; i++)
        {
          //the encoding given in the image is "bgr8"
          pixelRgbs[i].b = rgb->data[i * 3];
          pixelRgbs[i].g = rgb->data[i * 3 + 1];
          pixelRgbs[i].r = rgb->data[i * 3 + 2];
        }
        pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24(rgb->height, rgb->width, pixelRgbs, rgb->step);
        screenshot_manager_.saveImage(kinfu_->getCameraPose(), rgb24);
        delete[] pixelRgbs;
      }
    }*/
    }

    void start()
    {
        m_thread = boost::thread(&KinFuLSApp::run, this);
    }

    void prepareTermination()
    {
        boost::mutex::scoped_lock lock(m_mutex);
        m_request_termination = true;
        m_cond.notify_one();
    }

    void join()
    {
        prepareTermination();
        m_thread.join();
    }

    void setEnableTextureExtraction(bool e)
    {
        enable_texture_extraction_ = e;
    }

    bool isEnableTextureExtraction()
    {
        return enable_texture_extraction_;
    }

    void setExtractKnownPoints(bool e)
    {
        kinfu_->setExtractKnownPoints(e);
        if (e)
            kinfu_->setWeightCubeListener(m_world_download_manager.getWeightCubeListener());
    }

    void setTSDFPadding(const int padding)
    {
        kinfu_->setTSDFPadding(padding);
    }

    void enableExtractBorderPoints()
    {
        TIncompletePointsListener::Ptr listener = m_world_download_manager.getIncompletePointsListener();
        listener->AddAcceptedType(TIncompletePointsListener::TYPE_BORDERS);
        kinfu_->setIncompletePointsListener(listener);
        ROS_INFO("kinfu: border points extraction enabled.");
    }

    void enableExtractFrontierPoints()
    {
        TIncompletePointsListener::Ptr listener = m_world_download_manager.getIncompletePointsListener();
        listener->AddAcceptedType(TIncompletePointsListener::TYPE_FRONTIERS);
        kinfu_->setIncompletePointsListener(listener);
        ROS_INFO("kinfu: frontier points extraction enabled.");
    }

    void enableExtractSurfacePoints()
    {
        TIncompletePointsListener::Ptr listener = m_world_download_manager.getIncompletePointsListener();
        listener->AddAcceptedType(TIncompletePointsListener::TYPE_SURFACES);
        kinfu_->setIncompletePointsListener(listener);
        ROS_INFO("kinfu: surface points extraction enabled.");
    }

    void setMarchingCubesVolumeSize(int size)
    {
        m_world_download_manager.setMarchingCubesVolumeSize(size);
    }

    void setSnapshotRate(int frame_count)
    {
        snapshot_rate_ = frame_count;
    }

    void SetDepthWidth(const uint w)
    {
        m_depth_width = w;
    }

    void SetDepthHeight(const uint h)
    {
        m_depth_height = h;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
    bool scan_;
    bool scan_mesh_;
    bool scan_volume_;

    bool independent_camera_;
    int frame_counter_;
    bool enable_texture_extraction_;

    bool registration_;
    bool integrate_colors_;
    bool pcd_source_;
    float focal_length_;

    KinfuTracker *kinfu_;

    ResetSubscriber m_reset_subscriber;
    ImageSubscriber m_image_subscriber;
    CommandSubscriber m_command_subscriber;

    ImagePublisher m_image_publisher;
    PosePublisher m_pose_publisher;
    ICPIsLostPublisher m_icp_is_lost_publisher;
    ros::Publisher m_obstacle_force_publisher;

    WorldDownloadManager m_world_download_manager;

    KinfuTracker::DepthMap depth_device_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

    boost::thread m_thread;
    bool m_request_termination;

    uint m_depth_width, m_depth_height;

    std::vector<pcl::gpu::kinfuLS::PixelRGB> source_image_data_;
    std::vector<unsigned short> source_depth_data_;
    PtrStepSz<const unsigned short> depth_;
    PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24_;

    int time_ms_;
    std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;
    KinfuTracker::DepthMap generated_depth_;

    pcl::kinfuLS::ScreenshotManager screenshot_manager_;
    int snapshot_rate_;

    tf::TransformListener m_tf_listener;

    //the ros node handle used to shut down the node and stop capturing
    ros::NodeHandle &nh;

    boost::mutex m_mutex;
    boost::condition_variable m_cond;

    // Set up obstacle force feedback object.
    ObstacleForceFeedback* m_obstacle_force_feedback;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int print_cli_help()
{
    cout << "\nKinFu parameters:" << endl;
    cout << "    --help, -h                        : print this message" << endl;
    cout << "\nkinfuLS node parameters:" << endl;
    cout << "    volume_size <in_meters>, vs       : define integration volume size" << endl;
    cout
            << "    shifting_distance <in_meters>, sd : define shifting threshold (distance target-point / cube center)"
            << endl;
    cout
            << "    snapshot_rate <X_frames>, sr      : Extract RGB textures every <X_frames>. Default: 45"
            << endl;
    cout
            << "    extract_textures, et              : extract RGB PNG images to KinFuSnapshots folder. Default: true"
            << endl;

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    // check arguments
    if (pcl::console::find_switch(argc, argv, "--help") ||
        pcl::console::find_switch(argc, argv, "-h"))
        return print_cli_help();

    ros::init(argc, argv, "kinfuLS");
    ros::NodeHandle nh("~");

    bool param_bool;

    // assign value from parameter server, with default.
    int device;
    nh.param<int>(PARAM_NAME_CUDA_DEVICE_ID, device, PARAM_DEFAULT_CUDA_DEVICE_ID);
    pcl::gpu::setDevice(device);
    pcl::gpu::printShortCudaDeviceInfo(device);

    double volume_size = PARAM_DEFAULT_VOLUME_SIZE; //pcl::device::VOLUME_SIZE
    nh.getParam(PARAM_NAME_VOLUME_SIZE, volume_size);
    nh.getParam(PARAM_SNAME_VOLUME_SIZE, volume_size);

    int volume_resolution = PARAM_DEFAULT_VOLUME_RESOLUTION;
    nh.getParam(PARAM_NAME_VOLUME_RESOLUTION, volume_resolution);

    double shift_distance = PARAM_DEFAULT_SHIFT_DISTANCE; //pcl::device::DISTANCE_THRESHOLD;
    nh.getParam(PARAM_NAME_SHIFT_DISTANCE, shift_distance);
    nh.getParam(PARAM_SNAME_SHIFT_DISTANCE, shift_distance);

    double depth_height = PARAM_DEFAULT_DEPTH_HEIGHT, depth_width = PARAM_DEFAULT_DEPTH_WIDTH;
    nh.getParam(PARAM_NAME_DEPTH_HEIGHT, depth_height);
    nh.getParam(PARAM_NAME_DEPTH_WIDTH, depth_width);

    KinFuLSApp app(volume_size, volume_resolution, shift_distance, nh, depth_height, depth_width);

    int snapshot_rate = PARAM_DEFAULT_SNAPSHOT_RATE;
    nh.getParam(PARAM_NAME_SNAPSHOT_RATE, snapshot_rate);
    nh.getParam(PARAM_SNAME_SNAPSHOT_RATE, snapshot_rate);
    app.setSnapshotRate(snapshot_rate);

    bool enable_texture_extraction = PARAM_DEFAULT_EXTRACT_TEXTURES;
    nh.getParam(PARAM_NAME_EXTRACT_TEXTURES, enable_texture_extraction);
    nh.getParam(PARAM_SNAME_EXTRACT_TEXTURES, enable_texture_extraction);
    app.setEnableTextureExtraction(enable_texture_extraction);

    bool extract_known_points = PARAM_DEFAULT_EXTRACT_KNOWN_POINTS;
    nh.getParam(PARAM_NAME_EXTRACT_KNOWN_POINTS, extract_known_points);
    app.setExtractKnownPoints(extract_known_points);

    bool extract_border_points = PARAM_DEFAULT_EXTRACT_BORDER_POINTS;
    nh.getParam(PARAM_NAME_EXTRACT_BORDER_POINTS, extract_border_points);
    if (extract_border_points)
        app.enableExtractBorderPoints();

    bool extract_frontier_points = PARAM_DEFAULT_EXTRACT_FRONTIER_POINTS;
    nh.getParam(PARAM_NAME_EXTRACT_FRONTIER_POINTS, extract_frontier_points);
    if (extract_frontier_points)
    {
        app.enableExtractFrontierPoints();
        app.setTSDFPadding(1);
    }

    nh.param<bool>(PARAM_NAME_EXTRACT_SURFACE_POINTS, param_bool,
                   PARAM_DEFAULT_EXTRACT_SURFACE_POINTS);
    if (param_bool)
    {
        app.enableExtractSurfacePoints();
        app.setTSDFPadding(1);
    }

    int marching_cubes_volume_size = PARAM_DEFAULT_MARCHING_CUBE_SIZE;
    nh.getParam(PARAM_NAME_MARCHING_CUBE_SIZE, marching_cubes_volume_size);
    if (marching_cubes_volume_size > 0)
        app.setMarchingCubesVolumeSize(marching_cubes_volume_size);

    // start app main thread
    app.start();

    ros::spin();

    app.prepareTermination();
    app.join();

    return 0;
}
