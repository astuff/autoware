/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, All Rights Reserved.
*
* This file is part of the gpsins_localizer which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef GPSINS_LOCALIZER_GPSINSLOCALIZERNL_H
#define GPSINS_LOCALIZER_GPSINSLOCALIZERNL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <novatel_gps_msgs/Inspva.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

typedef message_filters::sync_policies::ApproximateTime<novatel_gps_msgs::Inspva, sensor_msgs::Imu> MySyncPolicy;

namespace gpsins_localizer {

class GpsInsLocalizerNl : public nodelet::Nodelet {
 public:
    GpsInsLocalizerNl();

 private:
    // Init
    virtual void onInit();
    void loadParams();

    // Subscriber callbacks
    void insDataCb(const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg,
        const sensor_msgs::Imu::ConstPtr& imu_msg);

    // Util functions
    void broadcastTf(tf2::Transform transform, ros::Time stamp);
    void publishPose(tf2::Transform pose, ros::Time stamp);
    void pubishVelocity(const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg,
        const sensor_msgs::Imu::ConstPtr& imu_msg);
    void createMapFrame(const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg);
    tf2::Transform calculateBaselinkPose(const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg);
    void checkInitialize(std::string ins_status);
    tf2::Transform convertLLHtoECEF(double latitude, double longitude, double height);
    tf2::Quaternion convertAzimuthToENU(double roll, double pitch, double yaw);
    tf2::Transform convertECEFtoMGRS(tf2::Transform pose, double roll, double pitch, double yaw);

    // Nodehandles, both public and private
    ros::NodeHandle nh, pnh;

    // Publishers
    ros::Publisher pose_pub;
    ros::Publisher velocity_pub;
    tf2_ros::TransformBroadcaster tf_bc;
    tf2_ros::StaticTransformBroadcaster stf_bc;

    // Subscribers
    message_filters::Subscriber<novatel_gps_msgs::Inspva> inspva_sub;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    message_filters::Synchronizer<MySyncPolicy>* sync;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Static transforms
    tf2::Transform earth_map_tf;
    tf2::Transform base_link_gps_tf;

    // Internal state
    bool initialized = false;
    bool map_frame_established = false;
    bool gps_frame_established = false;
    std::string mgrs_zone = "";
    tf2::Transform prev_mgrs_pose;
    bool mgrs_pose_frozen = false;

    // Parameters
    std::string imu_data_topic_name = "gps/imu";
    std::string ins_data_topic_name = "gps/inspva";
    bool create_map_frame = false;
    bool publish_earth_gpsm_tf = false;
    std::string measured_gps_frame = "gps_measured";
    std::string static_gps_frame = "gps";
    bool no_solution_init = false;
    bool mgrs_mode = false;
};

}  // namespace gpsins_localizer
#endif  // GPSINS_LOCALIZER_GPSINSLOCALIZERNL_H
