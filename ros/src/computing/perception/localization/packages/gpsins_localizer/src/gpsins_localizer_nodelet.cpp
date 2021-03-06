/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, All Rights Reserved.
*
* This file is part of the gpsins_localizer which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "gpsins_localizer/gpsins_localizer_nodelet.hpp"

#include <string>
#include <vector>
#include <math.h>
#include <GeographicLib/Geocentric.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gpsins_localizer
{

GpsInsLocalizerNl::GpsInsLocalizerNl() :
    tf_listener(this->tf_buffer)
{}

void GpsInsLocalizerNl::onInit()
{
    this->nh = getNodeHandle();
    this->pnh = getPrivateNodeHandle();
    this->loadParams();

    // Publishers
    this->pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 10);
    this->velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("current_velocity", 10);

    // Subscribers
    this->inspva_sub.subscribe(this->nh, this->ins_data_topic_name, 10);
    this->imu_sub.subscribe(this->nh, this->imu_data_topic_name, 10);

    this->sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), this->inspva_sub, this->imu_sub);
    this->sync->registerCallback(boost::bind(&GpsInsLocalizerNl::insDataCb, this, _1, _2));
}

void GpsInsLocalizerNl::loadParams()
{
    this->pnh.param<std::string>("imu_data_topic_name", this->imu_data_topic_name, "gps/imu");
    this->pnh.param<std::string>("ins_data_topic_name", this->ins_data_topic_name, "gps/inspva");
    this->pnh.param("create_map_frame", this->create_map_frame, false);
    this->pnh.param("publish_earth_gpsm_tf", this->publish_earth_gpsm_tf, false);
    this->pnh.param<std::string>("measured_gps_frame", this->measured_gps_frame, "gps_measured");
    this->pnh.param<std::string>("static_gps_frame", this->static_gps_frame, "gps");
    this->pnh.param("no_solution_init", this->no_solution_init, false);
    ROS_INFO("Parameters Loaded");
}

void GpsInsLocalizerNl::insDataCb(
    const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg,
    const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // We don't need any static TFs for these 2 functions, so no need to wait
    // for init
    if (this->create_map_frame)
    {
        createMapFrame(inspva_msg);
    }
    if (this->publish_earth_gpsm_tf)
    {
        bcMeasuredGpsFrame(inspva_msg);
    }

    // Don't continue if uninitialized
    checkInitialize(inspva_msg->status);
    if (!this->initialized)
    {
        return;
    }

    // Get position of measured GPS coordinates in earth frame
    tf2::Transform gps_point_earth = convertLLHtoECEF(
        inspva_msg->latitude, inspva_msg->longitude, inspva_msg->height);

    // Get position in map frame
    tf2::Transform gps_pose_map = earth_map_tf * gps_point_earth;

    // Orientation of the gps in the ENU(map) frame
    tf2::Quaternion orientation_gps_map = convertAzimuthToENU(
        inspva_msg->roll * M_PI / 180,
        inspva_msg->pitch * M_PI / 180,
        inspva_msg->azimuth * M_PI / 180);

    // Completed Pose of the gps in the map frame
    gps_pose_map.setRotation(orientation_gps_map);

    // Pose of base_link in the map frame
    tf2::Transform base_link_map = gps_pose_map * base_link_gps_tf;

    // Broadcast the map -> base_link transform
    geometry_msgs::TransformStamped map_baselink_tf;
    map_baselink_tf.header.frame_id = "map";
    map_baselink_tf.child_frame_id = "base_link";
    map_baselink_tf.header.stamp = inspva_msg->header.stamp;
    tf2::convert(base_link_map, map_baselink_tf.transform);
    this->tf_bc.sendTransform(map_baselink_tf);

    // Publish base_link pose in the map frame
    geometry_msgs::PoseStamped base_link_map_stamped;
    base_link_map_stamped.header.stamp = inspva_msg->header.stamp;
    base_link_map_stamped.header.frame_id = "map";
    tf2::toMsg(base_link_map, base_link_map_stamped.pose);
    this->pose_pub.publish(base_link_map_stamped);

    pubishVelocity(inspva_msg, imu_msg);
    return;
}

void GpsInsLocalizerNl::pubishVelocity(const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg,
    const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // GPS velocity
    double n_vel = inspva_msg->north_velocity;
    double e_vel = inspva_msg->east_velocity;
    double gps_velocity = sqrt(n_vel * n_vel + e_vel * e_vel);

    // Publish Twist in the base_link frame
    geometry_msgs::TwistStamped twist_bl;
    twist_bl.header.stamp = inspva_msg->header.stamp;
    twist_bl.header.frame_id = "base_link";
    twist_bl.twist.linear.x = gps_velocity;
    twist_bl.twist.linear.y = 0.0;
    twist_bl.twist.linear.z = 0.0;
    twist_bl.twist.angular.x = imu_msg->angular_velocity.x;
    twist_bl.twist.angular.y = imu_msg->angular_velocity.y;
    twist_bl.twist.angular.z = imu_msg->angular_velocity.z;
    this->velocity_pub.publish(twist_bl);
}

void GpsInsLocalizerNl::createMapFrame(const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg)
{
    tf2::Transform new_earth_map_tf = convertLLHtoECEF(
        inspva_msg->latitude, inspva_msg->longitude, inspva_msg->height);

    geometry_msgs::TransformStamped earth_map_tfs_msg;
    earth_map_tfs_msg.header.stamp = inspva_msg->header.stamp;
    earth_map_tfs_msg.header.frame_id = "earth";
    earth_map_tfs_msg.child_frame_id = "map";
    tf2::convert(new_earth_map_tf, earth_map_tfs_msg.transform);
    this->stf_bc.sendTransform(earth_map_tfs_msg);
    this->create_map_frame = false;

    // Also save internally, no need to wait for tf listener
    this->earth_map_tf = new_earth_map_tf.inverse();
    this->map_frame_established = true;
}

void GpsInsLocalizerNl::bcMeasuredGpsFrame(const novatel_gps_msgs::Inspva::ConstPtr& inspva_msg)
{
    // Get ENU TF of measured GPS coordinates
    tf2::Transform earth_gps_enu_tf = convertLLHtoECEF(
        inspva_msg->latitude, inspva_msg->longitude, inspva_msg->height);

    // Orientation of the gps in the ENU frame
    tf2::Quaternion orientation_gpsm = convertAzimuthToENU(
        inspva_msg->roll * M_PI / 180,
        inspva_msg->pitch * M_PI / 180,
        inspva_msg->azimuth * M_PI / 180);

    // Pose of the gps in the temporary measured gps ENU frame
    tf2::Transform tfpose_gpsm(orientation_gpsm);

    // Pose of gps in earth frame, with proper orientation
    tf2::Transform tfpose_earth = earth_gps_enu_tf * tfpose_gpsm;

    // Convert and broadcast
    geometry_msgs::TransformStamped output_tf2;
    output_tf2.header.stamp = inspva_msg->header.stamp;
    output_tf2.header.frame_id = "earth";
    output_tf2.child_frame_id = this->measured_gps_frame;
    tf2::convert(tfpose_earth, output_tf2.transform);

    this->tf_bc.sendTransform(output_tf2);
}

void GpsInsLocalizerNl::checkInitialize(std::string ins_status)
{
    if (this->initialized)
    {
        return;
    }

    // First check for required transforms
    // Check for earth -> map transform (Where is the map located in the world?)
    if (!this->map_frame_established)
    {
        try
        {
            geometry_msgs::TransformStamped tf_msg =
                this->tf_buffer.lookupTransform("map", "earth", ros::Time(0));
            tf2::convert(tf_msg.transform, this->earth_map_tf);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2, "%s", ex.what());
            ROS_WARN_THROTTLE(2, "Waiting for earth -> map transform");
            return;
        }
        this->map_frame_established = true;
    }

    // Check for base_link -> gps transform (Where is the gps sensor mounted on
    // the vehicle?)
    if (!this->gps_frame_established)
    {
        try
        {
            geometry_msgs::TransformStamped tf_msg =
                this->tf_buffer.lookupTransform(this->static_gps_frame, "base_link", ros::Time(0));
            tf2::convert(tf_msg.transform, this->base_link_gps_tf);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2, "%s", ex.what());
            ROS_WARN_THROTTLE(2, "Waiting for base_link -> %s transform", this->static_gps_frame.c_str());
            return;
        }
        this->gps_frame_established = true;
    }

    // Then check if we can initialize
    if (this->map_frame_established && this->gps_frame_established)
    {
        bool ins_alignment_complete = false;
        bool ins_solution_good = false;
        if (ins_status == "INS_ALIGNMENT_COMPLETE")
        {
            ins_alignment_complete = true;
        }
        if (ins_status == "INS_SOLUTION_GOOD")
        {
            ins_alignment_complete = true;
            ins_solution_good = true;
        }
        if (this->no_solution_init)
        {
            ins_solution_good = true;
        }

        if (ins_alignment_complete && ins_solution_good)
        {
            this->initialized = true;
            ROS_INFO("Localizer initialized");
        }
        else if (!ins_solution_good)
        {
            ROS_WARN_THROTTLE(2, "Waiting for INS_SOLUTION_GOOD status");
        }
        else
        {
            ROS_WARN_THROTTLE(2, "Waiting for INS_ALIGNMENT_COMPLETE status");
        }
    }

    if (!this->initialized)
    {
        ROS_WARN_THROTTLE(2, "Data received, but not ready to initialize");
    }
}

tf2::Transform GpsInsLocalizerNl::convertLLHtoECEF(double latitude, double longitude, double height)
{
    // This function is inspired by:
    // https://github.com/wavelab/libwave/tree/master/wave_geography

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    std::vector<double> rotation_vec(9, 0.0);
    double x, y, z;
    earth.Forward(latitude, longitude, height, x, y, z, rotation_vec);

    tf2::Vector3 origin(x, y, z);
    tf2::Matrix3x3 rotation;

    rotation.setValue(
        rotation_vec[0],
        rotation_vec[1],
        rotation_vec[2],
        rotation_vec[3],
        rotation_vec[4],
        rotation_vec[5],
        rotation_vec[6],
        rotation_vec[7],
        rotation_vec[8]);

    tf2::Transform ecef_enu_tf(rotation, origin);
    return ecef_enu_tf;
}

tf2::Quaternion GpsInsLocalizerNl::convertAzimuthToENU(double roll, double pitch, double yaw)
{
    // Convert from Azimuth (CW from North) to ENU (CCW from East)
    yaw = -yaw + M_PI / 2;

    // Clamp within 0 to 2 pi
    if (yaw > 2 * M_PI)
    {
        yaw = yaw - 2 * M_PI;
    }
    else if (yaw < 0)
    {
        yaw = yaw + 2 * M_PI;
    }

    // Novatel GPS uses different vehicle body frame (y forward, x right, z up)
    pitch = -pitch;

    // Broadcast map -> gps_measured tf
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);

    return orientation;
}

}  // namespace gpsins_localizer

PLUGINLIB_DECLARE_CLASS(gpsins_localizer,
                        GpsInsLocalizerNl,
                        gpsins_localizer::GpsInsLocalizerNl,
                        nodelet::Nodelet);
