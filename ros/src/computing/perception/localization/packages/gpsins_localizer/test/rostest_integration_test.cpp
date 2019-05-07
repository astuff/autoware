/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, All Rights Reserved.
*
* This file is part of the gpsins_localizer which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <novatel_gps_msgs/Inspva.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class NodeTest : public ::testing::Test
{
protected:
    NodeTest() : tf_listener(this->tf_buffer) {}

    ros::NodeHandle nh;

    // Publishers
    ros::Publisher inspva_pub =
      nh.advertise<novatel_gps_msgs::Inspva>("/gps/inspva", 1, true);
    ros::Publisher imu_pub =
      nh.advertise<sensor_msgs::Imu>("/gps/imu", 1, true);

    // Listen to TFs to ensure they are correct
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // To be used for cartesian coordinates in meters, not LLH comparison
    double error_bound = 0.001;

    void SetUp() override
    {
        ros::spinOnce();
    }

    void sendInspvaMsg(double lat, double lon, double hgt, double roll, double pitch, double azimuth)
    {
        novatel_gps_msgs::Inspva output_msg;
        output_msg.header.frame_id = "imu";
        output_msg.header.stamp = ros::Time(0.1);
        output_msg.status = "INS_SOLUTION_GOOD";
        output_msg.latitude = lat;
        output_msg.longitude = lon;
        output_msg.height = hgt;
        output_msg.roll = roll;
        output_msg.pitch = pitch;
        output_msg.azimuth = azimuth;

        sendInspvaMsg(output_msg);
    }

    void sendInspvaMsg(novatel_gps_msgs::Inspva output_msg)
    {
        this->inspva_pub.publish(output_msg);
        ros::spinOnce();
    }

    void sendImuMsg(double w_x, double w_y, double w_z)
    {
        sensor_msgs::Imu output_msg;
        output_msg.header.frame_id = "imu";
        output_msg.header.stamp = ros::Time(0.1);
        output_msg.angular_velocity.x = w_x;
        output_msg.angular_velocity.y = w_y;
        output_msg.angular_velocity.z = w_z;

        this->imu_pub.publish(output_msg);
        ros::spinOnce();
    }

    void compareTF(geometry_msgs::Transform actual_tf,
        double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, double r_w)
    {
        EXPECT_NEAR(actual_tf.translation.x, t_x, this->error_bound);
        EXPECT_NEAR(actual_tf.translation.y, t_y, this->error_bound);
        EXPECT_NEAR(actual_tf.translation.z, t_z, this->error_bound);
        EXPECT_NEAR(actual_tf.rotation.x, r_x, this->error_bound);
        EXPECT_NEAR(actual_tf.rotation.y, r_y, this->error_bound);
        EXPECT_NEAR(actual_tf.rotation.z, r_z, this->error_bound);
        EXPECT_NEAR(actual_tf.rotation.w, r_w, this->error_bound);
    }
};

TEST_F(NodeTest, mapFrameTest)
{
    // Test to ensure the map frame is created properly
    this->sendInspvaMsg(40.6117970767, -89.4833445072, 186.867369233, 0, 0, 0);
    this->sendImuMsg(0, 0, 0);
    ros::Duration(1.0).sleep();

    auto earth_map_tf = this->tf_buffer.lookupTransform("earth", "map", ros::Time(0));

    compareTF(earth_map_tf.transform, 43723.6310494, -4848708.00343, 4129913.74802,
    0.417769295391, 0.0018835965371, 0.00409634337214, 0.908541957129);
}

TEST_F(NodeTest, tfTreeTest)
{
    // This test is to ensure that the base_link -> map tf was updated properly

    // Push through two msgs to make sure it is initialized
    this->sendInspvaMsg(40.6117970767, -89.4833445072, 186.867369233, 0.1, 0.86, 2.5);
    this->sendImuMsg(0, 0, 0);
    ros::Duration(1.0).sleep();
    this->sendInspvaMsg(40.6117970767, -89.4833445072, 186.867369233, 0.1, 0.86, 2.5);
    this->sendImuMsg(0, 0, 0);

    auto earth_gps_tf = this->tf_buffer.lookupTransform("earth", "imu", ros::Time(0), ros::Duration(3.0));
    auto earth_gpsm_tf = this->tf_buffer.lookupTransform("earth", "gps_measured", ros::Time(0), ros::Duration(3.0));

    compareTF(earth_gpsm_tf.transform,
        earth_gps_tf.transform.translation.x,
        earth_gps_tf.transform.translation.y,
        earth_gps_tf.transform.translation.z,
        earth_gps_tf.transform.rotation.x,
        earth_gps_tf.transform.rotation.y,
        earth_gps_tf.transform.rotation.z,
        earth_gps_tf.transform.rotation.w);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_gpsins_localizer_node");
    return RUN_ALL_TESTS();
}
