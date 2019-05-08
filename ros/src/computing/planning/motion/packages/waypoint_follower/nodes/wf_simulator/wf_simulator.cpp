/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <random>

#include "autoware_msgs/VehicleCmd.h"
#include "waypoint_follower/libwaypoint_follower.h"

namespace
{
const std::string SIMULATION_FRAME = "sim_base_link";
const std::string LIDAR_FRAME = "sim_lidar";
const std::string MAP_FRAME = "map";

geometry_msgs::Pose _initial_pose;
bool _initial_set = false;
bool _pose_set = false;
bool _waypoint_set = false;
bool _use_ctrl_cmd = false;
bool g_is_closest_waypoint_subscribed = false;
WayPoints _current_waypoints;
ros::Publisher g_odometry_publisher;
ros::Publisher g_velocity_publisher;
int32_t g_closest_waypoint = -1;
double g_position_error;
double g_angle_error;
double g_linear_acceleration = 0;
double g_steering_angle = 0;
double g_wheel_base_m = 2.7;

constexpr int LOOP_RATE = 50;  // 50Hz

void CmdCallBack(const autoware_msgs::VehicleCmdConstPtr& msg, double accel_rate)
{
  if (_use_ctrl_cmd == true)
  {
    g_linear_acceleration = msg->ctrl_cmd.linear_acceleration;
    g_steering_angle = msg->ctrl_cmd.steering_angle;
  }
  else
  {
    static double previous_linear_velocity = 0;

    if (_current_velocity.linear.x < msg->twist_cmd.twist.linear.x)
    {
      _current_velocity.linear.x = previous_linear_velocity + accel_rate / (double)LOOP_RATE;

      if (_current_velocity.linear.x > msg->twist_cmd.twist.linear.x)
      {
        _current_velocity.linear.x = msg->twist_cmd.twist.linear.x;
      }
    }
    else
    {
      _current_velocity.linear.x = previous_linear_velocity - accel_rate / (double)LOOP_RATE;

      if (_current_velocity.linear.x < msg->twist_cmd.twist.linear.x)
      {
        _current_velocity.linear.x = msg->twist_cmd.twist.linear.x;
      }
    }

    previous_linear_velocity = _current_velocity.linear.x;

    _current_velocity.angular.z = msg->twist_cmd.twist.angular.z;

    //_current_velocity = msg->twist;
  }
}
void getTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform& transform)
{
  static tf::TransformListener listener;

  while (1)
  {
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& input)
{
  tf::StampedTransform transform;
  getTransformFromTF(MAP_FRAME, input->header.frame_id, transform);

  initial_pose_.position.x = input->pose.pose.position.x + transform.getOrigin().x();
  initial_pose_.position.y = input->pose.pose.position.y + transform.getOrigin().y();
  initial_pose_.position.z = input->pose.pose.position.z + transform.getOrigin().z();
  initial_pose_.orientation = input->pose.pose.orientation;

  initial_set_ = true;
  pose_set_ = false;
}

void callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr& msg)
{
  initial_pose_ = msg->pose;
  initial_set_ = true;
}

void waypointCallback(const autoware_msgs::LaneConstPtr& msg)
{
  // _path_og.setPath(msg);
  _current_waypoints.setPath(*msg);
  _waypoint_set = true;
  // ROS_INFO_STREAM("waypoint subscribed");
}

void callbackFromClosestWaypoint(const std_msgs::Int32ConstPtr& msg)
{
  closest_waypoint_ = msg->data;
  is_closest_waypoint_subscribed_ = true;
}

void updateVelocity()
{
  if (_use_ctrl_cmd == false)
    return;

  _current_velocity.linear.x += g_linear_acceleration / (double)LOOP_RATE;
  _current_velocity.angular.z = _current_velocity.linear.x * std::sin(g_steering_angle) / g_wheel_base_m;
}

void publishOdometry()
{
  static ros::Time current_time = ros::Time::now();
  static ros::Time last_time = ros::Time::now();
  static geometry_msgs::Pose pose;
  static double th = 0;
  static tf::TransformBroadcaster tf_broadcaster;

  if (!pose_set_)
  {
    pose.position = initial_pose_.position;
    pose.orientation = initial_pose_.orientation;
    th = tf::getYaw(pose.orientation);
    ROS_INFO_STREAM("pose set : (" << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << th
                                   << ")");
    pose_set_ = true;
  }

  /*int closest_waypoint = getClosestWaypoint(_current_waypoints.getCurrentWaypoints(), pose);
  if (closest_waypoint == -1)
  {
    ROS_INFO("cannot publish odometry because closest waypoint is -1.");
    return;
  }
  else
  {
    pose.position.z = _current_waypoints.getWaypointPosition(closest_waypoint).z;
  }
*/ if (
      _waypoint_set && g_is_closest_waypoint_subscribed)
    pose.position.z = _current_waypoints.getWaypointPosition(g_closest_waypoint).z;
  double vx = _current_velocity.linear.x;
  double vth = _current_velocity.angular.z;
  current_time = ros::Time::now();

  // compute odometry in a typical way given the velocities of the robot
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_real_distribution<double> rnd_dist(0.0, 2.0);
  double rnd_value_x = rnd_dist(mt) - 1.0;
  double rnd_value_y = rnd_dist(mt) - 1.0;
  double rnd_value_th = rnd_dist(mt) - 1.0;

  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th)) * dt + rnd_value_x * position_error_;
  double delta_y = (vx * sin(th)) * dt + rnd_value_y * position_error_;
  double delta_th = vth * dt + rnd_value_th * angle_error_ * M_PI / 180;

  pose.position.x += delta_x;
  pose.position.y += delta_y;
  th += delta_th;
  pose.orientation = tf::createQuaternionMsgFromYaw(th);

  // std::cout << "delta (x y th) : (" << delta_x << " " << delta_y << " " <<
  // delta_th << ")" << std::endl;
  // std::cout << "current_velocity(linear.x angular.z) : (" <<
  // _current_velocity.linear.x << " " <<
  // _current_velocity.angular.z << ")"<< std::endl;
  //    std::cout << "current_pose : (" << pose.position.x << " " <<
  //    pose.position.y<< " " << pose.position.z << " " <<
  //    th << ")" << std::endl << std::endl;

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = MAP_FRAME;
  odom_trans.child_frame_id = SIMULATION_FRAME;

  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;

  // send odom transform
  tf_broadcaster.sendTransform(odom_trans);

  geometry_msgs::TransformStamped lidar_trans;
  lidar_trans.header.stamp = odom_trans.header.stamp;
  lidar_trans.header.frame_id = SIMULATION_FRAME;
  lidar_trans.child_frame_id = LIDAR_FRAME;
  lidar_trans.transform.translation.z += lidar_height_;
  lidar_trans.transform.rotation.w = 1;

  // send lidar transform
  tf_broadcaster.sendTransform(lidar_trans);

  // next, we'll publish the odometry message over ROS
  std_msgs::Header h;
  h.stamp = current_time;
  h.frame_id = MAP_FRAME;

  geometry_msgs::PoseStamped ps;
  ps.header = h;
  ps.pose = pose;

  geometry_msgs::TwistStamped ts;
  ts.header = h;
  ts.twist.linear.x = vx;
  ts.twist.angular.z = vth;

  // publish the message
  odometry_publisher_.publish(ps);
  velocity_publisher_.publish(ts);

  last_time = current_time;
}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wf_simulator");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string initialize_source;
  private_nh.getParam("initialize_source", initialize_source);
  ROS_INFO_STREAM("initialize_source : " << initialize_source);

  double accel_rate;
  private_nh.param("accel_rate", accel_rate, double(1.0));
  ROS_INFO_STREAM("accel_rate : " << accel_rate);

  private_nh.param("position_error", g_position_error, double(0.0));
  private_nh.param("angle_error", g_angle_error, double(0.0));
  nh.param("vehicle_info/wheel_base", g_wheel_base_m, double(2.7));

  // publish topic
  odometry_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 10);
  velocity_publisher_ = nh.advertise<geometry_msgs::TwistStamped>("sim_velocity", 10);

  // subscribe topic
  ros::Subscriber cmd_subscriber =
      nh.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 10, boost::bind(CmdCallBack, _1, accel_rate));
  ros::Subscriber waypoint_subcscriber = nh.subscribe("base_waypoints", 10, waypointCallback);
  ros::Subscriber closest_sub = nh.subscribe("closest_waypoint", 10, callbackFromClosestWaypoint);
  ros::Subscriber initialpose_subscriber;

  if (initialize_source == "Rviz")
  {
    initialpose_subscriber = nh.subscribe("initialpose", 10, initialposeCallback);
  }
  else if (initialize_source == "lidar_localizer")
  {
    initialpose_subscriber = nh.subscribe("ndt_pose", 10, callbackFromPoseStamped);
  }
  else if (initialize_source == "GNSS")
  {
    initialpose_subscriber = nh.subscribe("gnss_pose", 10, callbackFromPoseStamped);
  }
  else
  {
    ROS_INFO("Set pose initializer!!");
  }

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();  // check subscribe topic

    if (!initial_set_)
    {
      loop_rate.sleep();
      continue;
    }

    updateVelocity();
    publishOdometry();

    loop_rate.sleep();
  }

  return 0;
}
