#ifndef REGION_TLR_TENSORFLOW_H
#define REGION_TLR_TENSORFLOW_H

#include <string>
#include <iostream>
#include <fstream>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "Context.h"
#include "autoware_msgs/Signals.h"

class RegionTLRTensorFlowROSNode
{
public:
  RegionTLRTensorFlowROSNode();
  ~RegionTLRTensorFlowROSNode();

  void RunRecognition();
  void ImageRawCallback(const sensor_msgs::ImageConstPtr &msg);
  void ROISignalCallback(const autoware_msgs::Signals::ConstPtr &extracted_pos);

  // The vector of data structure to save traffic light state, position, ...etc
  std::vector<Context> contexts_;

  // Service client
  ros::ServiceClient srv_client;

private:
  void GetROSParam();
  void StartSubscribersAndPublishers();
  void DetermineState(LightState in_current_state, Context& in_out_signal_context);
  void PublishTrafficLight(std::vector<Context> contexts);
  void PublishString(std::vector<Context> contexts);
  void PublishMarkerArray(std::vector<Context> contexts);
  void PublishImage(std::vector<Context> contexts);
  void SuperimposeCb(const std_msgs::Bool::ConstPtr &config_msg);

  // Execution parameter
  std::string image_topic_name_;
  double score_threshold_;
  int change_state_threshold_;// The threshold of state detected times to accept the state change
  bool swap_pole_lane_id;

  // Subscribers
  image_transport::Subscriber image_subscriber;
  ros::Subscriber roi_signal_subscriber;
  ros::Subscriber superimpose_sub;

  // Publishers
  ros::Publisher signal_state_publisher;
  ros::Publisher signal_state_string_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher superimpose_image_publisher;
  ros::Publisher roi_image_publisher;

  // Flag to show topic will be published in latch manner
  bool kAdvertiseInLatch_;

  // A frame image acquired from topic
  cv::Mat frame_;

  // Timestamp of a frame in process
  std_msgs::Header frame_header_;

  // constant values to pass recognition states to other nodes
  const int32_t kTrafficLightRed;
  const int32_t kTrafficLightGreen;
  const int32_t kTrafficLightUnknown;
  const std::string kStringRed;
  const std::string kStringGreen;
  const std::string kStringUnknown;
};

#endif  // REGION_TLR_TENSORFLOW_H
