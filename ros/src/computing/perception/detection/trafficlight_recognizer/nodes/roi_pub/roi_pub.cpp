#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Context.h>

#include <sstream>
#include <string>
#include <vector>

#include <std_msgs/Bool.h>
#include <autoware_msgs/Signals.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

static const std::string WINDOW_NAME = "superimpose result";

// A frame image acquired from topic
cv::Mat frame_;

std::string image_topic_name;
bool swap_pole_lane_id;

// Timestamp of a frame in process
std_msgs::Header frame_header_;

// Publishers
ros::Publisher roi_image_publisher;
ros::Publisher marker_publisher;
ros::Publisher superimpose_image_publisher;

void PublishMarkerArray(std::vector<Context> contexts)
{
  // Define color constants
  std_msgs::ColorRGBA color_black;
  color_black.r = 0.0f;
  color_black.g = 0.0f;
  color_black.b = 0.0f;
  color_black.a = 1.0f;

  std_msgs::ColorRGBA color_red;
  color_red.r = 1.0f;
  color_red.g = 0.0f;
  color_red.b = 0.0f;
  color_red.a = 1.0f;

  std_msgs::ColorRGBA color_yellow;
  color_yellow.r = 1.0f;
  color_yellow.g = 1.0f;
  color_yellow.b = 0.0f;
  color_yellow.a = 1.0f;

  std_msgs::ColorRGBA color_green;
  color_green.r = 0.0f;
  color_green.g = 1.0f;
  color_green.b = 0.0f;
  color_green.a = 1.0f;

  // publish all result as ROS MarkerArray
  for (const auto ctx : contexts)
  {
    visualization_msgs::MarkerArray signal_set;
    visualization_msgs::Marker red_light, yellow_light, green_light;

    // Set the frame ID
    red_light.header.frame_id = "map";
    yellow_light.header.frame_id = "map";
    green_light.header.frame_id = "map";

    // Set the namespace and ID for this markers
    red_light.ns = "tlr_result_red";
    red_light.id = ctx.signalID;

    yellow_light.ns = "tlr_result_yellow";
    yellow_light.id = ctx.signalID;

    green_light.ns = "tlr_result_green";
    green_light.id = ctx.signalID;

    // Set the markers type
    red_light.type = visualization_msgs::Marker::SPHERE;
    yellow_light.type = visualization_msgs::Marker::SPHERE;
    green_light.type = visualization_msgs::Marker::SPHERE;

    // Set the pose of the markers
    red_light.pose.position.x = ctx.redCenter3d.x;
    red_light.pose.position.y = ctx.redCenter3d.y;
    red_light.pose.position.z = ctx.redCenter3d.z;
    red_light.pose.orientation.x = 0.0;
    red_light.pose.orientation.y = 0.0;
    red_light.pose.orientation.z = 0.0;
    red_light.pose.orientation.w = 0.0;

    yellow_light.pose.position.x = ctx.yellowCenter3d.x;
    yellow_light.pose.position.y = ctx.yellowCenter3d.y;
    yellow_light.pose.position.z = ctx.yellowCenter3d.z;
    yellow_light.pose.orientation.x = 0.0;
    yellow_light.pose.orientation.y = 0.0;
    yellow_light.pose.orientation.z = 0.0;
    yellow_light.pose.orientation.w = 0.0;

    green_light.pose.position.x = ctx.greenCenter3d.x;
    green_light.pose.position.y = ctx.greenCenter3d.y;
    green_light.pose.position.z = ctx.greenCenter3d.z;
    green_light.pose.orientation.x = 0.0;
    green_light.pose.orientation.y = 0.0;
    green_light.pose.orientation.z = 0.0;
    green_light.pose.orientation.w = 0.0;

    // Set the scale of the markers. We assume lamp radius is 30cm in real world
    red_light.scale.x = 0.3;
    red_light.scale.y = 0.3;
    red_light.scale.z = 0.3;

    yellow_light.scale.x = 0.3;
    yellow_light.scale.y = 0.3;
    yellow_light.scale.z = 0.3;

    green_light.scale.x = 0.3;
    green_light.scale.y = 0.3;
    green_light.scale.z = 0.3;

    // Set the color for each marker
    switch (ctx.lightState)
    {
      case GREEN:
        red_light.color = color_black;
        yellow_light.color = color_black;
        green_light.color = color_green;
        break;
      case YELLOW:
        red_light.color = color_black;
        yellow_light.color = color_yellow;
        green_light.color = color_black;
        break;
      case RED:
        red_light.color = color_red;
        yellow_light.color = color_black;
        green_light.color = color_black;
        break;
      case UNDEFINED:
        red_light.color = color_black;
        yellow_light.color = color_black;
        green_light.color = color_black;
        break;
    }

    red_light.lifetime = ros::Duration(0.1);
    yellow_light.lifetime = ros::Duration(0.1);
    green_light.lifetime = ros::Duration(0.1);

    // Pack each light marker into one
    signal_set.markers.push_back(red_light);
    signal_set.markers.push_back(yellow_light);
    signal_set.markers.push_back(green_light);

    // Publish
    marker_publisher.publish(signal_set);
  }
}  // void PublishMarkerArray()

void PublishImage(std::vector<Context> contexts)
{
  // Copy the frame image for output
  cv::Mat result_image = frame_.clone();
  cv::cvtColor(result_image, result_image, cv::COLOR_RGB2BGR);

  // Define information for written label
  std::string label;
  const int k_font_face = cv::FONT_HERSHEY_COMPLEX_SMALL;
  const double k_font_scale = 0.8;
  int font_baseline = 0;
  CvScalar label_color;

  for (const auto ctx : contexts)
  {
    // Draw superimpose result on image
    circle(result_image, ctx.redCenter, ctx.lampRadius, CV_RGB(255, 0, 0), 1, 0);
    circle(result_image, ctx.yellowCenter, ctx.lampRadius, CV_RGB(255, 255, 0), 1, 0);
    circle(result_image, ctx.greenCenter, ctx.lampRadius, CV_RGB(0, 255, 0), 1, 0);

    // Draw recognition result on image
    switch (ctx.lightState)
    {
      case GREEN:
        label = "GREEN";
        label_color = CV_RGB(0, 255, 0);
        break;
      case YELLOW:
        label = "YELLOW";
        label_color = CV_RGB(255, 255, 0);
        break;
      case RED:
        label = "RED";
        label_color = CV_RGB(255, 0, 0);
        break;
      case UNDEFINED:
        label = "UNKNOWN";
        label_color = CV_RGB(0, 0, 0);
    }

    if (ctx.leftTurnSignal)
    {
      label += " LEFT";
    }
    if (ctx.rightTurnSignal)
    {
      label += " RIGHT";
    }
    // add lane # text
    label += " " + std::to_string(ctx.closestLaneId);

    cv::Point label_origin = cv::Point(ctx.topLeft.x, ctx.botRight.y + font_baseline);

    cv::putText(result_image, label, label_origin, k_font_face, k_font_scale, label_color);
  }

  // Publish superimpose result image
  cv_bridge::CvImage converter;
  converter.header = frame_header_;
  converter.encoding = sensor_msgs::image_encodings::BGR8;
  converter.image = result_image;
  superimpose_image_publisher.publish(converter.toImageMsg());
}  // void PublishImage()

void superimposeCb(const std_msgs::Bool::ConstPtr& config_msg)
{
  bool show_superimpose_result = config_msg->data;

  if (show_superimpose_result)
  {
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::startWindowThread();
  }

  if (!show_superimpose_result)
  {
    if (cvGetWindowHandle(WINDOW_NAME.c_str()) != NULL)
    {
      cv::destroyWindow(WINDOW_NAME);
      cv::waitKey(1);
    }
  }
}  // void superimposeCb()

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  auto se_frame = cv_bridge::toCvShare(msg);

  if (msg->encoding == "bgr8")
    cv::cvtColor(se_frame->image, frame_, cv::COLOR_BGR2RGB);
  else
    cv::cvtColor(se_frame->image, frame_, cv::COLOR_BayerRG2RGB);

  frame_header_ = msg->header;
}

void roiSignalCallback(const autoware_msgs::Signals::ConstPtr& extracted_pos)
{
  static ros::Time previous_timestamp;
  std::vector<Context> contexts_;
  // If frame has not been prepared, abort this callback
  if (frame_.empty() || frame_header_.stamp == previous_timestamp)
  {
    std::cout << "No Image" << std::endl;
    return;
  }

  // Acquire signal position on the image
  // Context::SetContexts(contexts_, extracted_pos, frame_.rows, frame_.cols, swap_pole_lane_id);
  Context::SetContexts(contexts_, extracted_pos, frame_.rows, frame_.cols);

  // Publish recognition result as some topic format
  if (extracted_pos->Signals.size() == 0 || contexts_.size() == 0)
  {
    std::cout << "no signals in the image" << std::endl;
  }
  else
  {
    // Extract ROI for top signal in vector (top signal has largest estimated
    // radius in every signals projected in a image)
    cv::Mat roi = frame_(cv::Rect(contexts_.at(0).topLeft, contexts_.at(0).botRight));

    cv_bridge::CvImage converter;
    converter.header = frame_header_;
    converter.encoding = sensor_msgs::image_encodings::RGB8;
    converter.image = roi;
    roi_image_publisher.publish(converter.toImageMsg());
  }

  PublishMarkerArray(contexts_);
  PublishImage(contexts_);

  previous_timestamp = frame_header_.stamp;
}

void getROSParams()
{
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("image_raw_topic", image_topic_name, "/image_raw");
  ROS_INFO("image_raw_topic: %s", image_topic_name.c_str());

  private_node_handle.param<bool>("swap_pole_lane_id", swap_pole_lane_id, "false");
  ROS_INFO("swap_pole_lane_id: %s", swap_pole_lane_id ? "true" : "false");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tlr_roi_pub");
  ros::NodeHandle n;

  getROSParams();

  // Register subscribers
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_subscriber = it.subscribe(image_topic_name, 1, &imageCallback, ros::VoidPtr());
  ros::Subscriber roi_signal_subscriber = n.subscribe("/roi_signal", 1, &roiSignalCallback);
  ros::Subscriber superimpose_sub = n.subscribe("/config/superimpose", 1, &superimposeCb);

  // Register publishers
  roi_image_publisher = n.advertise<sensor_msgs::Image>("tlr_roi_image", 1);
  marker_publisher = n.advertise<visualization_msgs::MarkerArray>("tlr_result", 1, true);
  superimpose_image_publisher = n.advertise<sensor_msgs::Image>("tlr_superimpose_image", 1);

  ros::spin();

  return 0;
}
