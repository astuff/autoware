/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, All Rights Reserved.
*
* This file is part of the gpsins_localizer which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/loader.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpsins_localizer_node");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(
      nodelet_name, "gpsins_localizer/gpsins_localizer_nodelet", remap, nargv);
    ros::spin();
    return 0;
}
