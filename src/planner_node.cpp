#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <fstream>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh("~");

  string map_file;
  double cube_length;
  nh.getParam("map_file", map_file);
  nh.getParam("cube_length", cube_length);

  // Store map into vector
  std::ifstream in;
  in.open(map_file);
  vector<int> map;
  int element;
  if (in.is_open()) {
    while (in >> element) {
      map.push_back(element);
    }
  }
  in.close();

  // Determine y extent of map
  int maprows = 0;
  int mapcols = 0;
  string line;
  in.open(map_file);
  while (getline(in, line)) {
    ++mapcols;
  }
  in.close();
  maprows = map.size()/mapcols;

  tf2_ros::TransformBroadcaster br;

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/drone/path", 1);
  nav_msgs::Path path;
  path.header.frame_id = "world";

  ros::Publisher map_pub = nh.advertise<visualization_msgs::MarkerArray>("/drone/map", 1);

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers = vector<visualization_msgs::Marker>(map.size());
  for (size_t i = 0; i < map.size(); ++i) {
    int row = i%maprows;
    int col = i/maprows;
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].ns = "planner_node";
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i].pose.position.x = cube_length*col;
    marker_array.markers[i].pose.position.y = cube_length*row;
    marker_array.markers[i].pose.position.z = 0;
    marker_array.markers[i].pose.orientation.w = 1;
    marker_array.markers[i].pose.orientation.x = 0;
    marker_array.markers[i].pose.orientation.y = 0;
    marker_array.markers[i].pose.orientation.z = 0;
    marker_array.markers[i].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[i].scale.x = 1.0;
    marker_array.markers[i].scale.y = 1.0;
    marker_array.markers[i].scale.z = 1.0;
    marker_array.markers[i].color.g = 1.0f;
    marker_array.markers[i].color.a = map[i];
    marker_array.markers[i].id = i;
  }

  ros::Rate r(20);
  while (ros::ok()) {
    // Publish drone position to tf
    geometry_msgs::TransformStamped pose;
    pose.transform.rotation.w = 1;
    pose.transform.rotation.x = 0;
    pose.transform.rotation.y = 0;
    pose.transform.rotation.z = 0;

    pose.transform.translation.x = 0;
    pose.transform.translation.y = 0;
    pose.transform.translation.z = 0;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.child_frame_id = "drone_pose";

    br.sendTransform(pose);

    // Publish path
    /*
    geometry_msgs::PoseStamped gpose;
    gpose.pose.orientation.w = 1;
    gpose.pose.orientation.x = 0;
    gpose.pose.orientation.y = 0;
    gpose.pose.orientation.z = 0;

    gpose.pose.position.x = 0;
    gpose.pose.position.y = 0;
    gpose.pose.position.z = 0;

    gpose.header.stamp = pose.header.stamp;
    gpose.header.frame_id = "drone_pose";

    path.poses.push_back(gpose);
    path.header.stamp = gpose.header.stamp;
    path_pub.publish(path);
    */

    // Display map in rviz
    for (vector<visualization_msgs::Marker>::iterator it = marker_array.markers.begin();
         it != marker_array.markers.end(); ++it) {
      it->header.stamp = pose.header.stamp;
    }

    map_pub.publish(marker_array);

    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
