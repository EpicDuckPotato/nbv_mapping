#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <fstream>
#include "map.h"
#include "planner.h"
#include "sensor_footprint.h"
#include <Eigen/Dense>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh("~");

  string map_file;
  double cube_length;
  double sf_depth;
  double sf_width;
  nh.getParam("map_file", map_file);
  nh.getParam("cube_length", cube_length);
  nh.getParam("sf_depth", sf_depth);
  nh.getParam("sf_width", sf_width);

  // Store map into vector
  std::ifstream in;
  in.open(map_file);
  vector<CellStatus> cells;
  int element;
  if (in.is_open()) {
    while (in >> element) {
      cells.push_back(element ? OCCUPIED : FREE);
    }
  }
  in.close();

  // Determine y extent of map
  int maprows = 0;
  int mapcols = 0;
  string line;
  in.open(map_file);
  while (getline(in, line)) {
    ++maprows;
  }
  in.close();
  mapcols = cells.size()/maprows;

  Map ground_truth_map(maprows, mapcols, cube_length, cells);

  // Initialize planner
  double x = 0;
  double y = 0;
  double theta = 0;
  Planner planner(maprows, mapcols, cube_length, x, y, theta, sf_depth, sf_width);

  // ROS stuff
  tf2_ros::TransformBroadcaster br;
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/drone/path", 1);
  nav_msgs::Path path;
  path.header.frame_id = "world";

  // Publish map as a marker array of cubes
  ros::Publisher map_pub = nh.advertise<visualization_msgs::MarkerArray>("/drone/map", 1);
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers = vector<visualization_msgs::Marker>(cells.size());
  for (size_t i = 0; i < cells.size(); ++i) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].ns = "planner_node";
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
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
    marker_array.markers[i].id = i;
  }

  ros::Publisher sensor_footprint_pub = nh.advertise<visualization_msgs::Marker>("/drone/sensor_footprint", 1);
  visualization_msgs::Marker sensor_footprint;
  sensor_footprint.header.frame_id = "world";
  sensor_footprint.ns = "planner_node";
  sensor_footprint.action = visualization_msgs::Marker::ADD;
  sensor_footprint.pose.position.x = 0;
  sensor_footprint.pose.position.y = 0;
  sensor_footprint.pose.position.z = 0;
  sensor_footprint.pose.orientation.w = 1;
  sensor_footprint.pose.orientation.x = 0;
  sensor_footprint.pose.orientation.y = 0;
  sensor_footprint.pose.orientation.z = 0;
  sensor_footprint.type = visualization_msgs::Marker::MESH_RESOURCE;
  sensor_footprint.scale.x = 1.0;
  sensor_footprint.scale.y = 1.0;
  sensor_footprint.scale.z = 1.0;
  sensor_footprint.color.r = 1.0f;
  sensor_footprint.color.g = 1.0f;
  sensor_footprint.color.a = 0.5;
  sensor_footprint.mesh_resource = "package://nbv_mapping/meshes/cone.stl";
  sensor_footprint.id = cells.size();

  ros::Rate r(20);
  while (ros::ok()) {
    // Update planner map using current sensor footprint
    const Map &map = planner.getMap();;
    SensorFootprint footprint(x, y, theta, sf_depth, sf_width);
    unordered_set<int> visible_cells;
    footprint.computeVisibleCells(visible_cells, ground_truth_map);
    unordered_map<int, CellStatus> cell_stati;
    for (unordered_set<int>::iterator it = visible_cells.begin();
         it != visible_cells.end(); ++it) {
      cell_stati[*it] = ground_truth_map.getStatus(*it);
    }
    planner.updateMap(cell_stati);
    planner.computeNextStep(x, y, theta);
    planner.updatePose(x, y, theta);

    // Publish drone pose to tf
    Eigen::Quaterniond drone_orientation = Eigen::AngleAxisd(theta - M_PI/2, Eigen::Vector3d(0, 0, 1))*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0));
    geometry_msgs::TransformStamped pose;
    pose.transform.rotation.w = drone_orientation.w();
    pose.transform.rotation.x = drone_orientation.x();
    pose.transform.rotation.y = drone_orientation.y();
    pose.transform.rotation.z = drone_orientation.z();

    pose.transform.translation.x = x;
    pose.transform.translation.y = y;
    pose.transform.translation.z = 0;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.child_frame_id = "drone_pose";

    br.sendTransform(pose);

    // Publish sensor footprint
    sensor_footprint.pose.position.x = pose.transform.translation.x;
    sensor_footprint.pose.position.y = pose.transform.translation.y;
    sensor_footprint.pose.position.z = pose.transform.translation.z;
    sensor_footprint.pose.orientation.w = pose.transform.rotation.w;
    sensor_footprint.pose.orientation.x = pose.transform.rotation.x;
    sensor_footprint.pose.orientation.y = pose.transform.rotation.y;
    sensor_footprint.pose.orientation.z = pose.transform.rotation.z;
    sensor_footprint.header.stamp = pose.header.stamp;
    sensor_footprint_pub.publish(sensor_footprint);

    // Publish path
    geometry_msgs::PoseStamped gpose;
    gpose.pose.orientation.w = pose.transform.rotation.w;
    gpose.pose.orientation.x = pose.transform.rotation.x;
    gpose.pose.orientation.y = pose.transform.rotation.y;
    gpose.pose.orientation.z = pose.transform.rotation.z;

    gpose.pose.position.x = pose.transform.translation.x;
    gpose.pose.position.y = pose.transform.translation.y;
    gpose.pose.position.z = pose.transform.translation.z;

    gpose.header.stamp = pose.header.stamp;
    gpose.header.frame_id = "drone_pose";

    path.poses.push_back(gpose);
    path.header.stamp = gpose.header.stamp;
    path_pub.publish(path);

    // Publish map
    for (size_t i = 0; i < cells.size(); ++i) {
      map.getCellPos(marker_array.markers[i].pose.position.x,
                     marker_array.markers[i].pose.position.y,
                     i);
      marker_array.markers[i].pose.position.x += cube_length/2;
      marker_array.markers[i].pose.position.y += cube_length/2;
      marker_array.markers[i].color.r = map.getStatus(i) == UNMAPPED;
      marker_array.markers[i].color.g = map.getStatus(i) == OCCUPIED;
      marker_array.markers[i].color.b = map.getStatus(i) == FREE;
      marker_array.markers[i].color.a = (ground_truth_map.getStatus(i) == OCCUPIED) ? 1 : 0.25;
      marker_array.markers[i].header.stamp = pose.header.stamp;
    }
    map_pub.publish(marker_array);

    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
