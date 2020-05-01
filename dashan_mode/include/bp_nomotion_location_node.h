#ifndef BP_NOMOTION_LOCATION_NODE__H
#define BP_NOMOTION_LOCATION_NODE__H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#pragma once

 void loadMapFromFile(nav_msgs::OccupancyGrid *map,
                     const char *fname, double res, bool negate,
                     double occ_th, double free_th, double *origin,
                     bool trinary);
 int readMap(nav_msgs::OccupancyGrid *map, const std::string &fname);
 bool getMapFileInfo(std::string fname, std::string &image, double &resolution, double &origin_x, double &origin_y, double &origin_z, int &negate, double &occupied_thresh, double &free_thresh);
 void testFun(const char *filepath);
 void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped &msg);
 int readMap(nav_msgs::OccupancyGrid *map, const std::string &fname);
 void testPoints();
 bool worldToMap(nav_msgs::OccupancyGrid &map, double wx, double wy, unsigned int &mx, unsigned int &my);
 bool canGenLineOnMap(double x1, double y1, double x2, double y2, std::string map, double r);




#endif
