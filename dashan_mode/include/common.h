#ifndef READFILE_H
#define READFILE_H

#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>






bool readWayPoints(const std::string path,std::map<int,geometry_msgs::PoseArray>& path_lists);

bool writeWayPoints(const std::string path, std::vector<geometry_msgs::PoseArray>& path_lists);

#endif
