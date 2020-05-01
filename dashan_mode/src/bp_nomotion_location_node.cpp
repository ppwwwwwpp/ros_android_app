
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <boost/thread/thread.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "std_srvs/Empty.h"
#include <std_msgs/Bool.h>

//#include "bp_std_msgs/PathTask.h"
#include "vector"
#include <ros/package.h>
#include <common.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <libgen.h>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <iostream>
#include "yaml-cpp/yaml.h"

#include <SDL/SDL_image.h>

#include <fstream>
#include "nav_msgs/GetMap.h"

#include <map_msgs/OccupancyGridUpdate.h>
#include <image_loader.h>

#include <bp_nomotion_location_node.h>

using namespace std;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node &node, T &i)
{
  i = node.as<T>();
}
#endif

void loadMapFromFile(nav_msgs::OccupancyGrid *map,
                     const char *fname, double res, bool negate,
                     double occ_th, double free_th, double *origin,
                     bool trinary)
{
  SDL_Surface *img;

  unsigned char *pixels;
  unsigned char *p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i, j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if (!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
                         std::string(fname) + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  map->info.width = img->w;
  map->info.height = img->h;
  map->info.resolution = res;
  map->info.origin.position.x = *(origin);
  map->info.origin.position.y = *(origin + 1);
  map->info.origin.position.z = 0.0;
  tf::Quaternion q;
  q.setRPY(0, 0, *(origin + 2));
  map->info.origin.orientation.x = q.x();
  map->info.origin.orientation.y = q.y();
  map->info.origin.orientation.z = q.z();
  map->info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  map->data.resize(map->info.width * map->info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  if (trinary || n_channels == 1)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure
  pixels = (unsigned char *)(img->pixels);
  for (j = 0; j < map->info.height; j++)
  {
    for (i = 0; i < map->info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (k = 0; k < avg_channels; k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
        alpha = 1;
      else
        alpha = *(p + n_channels - 1);

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      if (negate)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > occ_th)
        value = +100;
      else if (occ < free_th)
        value = 0;
      else if (trinary || alpha < 1.0)
        value = -1;
      else
      {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
      }

      map->data[MAP_IDX(map->info.width, i, map->info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);
}

int readMap(nav_msgs::OccupancyGrid *map, const std::string &fname)
{
  std::string mapfname = "";
  double origin[3];
  int negate;
  double occ_th, free_th, res;
  bool trinary = true;

  std::ifstream fin(fname.c_str());
  if (fin.fail())
  {
    ROS_ERROR("Map_server could not open %s.", fname.c_str());
    return 1;
  }

#ifdef HAVE_NEW_YAMLCPP
  // The document loading process changed in yaml-cpp 0.5.
  YAML::Node doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
#endif

  try
  {
    doc["resolution"] >> res;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    return 2;
  }

  try
  {
    doc["negate"] >> negate;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a negate tag or it is invalid.");
    return 2;
  }

  try
  {
    doc["occupied_thresh"] >> occ_th;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
    return 2;
  }

  try
  {
    doc["free_thresh"] >> free_th;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    return 2;
  }

  try
  {
    doc["trinary"] >> trinary;
  }
  catch (YAML::Exception)
  {
    ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
    trinary = true;
  }

  try
  {
    doc["origin"][0] >> origin[0];
    doc["origin"][1] >> origin[1];
    doc["origin"][2] >> origin[2];
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    return 2;
  }

  try
  {
    doc["image"] >> mapfname;
    // TODO: make this path-handling more robust
    if (mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      return 2;
    }
    if (mapfname[0] != '/')
    {
      // dirname can modify what you pass it
      char *fname_copy = strdup(fname.c_str());
      mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
      free(fname_copy);
    }
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    return 2;
  }

  ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
  loadMapFromFile(map, mapfname.c_str(), res, negate, occ_th, free_th, origin, trinary);
  map->info.map_load_time = ros::Time::now();
  map->header.frame_id = "map";
  map->header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           map->info.width,
           map->info.height,
           map->info.resolution);

  return 0;
}

bool getMapFileInfo(std::string fname, std::string &image, double &resolution, double &origin_x, double &origin_y, double &origin_z, int &negate, double &occupied_thresh, double &free_thresh)
{
  std::string mapfname = "";
  double origin[3];
  int neg;
  double occ_th, free_th, res;
  bool trinary = true;

  std::ifstream fin(fname.c_str());
  if (fin.fail())
  {
    ROS_ERROR("Map_server could not open %s.", fname.c_str());
    return 1;
  }
#ifdef HAVE_NEW_YAMLCPP
  // The document loading process changed in yaml-cpp 0.5.
  YAML::Node doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
// printf("ssssssssssssss");
#endif

  try
  {
    doc["resolution"] >> res;
    resolution = res;
    //  std::string aaa;
    //    doc["origin"].Scalar() >> aaa;
    //    ROS_ERROR("aaa = %s",aaa.c_str());
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    return false;
  }

  try
  {

    doc["negate"] >> negate;
    negate = neg;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a negate tag or it is invalid.");
    return false;
  }

  try
  {
    doc["occupied_thresh"] >> occ_th;
    occupied_thresh = occ_th;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
    return false;
  }

  try
  {
    doc["free_thresh"] >> free_th;
    free_thresh = free_th;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    return false;
  }

  try
  {
    doc["trinary"] >> trinary;
  }
  catch (YAML::Exception)
  {
    ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
    trinary = true;
  }

  try
  {
    doc["origin"][0] >> origin[0];
    doc["origin"][1] >> origin[1];
    doc["origin"][2] >> origin[2];
    origin_x = origin[0];
    origin_y = origin[1];
    origin_z = origin[2];
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    return false;
  }

  try
  {
    doc["image"] >> mapfname;
    // TODO: make this path-handling more robust
    if (mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      return false;
    }
    if (mapfname[0] != '/')
    {
      // dirname can modify what you pass it
      char *fname_copy = strdup(fname.c_str());
      mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
      free(fname_copy);
      image = mapfname;
    }
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    return false;
  }

  return true;
}

void testFun(const char *filepath)
{

  {
    // rename mapfile
    std::string newfile = "/tmp/111.txt";
    if (0 == rename("/tmp/a.txt", newfile.c_str()))
    {
      printf("rename %s\n", newfile.c_str());
    }
    else
    {
      perror("rename");
    }
    return;
  }

  {
    // remove mapfile
    if (0 == remove(filepath))
    // if(0== remove("/tmp/a.txt"))
    {
      printf("remove %s\n", filepath);
    }
    else
    {
      perror("remove");
    }
    return;
  }

  long FileNumber = 0; //存储文件个数

  DIR *dir = opendir(filepath); //先打开文件

  if (NULL == dir) //判断是否打开成功
  {
    perror("opendir ");
    return;
  }

  struct dirent *di; //dirent结构体指针，用于指向数据

  char p_file[1024]; //用于拼接字符串。遇到子目录。

  while ((di = readdir(dir)) != NULL)
  {
    //要忽略掉.和 .. 如果读到它们，就不要对他们操作。
    if (strcmp(di->d_name, ".") == 0 || strcmp(di->d_name, "..") == 0)
    {
      continue; //忽略掉
    }
    //遇到目录就要进入，使用递归
    else if (di->d_type == DT_DIR)
    {
      sprintf(p_file, "%s / %s", filepath, di->d_name);
      testFun(p_file);
    }
    else
    {
      struct stat st;
      std::string path(filepath);
      path = path + "/" + std::string(di->d_name);
      if (stat(path.c_str(), &st) == -1)
      {
        perror("stat");
        return;
      }

      printf("this is file %s,last file modification =%s\n", di->d_name, ctime(&st.st_mtime));
      std::string str();
      if (path.substr(path.size() - 5) == ".yaml")
      {
        printf("path = %s\n", path.c_str());
        std::string image;
        double resolution, origin_x, origin_y, origin_z, occupied_thresh, free_thresh;
        int negate;
        if (getMapFileInfo(path, image, resolution, origin_x, origin_y, origin_z, negate, occupied_thresh, free_thresh))
        {
          printf("image = %s,%f,%f,%f,%f,%d,%f,%f\n", image.c_str(), resolution, origin_x, origin_y, origin_z, negate, occupied_thresh, free_thresh);
        }
        else
        {
          printf("getMapFileInfo return false\n");
        }
      }
    }
  }

  closedir(dir);
}

void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  ROS_INFO("amclPoseCallBack");
  if (msg.pose.covariance[0] < 0.01 && msg.pose.covariance[7] < 0.01 && msg.pose.covariance[35] < 0.01)
  {
    ROS_INFO("pose right");
  }
  else
  {
    ROS_ERROR("pose error");
  }
}

struct MyPoint
{
  int type;
  int id;
  double x_world;
  double y_world;
  double z_world;
  double yaw_world;
};

template <class Type>
Type stringToNum2(const std::string &str)
{
  std::istringstream iss(str);
  Type num;
  iss >> num;

  return num;
}

typedef std::string::size_type string_size;
std::vector<std::string> splitString2(const std::string &s, const std::string &seperator)
{
  std::vector<std::string> result;
  string_size i = 0;

  while (i != s.size())
  {
    //找到字符串中首个不等于分隔符的字母；
    int flag = 0;
    while (i != s.size() && flag == 0)
    {
      flag = 1;
      for (string_size k = 0; k < seperator.size(); k++)
      {
        if (s[i] == seperator[k])
        {
          i++;
          flag = 0;
          break;
        }
      }
    }

    //找到又一个分隔符，将两个分隔符之间的字符串取出；
    flag = 0;
    string_size j = i;
    while (j != s.size() && flag == 0)
    {
      for (string_size k = 0; k < seperator.size(); k++)
      {
        if (s[j] == seperator[k])
        {
          flag = 1;
          break;
        }
      }
      if (flag == 0)
        j++;
    }

    if (i != j)
    {
      result.push_back(s.substr(i, j - i));
      i = j;
    }
  }
  return result;
}

void testPoints()
{
  char *home;
  home = getenv("HOME");
  printf("the home path is %s\n", home);
  std::string path;
  path = std::string(home) + "/data/points";

  std::vector<MyPoint> vec_points;
  std::ifstream fin(path.c_str());
  if (fin.is_open() == false)
  {
    std::cout << " Read  File Failed!!!" << std::endl;
    return;
  }
  std::string line;
  while (std::getline(fin, line))
  {
    std::vector<std::string> results;

    results = splitString2(line, " ");
    if (results.size() != 6)
      continue;
    MyPoint pt;
    pt.type = stringToNum2<int>(results[0]);
    pt.id = stringToNum2<int>(results[1]);
    pt.x_world = stringToNum2<double>(results[2]);
    pt.y_world = stringToNum2<double>(results[3]);
    pt.z_world = stringToNum2<double>(results[4]);
    pt.yaw_world = stringToNum2<double>(results[5]);

    std::cout << pt.type << "  " << pt.id << "  " << pt.x_world << "  " << pt.y_world << "  " << pt.z_world << "  " << pt.yaw_world << std::endl;
    vec_points.push_back(pt);
  }
  fin.close();

  {
      // add point
          std::fstream data_file;
      data_file.open(path.c_str(), std::ios::out);
      if (!data_file.is_open())
      {
        std::cout << "can not open data file fail" << std::endl;
        return;
      }

        MyPoint pt;
        pt.type = 0;
        pt.id = 0;
        pt.x_world = 1.1;
        pt.y_world = 2.1;
        pt.z_world = 3.0;
        pt.yaw_world = 4.0;
       vec_points.push_back(pt);
       data_file << pt.type << "  " << pt.id << "  " << pt.x_world << "  " << pt.y_world << "  " << pt.z_world << "  " << pt.yaw_world << std::endl;
        data_file.close();
  }

  {
    // delete point
    MyPoint del_pt;
    del_pt.type = 0;
    del_pt.id = 0;

    for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
    {
      if (it->type == del_pt.type && it->id == del_pt.id)
      {
        it = vec_points.erase(it);
      }
    }
    std::fstream data_file;
    data_file.open(path.c_str(), std::ios::out);
    if (!data_file.is_open())
    {
      std::cout << "can not open data file fail" << std::endl;
      return;
    }
    for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
    {
      MyPoint pt;
      pt = *it;
      data_file << pt.type << "  " << pt.id << "  " << pt.x_world << "  " << pt.y_world << "  " << pt.z_world << "  " << pt.yaw_world << std::endl;
    }
    data_file.close();
  }

  {
    //modify point
    MyPoint modify_pt;
    modify_pt.type = 1;
    modify_pt.id = 1;

    for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
    {
      if (it->type == modify_pt.type && it->id == modify_pt.id)
      {
        it->x_world = 100.0;
        break;
      }
    }
    std::fstream data_file;
    data_file.open(path.c_str(), std::ios::out);
    if (!data_file.is_open())
    {
      std::cout << "can not open data file fail" << std::endl;
      return;
    }
    for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
    {
      MyPoint pt;
      pt = *it;
      data_file << pt.type << "  " << pt.id << "  " << pt.x_world << "  " << pt.y_world << "  " << pt.z_world << "  " << pt.yaw_world << std::endl;
    }
    data_file.close();
  }

  {
    //look up point
    MyPoint look_pt;
    look_pt.type = 1;
    look_pt.id = 1;
    for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
    {
      if (it->type == look_pt.type && it->id == look_pt.id)
      {
        // to do ................
        break;
      }
    }
  }
}

ros::Publisher pub_cur_map;

bool worldToMap(nav_msgs::OccupancyGrid &map, double wx, double wy, unsigned int &mx, unsigned int &my)
{
  double origin_x, origin_y, resolution;
  origin_x = map.info.origin.position.x;
  origin_y = map.info.origin.position.y;
  resolution = map.info.resolution;
  unsigned int size_x, size_y;
  size_x = map.info.width;
  size_y = map.info.height;
  if (wx < origin_x || wy < origin_y)
    return false;

  mx = (int)((wx - origin_x) / resolution);
  my = (int)((wy - origin_y) / resolution);

  if (mx < size_x && my < size_y)
    return true;

  return false;
}

bool canGenLineOnMap(double x1, double y1, double x2, double y2, std::string map, double r)
{
  std::string fname("/home/junquan/maps/office.yaml");
  // std::string fname(map);
  nav_msgs::OccupancyGrid nav_map;
  readMap(&nav_map, fname);
  pub_cur_map.publish(nav_map);

  unsigned int start_x_u, start_y_u, goal_x_u, goal_y_u;

  if (!(worldToMap(nav_map, x1, y1, start_x_u, start_y_u)))
  {
    ROS_ERROR("canGenLineOnMap fail,start.pose.position.x = %f,start.pose.position.y = %f", x1, y1);
    return false;
  }
  if (!(worldToMap(nav_map, x2, y2, goal_x_u, goal_y_u)))
  {
    ROS_ERROR("canGenLineOnMap fail,goal.pose.position.x = %f,goal.pose.position.y = %f", x2, y2);
    return false;
  }

  int start_x, start_y, goal_x, goal_y;
  start_x = start_x_u;
  start_y = start_y_u;
  goal_x = goal_x_u;
  goal_y = goal_y_u;

  ROS_INFO("start_x= %d,start_y=%d,goal_x=%d,goal_y=%d", start_x, start_y, goal_x, goal_y);

  geometry_msgs::PoseStamped tmp;
  tmp.header.stamp = ros::Time::now();
  tmp.header.frame_id = "map";
  tmp.pose.orientation.w = 1.0;
  int dx = abs(start_x - goal_x);
  if (dx <= 2 && goal_y > start_y)
  {
    for (int i = start_y; i <= goal_y; i++)
    {
      if (nav_map.data[i * nav_map.info.width + start_x] == (char)100 || nav_map.data[i * nav_map.info.width + start_x] == (char)-1)
      {
        return false;
      }
    }
  }
  else if (dx <= 2 && goal_y < start_y)
  {
    for (int i = start_y; i >= goal_y; i--)
    {
      if (nav_map.data[i * nav_map.info.width + start_x] == (char)100 || nav_map.data[i * nav_map.info.width + start_x] == (char)-1)
      {
        return false;
      }
    }
  }

  // line y = k * x + b
  double k, b;
  k = (goal_y - start_y) * 1.0 / (goal_x - start_x);
  b = start_y - k * start_x;
  if (start_x < goal_x)
  {
    for (int i = start_x; i <= goal_x; i++)
    {
      int y;
      y = k * i + b;
      if (nav_map.data[y * nav_map.info.width + i] == (char)100 || nav_map.data[y * nav_map.info.width + i] == (char)-1)
      {
        return false;
      }
    }
  }
  else
  {
    for (int i = start_x; i >= goal_x; i--)
    {
      int y;
      y = k * i + b;
      if (nav_map.data[y * nav_map.info.width + i] == (char)100 || nav_map.data[y * nav_map.info.width + i] == (char)-1)
      {
        return false;
      }
    }
  }

  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "bp_nomotion_location_node");

  tf::TransformListener tf;
  tf::StampedTransform cur_transform;
  tf::StampedTransform last_transform;   
  ros::Rate loop_rate(1.0); //0.0637
  ros::ServiceClient client;
  ros::NodeHandle n;

  ros::Subscriber sub_amcl_pose = n.subscribe("amcl_pose", 1, amclPoseCallBack);
  pub_cur_map = n.advertise<nav_msgs::OccupancyGrid>("map2", 1, true);

  {
    char *home;
    home = getenv("HOME");
    printf("the home path is %s\n", home);
    testPoints();
    //   testFun("/home/junquan/maps");
    testFun((std::string(home) + "/data/maps").c_str());
    std::string fname("/home/junquan/maps/office.yaml");
    nav_msgs::OccupancyGrid nav_map;
    readMap(&nav_map, fname);
    // pub_cur_map.publish(nav_map);
  }
  //canGenLineOnMap(1,1,1,1,"",1);

  ros::spin();

  return 0;
}


/*
{ "op": "set_level",
  "level": "error"
}

{ "op": "status",
  "level": "error",
  "msg": {"pose right","pose right"}
}



    { "op": "png",
    
      "data": { "op":"subscribe","topic": "/map","type": "nav_msgs/OccupancyGrid" }

    }

{ "op": "subscribe",
  "topic": "/map",
  "type": "nav_msgs/OccupancyGrid"
}



*/
