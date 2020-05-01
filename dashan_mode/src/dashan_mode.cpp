#include <ros/ros.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "vector"
#include <ros/package.h>
#include <common.h>
#include "std_srvs/Empty.h"
#include <std_msgs/Bool.h>
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

#include <dashan_msg/Mode.h>
#include <dashan_msg/map.h>
#include <dashan_msg/posititions.h>
#include <dashan_msg/MapLists.h>

//#include <dashan_msg/PointLists.h>
#include <dashan_msg/pointlist.h>

#include <dashan_msg/pointgroup.h>
#include <dashan_msg/pointprocess.h>
#include <dashan_msg/verify_graph_line.h>
#include <dashan_msg/hand_draw_save.h>
#include <dashan_msg/canGenLineOnMap.h>
#include <dashan_msg/hand_draw_list.h>
#include <dashan_msg/system_status.h>


#include <bp_std_msgs/TaskData.h>
#include <bp_std_msgs/TaskDataList.h>
#include <bp_std_msgs/TaskEntry.h>

#include <bp_std_msgs/AlongWallBow.h>
#include <bp_std_msgs/StartAlongWall.h>
#include <bp_std_msgs/StartAlongWallOrBow.h>
#include <bp_std_msgs/StartSimplePointNav.h>

#include <std_msgs/UInt32.h>


#include "yaml-cpp/yaml.h"

#include <SDL/SDL_image.h>
#include <fstream>

#include <sys/socket.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/if_ether.h>
#include <net/if.h>
#include <linux/sockios.h>

#include "nav_msgs/GetMap.h"
#include <map_msgs/OccupancyGridUpdate.h>
#include <image_loader.h>
#include "nav_msgs/OccupancyGrid.h"


#include <sys/wait.h>
#include <unistd.h>

#define MAPPING_MODE 1
#define NAVIGATION_MODE 2
#define rm_MODE 3

ros::Publisher s_pub_mode_cmd;
ros::Publisher s_pub_MapLists_cmd;
ros::Publisher s_pub_pointgroup_cmd;
ros::Publisher s_pub_diagnose_cmd;
ros::Publisher s_pub_canGenLineOnMapStatus_cmd;
ros::Publisher s_pub_hand_draw_list_cmd;
ros::Publisher s_pub_mix_path_list_cmd;
ros::Publisher s_pub_current_map_cmd;

dashan_msg::Mode zhishanmode;
dashan_msg::MapLists MapLists_pub;
dashan_msg::pointgroup pointgroup_pub;

#define MAC0        0x00  
#define MAC1        0x1f
#define MAC2        0xc6
#define MAC3        0x82
#define MAC4        0x8c
#define MAC5        0xe8

/*extern void loadMapFromFile(nav_msgs::OccupancyGrid *map,
                     const char *fname, double res, bool negate,
                     double occ_th, double free_th, double *origin,
                     bool trinary);
extern int readMap(nav_msgs::OccupancyGrid *map, const std::string &fname);
extern bool getMapFileInfo(std::string fname, std::string &image, double &resolution, double &origin_x, double &origin_y, double &origin_z, int &negate, double &occupied_thresh, double &free_thresh);
extern void testFun(const char *filepath);
extern void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped &msg);
extern int readMap(nav_msgs::OccupancyGrid *map, const std::string &fname);
extern void testPoints();
extern bool worldToMap(nav_msgs::OccupancyGrid &map, double wx, double wy, unsigned int &mx, unsigned int &my);
extern bool canGenLineOnMap(double x1, double y1, double x2, double y2, std::string map, double r);
  */

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

struct stat buf;   //判断yaml文件是否为空
stat(fname.c_str(),&buf);
size_t size = buf.st_size;
if(size == 0)
{
	return false;
}


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
      //char *fname_copy = strdup(fname.c_str());
      //mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
      //free(fname_copy);
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
 /* ROS_INFO("amclPoseCallBack");
  if (msg.pose.covariance[0] < 0.01 && msg.pose.covariance[7] < 0.01 && msg.pose.covariance[35] < 0.01)
  {
    ROS_ERROR("pose right");
  }
  else
  {
    ROS_ERROR("pose error");
  }  */
}

struct MyPoint
{
  float angle;
  int map_id;
  double gridX;
  double gridY;
  double type;
  double id;
  double mapId;
  string mapName;
  string name;
  bool selected;
  double x_world;
  double y_world;
  double z_world;
  double yaw_world;
  double x;
  double y;
  double z;
 
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
  /*char *home;
  home = getenv("HOME");
  printf("the home path is %s\n", home);
  std::string path;
  path = std::string(home) + "/data/points"; */

  std::vector<MyPoint> vec_points;
  std::ifstream fin("/home/lx/data/points.csv");
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
      data_file.open("/home/lx/data/points.csv", std::ios::out);
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
    data_file.open("/home/lx/data/points.csv", std::ios::out);
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
    data_file.open("/home/lx/data/points.csv", std::ios::out);
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

ros::Publisher pub_cur_map_1;
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

int ReadImagePGM(char *filename, char **pBuffer, int *pWidth, int *pHeight){
   FILE *fp;
   char buf[71];
   int width, height;

   if ((fp = fopen(filename, "rb")) == NULL){
     fprintf(stderr, "Error reading the file %s in ReadImagePGM().\n", filename);
     return(0);
   } 

   fgets(buf, 70, fp);
   bool P2 = false;
   bool P5 = false;

   if      (strncmp(buf, "P2", 2) == 0) P2 = true;
   else if (strncmp(buf, "P5", 2) == 0) P5 = true;

   if (P2 == false && P5 == false){
      fprintf(stderr, "The file %s is not in PGM format in ", filename);
      fprintf(stderr, "ReadImagePGM().\n");
      fclose(fp);
      return 0;
   } //end-if

   do {fgets(buf, 70, fp);} while (buf[0] == '#'); 
   sscanf(buf, "%d %d", &width, &height);
   fgets(buf, 70, fp); 

   *pWidth = width;
   *pHeight = height;

   if (((*pBuffer) = (char *) malloc((*pWidth)*(*pHeight))) == NULL){
      fprintf(stderr, "Memory allocation failure in ReadImagePGM().\n");
      fclose(fp);
      return(0);
   } 

   if (P2){
      int index=0;
      char *p = *pBuffer;
      int col = 0;
      int read = 0;

      while (1){
        int c;
        if (fscanf(fp, "%d", &c) < 1) break;
        read++;

        if (col < *pWidth) p[index++] = (unsigned char)c;

        col++;
        if (col == width) col = 0;
      } 

      if (read != width*height){
        fprintf(stderr, "Error reading the image data in ReadImagePGM().\n");
        fclose(fp);
        free((*pBuffer));
        return(0);
      } 

   } else if (P5){
      int index=0;
      char *p = *pBuffer;
      int col = 0;
      int read = 0;

      while (1){
        unsigned char c;
        if (fread(&c, 1, 1, fp) < 1) break;
        read++;

        if (col < *pWidth) p[index++] = c;

        col++;
        if (col == width) col = 0;
      } //end-while

     if (read != width*height){
        fprintf(stderr, "Error reading the image data in ReadImagePGM().\n");
        fclose(fp);
        free((*pBuffer));
        return(0);
     } 
   } 

   fclose(fp);
   return 1;
} 


bool canGenLineOnMap(double x1, double y1, double x2, double y2, std::string map_name)
{

  std::string map_path = "/home/lx/map/"+map_name+".yaml";
  std::string fname(map_path.c_str());
  // std::string fname(map);
  nav_msgs::OccupancyGrid nav_map;
  readMap(&nav_map, fname);
  //pub_cur_map_1.publish(nav_map);

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

struct maplist 
{
   string image;
   double origin_x;
   double origin_y;
   double origin_z;
   double free_thresh;
   double resolution;
};



bool writeWayPoints(const std::string path, const std::string map_name,std::vector<geometry_msgs::PoseArray>& path_lists)
{
 
 if (path_lists.size() == 0)
 {
     ROS_ERROR("path_lists.size() == 0");
     return false;
 }
 

  std::fstream data_file;
  data_file.open(path.c_str(), std::ios::out);
  if (!data_file.is_open())
  {
    std::cout<<"can not open data file! writeWayPoints fail"<<std::endl;
    return false;
  }
  for (int i = 0; i < path_lists.size(); i++)
  {
     data_file<<map_name<<"  ";
     data_file<<i+1<<"  ";
     data_file<<path_lists[i].poses.size()<<"  ";
     for (int k = 0; k < path_lists[i].poses.size(); k++)
     {
        double x,y,yaw;
        x = path_lists[i].poses[k].position.x;
        y = path_lists[i].poses[k].position.y;
        yaw = tf::getYaw(path_lists[i].poses[k].orientation);
        data_file<<x<<"  "<<y<<"  "<<yaw<<"  ";
     }
     data_file<<std::endl;
  }

  data_file.close();
  return true;
}

template <class Type>
Type stringToNum(const std::string &str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;

    return num;
}

typedef std::string::size_type string_size;
std::vector<std::string> splitString(const std::string &s, const std::string &seperator)
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

bool readWayPoints(const std::string path, std::string& map_name,std::map<int,geometry_msgs::PoseArray>& path_lists)
{
    std::ifstream fin(path.c_str());
    if (fin.is_open() == false)
    {
        std::cout << "readWayPoints Read  File Failed" << std::endl;
        return false;
    }
    path_lists.clear();
    std::string line;
    while (std::getline(fin, line))
    {
        std::vector<std::string> results;

        results = splitString(line, " ");
        // for (int  i= 0; i< results.size(); i++)
        // {
        //     std::cout<<results[i]<<"  ";
        // }
        //  std::cout<<std::endl;
        map_name = results[0];
      //  std::cout << map_name<<std::endl;
        int path_id = stringToNum<int>(results[1]);
      //  std::cout << path_id << "  ";
        int waypoint_counts = stringToNum<int>(results[2]);
     //   std::cout << waypoint_counts << "  ";
        if (results.size() - 3 != waypoint_counts * 3)
        {
            std::cout << "readWayPoints results.size() -2 != waypoint_counts * 3!!!" << std::endl;
            return false;
        }

        geometry_msgs::PoseArray waypoints;
        for (int i = 0; i < waypoint_counts; i++)
        {
            double x = stringToNum<double>(results[i * 3 + 3 + 0]);
            double y = stringToNum<double>(results[i * 3 + 3 + 1]);
            double yaw = stringToNum<double>(results[i * 3 + 3 + 2]);
            geometry_msgs::Pose tmp;
            tmp.position.x = x;
            tmp.position.y = y;
            tmp.position.z = 0.0;
            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);
            waypoints.poses.push_back(tmp);
          //  std::cout << x << "  " << y << "  " << yaw << "  ";
        }
        std::cout << std::endl;
      path_lists[path_id] = waypoints;
    }

    // for (std::map<int,geometry_msgs::PoseArray>::iterator it =path_lists.begin(); it!=path_lists.end(); it++)
    // {
    //     std::cout<<it->first<<"  ";
        
    //     for (int i = 0; i < it->second.poses.size(); i++)
    //     {
    //         double x,y;
    //         x = it->second.poses[i].position.x;
    //         y = it->second.poses[i].position.y;
    //          std::cout<<x<<"  "<<y<<"  ";
    //     }
    //     std::cout<<std::endl;
    // }
    
    fin.close();
    return true;
}





 void modeCallback(const dashan_msg::Mode :: ConstPtr &msg)
{
   // ROS_ERROR("fff");
  
    const char *cstr_msg = msg->mode.c_str();

    if (msg->mode == "start_scan_map")
    {
       
       
        //system("source ~/cartographer_ws/install_isolated/setup.bash");
        // system("roslaunch cartographer_ros bprobot.launch &");
         //system("./cartographer.sh");
        //system("rosnode kill amcl");   
        //system("bash -c 'rosnode kill amcl;ps -ef |grep amcl | awk '{print $2}'| xargs kill -9'");
        //system("bash -c 'rosnode kill amcl;rosnode kill camera/realsense2_camera;rosnode kill camera/realsense2_camera_manager;rosnode kill move_base'");
        //system("bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;roslaunch cartographer_ros bprobot.launch &'");

       //system("/home/lx/start_scan_map.sh");   
 int ret;
    pid_t pid;
    int status;
    std::string cmd("/home/lx/start_scan_map.sh");
    if ((pid = fork()) < 0)
    {
      ROS_ERROR("fork < 0");
      return;
    }
    else if (pid == 0)
    {
      execl("/bin/sh", "sh", "-c", cmd.c_str(), (char *)0);
      ROS_INFO("after /bin/sh");
      exit(0);
    }
    else
    {
      if (waitpid(pid, &status, 0) < 0)
      {
        ROS_ERROR("waitpid error");
        return;
      }
      else
      {
        ROS_INFO("wait child process exit success");
      }
    }
        
    }

    if (msg->mode == "start_navigation_system")
    {
       
        //system("/home/lx/start_navigation_system.sh");   
     
        
        
    }

    if (msg->mode == "cancel_scan_map")
    {
          
          system("/home/lx/cancel_scan_map.sh");
         // popen("~/cancel_scan_map.sh","w");
        //system("rosnode kill /cartographer_node");
         // popen("rosnode kill /cartographer_node","w");
        //system("bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;rosnode kill /cartographer_node &'");
       
        //sleep(3);
        //system("bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;rosnode kill /cartographer_occupancy_grid_node'&");
        //system("bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;ps -ef |grep cartographer_occupancy_grid_node | awk '{print $2}'| xargs kill -9'&");        
        //system("bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;ps -ef |grep cartographer_occupancy_grid_node | awk '{print $2}'| xargs kill -9'&");
        //system("bash -c 'source /home/lx/bp_nav_ws/install/setup.bash;roslaunch sf_keyop bp_location.launch'&");
        
    }
    if (msg->mode == "stop_scan_map")
    {
            
        //system("rosservice call /finish_trajectory 0");
        //system("rosservice call /write_state filename=/home/lx/map/mymap.pbstream");       
        
        //system("rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/lx/map -pbstream_filename=/home/lx/map/mymap.pbstream -resolution=0.05");
       
    }
    if (msg->mode == "maps")
    {
     

long FileNumber = 0; //存储文件个数

  DIR *dir = opendir("/home/lx/map"); //先打开文件

  if (NULL == dir) //判断是否打开成功
  {
    perror("opendir ");

  }

  struct dirent *di; //dirent结构体指针，用于指向数据

  char p_file[1024]; //用于拼接字符串。遇到子目录。

  dashan_msg::MapLists vec_maplists;
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
      sprintf(p_file, "%s / %s", "/home/lx/map", di->d_name);
      //testFun(p_file);
    }
    else
    {
      struct stat st;
      std::string path("/home/lx/map");
      path = path + "/" + std::string(di->d_name);
      if (stat(path.c_str(), &st) == -1)
      {
        perror("stat");
       
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
          
		dashan_msg::maplist tmp;
		//tmp.id = 1;

          string str= image ;

	str.erase(str.end()-1);
        str.erase(str.end()-1);
        str.erase(str.end()-1);
	str.erase(str.end()-1);
 
        
        std::string temper = "/home/lx/map/" + str + ".pgm";
     
       /* int width, height;
        unsigned char *srcImg; 
        char *strin = (char *)temper.c_str();
        if (ReadImagePGM(strin, (char **)&srcImg, &width, &height) == 0){
                printf("Failed opening <%s>\n", strin);
                //return 1;
             } */
int width, height;
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
  if (!(img = IMG_Load(temper.c_str())))
  {
    std::string errmsg = std::string("failed to open image file \"") +
                         std::string(temper.c_str()) + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  width = img->w;
  height = img->h;
 

}



    tmp.name = str;
    
    tmp.mapInfo.gridHeight = height;
    tmp.mapInfo.gridWidth = width;
    tmp.mapInfo.originX = origin_x;
    tmp.mapInfo.originY = origin_y;
    
    tmp.mapInfo.resolution = resolution;


              vec_maplists.data.push_back(tmp);

         }   
         else     
	 {
              printf("getMapFileInfo return false\n");
         }
        }
     
      }
   }
   s_pub_MapLists_cmd.publish(vec_maplists);
 }

    if (msg->mode == "is_initialize_finished" )
    {
       

         //amclPoseCallBack
        
         
    }

    if (msg->mode == "is_graph_path_exist" )
    {
       
          

    }
 
    if (msg->mode == "point_list" )
    {
       
     
  /*char *home;
  home = getenv("HOME");
  //printf("the home path is %s\n", home);
  std::string path;
  path = std::string(home) + "/data/points.csv"; */


  /*std::vector<MyPoint> vec_points;
  std::ifstream fin("/home/lx/data/points.csv");
  if (fin.is_open() == false)
  {
    std::cout << " Read  File Failed!!!" << std::endl;
    return;
  }
  std::string line;

  dashan_msg::pointgroup pointgroup;

  while (std::getline(fin, line))
  {
    std::vector<std::string> results;

    results = splitString2(line, "  ");
    if (results.size() != 8)
      continue;
    MyPoint pt;
    pt.angle = stringToNum2<double>(results[0]);
    pt.mapName = results[1];
    pt.name = results[2];
    pt.type = stringToNum2<double>(results[3]);
    pt.selected = stringToNum2<bool>(results[4]);
    pt.x = stringToNum2<double>(results[5]);
    pt.y = stringToNum2<double>(results[6]);
    pt.z = stringToNum2<double>(results[7]);

    //std::cout << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
    vec_points.push_back(pt);

    dashan_msg::pointlist tmp;
    tmp.angle = pt.angle;
    tmp.mapName = pt.mapName.c_str();
    tmp.name = pt.name.c_str();
    tmp.type = pt.type;
    tmp.selected = pt.selected;
    tmp.pose.position.x = pt.x;
    tmp.pose.position.y = pt.y;
    tmp.pose.position.z = pt.z;

    pointgroup.data.push_back(tmp);  
  }
  s_pub_pointgroup_cmd.publish(pointgroup);
  fin.close();
*/
      /* std::ifstream fin("/home/lx/data/points.csv");        
      std::fstream data_file;
      data_file.open("/home/lx/data/points.csv", std::ios::app);  
      std::vector<MyPoint> vec_points;
       
          for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
             {
                  
                 dashan_msg::pointlist data;
                            

                 //MyPoint look_pt;
                 data.angle = it->yaw_world ;
                 data.gridX = it->gridX ;
                 data.gridY = it->gridY ;
                 data.id = it->id ;
                 data.mapName = it->mapName ;
                 data.name = it->name ;
                 position.x = it->x_world ;
                 position.y = it->y_world ;
                 position.z = it->z_world ;

                pointgroup_pub.data.push_back(data);
                  
             }
           s_pub_pointgroup_cmd.publish(pointgroup_pub);
         data_file.close();  */


    }
    
    


}

std::string current_map;

struct hand_draw
{

  string map_name;
  string name;
  geometry_msgs::PoseArray path_lists;
 
};



struct mix_path_list
{

  string name;
  string map_name;
  string map_id;
  bool loop;
  int loop_count;
  string task_name;
  bp_std_msgs::TaskData tasks_lists;
 
};


void mapCallback(const dashan_msg::map :: ConstPtr &msg)
{
    
   //string s_robotMode = msg->mode;
    const char *cstr_msg = msg->mode.c_str();

    if (msg->mode == "map_png")
    {
        const char *cstr_msg = msg->map_name.c_str();
        //string tmp("");
         std::string tem = "/home/lx/map/" + msg->map_name + ".yaml";
        std::string fname(tem.c_str());
        nav_msgs::OccupancyGrid nav_map;
        readMap(&nav_map, fname);
        pub_cur_map_1.publish(nav_map);

    }
    if (msg->mode == "delete_map")
    {
        const char *cstr_msg = msg->map_name.c_str();
        std::string rmyaml = "/home/lx/map/"+ msg->map_name + ".yaml";
        std::string rmpgm = "/home/lx/map/"+ msg->map_name + ".pgm";
        remove(rmyaml.c_str());
        remove(rmpgm.c_str());
        
         
    }
    if (msg->mode == "change_use_map" && !msg->map_name.empty())
    {
       //  const char *cstr_msg = msg->map_name.c_str(); 

     // if(current_map == msg->map_name)
     //	{
     //		return;
     //	}
	


        current_map = msg->map_name;
        
         nav_msgs::OccupancyGrid nav_map;
         std::string fname("/home/lx/map/" + msg->map_name + ".yaml");
         readMap(&nav_map, fname);
	 if(nav_map.info.height > 0 && nav_map.info.width> 0 && nav_map.data.size() > 0)
		{
 			pub_cur_map.publish(nav_map);
		}
        


   dashan_msg::system_status tem;
   tem.current_map_name = current_map;
   //tem.mapInfo.gridHeight = msg->info.height;
   //tem.mapInfo.gridWidth = msg->info.width;
   //tem.mapInfo.originX = msg->info.origin.position.x;
   //tem.mapInfo.originY = msg->info.origin.position.y;
   //tem.mapInfo.resolution = msg->info.resolution;
   
   s_pub_current_map_cmd.publish(tem);


    }

    if (msg->mode == "rename_map")
    {

         const char *cstr_msg = msg->origin_map_name.c_str();
         const char *astr_msg = msg->new_map_name.c_str();
          
 
  std::string msg = astr_msg;
  std::string msg1 = cstr_msg;
  std::string newfile = "/home/lx/map/"+msg+".yaml";
  std::string originfile = "/home/lx/map/"+ msg1 +".yaml";
    if (0 == rename(originfile.c_str(), newfile.c_str()))
    {
      printf("rename %s\n", newfile.c_str());
    }
    else
    {
      perror("rename");
    }
  

  std::string newfile_1 = "/home/lx/map/"+msg+".pgm";
  std::string originfile_1 = "/home/lx/map/"+ msg1 +".pgm";

    if (0 == rename(originfile_1.c_str(), newfile_1.c_str()))
    {
      printf("rename %s\n", newfile.c_str());
    }
    else
    {
      perror("rename");
    }


    std::string path1 = "/home/lx/map/"+ msg +".yaml";
         std::string image;
        double resolution, origin_x, origin_y, origin_z, occupied_thresh, free_thresh;
        int negate;
    getMapFileInfo(path1.c_str(), image, resolution, origin_x, origin_y, origin_z, negate, occupied_thresh, free_thresh);
    
    std::fstream data_file;
    data_file.open(path1.c_str(), std::ios::out);
    
    
     
     char buf1[32] = {0},buf2[32]={0},buf3[32]={0},buf4[32]={0};
     sprintf(buf1,"%f",resolution);

     sprintf(buf2,"%f",origin_x);
     sprintf(buf3,"%f",origin_y);
     sprintf(buf4,"%f",origin_z);
     
      
     
     image= msg +".pgm";

    std::string line;

    line=line+"image: "+ image +"\n"+"resolution: " + buf1 +"\n"+"origin: "+"["+ buf2 +"," + buf3 + "," + buf4 + "]"+"\n"+"negate: "+"0"+"\n"+"occupied_thresh: "+"0.65"+"\n"+"free_thresh: "+"0.196"+"\n" ;

     data_file<<line;
    
     data_file.close();

    std::string rm_originfile = "rm /home/lx/map/"+ msg1 +".yaml";
    system(rm_originfile.c_str());
        

    }

    if (msg->mode == "save_map")
    {
      
        const char *cstr_msg = msg->map_name.c_str();
        std::string map_name = msg->map_name.c_str();
        

        system("bash -c 'source /opt/ros/kinetic/setup.bash;rosservice call finish_trajectory 0 '");

       //std::string cmd= "bash -c 'source /opt/ros/kinetic/setup.bash;rosservice call /write_state ${HOME}/map/" + map_name + ".pbstream'&";
        
       //std::string tem="bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/lx/map/" + map_name + " -pbstream_filename=/home/lx/map/" + map_name +".pbstream -resolution=0.05'&";
       std::string cmd= "bash -c 'source /opt/ros/kinetic/setup.bash;rosservice call /write_state /home/lx/data/" + map_name + ".pbstream '";
        
       std::string tem="bash -c 'source /opt/ros/kinetic/setup.bash;rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/lx/map/" + map_name + " -pbstream_filename=/home/lx/data/" + map_name +".pbstream -resolution=0.05 '";
       


          sleep(1);
       
        //std::string cmd="rosservice call /write_state ${HOME}/map/" + map_name + ".pbstream";
        system(cmd.c_str());
        sleep(1);
        //system("bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;cmd.c_str()'");
        //std::string tem="rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/lx/map/" + map_name + " -pbstream_filename=/home/lx/map/" + map_name +".pbstream -resolution=0.05"; 
        system(tem.c_str());
        //system("bash -c 'source ~/cartographer_ws/install_isolated/setup.bash;tem.c_str()'");


        //sleep(3);
        system("/home/lx/cancel_scan_map.sh");

    }

    if(msg->mode == "hand_draw_list")
      {
           const char *cstr_msg = msg->map_name.c_str();
           dashan_msg::hand_draw_list hand_draw_pub;

 
  std::vector<MyPoint> vec_points;
  std::ifstream fin("/home/lx/data/waypoint3.dat");
  if (fin.is_open() == false)
  {
    std::cout << " Read  File Failed!!!" << std::endl;
    return;
  }
  std::string line;
          
  while (std::getline(fin, line))
  {
    std::vector<std::string> results;

    results = splitString2(line, "  ");
    
    std::string map_name = results[0];
    std::string name = results[1];
    //int path_id = stringToNum<int>(results[2]);
    int waypoint_counts = stringToNum<int>(results[2]);
        if (results.size() - 3 != waypoint_counts * 3)
        {
            std::cout << "readWayPoints results.size() -2 != waypoint_counts * 3!!!" << std::endl;
            
        }
 
       
     if(map_name == msg->map_name)
      {
          dashan_msg::hand_draw_save tep;
          tep.map_name = map_name.c_str();
          tep.name = name.c_str();
                   geometry_msgs::PoseArray waypoints;
                  for (int i = 0; i < waypoint_counts; i++)
                   {
                     double x = stringToNum<double>(results[i * 3 + 3 + 0]);
                     double y = stringToNum<double>(results[i * 3 + 3 + 1]);
                     double yaw = stringToNum<double>(results[i * 3 + 3 + 2]);
                     geometry_msgs::Pose tmp;
                     tmp.position.x = x;
                     tmp.position.y = y;
                     tmp.position.z = 0.0;
                     tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);
                     waypoints.poses.push_back(tmp);
            
                     //hand_draw_pub.data.push_back(tmp);  
             
                    }
           
          tep.path = waypoints;
          hand_draw_pub.data.push_back(tep); 
         
       // std::cout << std::endl;
      //path_lists[path_id] = waypoints;

     }//if(map_name == msg->map_name)

     

   } //while (std::getline(fin, line))

           s_pub_hand_draw_list_cmd.publish(hand_draw_pub);
      }

   
     if(msg->mode == "delete_hand_draw")
        {

    std::ifstream fin("/home/lx/data/waypoint3.dat"); 
    if (fin.is_open() == false)
    {
        std::cout << "readWayPoints Read  File Failed!!!" << std::endl;
    }

   std::vector<hand_draw> handdraws; 
   hand_draw tem;

   std::vector<geometry_msgs::PoseArray> path_lists;
   std::string map_name1=msg->map_name.c_str();
   std::string origin_name=msg->origin_map_name.c_str();

    std::string line;

    while (std::getline(fin, line))

    {
        std::vector<std::string> results;

        results = splitString(line, "  "); 

        std::string map_name = results[0];
        std::string name = results[1];

     if(map_name1 == map_name && origin_name == name)
       {
         continue;
       }


        int waypoint_counts = stringToNum<int>(results[2]);
    
        if (results.size() - 3 != waypoint_counts * 3)
        {
            std::cout << "readWayPoints results.size() -2 != waypoint_counts * 3!!!" << std::endl;
          
        }

        geometry_msgs::PoseArray waypoints;
        for (int i = 0; i < waypoint_counts; i++)
        {
            double x = stringToNum<double>(results[i * 3 + 3 + 0]);
            double y = stringToNum<double>(results[i * 3 + 3 + 1]);
            double yaw = stringToNum<double>(results[i * 3 + 3 + 2]);

            geometry_msgs::Pose tmp;
            tmp.position.x = x;
            tmp.position.y = y;
            tmp.position.z = 0.0;
            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);
            waypoints.poses.push_back(tmp);
          //  std::cout << x << "  " << y << "  " << yaw << "  ";
        //} //for (int i = 0; i < waypoint_counts; i++)

            //tem.path_lists.poses.push_back(tmp);


       } //for (int i = 0; i < waypoint_counts; i++)


       tem.map_name = map_name;     
       tem.name = name;    
       tem.path_lists = waypoints;         
       handdraws.push_back(tem);
       
 

    }  // while (std::getline(fin, line))

    fin.close();
 

  
   /*    for (std::vector<hand_draw>::iterator it = handdraws.begin(); it != handdraws.end(); it++)
       {
          if (it->map_name == map_name1 && it->name == origin_name)
            {
                 it = handdraws.erase(it);
             }
       }    */


   if(handdraws.size() == 0)
     {
        return;
     }
   else
     {
       std::fstream data_file;
       data_file.open("/home/lx/data/waypoint3.dat", std::ios::out);
    
       if (!data_file.is_open())
         {
            std::cout << "can not open data file! writeWayPoints fail" << std::endl;
          
        }
        for (std::vector<hand_draw>::iterator it = handdraws.begin(); it != handdraws.end(); it++)
        {
                
         
                        data_file<<it->map_name<<"  ";
                        data_file<<it->name<<"  ";

                        data_file<<it->path_lists.poses.size()<<"  ";
                        for (int k = 0; k < it->path_lists.poses.size(); k++)
                          {
                             double x,y,yaw;
                             x = it->path_lists.poses[k].position.x;
                             y = it->path_lists.poses[k].position.y;
                             yaw = tf::getYaw(it->path_lists.poses[k].orientation);
                             data_file<<x<<"  "<<y<<"  "<<yaw<<"  ";
                          }
                        data_file<<std::endl; 


         }    // for (std::vector<hand_draw>::iterator it = handdraws

         data_file.close();
         
     } //else
              
             
         }

   
     if(msg->mode == "rename_hand_draw")
        {
              

    std::ifstream fin("/home/lx/data/waypoint3.dat"); 

    if (fin.is_open() == false)
    {
        std::cout << "readWayPoints Read  File Failed!!!" << std::endl;
      
    }

   std::vector<hand_draw> handdraws; 
   hand_draw tem;

   std::vector<geometry_msgs::PoseArray> path_lists;
   std::string map_name1=msg->map_name.c_str();
   std::string origin_name=msg->origin_map_name.c_str();
   std::string new_name=msg->new_map_name.c_str();   

   cout << new_name << "  ";

    std::string line;
    while (std::getline(fin, line))
    {
        std::vector<std::string> results;

        results = splitString(line, "  ");

    //std::vector<hand_draw> handdraws;  

        std::string map_name = results[0];
        std::string name = results[1];
          
        if(map_name == map_name1 && name == origin_name)
         {
              name = new_name;
         }

        int waypoint_counts = stringToNum<int>(results[2]);
    
        if (results.size() - 3 != waypoint_counts * 3)
        {
            std::cout << "readWayPoints results.size() -2 != waypoint_counts * 3!!!" << std::endl;
          
        }

        geometry_msgs::PoseArray waypoints;
        for (int i = 0; i < waypoint_counts; i++)
        {
            double x = stringToNum<double>(results[i * 3 + 3 + 0]);
            double y = stringToNum<double>(results[i * 3 + 3 + 1]);
            double yaw = stringToNum<double>(results[i * 3 + 3 + 2]);
            geometry_msgs::Pose tmp;
            tmp.position.x = x;
            tmp.position.y = y;
            tmp.position.z = 0.0;
            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);
            waypoints.poses.push_back(tmp);
          //  std::cout << x << "  " << y << "  " << yaw << "  ";
        //} //for (int i = 0; i < waypoint_counts; i++)

            //tem.path_lists.poses.push_back(tmp);


       } //for (int i = 0; i < waypoint_counts; i++)
         
       tem.map_name = map_name;     
       tem.name = name;
       tem.path_lists = waypoints;
       handdraws.push_back(tem);

 
    }  // while (std::getline(fin, line))

    fin.close();
   
   
       
   /*    modify_pt.map_name = msg->map_name;
       modify_pt.name = msg->origin_map_name;
  
       for (std::vector<hand_draw>::iterator it = handdraws.begin(); it != handdraws.end(); it++)
       {

          if (it->map_name == modify_pt.map_name && it->name == modify_pt.name)

            {

                 it->name = msg->new_map_name;

             }

       }  */

       std::fstream data_file;
       data_file.open("/home/lx/data/waypoint3.dat", std::ios::out); 
       if (!data_file.is_open())
         {
            std::cout << "can not open data file! writeWayPoints fail" << std::endl;
          
        }
        for (std::vector<hand_draw>::iterator it = handdraws.begin(); it != handdraws.end(); it++)
        {
                           
 
              //for (int i = 0; i < path_lists.size(); i++)
                //   {
                        data_file<<it->map_name<<"  ";
                        data_file<<it->name<<"  ";
                        
                        data_file<<it->path_lists.poses.size()<<"  ";
                        for (int k = 0; k < it->path_lists.poses.size(); k++)
                          {
                             double x,y,yaw;
                             x = it->path_lists.poses[k].position.x;
                             y = it->path_lists.poses[k].position.y;
                             yaw = tf::getYaw(it->path_lists.poses[k].orientation);

                             data_file<<x<<"  "<<y<<"  "<<yaw<<"  ";
                          }
                    // }     
                           
                      data_file<<std::endl; 
         }    // for (std::vector<hand_draw>::iterator it = handdraws

      
         data_file.close();   
         
 


                  
             
       }  //if(msg->mode == "rename_hand_draw")

/*      if(msg->mode == "mix_path_list")
        {
        
  //std::vector<mix_path_list> mix_path_lists;


  bp_std_msgs::TaskDataList  mix_path_lists_pub;
  std::ifstream fin("/home/lx/data/mixpath.dat");

  

  if (fin.is_open() == false)
  {
    std::cout << " Read  File Failed!!!" << std::endl;
    return;
  }
  std::string line;

  while (std::getline(fin, line))
  {
    std::vector<std::string> results;

    results = splitString2(line, "  ");
    
    bp_std_msgs::TaskData pt;
    pt.name = results[0];
    pt.map_name = results[1];
    pt.map_id = results[2];
    pt.loop = stringToNum<int>(results[3]);
    pt.loop_count = stringToNum<int>(results[4]);
    pt.task_name = results[5];
             
      if(pt.map_name == msg->map_name)
      {       
                  bp_std_msgs::TaskEntry tmp;                 
                  for (int i = 0; i < pt.loop_count; i++)
                   {

                       tmp.name = results[i * 3 + 6 + 0];
                       tmp.start_param.map_name = results[i * 3 + 6 + 1];
                       tmp.start_param.path_name = results[i * 3 + 6 + 2];
                       tmp.start_param.graph_name = results[i * 3 + 6 + 3];
                       tmp.start_param.graph_path_name = results[i * 3 + 6 + 4];
                       tmp.start_param.graph_path_group_name = results[i * 3 + 6 + 5];
                       tmp.start_param.position_name = results[i * 3 + 6 + 6];

                       tmp.start_param.destination.angle = stringToNum<int>(results[i * 3 + 6 + 7]);
                       tmp.start_param.destination.gridPosition.x = stringToNum<int>(results[i * 3 + 6 + 8]);
                       tmp.start_param.destination.gridPosition.y = stringToNum<int>(results[i * 3 + 6 + 9]);
                       pt.tasks.push_back(tmp);

                    }
               
       


       
        mix_path_lists_pub.data.push_back(pt);
      
       } //if(map_name == msg->map_name)

   }  //while (std::getline(fin, line))
   fin.close();

       s_pub_mix_path_list_cmd.publish(mix_path_lists_pub);
 

        }  //if(msg->mode == "mix_path_list")  
*/

        if(msg->mode == "delete_mix_path")
        {

    std::ifstream fin("/home/lx/data/mixpath.dat"); 
    if (fin.is_open() == false)
    {
        std::cout << "readWayPoints Read  File Failed!!!" << std::endl;
    }

   std::vector<mix_path_list> mix_path_lists;

   mix_path_list tem;

   std::vector<bp_std_msgs::TaskData> tasks_lists;

   std::string map_name1=msg->map_name.c_str();
   std::string origin_name=msg->origin_map_name.c_str();

    std::string line;

    while (std::getline(fin, line))

    {
        std::vector<std::string> results;

        results = splitString(line, "  "); 

        tem.name = results[0];
        tem.map_name = results[1];
        tem.map_id = results[2];
        tem.loop = stringToNum<int>(results[3]);
        tem.loop_count = stringToNum<int>(results[4]);
        tem.task_name = results[5];
     if(map_name1 == tem.map_name && origin_name == tem.name)
       {
         continue;
       }

                  bp_std_msgs::TaskData TaskData;  
                  for (int i = 0; i < tem.loop_count; i++)
                   {
                       bp_std_msgs::TaskEntry tasks_list;
                       tasks_list.name = results[i * 3 + 6 + 0];
                       tasks_list.start_param.map_name = results[i * 3 + 6 + 1];
                       tasks_list.start_param.path_name = results[i * 3 + 6 + 2];
                       tasks_list.start_param.graph_name = results[i * 3 + 6 + 3];
                       tasks_list.start_param.graph_path_name = results[i * 3 + 6 + 4];
                       tasks_list.start_param.graph_path_group_name = results[i * 3 + 6 + 5];
                       tasks_list.start_param.position_name = results[i * 3 + 6 + 6];

                       tasks_list.start_param.destination.angle = stringToNum<int>(results[i * 3 + 6 + 7]);
                       tasks_list.start_param.destination.gridPosition.x = stringToNum<int>(results[i * 3 + 6 + 8]);
                       tasks_list.start_param.destination.gridPosition.y = stringToNum<int>(results[i * 3 + 6 + 9]);
                       
                 
                       TaskData.tasks.push_back(tasks_list);



                       //tem.tasks_lists.push_back(tasks_list);
                    }
 

              
            tem.tasks_lists = TaskData;
       mix_path_lists.push_back(tem);
       
 

     }  // while (std::getline(fin, line))

    fin.close();
     
     if(mix_path_lists.size() == 0)
     {
        return;
     }                      
     else
     {

       std::fstream data_file;
       data_file.open("/home/lx/data/mixpath.dat", std::ios::out);
    
       if (!data_file.is_open())
         {
            std::cout << "can not open mixpath file" << std::endl;
          
        }
        for (std::vector<mix_path_list>::iterator it = mix_path_lists.begin(); it != mix_path_lists.end(); it++)
        {
                
         
       data_file<<it->name<<"  ";
       data_file<<it->map_name<<"  ";        
       data_file<<it->map_id<<"  ";
       data_file<<it->loop<<"  "; 
       data_file<<it->loop_count<<"  ";
       data_file<<it->task_name<<"  "; 



         for (int i = 0; i < it->tasks_lists.tasks.size(); i++)
          {
              data_file<<it->tasks_lists.tasks[i].name<<"  ";
              data_file<<it->tasks_lists.tasks[i].start_param.map_name<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.path_name<<"  ";                         
              data_file<<it->tasks_lists.tasks[i].start_param.graph_name<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.graph_path_name<<"  ";  
              data_file<<it->tasks_lists.tasks[i].start_param.graph_path_group_name<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.position_name<<"  ";  
              data_file<<it->tasks_lists.tasks[i].start_param.destination.angle<<"  ";  
              data_file<<it->tasks_lists.tasks[i].start_param.destination.gridPosition.x<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.destination.gridPosition.y<<"  ";              
   
             
          }  //for (int i = 0; i < it->loop_count; i++)
        data_file<<std::endl;


         }    // for (std::vector<mix_path_list>::iterator

         data_file.close();
  
     }   //else            

        } //if(msg->mode == "delete_mix_path")
       
         if(msg->mode == "rename_mix_path")
        {
                       
    std::ifstream fin("/home/lx/data/mixpath.dat"); 
    if (fin.is_open() == false)
    {
        std::cout << "readWayPoints Read  File Failed!!!" << std::endl;
    }

   std::vector<mix_path_list> mix_path_lists;

   mix_path_list tem;

   std::vector<bp_std_msgs::TaskData> tasks_lists;

   std::string map_name1=msg->map_name.c_str();
   std::string origin_name=msg->origin_map_name.c_str();
   std::string new_name=msg->new_map_name.c_str();   

    std::string line;

    while (std::getline(fin, line))

    {
        std::vector<std::string> results;

        results = splitString(line, "  "); 

        tem.name = results[0];
        tem.map_name = results[1];
        tem.map_id = results[2];
        tem.loop = stringToNum<int>(results[3]);
        tem.loop_count = stringToNum<int>(results[4]);
        tem.task_name = results[5];

        if(tem.map_name == map_name1 && tem.name == origin_name)
         {
              tem.name = new_name;
         }


                  bp_std_msgs::TaskData TaskData;  
                  for (int i = 0; i < tem.loop_count; i++)
                   {
                       bp_std_msgs::TaskEntry tasks_list;
                       tasks_list.name = results[i * 3 + 6 + 0];
                       tasks_list.start_param.map_name = results[i * 3 + 6 + 1];
                       tasks_list.start_param.path_name = results[i * 3 + 6 + 2];
                       tasks_list.start_param.graph_name = results[i * 3 + 6 + 3];
                       tasks_list.start_param.graph_path_name = results[i * 3 + 6 + 4];
                       tasks_list.start_param.graph_path_group_name = results[i * 3 + 6 + 5];
                       tasks_list.start_param.position_name = results[i * 3 + 6 + 6];

                       tasks_list.start_param.destination.angle = stringToNum<int>(results[i * 3 + 6 + 7]);
                       tasks_list.start_param.destination.gridPosition.x = stringToNum<int>(results[i * 3 + 6 + 8]);
                       tasks_list.start_param.destination.gridPosition.y = stringToNum<int>(results[i * 3 + 6 + 9]);
                       
                 
                       TaskData.tasks.push_back(tasks_list);



                       //tem.tasks_lists.push_back(tasks_list);
                    }
 

              
            tem.tasks_lists = TaskData;
       mix_path_lists.push_back(tem);
       
 

     }  // while (std::getline(fin, line))

    fin.close();


     if(mix_path_lists.size() == 0)
     {
        return;
     }                      
     else
     {

       std::fstream data_file;
       data_file.open("/home/lx/data/mixpath.dat", std::ios::out);
    
       if (!data_file.is_open())
         {
            std::cout << "can not open mixpath file" << std::endl;
          
        }
        for (std::vector<mix_path_list>::iterator it = mix_path_lists.begin(); it != mix_path_lists.end(); it++)
        {
                
         
       data_file<<it->name<<"  ";
       data_file<<it->map_name<<"  ";        
       data_file<<it->map_id<<"  ";
       data_file<<it->loop<<"  "; 
       data_file<<it->loop_count<<"  ";
       data_file<<it->task_name<<"  "; 



         for (int i = 0; i < it->tasks_lists.tasks.size(); i++)
          {
              data_file<<it->tasks_lists.tasks[i].name<<"  ";
              data_file<<it->tasks_lists.tasks[i].start_param.map_name<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.path_name<<"  ";                         
              data_file<<it->tasks_lists.tasks[i].start_param.graph_name<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.graph_path_name<<"  ";  
              data_file<<it->tasks_lists.tasks[i].start_param.graph_path_group_name<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.position_name<<"  ";  
              data_file<<it->tasks_lists.tasks[i].start_param.destination.angle<<"  ";  
              data_file<<it->tasks_lists.tasks[i].start_param.destination.gridPosition.x<<"  "; 
              data_file<<it->tasks_lists.tasks[i].start_param.destination.gridPosition.y<<"  ";              
   
             
          }  //for (int i = 0; i < it->loop_count; i++)
        data_file<<std::endl;


         }    // for (std::vector<mix_path_list>::iterator

         data_file.close();
  
     }   //else  


                  

        } //if(msg->mode == "rename_mix_path")

        if(msg->mode == "point_list")
         {
                std::vector<MyPoint> vec_points;
  std::ifstream fin("/home/lx/data/points.csv");
  if (fin.is_open() == false)
  {
    std::cout << " Read  File Failed!!!" << std::endl;
    return;
  }
  std::string line;

  dashan_msg::pointgroup pointgroup;

  while (std::getline(fin, line))
  {
    std::vector<std::string> results;

    results = splitString2(line, "  ");
    if (results.size() != 8)
      continue;
    MyPoint pt;
    pt.angle = stringToNum2<double>(results[0]);
    pt.mapName = results[1];

     if(pt.mapName != msg->map_name)
     
         continue;
       

    pt.name = results[2];
    pt.type = stringToNum2<double>(results[3]);
    pt.selected = stringToNum2<bool>(results[4]);
    pt.x = stringToNum2<double>(results[5]);
    pt.y = stringToNum2<double>(results[6]);
    pt.z = stringToNum2<double>(results[7]);



    //std::cout << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
    vec_points.push_back(pt);

    dashan_msg::pointlist tmp;
    tmp.angle = pt.angle;
    tmp.mapName = pt.mapName.c_str();
    tmp.name = pt.name.c_str();
    tmp.type = pt.type;
    tmp.selected = pt.selected;
    tmp.pose.position.x = pt.x;
    tmp.pose.position.y = pt.y;
    tmp.pose.position.z = pt.z;

    pointgroup.data.push_back(tmp);  
  } //while (std::getline(fin, line))
  s_pub_pointgroup_cmd.publish(pointgroup);
  fin.close();
    
         }  //if(msg->mode == "point_list")
     

}



std::vector<MyPoint> vec_points;

void posititionsCallback(const dashan_msg::posititions :: ConstPtr &msg)
{
    
   //string s_robotMode = msg->mode;
    const char *cstr_msg = msg->mode.c_str();
    if (msg->mode == "add_position")
    {
        
    }
    if (msg->mode == "add_init_point")
    {
       
        std::ifstream fin("/home/lx/data/points.csv");
        
        dashan_msg::worldPosition worldPosition;
        dashan_msg::Point position;

        double yaw;
        geometry_msgs::Quaternion orientation;
        orientation.x = orientation.x;
        orientation.y = orientation.y;
        orientation.z = orientation.z;
        orientation.w = orientation.w;
	//geometry_msgs::Quaternion orientation(x,y,z,w) = msg->worldPosition.orientation;    
        yaw=tf::getYaw(orientation);    
            
        //mat.getEulerYPR(yaw, pitch, roll);  
        
       

      
        MyPoint pt;
        pt.type = 0;
        pt.id = 0;
        pt.x_world = position.x;
        pt.y_world = position.y;
        pt.z_world = position.z;
        pt.yaw_world = yaw;
        std::cout << pt.type << "  " << pt.id << "  " << pt.x_world << "  " << pt.y_world << "  " << pt.z_world << "  " << pt.yaw_world << std::endl;
        vec_points.push_back(pt);
          

    }

    if (msg->mode == "add_navi_position")
    {
       
        
        

    }

}

void addpointCallback(const dashan_msg::pointlist :: ConstPtr &msg)
{
    
    //const char *cstr_msg = msg->mode.c_str();
    //dashan_msg::pointlist data;

 
        //std::ifstream fin("/home/lx/data/points.csv");
        
       std::fstream data_file;

       data_file.open("/home/lx/data/points.csv", std::ios::app);

        dashan_msg::worldPosition pose;
        dashan_msg::Point position;    
             
        MyPoint pt;
        pt.angle = msg->angle;      
        //pt.gridX = msg->gridX;
        //pt.gridY = msg->gridY;
        //pt.id = msg->id;      
        //pt.mapId = msg->mapId;
        pt.mapName = msg->mapName.c_str();
        pt.name = msg->name.c_str();
        pt.type = msg->type;
        pt.selected = msg->selected;
        pt.x = msg->pose.position.x;
        pt.y = msg->pose.position.y;
        pt.z = msg->pose.position.z;
        //pt.x_world = position.x;
        //pt.y_world = position.y;
       // pt.z_world = position.z;
        //pt.yaw_world = data.angle;

        //vec_points.push_back(pt);

       // std::cout << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
        

       data_file << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
         data_file.close();
                 
  

  
    /*if (data.id == 1)  //0:初始点 1:充电点  2.导航点
    {
       
        std::ifstream fin("/home/lx/data/points.csv");
        
       std::fstream data_file;
      // data_file.open(path.c_str(), std::ios::out);
      data_file.open("/home/lx/data/points.csv", std::ios::app);

        dashan_msg::worldPosition pose;
        dashan_msg::Point position;    
             
        MyPoint pt;
        pt.map_id = data.mapId;
        pt.gridX = data.gridX;
        pt.gridY = data.gridY;
        pt.type = data.type;
        pt.id = data.id;
        pt.mapName = data.mapName;
        pt.name = data.name;
        pt.x_world = position.x;
        pt.y_world = position.y;
        pt.z_world = position.z;
        pt.yaw_world = data.angle;

        vec_points.push_back(pt);

        std::cout << pt.map_id << "  " << pt.gridX << "  " << pt.gridY << "  " << pt.type << "  "<< pt.id << "  " << pt.mapName << "  " << pt.name << "  " << pt.x_world << "  " << pt.y_world << "  " << pt.z_world << "  " << pt.yaw_world << std::endl;
        vec_points.push_back(pt);

         data_file.close();
          

    }


    if (data.id == 2)
    {
       
       std::ifstream fin("/home/lx/data/points.csv");
        
       std::fstream data_file;
      // data_file.open(path.c_str(), std::ios::out);
      data_file.open("/home/lx/data/points.csv", std::ios::app);

        dashan_msg::worldPosition pose;
        dashan_msg::Point position;    
             

        MyPoint pt;
        pt.map_id = data.mapId;
        pt.gridX = data.gridX;
        pt.gridY = data.gridY;

        pt.type = data.type;
        pt.id = data.id;
        pt.mapName = data.mapName;
        pt.name = data.name;
        pt.x_world = position.x;
        pt.y_world = position.y;
        pt.z_world = position.z;
        pt.yaw_world = data.angle;

        vec_points.push_back(pt);

        std::cout << pt.map_id << "  " << pt.gridX << "  " << pt.gridY << "  " << pt.type << "  "<< pt.id << "  " << pt.mapName << "  " << pt.name << "  " << pt.x_world << "  " << pt.y_world << "  " << pt.z_world << "  " << pt.yaw_world << std::endl;
        vec_points.push_back(pt);

         data_file.close();  
        

    }    */

}

struct pointprocess
{
  float angle;
  string mapName;
  string name;
  double type;
  bool selected;
  double x;
  double y;
  double z; 
};

void pointprocessCallback(const dashan_msg::pointprocess :: ConstPtr &msg)
{

    const char *cstr_msg = msg->mode.c_str();

    if (msg->mode == "delete_position")
    {
  /*char *home;
  home = getenv("HOME");
  //printf("the home path is %s\n", home);
  std::string path;
  path = std::string(home) + "/data/points.csv";  */



  std::vector<MyPoint> vec_points;
  std::ifstream fin("/home/lx/data/points.csv");
  if (fin.is_open() == false)
  {
    std::cout << " Read  File Failed!!!" << std::endl;
    return;
  }
  std::string line;
  while (std::getline(fin, line))
  {
    std::vector<std::string> results;

    results = splitString2(line, "  ");
    if (results.size() != 8)
      continue;
    MyPoint pt;
    pt.angle = stringToNum2<double>(results[0]);
    pt.mapName = results[1];
    pt.name = results[2];
    pt.type = stringToNum2<double>(results[3]);
    pt.selected = stringToNum2<bool>(results[4]);
    pt.x = stringToNum2<double>(results[5]);
    pt.y = stringToNum2<double>(results[6]);
    pt.z = stringToNum2<double>(results[7]);

    std::cout << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
    vec_points.push_back(pt);
  }
  fin.close();

       MyPoint del_pt;
       del_pt.mapName = msg->map_name;
       del_pt.name =  msg->position_name;

       for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
       {
          if (it->mapName == del_pt.mapName && it->name == del_pt.name)
            {
                 it = vec_points.erase(it);
             }
       }
          
       std::fstream data_file;
       data_file.open("/home/lx/data/points.csv", std::ios::out);
    
       if (!data_file.is_open())
         {
            std::cout << "can not open data file fail" << std::endl;
             return;
        }
        for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
        {
            MyPoint pt;
            pt = *it;
            data_file << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
         }
            data_file.close();
         
        
    }

    if (msg->mode == "rename_position")
    {

  /*char *home;
  home = getenv("HOME");
  //printf("the home path is %s\n", home);
  std::string path;
  path = std::string(home) + "/data/points.csv";  */

  std::vector<MyPoint> vec_points;
  std::ifstream fin("/home/lx/data/points.csv");
  if (fin.is_open() == false)
  {
    std::cout << " Read  File Failed!!!" << std::endl;
    return;
  }
  std::string line;
  while (std::getline(fin, line))
  {
    std::vector<std::string> results;

    results = splitString2(line, "  ");
    if (results.size() != 8)
      continue;
    MyPoint pt;
    pt.angle = stringToNum2<double>(results[0]);
    pt.mapName = results[1];
    pt.name = results[2];
    pt.type = stringToNum2<double>(results[3]);
    pt.selected = stringToNum2<bool>(results[4]);
    pt.x = stringToNum2<double>(results[5]);
    pt.y = stringToNum2<double>(results[6]);
    pt.z = stringToNum2<double>(results[7]);

    std::cout << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
    vec_points.push_back(pt);
  }
fin.close();      

       MyPoint modify_pt;
       modify_pt.mapName = msg->map_name;
       modify_pt.name =  msg->origin_name;
       
    for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
    {
      if (it->mapName == modify_pt.mapName && it->name == modify_pt.name)
      {
        it->name = msg->new_name ;
        break;
      }
    }

       std::fstream data_file;
       data_file.open("/home/lx/data/points.csv", std::ios::out);

    if (!data_file.is_open())
    {
      std::cout << "can not open data file fail" << std::endl;
      return;
    }
    for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
    {
      MyPoint pt;
      pt = *it;
      data_file << pt.angle << "  " << pt.mapName << "  " << pt.name << "  " << pt.type << "  "<< pt.selected << "  " << pt.x << "  " << pt.y << "  " << pt.z << std::endl;
    }
      data_file.close();      
       
   }
    
    if (msg->mode == "init_position_list")
    {

      std::ifstream fin("/home/lx/data/points.csv");        
      std::fstream data_file;
      data_file.open("/home/lx/data/points.csv", std::ios::app);  
      std::vector<MyPoint> vec_points;
       MyPoint select_pt;
       select_pt.mapName = msg->map_name;
       select_pt.id = msg->type;

         for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
             {
               if(it->mapName != select_pt.mapName || it->type != select_pt.type) 
                  {

                        it = vec_points.erase(it);
                             
                   }
             
                      dashan_msg::pointlist data;
                      dashan_msg::worldPosition pose;
                      dashan_msg::Point position;             

                       data.angle = it->yaw_world ;
                       data.gridX = it->gridX ;
                       data.gridY = it->gridY ;
                       data.id = it->id ;
                       data.mapName = it->mapName ;
                        data.name = it->name ;
                       position.x = it->x_world ;
                       position.y = it->y_world ;
                       position.z = it->z_world ;
               
                       pointgroup_pub.data.push_back(data);
                 }   
                  
             
          // s_pub_pointgroup_cmd.publish(pointgroup_pub);       


     }

    if (msg->mode == "charge_position_list")
    {

 
      std::ifstream fin("/home/lx/data/points.csv");        
      std::fstream data_file;
      data_file.open("/home/lx/data/points.csv", std::ios::out);  
      std::vector<MyPoint> vec_points;
       MyPoint select_pt;
       select_pt.mapName = msg->map_name;
       select_pt.id = msg->type;

         for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
             {
               if(it->mapName != select_pt.mapName || it->type != select_pt.type) 
                  {

                        it = vec_points.erase(it);
                             
                   }
             
                      dashan_msg::pointlist data;
                      dashan_msg::worldPosition pose;
                      dashan_msg::Point position;             

                       data.angle = it->yaw_world ;
                       data.gridX = it->gridX ;
                       data.gridY = it->gridY ;
                       data.id = it->id ;
                       data.mapName = it->mapName ;
                        data.name = it->name ;
                       position.x = it->x_world ;
                       position.y = it->y_world ;
                       position.z = it->z_world ;
               
                       pointgroup_pub.data.push_back(data);
                 }   
                  
             
         // s_pub_pointgroup_cmd.publish(pointgroup_pub);       
       


     }
    if (msg->mode == "navi_position_list")
    {

      std::ifstream fin("/home/lx/data/points.csv");        
      std::fstream data_file;
      data_file.open("/home/lx/data/points.csv", std::ios::out);  
      std::vector<MyPoint> vec_points;
       MyPoint select_pt;
       select_pt.mapName = msg->map_name;
       select_pt.id = msg->type;

         for (std::vector<MyPoint>::iterator it = vec_points.begin(); it != vec_points.end(); it++)
             {
               if(it->mapName != select_pt.mapName || it->type != select_pt.type) 
                  {

                        it = vec_points.erase(it);
                             
                   }
             
                      dashan_msg::pointlist data;
                      dashan_msg::worldPosition pose;
                      dashan_msg::Point position;             

                       data.angle = it->yaw_world ;
                       data.gridX = it->gridX ;
                       data.gridY = it->gridY ;
                       data.id = it->id ;
                       data.mapName = it->mapName ;
                        data.name = it->name ;
                       position.x = it->x_world ;
                       position.y = it->y_world ;
                       position.z = it->z_world ;
               
                       pointgroup_pub.data.push_back(data);
                 }   
                  
             
          // s_pub_pointgroup_cmd.publish(pointgroup_pub);       
        


     }
}

ros::Publisher s_pub_alongwall_task_cmd;
ros::Publisher s_pub_bow_task_cmd;
//ros::Publisher key_pub_goal;
ros::Publisher s_pub_point_nav_cmd;
ros::Publisher s_pub_hand_draw_cmd;

void pathprocessCallback(const bp_std_msgs::TaskData :: ConstPtr &msg)
{

         const char *cstr_msg = msg->map_name.c_str();
        
         //nav_msgs::OccupancyGrid nav_map;
         //std::string fname("/home/lx/map/" + msg->map_name + ".yaml");
         //readMap(&nav_map, fname);
         //pub_cur_map.publish(nav_map); 

                        
        /* if(msg->name == "path_along_wall")
         {
              bp_std_msgs::StartAlongWallOrBow AlongWall;

              AlongWall.loop_count = msg->loop_count;
              AlongWall.task_name = msg->task_name;

              //std::string data_file_name = "/home/lx/data/waypoint1.dat";
              
              string map_name = msg->map_name.c_str();
              std::ifstream fin("/home/lx/data/waypoint1.dat");
              std::map<int, geometry_msgs::PoseArray> path_lists;
              if (fin.is_open() == false)
              {
                  std::cout << "readWayPoints Read File Failed!" << std::endl;                
              }
              
              //path_lists.clear();
              std::string line;
              while (std::getline(fin, line))
              {
                  std::vector<std::string> results;

                  results = splitString(line, " ");
                   map_name = results[0];
                  int path_id = stringToNum<int>(results[1]);
      
                  int waypoint_counts = stringToNum<int>(results[2]);
   
                  if (results.size() - 3 != waypoint_counts * 3)
                       {
                           std::cout << "readWayPoints results.size() -2 != waypoint_counts * 3!!!" << std::endl;
                        }

        geometry_msgs::PoseArray waypoints;
        for (int i = 0; i < waypoint_counts; i++)
        {
            double x = stringToNum<double>(results[i * 3 + 3 + 0]);
            double y = stringToNum<double>(results[i * 3 + 3 + 1]);
            double yaw = stringToNum<double>(results[i * 3 + 3 + 2]);
            geometry_msgs::Pose tmp;
            tmp.position.x = x;
            tmp.position.y = y;
            tmp.position.z = 0.0;
            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);
            //waypoints.poses.push_back(tmp);
            
            AlongWall.path.poses.push_back(tmp);
            
            
          
        } //for (int i = 0; i < waypoint_counts; i++)
        //std::cout << std::endl;
      path_lists[path_id] = waypoints;
    }  //while (std::getline(fin, line))
    fin.close();         
              


	      AlongWall.path.header.frame_id = "map";
  	      AlongWall.path.header.stamp = ros::Time::now();
              s_pub_alongwall_task_cmd.publish(AlongWall);

         } //if(msg->name == "path_along_wall")
     */


        if(msg->name == "path_along_wall")
         {
           
              bp_std_msgs::StartAlongWall AlongWall;
              AlongWall.loop_count = msg->loop_count;
              AlongWall.task_name = msg->task_name;


              string map_name = msg->map_name.c_str();

              std::ifstream fin("/home/lx/data/waypoint1.dat");
              //std::map<int, geometry_msgs::PoseArray> path_lists;
              if (fin.is_open() == false)
              {
                  std::cout << "readWayPoints Read File Failed!" << std::endl;                
              }
              
              
              std::string line;

              if (std::getline(fin, line))
	      {
		 std::vector<std::string> results;

                  results = splitString(line, " ");
                   map_name = results[0];
	      }
	      if (std::getline(fin, line))
	      {
		 std::vector<std::string> results;
                  results = splitString(line, " ");
                  // path_count = results[0];
	      }
              //{
                  //std::vector<std::string> results;

                  //results = splitString(line, " ");
                  //  = results[0];
                  //int path_count = stringToNum<int>(results[1]);
                  //int path_id = stringToNum<int>(results[2]);
                  //int waypoint_counts = stringToNum<int>(results[3]);
     while (std::getline(fin, line))   
        {
      
      //for (int j = 0; j < path_count; j++)
       //{

             std::vector<std::string> results;
             results = splitString(line, "  ");
             //int waypoint_counts = 0;
             int path_id = stringToNum<int>(results[0]);
             int waypoint_counts = stringToNum<int>(results[1]);
             
                         
          //ROS_INFO("waypoint_countsasfsfadfdfd: \"%d\"", waypoint_counts);

        geometry_msgs::PoseArray waypoints;
        for (int i = 0; i < waypoint_counts; i++)
         {


            double x = stringToNum<double>(results[i * 3 + 2]);
            double y = stringToNum<double>(results[i * 3 + 3]);
            double yaw = stringToNum<double>(results[i * 3 + 4]);
            geometry_msgs::Pose tmp;
            tmp.position.x = x;
            tmp.position.y = y;
            tmp.position.z = 0.0;
            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);


            //ROS_INFO("waypoint_countsasf: \"%d\"", x);          

            waypoints.poses.push_back(tmp);
            //AlongWall.path[j].poses.push_back(tmp);
            
            
          
         } //for (int i = 0; i < waypoint_counts; i++)
         //std::cout << std::endl;
                  

       
          //AlongWall.path[j].header.frame_id = "map";
          //AlongWall.path[j].header.stamp = ros::Time::now();
          //AlongWall.path[j] = waypoints;
          waypoints.header.frame_id = "map"; 
          waypoints.header.stamp = ros::Time::now();


          AlongWall.path.push_back(waypoints);

       // }  // for (int i = 0; i < path_id; i++)
      //path_lists[path_id] = waypoints;
        

    }  //while (std::getline(fin, line))
    fin.close();         
              

	      //AlongWall.path.header.frame_id = "map";
  	      //AlongWall.path.header.stamp = ros::Time::now();
              s_pub_alongwall_task_cmd.publish(AlongWall);
                                     


         } //if(msg->name == "path_along_wall")   

         if(msg->name == "path_bow")
         {
              bp_std_msgs::StartAlongWallOrBow Bow;

              Bow.loop_count = msg->loop_count;
              Bow.task_name = msg->task_name;
              

              std::map<int, geometry_msgs::PoseArray> path_lists;
              string map_name = msg->map_name.c_str();
              std::ifstream fin("/home/lx/data/waypoint2.dat");
              if (fin.is_open() == false)
              {
                  std::cout << "readWayPoints Read  File Failed!" << std::endl;                
              }
              
              //path_lists.clear();
              std::string line;
              while (std::getline(fin, line))
              {
                  std::vector<std::string> results;

                  results = splitString(line, " ");
                   map_name = results[0];
                  int path_id = stringToNum<int>(results[1]);
      
                  int waypoint_counts = stringToNum<int>(results[2]);
   
                  if (results.size() - 3 != waypoint_counts * 3)
                       {
                           std::cout << "readWayPoints results.size() -2 != waypoint_counts * 3!!!" << std::endl;
                        }

        geometry_msgs::PoseArray waypoints;
        for (int i = 0; i < waypoint_counts; i++)
        {
            double x = stringToNum<double>(results[i * 3 + 3 + 0]);
            double y = stringToNum<double>(results[i * 3 + 3 + 1]);
            double yaw = stringToNum<double>(results[i * 3 + 3 + 2]);
            geometry_msgs::Pose tmp;
            tmp.position.x = x;
            tmp.position.y = y;
            tmp.position.z = 0.0;
            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);
            //waypoints.poses.push_back(tmp);
            
            Bow.path.poses.push_back(tmp);
            
            
          
        }
        std::cout << std::endl;
      path_lists[path_id] = waypoints;
    }

	      Bow.path.header.frame_id = "map";
  	      Bow.path.header.stamp = ros::Time::now();
              s_pub_bow_task_cmd.publish(Bow);
         }

         if(msg->name == "FreeNavi")
         {
              
             //geometry_msgs::PoseStamped goal; 
             
             //goal.header.frame_id = "map";
             //goal.header.stamp    = ros::Time::now();  

             
             //goal.pose.position.x = msg->tasks[0].start_param.destination.gridPosition.x;
             //goal.pose.position.y = msg->tasks[0].start_param.destination.gridPosition.y;
    
             //goal.pose.orientation = tf::createQuaternionMsgFromYaw(msg->tasks[0].start_param.destination.angle);
            

             //key_pub_goal.publish(goal);

             
             bp_std_msgs::StartSimplePointNav goal; 
             
             goal.task_name = msg->name;

             goal.pt.header.frame_id = "map";
             goal.pt.header.stamp = ros::Time::now();  

             
             goal.pt.pose.position.x = msg->tasks[0].start_param.destination.gridPosition.x;
             goal.pt.pose.position.y = msg->tasks[0].start_param.destination.gridPosition.y;
    
             goal.pt.pose.orientation = tf::createQuaternionMsgFromYaw(msg->tasks[0].start_param.destination.angle);

             
             
            s_pub_point_nav_cmd.publish(goal);
         }  

         if(msg->name == "hand_draw")
         {
                                               
           bp_std_msgs::StartAlongWallOrBow handdraw;

           handdraw.loop_count = msg->loop_count;
           handdraw.task_name = msg->task_name;


           std::ifstream fin("/home/lx/data/waypoint3.dat");        
           //std::fstream data_file;
           //data_file.open("/home/lx/data/waypoint3.dat", std::ios::app);
           
           std::map<int,geometry_msgs::PoseArray> path_lists;

           std::string line;

      while (std::getline(fin, line))
      {
        std::vector<std::string> results;

        results = splitString(line, "  ");
        // for (int  i= 0; i< results.size(); i++)
        // {
        //     std::cout<<results[i]<<"  ";
        // }
     
        std::string map_name = results[0];
        std::string name = results[1];

        //int path_id = stringToNum<int>(results[2]);

        if(msg->map_name == map_name && msg->task_name == name )
         {
      //  std::cout << path_id << "  ";
        int waypoint_counts = stringToNum<int>(results[2]);
     //   std::cout << waypoint_counts << "  ";
        if (results.size() - 3 != waypoint_counts * 3)
        {
            std::cout << "readWayPoints results.size() -2 != waypoint_counts * 3!!!" << std::endl;
        
        }

        geometry_msgs::PoseArray waypoints;
        for (int i = 0; i < waypoint_counts; i++)
        {
            double x = stringToNum<double>(results[i * 3 + 3 + 0]);
            double y = stringToNum<double>(results[i * 3 + 3 + 1]);
            double yaw = stringToNum<double>(results[i * 3 + 3 + 2]);
            geometry_msgs::Pose tmp;
            tmp.position.x = x;
            tmp.position.y = y;
            tmp.position.z = 0.0;
            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw);
            waypoints.poses.push_back(tmp);
            
            handdraw.path.poses.push_back(tmp);

          //  std::cout << x << "  " << y << "  " << yaw << "  ";
        }
       // std::cout << std::endl;
      //path_lists[path_id] = waypoints;

     }  //if(msg->map_name == map_name && msg->task_name == name )


    }

    // for (std::map<int,geometry_msgs::PoseArray>::iterator it =path_lists.begin(); it!=path_lists.end(); it++)
    // {
    //     std::cout<<it->first<<"  ";
        
    //     for (int i = 0; i < it->second.poses.size(); i++)
    //     {
    //         double x,y;
    //         x = it->second.poses[i].position.x;
    //         y = it->second.poses[i].position.y;
    //          std::cout<<x<<"  "<<y<<"  ";
    //     }
    //     std::cout<<std::endl;
    // }
    
    fin.close();





	   handdraw.path.header.frame_id = "map";
  	   handdraw.path.header.stamp = ros::Time::now();
           s_pub_hand_draw_cmd.publish(handdraw);
         }

          
}

struct path_along_wall
{

  string name_map;
  geometry_msgs::PoseArray path_along_wall;
 
}; 

void pathpointsaveCallback(const bp_std_msgs::AlongWallBow :: ConstPtr &msg)
{
       
     
       std::string map_name = msg->name_map.c_str();

       //bp_std_msgs::AlongWallBow path_along_wall;
  
       //std::vector<geometry_msgs::PoseArray> waypoints_lists_1;
   
        //waypoints_lists_1.push_back(msg->path_along_wall);

    {   
       std::vector<geometry_msgs::PoseArray> AlongWalls;

       AlongWalls = msg->path_along_wall;

       std::fstream data_file;
       data_file.open("/home/lx/data/waypoint1.dat", std::ios::out);
       if (!data_file.is_open())
        {
           std::cout<<"can not open data waypoint1 file!"<<std::endl;
           //return false;
         }

     data_file<<map_name<<endl; 
     data_file<<AlongWalls.size()<<endl;
       
   for (int i = 0; i < AlongWalls.size(); i++)
   {
     data_file<<i+1<<"  ";
     
     data_file<<AlongWalls[i].poses.size()<<"  ";
     for (int k = 0; k < AlongWalls[i].poses.size(); k++)
     {
        double x,y,yaw;
        x = AlongWalls[i].poses[k].position.x;
        y = AlongWalls[i].poses[k].position.y;
        yaw = tf::getYaw(AlongWalls[i].poses[k].orientation);
        data_file<<x<<"  "<<y<<"  "<<yaw<<"  ";
     } //for (int k = 0; k < path_lists[i].poses.size(); k++)
              
      data_file<<endl;   
       //data_file<<std::endl;
    } //for (int i = 0; i < path_lists.size(); i++)
        data_file.close();
 
      //ROS_INFO("124324333333333");                 
     }

    
  

      // if (!writeWayPoints("/home/lx/data/waypoint1.dat",map_name, waypoints_lists_1))
       //{
       //     ROS_ERROR(" save path fail");
       //}
       //else
       //{
       //     ROS_INFO(" save path success ");
       //} 

    /* {
       bp_std_msgs::AlongWallBow path_bow;
       std::vector<geometry_msgs::PoseArray> waypoints_lists_2;
       waypoints_lists_2.push_back(msg->path_bow);
       if (!writeWayPoints("/home/lx/data/waypoint2.dat",map_name, waypoints_lists_2))
       {
            ROS_ERROR(" save path fail !");
       }
       else
       {
            ROS_INFO(" save path success ");
       }   

     }  */


      {

  bp_std_msgs::AlongWallBow path_bow;
  std::vector<geometry_msgs::PoseArray> waypoints_lists_2;
  waypoints_lists_2.push_back(msg->path_bow);
 if (waypoints_lists_2.size() == 0)
 {
     ROS_ERROR("waypoints_lists_2.size() == 0");
     //return false;
 }
 

  std::fstream data_file;
  data_file.open("/home/lx/data/waypoint2.dat", std::ios::out);
  if (!data_file.is_open())
  {
    std::cout<<"can not open waypoint2 file"<<std::endl;
    //return false;
  }
  for (int i = 0; i < waypoints_lists_2.size(); i++)
  {
     data_file<<map_name<<"  ";
     data_file<<i+1<<"  ";
     data_file<<waypoints_lists_2[i].poses.size()<<"  ";
     for (int k = 0; k < waypoints_lists_2[i].poses.size(); k++)
     {
        double x,y,yaw;
        x = waypoints_lists_2[i].poses[k].position.x;
        y = waypoints_lists_2[i].poses[k].position.y;
        yaw = tf::getYaw(waypoints_lists_2[i].poses[k].orientation);
        data_file<<x<<"  "<<y<<"  "<<yaw<<"  ";
     } //for (int k = 0; k < path_lists[i].poses.size(); k++)
     //data_file<<std::endl;
  } // for (int i = 0; i < path_lists.size(); i++)

  data_file.close();  
      //ROS_INFO("12432411111111111");                   
      }
      
    
    
  
}

void diagnoseCallback(const std_msgs::UInt32 :: ConstPtr &msg)
{
      std_msgs::UInt32 tep;
      tep.data=0x200000/2;
     s_pub_diagnose_cmd.publish(tep);     
}


void handdrawsaveCallback(const dashan_msg::hand_draw_save :: ConstPtr &msg)
{

      std::ifstream fin("/home/lx/data/waypoint3.dat");        
      std::fstream data_file;
      
      std::string map_name = msg->map_name; 
      std::string name = msg->name;
  
      dashan_msg::hand_draw_save path;
      std::vector<geometry_msgs::PoseArray> path_lists;
      path_lists.push_back(msg->path);

      data_file.open("/home/lx/data/waypoint3.dat", std::ios::app);
        if (!data_file.is_open())
         {
           std::cout<<"can not open data file! writeWayPoints fail"<<std::endl;
          
         } 
     // std::vector<geometry_msgs::PoseArray> path_lists;
        for (int i = 0; i < path_lists.size(); i++)
     {
     data_file<<map_name<<"  ";
     data_file<<name<<"  ";
     //data_file<<i+1<<"  ";
     data_file<<path_lists[i].poses.size()<<"  ";
     for (int k = 0; k < path_lists[i].poses.size(); k++)
     {
        double x,y,yaw;
        x = path_lists[i].poses[k].position.x;
        y = path_lists[i].poses[k].position.y;
        yaw = tf::getYaw(path_lists[i].poses[k].orientation);
        data_file<<x<<"  "<<y<<"  "<<yaw<<"  ";
     }
    data_file<<std::endl;
     }
  data_file.close();

 
}


void canGenLineOnMapCallback(const dashan_msg::canGenLineOnMap :: ConstPtr &msg)
{
     dashan_msg::Mode tep;
     tep.mode=(canGenLineOnMap(msg->x1, msg->y1, msg->x2, msg->y2, msg->map_name)?"true":"false");
    

     
     s_pub_canGenLineOnMapStatus_cmd.publish(tep);
}


void mixpathCallback(const bp_std_msgs::TaskData :: ConstPtr &msg)
{
      
       //std::vector<mixpath> mixpaths;
       //mixpath = tem;
       
       //std::vector<bp_std_msgs::TaskData> mixpaths;
       //bp_std_msgs/TaskEntry[] tasks

       std::vector<bp_std_msgs::TaskEntry> mix_paths;
       mix_paths =msg->tasks;
       

       std::fstream data_file;
       data_file.open("/home/lx/data/mixpath.dat", std::ios::app);
       if (!data_file.is_open())
        {
           std::cout<<"can not open data mixpaths file!"<<std::endl;
          // return false;
         }
       data_file<<msg->name<<"  ";
       data_file<<msg->map_name<<"  ";        
       data_file<<msg->map_id<<"  ";
       data_file<<msg->loop<<"  "; 
       data_file<<msg->loop_count<<"  ";
       data_file<<msg->task_name<<"  "; 

       for (int i = 0; i < mix_paths.size(); i++)
         {
              data_file<<mix_paths[i].name<<"  ";
              data_file<<mix_paths[i].start_param.map_name<<"  "; 
              data_file<<mix_paths[i].start_param.path_name<<"  ";                         
              data_file<<mix_paths[i].start_param.graph_name<<"  "; 
              data_file<<mix_paths[i].start_param.graph_path_name<<"  ";  
              data_file<<mix_paths[i].start_param.graph_path_group_name<<"  "; 
              data_file<<mix_paths[i].start_param.position_name<<"  ";  
              data_file<<mix_paths[i].start_param.destination.angle<<"  ";  
              data_file<<mix_paths[i].start_param.destination.gridPosition.x<<"  "; 
              data_file<<mix_paths[i].start_param.destination.gridPosition.y<<"  "; 
   
              
          }    
        data_file<<std::endl;

        data_file.close();
                  

}


/*void currentmapCallback(const nav_msgs::OccupancyGrid :: ConstPtr &msg)
{
   dashan_msg::system_status tem;
   tem.current_map_name = current_map;
   tem.mapInfo.gridHeight = msg->info.height;
   tem.mapInfo.gridWidth = msg->info.width;
   tem.mapInfo.originX = msg->info.origin.position.x;
   tem.mapInfo.originY = msg->info.origin.position.y;
   tem.mapInfo.resolution = msg->info.resolution;
   
   s_pub_current_map_cmd.publish(tem);
}
*/


int main(int argc, char** argv)
{
    int iRet;
     //ROS_INFO("dashan_mode node loaded!\n");

    ros::init(argc, argv, "dashan_mode");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10); 
    ros::Subscriber sub_amcl_pose = nh.subscribe("amcl_pose", 1, amclPoseCallBack);
    ros::Subscriber sub_robot_mode_cmd = nh.subscribe("/dashan_mode", 10, modeCallback);
    ros::Subscriber sub_robot_map_cmd = nh.subscribe("/dashan_map", 1, mapCallback);
    ros::Subscriber sub_robot_posititions_cmd = nh.subscribe("/dashan_posititions", 1, posititionsCallback);  
    ros::Subscriber sub_robot_addpoint_cmd = nh.subscribe("/dashan_addpoint", 1, addpointCallback); 
    ros::Subscriber sub_robot_pointprocess_cmd = nh.subscribe("/dashan_pointprocess", 1, pointprocessCallback); 
    ros::Subscriber sub_robot_pathprocess_cmd = nh.subscribe("/dashan_pathprocess", 1, pathprocessCallback);
    ros::Subscriber sub_robot_path_point_save_cmd = nh.subscribe("/bp_nav_to_android_alongwallbow", 1, pathpointsaveCallback);
    ros::Subscriber sub_robot_hand_draw_save_cmd = nh.subscribe("/hand_draw_save", 1, handdrawsaveCallback);

    ros::Subscriber sub_diagnose_cmd = nh.subscribe("/bp_node_ask", 1, diagnoseCallback);

    ros::Subscriber sub_canGenLineOnMap_cmd = nh.subscribe("/canGenLineOnMap", 1, canGenLineOnMapCallback);

    ros::Subscriber sub_mixpath_cmd = nh.subscribe("/dashan_mixpath", 1, mixpathCallback); 

    //ros::Subscriber sub_currentmap_cmd = nh.subscribe("/map", 1, currentmapCallback);


    s_pub_mode_cmd = nh.advertise<dashan_msg::Mode>("/dashan_mode", 1000);
    s_pub_MapLists_cmd = nh.advertise<dashan_msg::MapLists>("/dashan_MapLists", 1);
    s_pub_pointgroup_cmd = nh.advertise<dashan_msg::pointgroup>("/dashan_pointgroup", 1000);
    
    s_pub_diagnose_cmd = nh.advertise<std_msgs::UInt32>("/bp_node_feedback", 1000);

    pub_cur_map_1 = nh.advertise<nav_msgs::OccupancyGrid>("map1", 1, true);
    pub_cur_map = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    s_pub_alongwall_task_cmd = nh.advertise<bp_std_msgs::StartAlongWall>("/start_alongwall_task", 1);
   
    s_pub_bow_task_cmd = nh.advertise<bp_std_msgs::StartAlongWallOrBow>("/start_bow_task", 1);
    //key_pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",100);
    s_pub_point_nav_cmd = nh.advertise<bp_std_msgs::StartSimplePointNav>("/simple_pose_nav", 1);
    s_pub_hand_draw_cmd = nh.advertise<bp_std_msgs::StartAlongWallOrBow>("/start_handle_draw_task", 1);
    s_pub_canGenLineOnMapStatus_cmd = nh.advertise<dashan_msg::Mode>("/canGenLineOnMapStatus", 1);
    s_pub_hand_draw_list_cmd = nh.advertise<dashan_msg::hand_draw_list>("/hand_draw_list", 1000);
    s_pub_mix_path_list_cmd = nh.advertise<bp_std_msgs::TaskDataList>("/mix_path_list", 1000);

    s_pub_current_map_cmd = nh.advertise<dashan_msg::system_status>("/current_map", 1000);

    ros::spin();

   // while(1) 
    //{
	//    ros::spinOnce();
	  //  loop_rate.sleep();
    
        

   //}



}




