#include "bp_nomotion_location_node/common.h"

#include <fstream>
//#include <boost/algorithm/string.hpp>

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
        std::cout << "readWayPoints Read  File Failed!!!" << std::endl;
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
