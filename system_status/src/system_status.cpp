


#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include <nav_msgs/OccupancyGrid.h>
#include <dashan_msg/system_status.h>
#include <dashan_msg/Mode.h>
#include <dashan_msg/map.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <bp_std_msgs/TaskData.h>
#include <bp_std_msgs/TaskEntry.h>
#include <bp_std_msgs/TaskOption.h>



ros::Publisher s_pub_system_status_cmd;

std::string map_name; 
float gridHeight = 0.0;
float gridWidth = 0.0;
float originX = 0.0;
float originY = 0.0;
float resolution = 0.0;

std::string status; 

std::string name;

std::string map_id;
int loop_count;
std::string task_name;

std::vector<bp_std_msgs::TaskEntry> tasks;

int task_status; 


void systemState(void);


void modeCallback(const dashan_msg::Mode :: ConstPtr &msg)
{
    const char *cstr_msg = msg->mode.c_str();

    if (msg->mode == "system_status")
    {  
         systemState();




    }

}


void mapnameCallback(const dashan_msg::map :: ConstPtr &msg)
{
      const char *cstr_msg = msg->mode.c_str();
      if (msg->mode == "change_use_map" && !msg->map_name.empty())
      {
          map_name = msg->map_name;
      }
}

void mapinfoCallback(const nav_msgs::OccupancyGrid :: ConstPtr &msg)
{
   gridHeight = msg->info.height;
   gridWidth = msg->info.width;
   originX = msg->info.origin.position.x;
   originY = msg->info.origin.position.y;
   resolution = msg->info.resolution;
       

   ROS_INFO("abcdef %f  what %f",gridHeight,gridWidth); 

}


void scanmapCallback(const visualization_msgs::MarkerArray :: ConstPtr &msg)
{
      //status = "SCAN_MAP";   
}

void movestatusCallback(const actionlib_msgs::GoalStatusArray :: ConstPtr &msg)
{
       
}



void pathprocessCallback(const bp_std_msgs::TaskData :: ConstPtr &msg)
{
       status == "RUNNING_TASK";
       map_name = msg->map_name;
       name = msg->name;
       map_id = msg->map_id;
       loop_count = msg->loop_count;
       task_name = msg->task_name;       
       tasks = msg->tasks;

}


void taskstatusCallback(const bp_std_msgs::TaskOption :: ConstPtr &msg)
{
       task_status = msg->id;
}


void systemState(void)
{
     

     ROS_INFO("123 %s",status.c_str()); 

     dashan_msg::system_status tem;

     tem.current_map_name = map_name;
     tem.mapInfo.gridHeight = gridHeight;
     tem.mapInfo.gridWidth = gridWidth;
     tem.mapInfo.originX = originX;
     tem.mapInfo.originY = originY;
     tem.mapInfo.resolution = resolution;

     tem.work_status = status;

     if(status == "SCAN_MAP")
       {
          s_pub_system_status_cmd.publish(tem);
       }
     if(status == "RUNNING_TASK")
       {
          tem.task_data.name = name;
          tem.task_data.map_name = map_name;
          tem.task_data.map_id = map_id;          
          tem.task_data.loop_count = loop_count;
          tem.task_data.task_name = task_name;
          tem.task_data.tasks = tasks;

          tem.task_status = task_status;
          s_pub_system_status_cmd.publish(tem);
       }


        
} 





int main(int argc, char** argv)
{

     int iRet;

     ros::init(argc, argv, "system_status");
     ros::NodeHandle nh;
     ros::Rate loop_rate(10); 
    

     ros::Subscriber sub_mapinfo_cmd = nh.subscribe("/map", 1, mapinfoCallback);
     ros::Subscriber sub_scanmap_cmd = nh.subscribe("/constraint_list", 1, scanmapCallback);
     ros::Subscriber sub_robot_mode_cmd = nh.subscribe("/dashan_mode", 1, modeCallback);
     ros::Subscriber sub_mapname_cmd = nh.subscribe("/dashan_map", 1, mapnameCallback);

    
     ros::Subscriber sub_movestatus_cmd = nh.subscribe("/move_base/status", 1, movestatusCallback);

      ros::Subscriber sub_robot_pathprocess_cmd = nh.subscribe("/dashan_pathprocess", 1, pathprocessCallback);  
      ros::Subscriber sub_taskstatus_cmd = nh.subscribe("/bp_task_feedback", 1, taskstatusCallback);
   

     s_pub_system_status_cmd = nh.advertise<dashan_msg::system_status>("/system_status", 1);



       

       

     ros::spin();


}



