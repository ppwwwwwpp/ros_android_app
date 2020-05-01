cmake_minimum_required(VERSION 2.8.3)
project(dashan_msg)


find_package(catkin REQUIRED COMPONENTS std_msgs message_generation geometry_msgs nav_msgs bp_std_msgs)



add_message_files(DIRECTORY msg
                      FILES 
                            diagnosis.msg
                            posititions.msg
                            worldPosition.msg	
                            Mode.msg
                            gridPosition.msg
                            hmiStatus.msg
                            map.msg
                            mapInfo.msg
                            maplist.msg
                            MapLists.msg
                            Point.msg
                            Quaternion.msg
                            pointprocess.msg
                             PointLists.msg
                             pointlist.msg
                             pointgroup.msg
                             verify_graph_line.msg
                           hand_draw_save.msg
                           canGenLineOnMap.msg
                           hand_draw_list.msg
  
                           PngMapData.msg
                           system_status.msg
      PathTask.msg
    ActionEntry.msg
    Actions.msg
    GridPoint.msg
    LineEntry.msg
    Lines.msg
    PointEntry.msg
    Points.msg
    RotationData.msg
    GraphPath.msg
    PathEntry.msg
    PathGroupEntry.msg
    PathGroups.msg
    Paths.msg 
    Pose2D.msg  
    StartParam.msg
    TaskData.msg
    TaskEntry.msg   
                       
           center.msg
           circles.msg
           edit_origin_map.msg
           lines.msg
            obstacles.msg
            polygons.msg
            polylines.msg
            rectangles.msg
			    customRoute.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs bp_std_msgs)

catkin_package(
      
)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs bp_std_msgs)



include_directories(
  ${catkin_INCLUDE_DIRS}
)





