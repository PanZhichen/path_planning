1、打开三个终端，分别 cd moveit_ws/ && source devel/setup.bash
2、一个终端打开rviz，一个rosrun path_planning path_planning_node ，另一个rosrun process_map pub_octomap 
3、然后在rviz里可以用‘publish point’按钮点击地图中的一些点，可以规划到目标点的路径

#######################################################################################
地图处理方法：
1. cd moveit_ws/ && source devel/setup.bash后运行$roslaunch process_map processMap.launch 

