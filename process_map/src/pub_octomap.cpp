#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <eigen_conversions/eigen_msg.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap/OcTree.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_octomap");

  ros::NodeHandle n;

  std::string file_path;
  if(argc<2)
      file_path="/home/nrsl/moveit_ws/map/8-10-octomap.bt";
  else
      file_path=argv[1];
  //publisher for the planning scene
  ros::Publisher octomap_pub = n.advertise<octomap_msgs::Octomap>("/octomap_binary", 1);

  octomap_msgs::Octomap octomap;
  octomap::OcTree myOctomap(file_path);
  std::cout<<&myOctomap<<std::endl;

  octomap_msgs::binaryMapToMsg(myOctomap, octomap);
  //octomap.header.stamp = ros::Time::now();
  octomap.header.frame_id = "odom";

  std::stringstream datastream;
  if (octomap.data.size() > 0){
    datastream.write((const char*) &octomap.data[0], octomap.data.size());
    //std::cout<<"aoaoaoao!!!"<<datastream<<std::endl;
  }
  static int i=0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
//       if(i<10)
//       {
          octomap.header.stamp = ros::Time::now();
          octomap_pub.publish(octomap);
//       }
//       else{
//           ros::shutdown();
//       }
//       i++;
    //ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
