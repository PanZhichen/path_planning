/*#include <ros/ros.h>
#include <iostream>
#include <assert.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
ros::Publisher pub_octomap;
geometry_msgs::TransformStamped OpticalToMap;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    //cout<<input->header.frame_id<<endl;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud ;
    pcl::fromROSMsg(*input,cloud);

    geometry_msgs::PointStamped in,out;

    for(int i=0;i<cloud.points.size();i++)
      {
        in.point.x=cloud.points[i].x;
        in.point.y=cloud.points[i].y;
        in.point.z=cloud.points[i].z;
        tf2::doTransform(in,out,OpticalToMap);
        cloud.points[i].x=out.point.x;
        cloud.points[i].y=out.point.y;
        cloud.points[i].z=out.point.z;
      }

    octomap::OcTree tree( 0.05 );

    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    octomap_msgs::Octomap map_msg;
    octomap_msgs::binaryMapToMsg(tree,map_msg);
    map_msg.header.stamp=ros::Time::now();
    map_msg.header.frame_id="map";
    pub_octomap.publish(map_msg);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "realtime_octomap");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe ("voxel_cloud", 1, cloud_cb);
    pub_octomap = nh.advertise<octomap_msgs::Octomap> ("realtime_map", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(25);

    while (ros::ok())
      {
        try{
                OpticalToMap = tfBuffer.lookupTransform("map", "camera_rgb_optical_frame",ros::Time(0));
              }
              catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
              }

        ros::spinOnce();
        loop_rate.sleep();
        //cout<<endl;
      }

    return 0;
}*/

#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap 
#include <octomap/octomap.h>
using namespace std;

int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: pcd2octomap <input_file> <output_file>"<<endl;
        return -1;
    }

    string input_file = argv[1], output_file = argv[2];
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_temp;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );
    //cloud_temp.width=cloud.width;
    //cloud_temp.height=cloud.height;
    //cloud_temp.is_dense=false;
   // cloud_temp.points.resize(cloud_temp.width*cloud_temp.height);

    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;
    //for(uint i=0;i<cloud.points.size();i++)
    //{
    //    float temp;
    //    temp=cloud.points[i].y;
     //   cloud.points[i].y=-cloud.points[i].z;
    //    cloud.points[i].z=-temp;
    //    if((cloud.points[i].z>0)&&(cloud.points[i].z<2.5))
    //    {
    ////        cloud_temp.points[i].x=cloud.points[i].x;
     //       cloud_temp.points[i].y=cloud.points[i].y;
     //       cloud_temp.points[i].z=cloud.points[i].z;
     //   }
    //}
    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.05 );

    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary( output_file );
    cout<<"done."<<endl;

    return 0;
}

