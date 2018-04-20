#include <ros/ros.h>
#include <iostream>
#include <assert.h>
#include <vector>
#include <stdlib.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include<pcl/common/transforms.h>
#include<pcl/common/eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>

using namespace std;
const double PI = 3.1415926;
ros::Publisher PointCloudPub;
ros::Publisher marker_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudLoad(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
vector<pcl::PointXYZ> polygonVertex;
bool New_Vertex = false;
visualization_msgs::Marker mline_strip, mpoints;

void PolygonCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  pcl::PointXYZ point_t;
  point_t.x = msg->point.x;
  point_t.y = msg->point.y;
  point_t.z = msg->point.z;
  polygonVertex.push_back(point_t);
  New_Vertex = true;

  mpoints.action = visualization_msgs::Marker::ADD;
  mpoints.header.stamp = ros::Time::now();
  mpoints.points.clear();
  for(size_t i=0;i<polygonVertex.size();i++){
    geometry_msgs::Point p;
    p.x = polygonVertex[i].x;
    p.y = polygonVertex[i].y;
    p.z = polygonVertex[i].z;
    mpoints.points.push_back(p);
  }
  marker_pub.publish(mpoints);
}

bool pointInRegion(pcl::PointXYZ& pt)
{
    int nCross = 0;    

    for (int i = 0; i < polygonVertex.size(); i++) {   
      pcl::PointXYZ p1;
      pcl::PointXYZ p2;
      p1 = polygonVertex[i];
      p2 = polygonVertex[(i+1)%polygonVertex.size()]; 

      if ( p1.z == p2.z )
	      continue;   
      if ( pt.z < min(p1.z, p2.z)) 
	      continue; 
      if ( pt.z >= max(p1.z, p2.z)) 
	      continue; 
      double x = (double)(pt.z - p1.z) * (double)(p2.x - p1.x) / (double)(p2.z - p1.z) + p1.x; 
      if ( x > pt.x ) 
	      nCross++; 
    } 
    if (nCross % 2 == 1) {
	    return true; //in region
    }
    else {
	    return false; //out or on edge
    }
}

void remove_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_temp)
{
  size_t pointsNum = PointCloudLoad->size();
  pcl::PointXYZ center = polygonVertex.back();
  polygonVertex.pop_back();
  
  for(size_t i=0;i<pointsNum;i++){
    pcl::PointXYZ point_t = PointCloudLoad->points[i];
    double dist = sqrt((center.x - point_t.x)*(center.x - point_t.x) + (center.z - point_t.z)*(center.z - point_t.z));
    if(/*point_t.y<-1.35|| */dist>50.0){
      cloud_temp->push_back(point_t);
    }
    else{
      if(!pointInRegion(point_t)){
	cloud_temp->push_back(point_t);
      }
    }
  }
}

void delete_one(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_temp)
{
  size_t pointsNum = PointCloudLoad->size();
  pcl::PointXYZ point_c = polygonVertex.front();
  
  for(size_t i=0;i<pointsNum;i++){
    pcl::PointXYZ point_t = PointCloudLoad->points[i];
    double dist = sqrt((point_c.x - point_t.x)*(point_c.x - point_t.x) + (point_c.z - point_t.z)*(point_c.z - point_t.z) + (point_c.y - point_t.y)*(point_c.y - point_t.y));
    if(dist>0.2){
      cloud_temp->push_back(point_t);
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "processPointCloud");
  ros::NodeHandle nh;
  if (argc != 3)
  {
      cout<<"Usage: processPointCloud <input_file> <output_path(with '/' at the end)>"<<endl;
      return -1;
  }

  string input_file = argv[1], output_file = argv[2];
  pcl::io::loadPCDFile<pcl::PointXYZ> ( input_file, *PointCloudLoad );
  size_t pointsNum = PointCloudLoad->size(); 
  for(size_t i=0;i<pointsNum;i++){
    pcl::PointXYZ point_t = PointCloudLoad->points[i];
    if(/*point_t.y<0.55 && */point_t.y>-2.5){
      cloud_temp->push_back(point_t);
    }
  }
  PointCloudLoad->clear();
  pcl::PointCloud<pcl::PointXYZ>::Ptr exchange=PointCloudLoad;
  PointCloudLoad=cloud_temp;
  cloud_temp=exchange;

  ros::Subscriber vertex_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, PolygonCb);
  PointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_map", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  
  mpoints.header.frame_id =mline_strip.header.frame_id = "odom";
  mpoints.ns =mline_strip.ns = "points_line";
  mpoints.id = 0;
  mline_strip.id = 1;
  mpoints.action = mline_strip.action = visualization_msgs::Marker::ADD;
  mpoints.pose.orientation.w= mline_strip.pose.orientation.w = 1.0;
  mpoints.type = visualization_msgs::Marker::POINTS;
  mline_strip.type = visualization_msgs::Marker::LINE_STRIP;
  mpoints.scale.x = 0.3;
  mpoints.scale.y = 0.3;
  mline_strip.scale.x = 0.1;
  mpoints.color.r = 1.0f;
  mpoints.color.g = 1.0f;
  mpoints.color.a = 1.0;
  mline_strip.color.g = 1.0f;
  mline_strip.color.a = 1.0;
  
    static uint8_t PUB_INIT_PC = 0;
    while(PUB_INIT_PC<1){
      sensor_msgs::PointCloud2 pointCloud2msg;
      pcl::toROSMsg(*PointCloudLoad, pointCloud2msg);
      pointCloud2msg.header.frame_id = "odom"; 
      pointCloud2msg.header.stamp = ros::Time::now();
      cout<<"Wating for PointCloud to ROS message ...."<<endl;
      ros::Duration(1).sleep();
      cout<<"Ready for publishing!!"<<endl;
      PointCloudPub.publish(pointCloud2msg);
      PUB_INIT_PC++;
    }
  
  ros::Rate loop_rate(60);
  while(ros::ok())
  {
    ros::spinOnce();
    
    if(New_Vertex){
      system("clear");
      cout<<"\033[32m The new polygon vertex is ["<<polygonVertex.back().x<<","<<polygonVertex.back().y<<","<<polygonVertex.back().z<<"]. \033[0m"<<endl;
      cout<<"\033[32m Do you want to keep this vertex [Y/N]?"<<endl
          <<" Or [S] for saving & processing PointCloud, [D] for deleting all vertices: "<<"\033[0m";

      char c_in;
      std::cin>>c_in;
      switch(c_in){
	case 'P' : case 'p' :{
	  cout<<endl<<"\033[32m Save the new vertex, and there are a total of "<<polygonVertex.size()<<" vertices."<<"\033[0m"<<endl;
	  New_Vertex = false;break;
	}
	case 'Y' : case 'y' :{
	  cout<<endl<<"\033[32m Delete the specified point."<<"\033[0m"<<endl;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	  delete_one(cloud_temp);
	  PointCloudLoad->clear();
	  pcl::PointCloud<pcl::PointXYZ>::Ptr exchange=PointCloudLoad;
	  PointCloudLoad=cloud_temp;
	  cloud_temp=exchange;
	  static uint countPCd = 0;
	  countPCd++;
	  ostringstream oss;
	  oss << output_file <<"DeleteOneMap_"<< countPCd << ".pcd";
	  pcl::io::savePCDFileASCII (oss.str(), *PointCloudLoad);
	  cout<<endl<<"\033[32m Save the DeleteOne PointCloud: "<<oss.str()<<"\033[0m"<<endl;
	  sensor_msgs::PointCloud2 pointCloud2msg;
	  pcl::toROSMsg(*PointCloudLoad, pointCloud2msg);
	  pointCloud2msg.header.frame_id = "odom"; 
	  pointCloud2msg.header.stamp = ros::Time::now();
	  cout<<"Wating for PointCloud to ROS message ...."<<endl;
	  ros::Duration(1).sleep();
	  cout<<"Ready for publishing!!"<<endl;
	  PointCloudPub.publish(pointCloud2msg);
	  polygonVertex.clear();
	  mpoints.action = visualization_msgs::Marker::DELETEALL;
	  marker_pub.publish(mpoints);
	  New_Vertex = false;break;
	}
	case 'N' : case 'n' :{
	  polygonVertex.pop_back();
	  
	  mpoints.action = visualization_msgs::Marker::DELETE;
	  marker_pub.publish(mpoints);
	  mpoints.action = visualization_msgs::Marker::ADD;
	  mpoints.header.stamp = ros::Time::now();
	  mpoints.points.clear();
	  for(size_t i=0;i<polygonVertex.size();i++){
	    geometry_msgs::Point p;
	    p.x = polygonVertex[i].x;
	    p.y = polygonVertex[i].y;
	    p.z = polygonVertex[i].z;
	    mpoints.points.push_back(p);
	  }
	  marker_pub.publish(mpoints);
	  
	  cout<<endl<<"\033[32m Delete the new vertex, and there are "<<polygonVertex.size()<<" vertices left."<<"\033[0m"<<endl;
	  New_Vertex = false;break;
	}
	case 'D' : case 'd' :{
	  polygonVertex.clear();
	  
	  mpoints.action = visualization_msgs::Marker::DELETEALL;
	  marker_pub.publish(mpoints);
	  cout<<endl<<"\033[32m Delete all vertices! "<<"\033[0m"<<endl;
	  New_Vertex = false;break;
	}
	case 'S' : case 's' :{
	  if(polygonVertex.size()>=3){
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	    remove_points(cloud_temp);
	    PointCloudLoad->clear();
	    pcl::PointCloud<pcl::PointXYZ>::Ptr exchange=PointCloudLoad;
	    PointCloudLoad=cloud_temp;
	    cloud_temp=exchange;
	    static uint countPC = 0;
	    countPC++;
	    ostringstream oss;
	    oss << output_file <<"processedMap_"<< countPC << ".pcd";
	    pcl::io::savePCDFileASCII (oss.str(), *PointCloudLoad);
	    cout<<endl<<"\033[32m Save the processed PointCloud: "<<oss.str()<<"\033[0m"<<endl;
	    sensor_msgs::PointCloud2 pointCloud2msg;
	    pcl::toROSMsg(*PointCloudLoad, pointCloud2msg);
	    pointCloud2msg.header.frame_id = "odom"; 
	    pointCloud2msg.header.stamp = ros::Time::now();
	    cout<<"Wating for PointCloud to ROS message ...."<<endl;
	    ros::Duration(1).sleep();
	    cout<<"Ready for publishing!!"<<endl;
	    PointCloudPub.publish(pointCloud2msg);
	  }
	  else{
	    cout<<"\033[31m Less than 3 points cannot form a polygon. Pick again. "<<"\033[0m"<<endl;
	  }
	  polygonVertex.clear();
	  
	  mpoints.action = visualization_msgs::Marker::DELETEALL;
	  marker_pub.publish(mpoints);
	  New_Vertex = false;break;
	}
      }
    }

    loop_rate.sleep();
  }
  cout<<"\033[31m Waiting for filtering ...... "<<PointCloudLoad->size()<<"\033[0m"<<endl;
  cloud_temp->clear();
  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
  downSizeFilter.setInputCloud(PointCloudLoad);
  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilter.filter(*cloud_temp);
  PointCloudLoad->clear();
  
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud_temp);
  outrem.setRadiusSearch(0.3);
  outrem.setMinNeighborsInRadius (5);
  // apply filter
  outrem.filter (*PointCloudLoad);
  
  //----------------------------------------------------------------------------------  
  //旋转方向按照z轴->y轴->x轴的顺序
  cloud_temp->clear();
  Eigen::Affine3f transf = pcl::getTransformation(0.0,0.0,0.0,-PI/2,0.0,-PI/2);
  pcl::transformPointCloud(*PointCloudLoad, *cloud_temp, transf);
 //----------------------------------------------------------------------------------
  
  string filtered_path = output_file + "filteredMap.pcd";
  pcl::io::savePCDFileASCII (filtered_path, *cloud_temp);
  
  octomap::OcTree tree( 0.3 );

    for (auto p:cloud_temp->points){
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }
    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary( output_file + "OctoMap.bt" );
  
  cout<<"\033[31m Finish saving filtered map."<<cloud_temp->size()<<"\033[0m"<<endl;
  return 0;

}