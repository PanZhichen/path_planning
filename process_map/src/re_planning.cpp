/*#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <stdlib.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <vector>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

#include <assert.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>*/
#include "re_plan_head.h"
#include <pathwithflag_msgs/PathWithFlag.h>
#define RECTIFY 0.0
namespace ob = ompl::base;
namespace og = ompl::geometric;
//using namespace std;

// Declear some global variables

//ROS publishers
ros::Publisher path_pub;
ros::Publisher octomap_pub;
ros::Publisher octomap_added;
bool get_globalPath=false;
bool get_globalMap=false;
tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped OpticalToMap;
static bool waiting_free=false;
//octomap_msgs::Octomap::ConstPtr global_map;
double x11,y11,z11,x22,y22,z22;

class planner {
public:
    /////////////////////////////
    pathwithflag_msgs::PathWithFlag planned_path;
    //nav_msgs::Path planned_path;
    octomap_msgs::Octomap::ConstPtr global_map;

    void setStart(geometry_msgs::PoseStamped startP)
    {
        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(startP.pose.position.x,
                      startP.pose.position.y,
                      startP.pose.position.z);
        //start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        start->as<ob::SO3StateSpace::StateType>(1)->w=startP.pose.orientation.w;
        start->as<ob::SO3StateSpace::StateType>(1)->x=startP.pose.orientation.x;
        start->as<ob::SO3StateSpace::StateType>(1)->y=startP.pose.orientation.y;
        start->as<ob::SO3StateSpace::StateType>(1)->z=startP.pose.orientation.z;
        pdef->clearStartStates();
        pdef->addStartState(start);
    }
    void setGoal(geometry_msgs::PoseStamped goalP)
    {
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(goalP.pose.position.x,
                     goalP.pose.position.y,
                     goalP.pose.position.z);
        goal->as<ob::SO3StateSpace::StateType>(1)->w=goalP.pose.orientation.w;
        goal->as<ob::SO3StateSpace::StateType>(1)->x=goalP.pose.orientation.x;
        goal->as<ob::SO3StateSpace::StateType>(1)->y=goalP.pose.orientation.y;
        goal->as<ob::SO3StateSpace::StateType>(1)->z=goalP.pose.orientation.z;
        pdef->clearGoal();
        pdef->setGoalState(goal);
        std::cout << "goal set to: " << goalP.pose.position.x << " "
                  << goalP.pose.position.y << " " << goalP.pose.position.z << std::endl;
    }
    void updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
    {
        tree_obj = map;
        std::cout<<"ininininininin    "<<std::endl;

    }
    void updateCollideMap(std::shared_ptr<fcl::CollisionGeometry> map)
    {
        collide_obj = map;
    }
    // Constructor
    planner(void)
    {
        //四旋翼的障碍物几何形状
        Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.4, 0.4, 0.1));
        //分辨率参数设置
        fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.05)));
        tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);

        //解的状态空间
        space = ob::StateSpacePtr(new ob::SE3StateSpace());

        // create a start state
        ob::ScopedState<ob::SE3StateSpace> start(space);

        // create a goal state
        ob::ScopedState<ob::SE3StateSpace> goal(space);

        // set the bounds for the R^3 part of SE(3)
        // 搜索的三维范围设置
        ob::RealVectorBounds bounds(3);

        bounds.setLow(0,-5);
        bounds.setHigh(0,5);
        bounds.setLow(1,-5);
        bounds.setHigh(1,5);
        bounds.setLow(2,RECTIFY-0.1);
        bounds.setHigh(2,RECTIFY+0.2);

        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        start->setXYZ(x11,y11,z11);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // start.random();

        goal->setXYZ(x22,y22,z22);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // goal.random();


        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));
        /*Si中设置了一个函数setStateValidityChecker，用来检测求解的每个路径点是否合理，即是否发生碰撞，
         * 这里将这个setStateValidityChecker函数指向了planner::isStateValid( )函数，是一个自定义的函数，
         * 返回值是true/false,在这个函数中，地图指向了tree_obj,飞机尺寸指向了quadcopter，这就是地图在
         * 路径规划中的应用*/
        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // set Optimizattion objective
        pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

        std::cout << "Initialized: " << std::endl;
    }
    // Destructor
    ~planner()
    {
    }

    void plan(void)
    {

        // create a planner for the defined space
        og::InformedRRTstar* rrt = new og::InformedRRTstar(si);

        //设置rrt的参数range
        rrt->setRange(0.2);

        ob::PlannerPtr plan(rrt);

        // set the problem we are trying to solve for the planner
        plan->setProblemDefinition(pdef);

        // perform setup steps for the planner
        plan->setup();

        // print the settings for this space
        si->printSettings(std::cout);

        std::cout << "problem setting\n";
        // print the problem settings
        pdef->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = plan->solve(1);

        if (solved)
        {
            //Path smoothing using bspline
            //B样条曲线优化
            og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
            path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            pathBSpline->smoothBSpline(*path_smooth,3);

            nav_msgs::Path smooth_msg;
            smooth_msg.header.stamp = ros::Time::now();
            smooth_msg.header.frame_id = "map";

            for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
            {
                    // cast the abstract state type to the type we expect
                const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

                // extract the first component of the state and cast it to what we expect
                const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                // extract the second component of the state and cast it to what we expect
                const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                geometry_msgs::PoseStamped point;

//				pose.header.frame_id = "/world"

                point.pose.position.x = pos->values[0];
                point.pose.position.y = pos->values[1];
                //point.pose.position.z = pos->values[2];
                point.pose.position.z = RECTIFY;

                point.pose.orientation.x = rot->x;
                point.pose.orientation.y = rot->y;
                point.pose.orientation.z = rot->z;
                point.pose.orientation.w = rot->w;

                smooth_msg.poses.push_back(point);

                std::cout << "Published marker: " << idx << std::endl;
            }
            planned_path.path=smooth_msg;

            pdef->clearSolutionPaths();
            replan_flag = false;
            std::cout<<"DoneDoneDone!!!!"<<std::endl;

        }
        else
            std::cout << "No solution found" << std::endl;
    }
    bool ifCollide()
    {
        fcl::CollisionObject treeObj((collide_obj));
        /// //!!!!!!!!!!!!!!!
        ////fcl::CollisionObject treeObj((tree_obj));
        std::shared_ptr<fcl::CollisionGeometry> Quadcopter_t = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.2, 0.2, 0.1));
        fcl::CollisionObject aircraftObject(Quadcopter_t);
        ////!!!!!!!!!!!!!!!!!!!!!!!!!!!
        for(int idx=0;idx<planned_path.path.poses.size();idx++)
        {
            // check validity of state defined by pos & rot
            fcl::Vec3f translation(planned_path.path.poses[idx].pose.position.x,
                                   planned_path.path.poses[idx].pose.position.y,
                                   planned_path.path.poses[idx].pose.position.z);
            fcl::Quaternion3f rotation(planned_path.path.poses[idx].pose.orientation.w,
                                       planned_path.path.poses[idx].pose.orientation.x,
                                       planned_path.path.poses[idx].pose.orientation.y,
                                       planned_path.path.poses[idx].pose.orientation.z);
            aircraftObject.setTransform(rotation, translation);
            fcl::CollisionRequest requestType(1,false,1,false);
            fcl::CollisionResult collisionResult;
            fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
            if(collisionResult.isCollision())
            {
                std::cout<<"Need Re-Planning!!!"<<std::endl;
                return true;
            }
        }
        std::cout<<"FreeFree"<<std::endl;
        return false;
    }
private:

    // construct the state space we are planning in
    ob::StateSpacePtr space;

    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si;

    // create a problem instance
    ob::ProblemDefinitionPtr pdef;

    og::PathGeometric* path_smooth;

    std::vector<int> collision_idx;

    bool replan_flag = false;

    std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

    std::shared_ptr<fcl::CollisionGeometry> tree_obj;
    std::shared_ptr<fcl::CollisionGeometry> collide_obj;

    bool isStateValid(const ob::State *state)
    {
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        fcl::CollisionObject treeObj((tree_obj));
        fcl::CollisionObject aircraftObject(Quadcopter);

        // check validity of state defined by pos & rot
        fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
        aircraftObject.setTransform(rotation, translation);
        fcl::CollisionRequest requestType(1,false,1,false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

        return(!collisionResult.isCollision());
    }

    // Returns a structure representing the optimization objective to use
    // for optimal motion planning. This method returns an objective which
    // attempts to minimize the length in configuration space of computed
    // paths.
    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        // obj->setCostThreshold(ob::Cost(1.51));
        return obj;
    }

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
        return obj;
    }

};


void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
{
    if(!get_globalMap)
    {
        std::stringstream datastream;
        if (msg->data.size() > 0){
          datastream.write((const char*) &msg->data[0], msg->data.size());
          //std::cout<<"aoaoaoao!!!"<<datastream<<std::endl;
          //while (ros::ok());
        }
        planner_ptr->global_map=msg;
        // convert octree to collision object
        octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
        fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
        // Update the octree used for collision checking
        planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
        planner_ptr->plan();
        planner_ptr->planned_path.renew=true;
        path_pub.publish(planner_ptr->planned_path);
        get_globalMap=true;  
        planner_ptr->planned_path.renew=false;
    }
    //path_pub.publish(planner_ptr->planned_path.path);
}

void pointcloudCallback (const sensor_msgs::PointCloud2ConstPtr& input, planner* planner_ptr)
{
    try{
            OpticalToMap = tfBuffer.lookupTransform("map", "camera_rgb_optical_frame",ros::Time(0));
          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(0.2).sleep();
            return;}
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

    //octomap::OcTree tree_ptr(0.05);
    octomap::OcTree* tree_ptr=new octomap::OcTree(0.05);
    //tree_ptr->setResolution(0.05);
    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
        tree_ptr->updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }
    // 更新octomap
    tree_ptr->updateInnerOccupancy();

    octomap_msgs::Octomap map_msg;
    octomap_msgs::binaryMapToMsg(*tree_ptr,map_msg);
    map_msg.header.stamp=ros::Time::now();
    map_msg.header.frame_id="map";
    octomap_pub.publish(map_msg);

    octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map_msg));
    fcl::OcTree* tree_1 = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
    planner_ptr->updateCollideMap(std::shared_ptr<fcl::CollisionGeometry>(tree_1));
    /*///////////////////gaigaigai    updateCollideMap /////////////////////////////////
    planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree_1));
    if(if_first)
    {
        planner_ptr->plan();
        planner_ptr->planned_path.renew=true;
        path_pub.publish(planner_ptr->planned_path.path);
        if_first=false;
    }

    if(planner_ptr->ifCollide())//if collide
    {
        try{
                OpticalToMap = tfBuffer.lookupTransform("map", "camera_rgb_optical_frame",ros::Time(0));
              }
              catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                //ros::Duration(1.0).sleep();
                return;}
        geometry_msgs::PoseStamped startP;
        startP.pose.position.x=OpticalToMap.transform.translation.x;
        startP.pose.position.y=OpticalToMap.transform.translation.y;
        startP.pose.position.z=OpticalToMap.transform.translation.z;
        //if((startP.pose.position.z<0)||(startP.pose.position.z>1.0))
            startP.pose.position.z=0.4;
        //startP.pose.orientation.w=1;
        //startP.pose.orientation.x=startP.pose.orientation.y=startP.pose.orientation.z=0;
        startP.pose.orientation.w=OpticalToMap.transform.rotation.w;
        startP.pose.orientation.x=OpticalToMap.transform.rotation.x;
        startP.pose.orientation.y=OpticalToMap.transform.rotation.y;
        startP.pose.orientation.z=OpticalToMap.transform.rotation.z;
        planner_ptr->setStart(startP);
        planner_ptr->plan();
        planner_ptr->planned_path.renew=true;
    }*/
/////////////////////////////!!!!!!!!//////////////////////////////
    if(planner_ptr->ifCollide())//if collide
    {
        if(!waiting_free){
            pathwithflag_msgs::PathWithFlag wait_path;
            wait_path.renew=true;
            wait_path.path.poses.clear();
            path_pub.publish(wait_path);
            waiting_free=true;
            ros::Duration(1.0).sleep();
        }
        else{
            //octomap::OcTree temp_map=*(planner_ptr->globalMap);
            octomap::OcTree* temp_mapPtr = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*(planner_ptr->global_map)));
            for (auto p:cloud.points){
                temp_mapPtr->updateNode( octomap::point3d(p.x, p.y, p.z), true );
            }
            temp_mapPtr->updateInnerOccupancy();

            fcl::OcTree* tree_2 = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(temp_mapPtr));

            planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree_2));

            geometry_msgs::PoseStamped startP;
            startP.pose.position.x=OpticalToMap.transform.translation.x;
            startP.pose.position.y=OpticalToMap.transform.translation.y;
            startP.pose.position.z=OpticalToMap.transform.translation.z;
            //if((startP.pose.position.z<0)||(startP.pose.position.z>1.0))
                startP.pose.position.z=RECTIFY;
            //startP.pose.orientation.w=1;
            //startP.pose.orientation.x=startP.pose.orientation.y=startP.pose.orientation.z=0;
            startP.pose.orientation.w=OpticalToMap.transform.rotation.w;
            startP.pose.orientation.x=OpticalToMap.transform.rotation.x;
            startP.pose.orientation.y=OpticalToMap.transform.rotation.y;
            startP.pose.orientation.z=OpticalToMap.transform.rotation.z;
            planner_ptr->setStart(startP);
            planner_ptr->plan();
            planner_ptr->planned_path.renew=true;
            waiting_free=false;
        }
    }
    else{
        waiting_free=false;
    }
    if(!waiting_free){
        path_pub.publish(planner_ptr->planned_path);
        planner_ptr->planned_path.renew=false;
    }

    delete tree_ptr;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_planner");
    ros::NodeHandle n;
    tf2_ros::TransformListener tfListener(tfBuffer);
    if(argc!=7)
    {
     x11=0;
     y11=0;
     z11=0.4;
     x22=-3.0;
     y22=2.0;
     z22=0.4;
    }
    else{
        x11=strtod(argv[1],NULL);y11=strtod(argv[2],NULL);z11=strtod(argv[3],NULL);
        x22=strtod(argv[4],NULL);y22=strtod(argv[5],NULL);z22=strtod(argv[6],NULL);
    }
    planner planner_object;

    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
 //////   ros::Subscriber pointcloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/filtered_cloud", 1, boost::bind(&pointcloudCallback, _1, &planner_object));
    /*boost::bind()用法参考http://blog.csdn.net/adcxf/article/details/3970116，_1表示缺省形参，可以不给第一个参数指定输入，
     *另外boost::bind()返回值为第一个参数也就是要绑定函数的地址，相当于 &function */

    //path_pub = n.advertise<nav_msgs::Path>( "republished_path", 1);
    path_pub = n.advertise<pathwithflag_msgs::PathWithFlag>( "republished_path", 1);
    octomap_pub=n.advertise<octomap_msgs::Octomap>("realtime_map",1);
    octomap_added=n.advertise<octomap_msgs::Octomap>("Added_map",1);
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    ros::Rate loop_rate(5);
    while (ros::ok())
    {
      ros::spinOnce();

      loop_rate.sleep();
    }

    return 0;
}

