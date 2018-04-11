#include "ros/ros.h"
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
#include <ompl/base/spaces/SE2StateSpace.h>
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

namespace ob = ompl::base;
namespace og = ompl::geometric;
//using namespace std;

// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;
ros::Publisher marker_pub;
bool get_globalPath=false;

double x11,y11,z11,x22,y22,z22;

class planner {
public:
    nav_msgs::Path global_path;

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
	}
	// Constructor
	planner(void)
	{
		//四旋翼的障碍物几何形状
        Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.5, 0.5, 0.3));
		//分辨率参数设置
		fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
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
		bounds.setLow(2,0);
		bounds.setHigh(2,3);

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
	void replan(void)
	{
/*
        std::cout << "Total Points:" << global_path.poses.size() << std::endl;
        std::cout<<"ininininin"<<std::endl;
        //if(global_path->getStateCount () <= 2)
        //	plan();
        //else
        //{
            for (std::size_t idx = 0; idx < global_path.poses.size(); idx++)
			{
				if(!replan_flag)
                    replan_flag = !isStateValid(global_path.poses[idx]);
				else
					break;
			}
			if(replan_flag)
                //plan();
                std::cout<<"collision!!!"<<std::endl;
			else
				std::cout << "Replanning not required" << std::endl;
        //}*/



            fcl::CollisionObject treeObj((tree_obj));
            fcl::CollisionObject aircraftObject(Quadcopter);

            // check validity of state defined by pos & rot
            fcl::Vec3f translation(x11,y11,z11);
            fcl::Quaternion3f rotation(1, 0, 0, 0);
            aircraftObject.setTransform(rotation, translation);
            fcl::CollisionRequest requestType(1,false,1,false);
            fcl::CollisionResult collisionResult;
            fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

            if(collisionResult.isCollision())
                std::cout<<"collision!!!!!!!!!"<<std::endl;
            else
                std::cout<<"FreeFree"<<std::endl;

            visualization_msgs::Marker points;
            points.header.frame_id = "/map";
            points.header.stamp  = ros::Time::now();
            points.ns = "points_and_lines";
            points.action  = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;

            points.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.2;
            points.scale.y = 0.2;
            // Points are green
            points.color.g = 1.0f;
            points.color.a = 1.0;

            geometry_msgs::Point p;
            p.x = x11;
            p.y = y11;
            p.z = z11;
            points.points.push_back(p);
            marker_pub.publish(points);

		
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
	        // get the goal representation from the problem definition (not the same as the goal state)
	        // and inquire about the found path
			std::cout << "Found solution:" << std::endl;
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			pth->printAsMatrix(std::cout);
	        // print the path to screen
	        // path->print(std::cout);

/*
			nav_msgs::Path msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "map";

            for (std::size_t path_idx = 0; path_idx < collision_idx.front()-2; path_idx++)
            {
                msg.poses.push_back(global_path.poses[path_idx]);
            }

            for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
			{
				const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

				geometry_msgs::PoseStamped pose;

//				pose.header.frame_id = "/world"

				pose.pose.position.x = pos->values[0];
				pose.pose.position.y = pos->values[1];
				pose.pose.position.z = pos->values[2];

				pose.pose.orientation.x = rot->x;
				pose.pose.orientation.y = rot->y;
				pose.pose.orientation.z = rot->z;
				pose.pose.orientation.w = rot->w;

				msg.poses.push_back(pose);
			}
            for (std::size_t path_idx = collision_idx.back()+3; path_idx < global_path.poses.size(); path_idx++)
            {
                msg.poses.push_back(global_path.poses[path_idx]);
            }
            global_path=msg;
            //traj_pub.publish(global_path);
*/
			
	        //Path smoothing using bspline
			//B样条曲线优化
			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth,3);
			// std::cout << "Smoothed Path" << std::endl;
			// path_smooth.print(std::cout);

			
			//Publish path as markers

			nav_msgs::Path smooth_msg;
			smooth_msg.header.stamp = ros::Time::now();
			smooth_msg.header.frame_id = "map";

            for (std::size_t path_idx = 0; path_idx < collision_idx.front()-10; path_idx++)
            {
                smooth_msg.poses.push_back(global_path.poses[path_idx]);
            }

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
				point.pose.position.z = pos->values[2];

				point.pose.orientation.x = rot->x;
				point.pose.orientation.y = rot->y;
				point.pose.orientation.z = rot->z;
				point.pose.orientation.w = rot->w;

				smooth_msg.poses.push_back(point);

				std::cout << "Published marker: " << idx << std::endl;
			}

            for (std::size_t path_idx = collision_idx.back()+11; path_idx < global_path.poses.size(); path_idx++)
            {
                smooth_msg.poses.push_back(global_path.poses[path_idx]);
            }
            global_path=smooth_msg;

    ///////////        vis_pub.publish(global_path);
			// ros::Duration(0.1).sleep();

			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;

		}
		else
			std::cout << "No solution found" << std::endl;
	}
    void ifCollide()
    {
        std::vector<int> temp;
        collision_idx.swap(temp);
        fcl::CollisionObject treeObj((tree_obj));
        fcl::CollisionObject aircraftObject(Quadcopter);
        for(int idx=0;idx<global_path.poses.size();idx++)
        {
            // check validity of state defined by pos & rot
            fcl::Vec3f translation(global_path.poses[idx].pose.position.x,
                                   global_path.poses[idx].pose.position.y,
                                   global_path.poses[idx].pose.position.z);
            fcl::Quaternion3f rotation(global_path.poses[idx].pose.orientation.w,
                                       global_path.poses[idx].pose.orientation.x,
                                       global_path.poses[idx].pose.orientation.y,
                                       global_path.poses[idx].pose.orientation.z);
            aircraftObject.setTransform(rotation, translation);
            fcl::CollisionRequest requestType(1,false,1,false);
            fcl::CollisionResult collisionResult;
            fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
            if(collisionResult.isCollision())
            {
                collision_idx.push_back(idx);
            }
        }

        if(collision_idx.size()!=0)
        {
            int start=collision_idx.front();
            int goal=collision_idx.back();
            setStart(global_path.poses[start-10]);
            setGoal(global_path.poses[goal+10]);
            std::cout<<"Need Re-Planning!!!"<<std::endl;
            plan();
        }
        else
            std::cout<<"FreeFree"<<std::endl;

        vis_pub.publish(global_path);
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
    //loading octree from binary
	// const std::string filename = "/home/xiaopeng/dense.bt";
	// octomap::OcTree temp_tree(0.1);
	// temp_tree.readBinary(filename);
	// fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	
	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
    /*shared_ptr参考http://www.cnblogs.com/TianFang/archive/2008/09/19/1294521.html*/
	// Update the octree used for collision checking
	planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
    planner_ptr->ifCollide();
    //planner_ptr->replan();
}
void path_callback(const nav_msgs::Path::ConstPtr &msg, planner* planner_ptr)
{
    //vis_pub.publish(*msg);
    if(!get_globalPath)
    {
        planner_ptr->global_path=*msg;
        get_globalPath=true;
    }

    /*std::cout<<msg->poses.size()<<std::endl;
    std::cout<<msg->poses[msg->poses.size()-1].pose.position.x<<std::endl;
    std::cout<<msg->poses[msg->poses.size()-1].pose.position.y<<std::endl;
    std::cout<<msg->poses[msg->poses.size()-1].pose.position.z<<std::endl;
    std::cout<<"---------------------------"<<std::endl;*/
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_planner");
    ros::NodeHandle n;

    if(argc!=7)
    {
     x11=0.5;
     y11=-2.5;
     z11=0.2;
     x22=-2.0;
     y22=1.5;
     z22=0.2;
    }
    else{
        x11=strtod(argv[1],NULL);y11=strtod(argv[2],NULL);z11=strtod(argv[3],NULL);
        x22=strtod(argv[4],NULL);y22=strtod(argv[5],NULL);z22=strtod(argv[6],NULL);
        //planner_object.setStart(strtod(argv[1],NULL),strtod(argv[2],NULL),strtod(argv[3],NULL));
        //planner_object.setGoal(strtod(argv[4],NULL),strtod(argv[5],NULL),strtod(argv[6],NULL));
    }
    planner planner_object;

    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/realtime_map", 1, boost::bind(&octomapCallback, _1, &planner_object));
    /*boost::bind()用法参考http://blog.csdn.net/adcxf/article/details/3970116，_1表示缺省形参，可以不给第一个参数指定输入，
     *另外boost::bind()返回值为第一个参数也就是要绑定函数的地址，相当于 &function */
    ros::Subscriber sub_path = n.subscribe<nav_msgs::Path>("/visualization_marker", 1, boost::bind(&path_callback, _1, &planner_object));

    vis_pub = n.advertise<nav_msgs::Path>( "republished_path", 1);
    marker_pub=n.advertise<visualization_msgs::Marker>("MarkerMarker",1);
	
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	ros::spin();

	return 0;
}

