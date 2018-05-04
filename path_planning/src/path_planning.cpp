#include "path_planning.h"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
const double PI = 3.1415926;
//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr PathCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
vector<vector<float>> InitPath;

class planner {
public:
	ob::ProblemDefinitionPtr get_pdef(){
	  return pdef;
	}
		// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;
	
	void init_start(void)
	{
		if(!set_start)
			std::cout << "Initialized!!" << std::endl;
		set_start = true;
	}
	void setStart(double x, double y, double z)
	{
		ob::ScopedState<ob::SE3StateSpace> start(space);
		start->setXYZ(x,y,z);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearStartStates();
		pdef->addStartState(start);
	}
	void setGoal(double x, double y, double z)
	{
		if(prev_goal[0] != x || prev_goal[1] != y || prev_goal[2] != z)
		{
			ob::ScopedState<ob::SE3StateSpace> goal(space);
			goal->setXYZ(x,y,z);
			prev_goal[0] = x;
			prev_goal[1] = y;
			prev_goal[2] = z;
			goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
			pdef->clearGoal();
			pdef->setGoalState(goal);
			std::cout << "Goal point set to: " << x << " " << y << " " << z << std::endl;
			if(set_start)
				plan();
			
		}
	}
	void updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
	{
		tree_obj = map;
	}
	// Constructor
	planner(void)
	{
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(1.0, 1.0, 0.5));
		fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
		tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
		
		space = ob::StateSpacePtr(new ob::SE3StateSpace());

		std::cout << "Initialized: " << std::endl;
	}
	// Destructor
	~planner()
	{
	}
	void replan(void)
	{
		if(path_smooth != NULL && set_start)
		{
			std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
			if(path_smooth->getStateCount () <= 2)
				plan();
			else
			{
				for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
				{
					if(!replan_flag)
						replan_flag = !isStateValid(path_smooth->getState(idx));
					else
						break;

				}
				if(replan_flag)
					plan();
				else
					std::cout << "Replanning not required" << std::endl;
			}
		}
	}

	void plan(void)
	{

	    // create a planner for the defined space
		ob::PlannerPtr plan(new og::InformedRRTstar(si));

	    // set the problem we are trying to solve for the planner
		plan->setProblemDefinition(pdef);

	    // perform setup steps for the planner
		plan->setup();

	    // print the settings for this space
		si->printSettings(std::cout);

	    // print the problem settings
		pdef->print(std::cout);

	    // attempt to solve the problem within one second of planning time
		ob::PlannerStatus solved = plan->solve(0.5);

		if (solved)
		{			
	        //Path smoothing using bspline

			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth,3);
			// std::cout << "Smoothed Path" << std::endl;
			// path_smooth.print(std::cout);

			
			//Publish path as markers

			visualization_msgs::Marker marker;
			marker.action = visualization_msgs::Marker::DELETEALL;
			vis_pub.publish(marker);

			for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
			{
	                // cast the abstract state type to the type we expect
				const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
				
				marker.header.frame_id = "odom";
				marker.header.stamp = ros::Time();
				marker.ns = "path";
				marker.id = idx;
				marker.type = visualization_msgs::Marker::CUBE;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = pos->values[0];
				marker.pose.position.y = pos->values[1];
				marker.pose.position.z = pos->values[2];
				marker.pose.orientation.x = rot->x;
				marker.pose.orientation.y = rot->y;
				marker.pose.orientation.z = rot->z;
				marker.pose.orientation.w = rot->w;
				marker.scale.x = 0.35;
				marker.scale.y = 0.35;
				marker.scale.z = 0.35;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				vis_pub.publish(marker);
				// ros::Duration(0.1).sleep();
				std::cout << "Published marker: " << idx << std::endl;  
			}
			
			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;

		}
		else
			std::cout << "No solution found" << std::endl;
	}
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
private:

// 	// construct the state space we are planning in
// 	ob::StateSpacePtr space;
// 
// 	// construct an instance of  space information from this state space
// 	ob::SpaceInformationPtr si;
// 
// 	// create a problem instance
// 	ob::ProblemDefinitionPtr pdef;

	// goal state
	double prev_goal[3];

	og::PathGeometric* path_smooth = NULL;

	bool replan_flag = false;

	std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

	std::shared_ptr<fcl::CollisionGeometry> tree_obj;

	// Flag for initialization
	bool set_start = false;

// 	bool isStateValid(const ob::State *state)
// 	{
// 	    // cast the abstract state type to the type we expect
// 		const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
// 
// 	    // extract the first component of the state and cast it to what we expect
// 		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
// 
// 	    // extract the second component of the state and cast it to what we expect
// 		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
// 
// 		fcl::CollisionObject treeObj((tree_obj));
// 		fcl::CollisionObject aircraftObject(Quadcopter);
// 
// 	    // check validity of state defined by pos & rot
// 		fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
// 		fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
// 		aircraftObject.setTransform(rotation, translation);
// 		fcl::CollisionRequest requestType(1,false,1,false);
// 		fcl::CollisionResult collisionResult;
// 		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
// 
// 		return(!collisionResult.isCollision());
// 	}

	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
// 	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
// 	{
// 		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
// 		// obj->setCostThreshold(ob::Cost(1.51));
// 		return obj;
// 	}
// 
// 	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
// 	{
// 		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
// 		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
// 		return obj;
// 	}

};

void ReadInitPath()
{
  std::string pkg_loc = ros::package::getPath("path_planning");
  std::ifstream infile(pkg_loc + "/conf/KeyFrameTrajectory.txt");

  istringstream istr;
  string str;
  vector<float> tmpvec;
  while(getline(infile,str)){
    istr.str(str);
    float tmpf;
    while(istr>>tmpf){
      tmpvec.push_back(tmpf);
    }
    InitPath.push_back(tmpvec);
    tmpvec.clear();
    istr.clear();
  }
  infile.close();
}

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
{
    //loading octree from binary
// 	const std::string filename = "/home/nrsl/moveit_ws/map/Octomap_Origin.bt";
// 	octomap::OcTree temp_tree(0.1);
// 	temp_tree.readBinary(filename);
// 	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	

	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	
	// Update the octree used for collision checking
	planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
	planner_ptr->replan();
}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	planner_ptr->init_start();
}

void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
	planner_ptr->init_start();
}

void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqrDis;
  pcl::PointXYZ ips;
  ips.x = msg->point.x;
  ips.y = msg->point.y;
  ips.z = msg->point.z;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  traj_pub.publish(marker);
  pcl::PointXYZ path_n;
  kdTree->nearestKSearch(ips, 1, pointSearchInd, pointSearchSqrDis);
  if (pointSearchSqrDis[0] < 250.0 && pointSearchInd.size() == 1) 
  {
    path_n = PathCloud->points[pointSearchInd[0]];
//     planner_ptr->setStart(path_n.x, path_n.y, path_n.z);
    std::cout<<"!!!!!!!!!!!"<<path_n.x<<"  "<<path_n.y<<"  "<<path_n.z<<std::endl;
    
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "initpath";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = path_n.x;
    marker.pose.position.y = path_n.y;
    marker.pose.position.z = path_n.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    ros::Duration(0.05).sleep();
    traj_pub.publish(marker);
  //--------------------------------//////////////////////////////////  
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "initpath";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = ips.x;
    marker.pose.position.y = ips.y;
    marker.pose.position.z = path_n.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    ros::Duration(0.05).sleep();
    traj_pub.publish(marker);

   // set the bounds for the R^3 part of SE(3)
    double lx = ((path_n.x<ips.x)?path_n.x:ips.x)-5.0;
    double hx = ((path_n.x>ips.x)?path_n.x:ips.x)+5.0;
    double ly = ((path_n.y<ips.y)?path_n.y:ips.y)-5.0;
    double hy = ((path_n.y>ips.y)?path_n.y:ips.y)+5.0;
    double lz = path_n.z-0.7;
    double hz = path_n.z+0.5;;
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,lx);
    bounds.setHigh(0,hx);
    bounds.setLow(1,ly);
    bounds.setHigh(1,hy);
    bounds.setLow(2,lz);
    bounds.setHigh(2,hz);

    planner_ptr->space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
    planner_ptr->si = ob::SpaceInformationPtr(new ob::SpaceInformation(planner_ptr->space));

    ob::ScopedState<ob::SE3StateSpace> start(planner_ptr->space);
    ob::ScopedState<ob::SE3StateSpace> goal(planner_ptr->space);
    start->setXYZ(path_n.x,path_n.y,path_n.z);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    goal->setXYZ(ips.x,ips.y,path_n.z);
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
  // set state validity checking for this space
    planner_ptr->si->setStateValidityChecker(boost::bind(&planner::isStateValid, planner_ptr,_1));

    // create a problem instance
    planner_ptr->pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(planner_ptr->si));

    // set the start and goal states
    planner_ptr->pdef->setStartAndGoalStates(start, goal);

  // set Optimizattion objective
    planner_ptr->pdef->setOptimizationObjective(boost::bind(&planner::getPathLengthObjWithCostToGo,planner_ptr,_1)(planner_ptr->si));
    planner_ptr->plan();
//----------------------------------------------------------------------------------
  } 
  //    planner_ptr->setGoal(ips.x, ips.y, ips.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_planner");
  ros::NodeHandle n;
  planner planner_object;
  planner_object.init_start();
  ReadInitPath();
  
  ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/bebop2/odometry_sensor1/odometry", 1, boost::bind(&odomCb, _1, &planner_object));
  ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));
  // ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));

  vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  traj_pub = n.advertise<visualization_msgs::Marker>("init_path",1);
  int count =0;
  ros::Duration(1).sleep();
//   visualization_msgs::Marker marker;
  for(uint i=0;i<InitPath.size();i++){
    count++;
    tfScalar yaw=PI/2,pitch=0.0,roll=-PI/2;
    tf::Quaternion q = tf::Quaternion(yaw,pitch,roll);
    tf::Vector3 v_T = tf::Matrix3x3(q)*tf::Vector3(InitPath[i][2],InitPath[i][3],InitPath[i][4]);
    pcl::PointXYZ p_n;
    p_n.x = v_T.getX();
    p_n.y = v_T.getY();
    p_n.z = v_T.getZ();
    PathCloud->push_back(p_n);
//     marker.header.frame_id = "odom";
//     marker.header.stamp = ros::Time();
//     marker.ns = "initpath";
//     marker.id = i;
//     marker.type = visualization_msgs::Marker::CUBE;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose.position.x = v_T.getX();
//     marker.pose.position.y = v_T.getY();
//     marker.pose.position.z = v_T.getZ();
//     marker.pose.orientation.x = InitPath[i][7];
//     marker.pose.orientation.y = -InitPath[i][5];
//     marker.pose.orientation.z = -InitPath[i][6];
//     marker.pose.orientation.w = InitPath[i][8];
//     marker.scale.x = 0.35;
//     marker.scale.y = 0.35;
//     marker.scale.z = 0.35;
//     marker.color.a = 1.0;
//     marker.color.r = 1.0;
//     marker.color.g = 0.5;
//     marker.color.b = 0.5;
//     ros::Duration(0.05).sleep();
//     traj_pub.publish(marker);
  }
  kdTree->setInputCloud(PathCloud);
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  ros::spin();

  return 0;
}