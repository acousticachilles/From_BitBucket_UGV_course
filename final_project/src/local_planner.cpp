#include <ros/ros.h>

#include <base_local_planner/trajectory_planner_ros.h>
#include <tf/transform_listener.h>
#include <navfn/navfn_ros.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Class pointers
ros ::Publisher pub;
base_local_planner::TrajectoryPlannerROS* local_planner;
navfn::NavfnROS* global_planner;
costmap_2d::Costmap2DROS* local_costmap;
tf::TransformListener* listener;
costmap_2d::Costmap2DROS* global_costmap;
// Publishers
ros::Publisher pub_twist;
ros::Publisher pub_twisttest;
ros::Publisher pub_goal;
ros::Publisher pub_shortestPash;

tf::Stamped<tf::Pose> pose_stamped;
double xTarget;
double yTarget;
double epsilon ;
double xCurrent ;

double yCurrent ;
double xCurrent1 ;
double yCurrent1 ;
double x_dif ;
double y_dif ;
long double distToTarget[6];
long double distCalc[6];
double distToMark;
int h = 0;
int firstGo = 0;
// Current goal set by Rviz
geometry_msgs::PoseStamped current_goal;
geometry_msgs::PoseStamped temp_st;
geometry_msgs::PoseStamped temp_st1;
geometry_msgs::PoseStamped temp_st2;
geometry_msgs::PoseStamped home;
geometry_msgs::PoseStamped spath;
//std_msgs::Float64MultiArray current_goal;
std::vector<geometry_msgs::PoseStamped> plan;
std::vector<geometry_msgs::PoseStamped> s_path;
std::vector<geometry_msgs::PoseStamped> plantopoint[6];
std::vector<geometry_msgs::PoseStamped> s2to3;
std::vector<geometry_msgs::PoseStamped> s3to1;
geometry_msgs::PoseArray something;
geometry_msgs::Pose something2;
std::vector<geometry_msgs::PoseStamped> home1;
std::vector<geometry_msgs::PoseStamped> home2;
std::vector<geometry_msgs::PoseStamped> home3;

std::vector<geometry_msgs::PoseStamped> plan23;
geometry_msgs::Twist twist_msgtest;
// Booleans to wait for valid inputs before acting
bool active = true;
bool valid_goal = false;

// Receive path from global planner and pass it to the local local_planner class instance
void recvPath(const nav_msgs::Path::ConstPtr& msg)
{
	if (msg->poses.size() > 0){
		active = true;
	}else{
		active = false;
	}

	local_planner->setPlan(msg->poses);
}

// Receive goal from Rviz and store in global variable
void recvGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	valid_goal = true;

	current_goal.header.frame_id ="/map";
	current_goal.header.stamp = ros::Time::now();
	current_goal.pose.position.x = 4;
	current_goal.pose.position.y = 9;
	current_goal.pose.position.z = 0;
	current_goal.pose.orientation.x = 0;
	current_goal.pose.orientation.y = 0;
	current_goal.pose.orientation.z = 0;
	current_goal.pose.orientation.w = 1;
	//current_goal = the_goal_hc;
	//current_goal = *msg;
}

// Receive goal from Rviz and store in global variable
void setGoal()
{
	valid_goal = true;

	current_goal.header.frame_id ="/map";
	current_goal.header.stamp = ros::Time::now();
	current_goal.pose.position.x = 4;
	current_goal.pose.position.y = 9;
	current_goal.pose.position.z = 0;
	current_goal.pose.orientation.x = 0;
	current_goal.pose.orientation.y = 0;
	current_goal.pose.orientation.z = 0;
	current_goal.pose.orientation.w = 1;
	//current_goal = the_goal_hc;
	//current_goal = *msg;
}

// Timer to extract control signals from local local_planner class instance
void timerCallback(const ros::TimerEvent& event)
{
	if (active){
		geometry_msgs::Twist twist_msg;
		local_planner->computeVelocityCommands(twist_msg);
		pub_twist.publish(twist_msg);
	}
}
 //Timer to extract control signals from local local_planner class instance
void timerCallback_new(const ros::TimerEvent& event)
{
	if (active){
		geometry_msgs::Twist twist_msg;
		local_planner->computeVelocityCommands(twist_msg);
		pub_twist.publish(twist_msg);
	}
}





void parseCustomers(const geometry_msgs::PoseArray::ConstPtr  &pose_array)
{

	//temp space
if (firstGo ==0)
{
		//something.poses.clear();
		temp_st.header.frame_id = "/map";
		temp_st1.header.frame_id = "/map";
		temp_st2.header.frame_id = "/map";
		home.header.frame_id = "/map";
		temp_st.pose =pose_array->poses[0];
		temp_st1.pose =pose_array->poses[1];
		temp_st2.pose =pose_array->poses[2];
		home.pose.position.x =0;

		home.pose.position.y =0;
		something.poses.size() ;
	//	something.poses.insert(temp_st.pose);
		//something.poses.resize()
		//something.poses[0].position.x=temp_st.pose.position.x ;
		something.poses.push_back(temp_st.pose);
		something.poses.push_back(temp_st1.pose);
		something.poses.push_back(temp_st2.pose);
		ros::Duration d = ros::Duration(3.0, 0);
	    global_planner->makePlan(temp_st, temp_st1, plantopoint[0]);
	    d.sleep();
	    global_planner->makePlan( temp_st1, temp_st2,plantopoint[1]);
	    d.sleep();
	    global_planner->makePlan(temp_st, temp_st2, plantopoint[2]);
	    d.sleep();
	    global_planner->makePlan( home, temp_st,plantopoint[3]);
	    d.sleep();
	    global_planner->makePlan( home, temp_st1,plantopoint[4]);
	    d.sleep();
	    global_planner->makePlan( home, temp_st2,plantopoint[5]);
	    d.sleep();



	    	//x = msg->poses.size();


	    int y = 0;
	    while (y < 6)
	    {
	    	distToTarget[y] = 0;
	    	int x = 1;
	    	while ( x < plantopoint[y].size() )
	    	{


	    		xCurrent = plantopoint[y][x-1].pose.position.x;
	    		yCurrent = plantopoint[y][x-1].pose.position.y;


	    		xCurrent1 = plantopoint[y][x].pose.position.x;
	    		yCurrent1 = plantopoint[y][x].pose.position.y;


	    		x_dif = pow((xCurrent1 -xCurrent), 2);
	    		y_dif = pow((yCurrent1 - yCurrent), 2);
	    		distToTarget[y] =distToTarget[y] + sqrt(x_dif + y_dif);

	    		x = x+ 1;
	    	}


	    	y++;
	    }
	    distCalc[1] =distToTarget[5]+distToTarget[3]+distToTarget[1]+distToTarget[0];
	    distCalc[0] = distToTarget[3]+distToTarget[4]+distToTarget[1]+distToTarget[2];
	    distCalc[2] = distToTarget[5]+distToTarget[4]+distToTarget[0]+distToTarget[2];
	    ROS_INFO_STREAM("Here is a list of all partial distances\n");
	    for(int kk = 0; kk < 7; kk++)
	    {
	    	ROS_INFO_STREAM("partial distance (PD) " << kk << " = " << distToTarget[kk]);
	    }
	    ROS_INFO_STREAM("The total path distance are define by the following sum of partial distances \n");
	    std::string smallest = "";
	    for(int ll = 0; ll < 3; ll++)
		{
	    	switch(ll){
				case 0:
					ROS_INFO_STREAM("path " << ll << " : PD4 + PD3 + PD2 + PD1 = " << distCalc[ll]);
					break;
				case 1:
					ROS_INFO_STREAM("path " << ll << " : PD5 + PD3 + PD1 + PD0 = " << distCalc[ll]);
					break;
				case 2:
					ROS_INFO_STREAM("path " << ll << " : PD5 + PD4 + PD2 + PD0 = " << distCalc[ll]);
					break;
	    	}

		}
	    something.poses[0].position.x = temp_st.pose.position.x ;
	    something.poses[1].position.x = temp_st2.pose .position.x;
	    something.poses[2].position.x = temp_st1.pose.position.x;
	    something.poses[0].position.y = temp_st.pose.position.y ;
	    something.poses[1].position.y = temp_st2.pose .position.y;
	    something.poses[2].position.y = temp_st1.pose.position.y;

	    ROS_INFO_STREAM("Which is smaller path 0 or 1 or 2? ");
	    smallest = "0";
	    if (distCalc[1] <  distCalc[0])
	    {
	    	smallest = "1";
	    	distCalc[0] = distCalc[1];
	    	smallest = 1;
		    something.poses[0].position.x=temp_st.pose.position.x ;
		    something.poses[1].position.x=temp_st1.pose.position.x ;
		    something.poses[2].position.x = temp_st2.pose.position.x;
		    something.poses[0].position.y=temp_st.pose.position.y ;
		    something.poses[1].position.y=temp_st1.pose.position.y ;
		    something.poses[2].position.y = temp_st2.pose.position.y;

	    }

		if ( distCalc[2] < distCalc[0])

		{
			smallest = "2";
			something.poses[0].position.x=temp_st1.pose.position.x ;
			something.poses[1].position.x=temp_st.pose.position.x ;
			something.poses[2].position.x = temp_st2.pose.position.x;
			something.poses[0].position.y=temp_st1.pose.position.y ;
			something.poses[1].position.y=temp_st.pose.position.y ;
			something.poses[2].position.y = temp_st2.pose.position.y;

		}

		ROS_INFO_STREAM(smallest);
	    twist_msgtest.linear.x =distToTarget[0];
	    twist_msgtest.linear.y =distToTarget[1];
	    twist_msgtest.linear.z =distToTarget[2];
	    twist_msgtest.angular.x =distToTarget[3];
	    twist_msgtest.angular.y =distToTarget[4];
	    twist_msgtest.angular.z =distToTarget[5];
	    firstGo =1;
}
pub_shortestPash.publish(something);
pub_twisttest.publish(twist_msgtest);
	//pose_to_target.header.stamp =  ros::Time::now();
	//pose_to_target.header.frame_id = "/map";
	//ros::Duration(1.5).sleep();
	if(pose_array->header.seq > 10){
		active = true;
		//ROS_INFO_STREAM(pose_to_target);
		//ros::Duration(1.5).sleep();
		//publishMarkerArray();
		//publishing PosedStamped of winning customer
		//pub_way_points.publish(pose_to_target);
//	}
	//if(currentWay < pose_array->poses.size())
	//{
	//	pose_to_target.pose = pose_array->poses[currentWay];

	//}
	//else
	//{
	//	pose_to_target.pose.position.x = 0.0;
	//	pose_to_target.pose.position.y = 0.0;
	//}
//	pub_way_points.publish(pose_to_target);

}
	}
// Timer to re-publish goal such that the global planner reacts to
// new local_costmap information
void goalCallback(const ros::TimerEvent& event)
{
	  if (valid_goal)
	  {
	    listener->transformPose("/map", ros::Time(0), pose_stamped, pose_stamped.frame_id_, pose_stamped);
	    //ROS_INFO("^^^^^^^^^^^^^^   GOOD GOAL");
	    geometry_msgs::PoseStamped robot_pose;
	    robot_pose.header.frame_id = "/map";
	    robot_pose.header.stamp = pose_stamped.stamp_;
	    tf::quaternionTFToMsg(pose_stamped.getRotation(), robot_pose.pose.orientation);
	    robot_pose.pose.position.x = pose_stamped.getOrigin().x();
	    robot_pose.pose.position.y = pose_stamped.getOrigin().y();
	    robot_pose.pose.position.z = pose_stamped.getOrigin().z();


	    global_planner->makePlan(robot_pose, current_goal, plan);




	    local_planner->setPlan(plan);
	  }
	 else
	  {
		//  ROS_INFO("bad goal******************************");
	  }
}


void recvOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_stamped.frame_id_ = msg->header.frame_id;
  pose_stamped.stamp_ = msg->header.stamp;

  tf::Vector3 vehicle_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  pose_stamped.setOrigin(vehicle_position);

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  pose_stamped.setRotation(q);
}

// Receive goal from Rviz and store in global variable
void recvGoal_new(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	valid_goal = true;
	current_goal = *msg;
	//ROS_INFO("In goal reception area $$$$$$$$$$$$$$$$$$$$$$$$$$$");
}


int main(int argc, char** argv)
{

	//turn off logging by uncommenting
	//ros::console::shutdown();

	// Init node and node handles
	ros::init(argc, argv, "local_planner");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//setGoal();
	// Instantiate classess
	listener = new tf::TransformListener;
	local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *listener);
	local_planner = new base_local_planner::TrajectoryPlannerROS("local_planner", listener, local_costmap);

	global_costmap = new costmap_2d::Costmap2DROS("global_costmap", *listener);
	global_planner = new navfn::NavfnROS("global_planner", global_costmap);
	  pub = n.advertise<std_msgs::Float64>("/topic_out", 1);
	// Subscribe to and advertise topics
	//ros::Subscriber sub_path = n.subscribe("/global_planner/navfn_planner/plan", 1, recvPath); //using navfn_global planner instead
	ros::Subscriber sub_goal_new = n.subscribe("/marker_waypoints", 1, recvGoal_new);
	ros::Subscriber sub_odom = n.subscribe("odom", 1, recvOdom);
	//ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1, recvGoal); //uncomment to have goal be rviz 2d goal
	//ros::Subscriber sub_customer = n.subscribe("/customer_array_markers", 1, recvCustomer);
	pub_twist = n.advertise<geometry_msgs::Twist>("/roundbot/cmd_vel", 1);
	pub_twisttest = n.advertise<geometry_msgs::Twist>("/test", 1);
	pub_shortestPash = n.advertise<geometry_msgs::PoseArray>("/shortest_waypoints", 1);
	//pub_goal = n.advertise<geometry_msgs::PoseStamped>("/global_planner/goal", 1);
	ros::Subscriber sub_customers = n.subscribe("/customer_position_array", 1, parseCustomers);
	// Set up timers
	double freq;
	pn.param("freq", freq, 20.0);
	ros::Timer control_timer = n.createTimer(ros::Duration(1.0 / freq), timerCallback);
	//ros::Timer control_timer_new = n.createTimer(ros::Duration(1.0 / freq), timerCallback_new);
	ros::Timer goal_timer = n.createTimer(ros::Duration(1.0), goalCallback);

	ros::spin();
}
