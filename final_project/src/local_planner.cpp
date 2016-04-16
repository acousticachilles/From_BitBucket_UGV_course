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

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Class pointers
base_local_planner::TrajectoryPlannerROS* local_planner;
navfn::NavfnROS* global_planner;
costmap_2d::Costmap2DROS* local_costmap;
tf::TransformListener* listener;
costmap_2d::Costmap2DROS* global_costmap;
// Publishers
ros::Publisher pub_twist;
ros::Publisher pub_goal;
tf::Stamped<tf::Pose> pose_stamped;
// Current goal set by Rviz
geometry_msgs::PoseStamped current_goal;
//std_msgs::Float64MultiArray current_goal;

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

	    std::vector<geometry_msgs::PoseStamped> plan;
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

	// Subscribe to and advertise topics
	//ros::Subscriber sub_path = n.subscribe("/global_planner/navfn_planner/plan", 1, recvPath); //using navfn_global planner instead
	ros::Subscriber sub_goal_new = n.subscribe("/marker_waypoints", 1, recvGoal_new);
	ros::Subscriber sub_odom = n.subscribe("odom", 1, recvOdom);
	//ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1, recvGoal); //uncomment to have goal be rviz 2d goal
	//ros::Subscriber sub_customer = n.subscribe("/customer_array_markers", 1, recvCustomer);
	pub_twist = n.advertise<geometry_msgs::Twist>("/roundbot/cmd_vel", 1);
	//pub_goal = n.advertise<geometry_msgs::PoseStamped>("/global_planner/goal", 1);

	// Set up timers
	double freq;
	pn.param("freq", freq, 20.0);
	ros::Timer control_timer = n.createTimer(ros::Duration(1.0 / freq), timerCallback);
	//ros::Timer control_timer_new = n.createTimer(ros::Duration(1.0 / freq), timerCallback_new);
	ros::Timer goal_timer = n.createTimer(ros::Duration(1.0), goalCallback);

	ros::spin();
}
