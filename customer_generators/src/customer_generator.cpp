
#include <ros/ros.h>

#include <base_local_planner/trajectory_planner_ros.h>
#include <tf/transform_listener.h>
#include <navfn/navfn_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

//----------------------------------------end includes
/*
 * c1:x1, cy1
 * c2:x2, cy2
 * .
 * .
 * .
 * cn:xn, cyn
 *
 *
 */
using namespace std;
ros::Publisher publish_customers;
vector<vector<int> > c_container;



//Timer to publish all customer positions which the manager will use to generate its path planner
void timerCallback(const ros::TimerEvent &event)
{
	geometry_msgs::PoseArray customer_pose_array;
	geometry_msgs::Pose customer_pose;

	int position = 0;

	for (int i = 0; i < c_container.size(); i++)
	{
		customer_pose_array.header.frame_id = "/map";
		customer_pose_array.header.stamp = event.current_real;

		for (int j = 0; j < c_container[i].size(); j++)
		{
			customer_pose.position.z = 0;
			customer_pose.orientation.w = 1;
			customer_pose.orientation.x = 0;
			customer_pose.orientation.y = 0;
			customer_pose.orientation.z = 0;
			//ROS_INFO_STREAM(*col << "Heres some info ^^**^^__");
			//ROS_INFO_STREAM("Heres some info ^^**^^__" << *col<<" "<<  "  " );
			//ROS_INFO_STREAM(c_container[i][j]);
			position = c_container[i][j];
			if( (j % 2) == 0)
			{
				customer_pose.position.x = position;
				//ROS_INFO_STREAM(customer_pose.pose.position.x << "Heres some info ^^**^^__");
			}
			else
			{
				customer_pose.position.y = position;
				//ROS_INFO_STREAM(customer_pose.pose.position.y << "  ^^**^^__ Heres some info ");
			}

		}
		customer_pose_array.poses.push_back(customer_pose);

		//ROS_INFO_STREAM(customer_pose << "** seeing something yet **__**");
	}
	publish_customers.publish(customer_pose_array);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "generating_customers_you_know");

  ros::NodeHandle n("~");

  std::vector<int> p1, p2, p3;
  n.getParam("p1", p1);
  n.getParam("p2", p2);
  n.getParam("p3", p3);
  c_container.push_back(p1);
  c_container.push_back(p2);
  c_container.push_back(p3);	//should find a way to generalize this not just push 3 everytime
  ROS_INFO("P1: (%i, %i)", p1[0], p1[1]);
  ROS_INFO("P2: (%i, %i)", p2[0], p2[1]);
  ROS_INFO("P3: (%i, %i)", p3[0], p3[1]);
  publish_customers = n.advertise<geometry_msgs::PoseArray>("/customer_position_array", 1);
  ros::Timer control_timer = n.createTimer(ros::Duration(1.0/2), timerCallback);
  ros::spin();
}
