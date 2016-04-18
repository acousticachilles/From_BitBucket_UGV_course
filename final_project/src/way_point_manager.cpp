/*
 * sim_command.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: dlebowski
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <visualization_msgs/MarkerArray.h>

#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ugv_course_libs/gps_conv.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>



using namespace visualization_msgs;


ros::Publisher pub_Waypoints;
ros::Publisher pub_markers;
ros::Publisher pub_path;
ros::Publisher pub_currentPoint;
ros::Publisher pub_way_points;

UTMCoords ref_utm;
MarkerArray marker_msg;
std_msgs::UInt16 currentWays;
std_msgs::UInt16 currentWays2;


uint currentWay = 0;
uint currentWay2 = 0;
//tf::StampedTransform transform;
nav_msgs::Path path_msg;
std_msgs::Float64MultiArray currentMarker;

//new position marker to nav_stack_example
geometry_msgs::PoseStamped currentMark;
geometry_msgs::PoseStamped currentMark2;
geometry_msgs::PoseArray customer_poses;
geometry_msgs::PoseStamped pose_to_target;

bool active = false;
double xTarget;
double yTarget;
double epsilon ;
double xCurrent ;
double yCurrent ;
double xCurrent1 ;
double yCurrent1 ;
double x_dif ;
double y_dif ;
long double distToTarget;
double distToMark;
int h = 0;



/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
*/

void timerCallback(const ros::TimerEvent& event)
{
	if(active){

	//	static tf::TransformBroadcaster broadcaster;
	//	transform.stamp_ = event.current_real;
	//	broadcaster.sendTransform(transform);
		for (int i=0; i<marker_msg.markers.size(); i++)
		{
		//	marker_msg.markers[i].header.stamp = event.current_expected;
		}
		//pub_markers.publish(marker_msg);
		//ROS_INFO_STREAM(marker_msg);

	}
}

void recvCurrentPos(const nav_msgs::Odometry::ConstPtr& msg)
{
	//pub_currentPoint.publish(currentWays);
	if (active)
	{
		xTarget = customer_poses.poses[currentWay].position.x;
		yTarget = customer_poses.poses[currentWay].position.y;
		epsilon = 1;
		xCurrent = msg->pose.pose.position.x;
		yCurrent = msg->pose.pose.position.y;
		//pub_Waypoints.publish(currentMark);
		//ROS_INFO("marker %d at x: %G and y: %G \n", currentWay, xTarget, yTarget);

		// find the difference
		x_dif = pow((xTarget -xCurrent), 2);
		y_dif = pow((yTarget - yCurrent), 2);

	    //use difference to find distance
		distToMark = sqrt(x_dif + y_dif);

		//if pint hit, increment x (tracks current way point)
		if (distToMark <= 0.8)
		{
			currentWay++;
			//pub_currentPoint.publish(currentWays);
			if(currentWay < customer_poses.poses.size())
			{
				currentMark.pose.position.x = customer_poses.poses[currentWay].position.x;
				currentMark.pose.position.y = customer_poses.poses[currentWay].position.y;
			}
			else
			{
				//currentWay--;
				currentMark.pose.position.x = 0.0;
				currentMark.pose.position.y = 0.0;

			}
			//ROS_INFO("marker %d at x: %G and y: %G \n", currentWay, xTarget, yTarget);
			//pub_way_points.publish(currentMark);
			//currentWays.data = currentWay;
			//pub_currentPoint.publish(currentWays);
		}
	}



}


void parseCustomers(const geometry_msgs::PoseArray::ConstPtr  &pose_array)
{

	/**
	 *
	 *
	 * PUT SOME FUNCTIONALITY IN HERE TO DETERMINE WHICH POSITION IS THE RIGHT CHOICE
	 * 			EVENTUALLY THE WINNER WILL END UP BEING PUBLISHED IN
	 * 						pub_way_points.publish(winning_customer);
	 * 						or something
	 *
	 * 							but for now just get the second one...
	 *
	 * 							currentWay could be used as pointer to the next customer to go to
	 */
	customer_poses = *pose_array;
	pose_to_target.header.stamp =  ros::Time::now();
	pose_to_target.header.frame_id = "/map";
	//ros::Duration(1.5).sleep();
	if(pose_array->header.seq > 10){
		active = true;
		//ROS_INFO_STREAM(pose_to_target);
		//ros::Duration(1.5).sleep();
		//publishMarkerArray();
		//publishing PosedStamped of winning customer
		pub_way_points.publish(pose_to_target);
	}
	if(currentWay < customer_poses.poses.size())
	{
		pose_to_target.pose = pose_array->poses[currentWay];

	}
	else
	{
		pose_to_target.pose.position.x = 0.0;
		pose_to_target.pose.position.y = 0.0;
	}
	pub_way_points.publish(pose_to_target);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "manage_way_points");
	ros::NodeHandle n;

	int number_of_customers = 1;
//	transform.frame_id_ = "/map";
//	transform.child_frame_id_ = "/base_footprint";



	//initialize the publishers
	pub_markers = n.advertise<visualization_msgs::MarkerArray>("/marker_array", 1);		//for rviz
	pub_Waypoints = n.advertise<geometry_msgs::PoseStamped>("/marker_waypo", 1);	//for control alg.  old
	pub_currentPoint = n.advertise<std_msgs::UInt16>("/current_point", 1);
	pub_way_points = n.advertise<geometry_msgs::PoseStamped>("/marker_waypoints", 1);

	//initialize the subscribers
	ros::Subscriber sub_waypoints = n.subscribe("/odom", 1, recvCurrentPos);
	ros::Subscriber sub_customers = n.subscribe("/shortest_waypoints", 1, parseCustomers);
	// publish_customers = n.advertise<geometry_msgs::PoseArray>("/customer_position_array", 1);

	ros::Timer timer = n.createTimer(ros::Duration(1),timerCallback);
	ros::spin();
}


