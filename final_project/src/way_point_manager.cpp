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
#include <dynamic_reconfigure/server.h>
using namespace visualization_msgs;



ros::Publisher pub_Waypoints;
ros::Publisher pub_markers;
ros::Publisher pub_path;
ros::Publisher pub_currentPoint;
//ros::Publisher pub_yaw;

UTMCoords ref_utm;
MarkerArray marker_msg;
std_msgs::UInt16 currentWays;
uint currentWay;
tf::StampedTransform transform;
nav_msgs::Path path_msg;
std_msgs::Float64MultiArray currentMarker;

//new position marker to nav_stack_example
geometry_msgs::PoseStamped currentMark;
bool onetimer = true;
double xTarget;
	double yTarget;
	double epsilon ;
	double xCurrent ;
	double yCurrent ;
	double x_dif ;
	double y_dif ;
	double distToMark ;

void publishMarkerArray()
{

	Marker new_marker;

	//ROS_INFO("oh yeah");
	new_marker.header.frame_id = "/map";
	new_marker.header.stamp = ros::Time::now();
	new_marker.action = Marker::ADD;
	new_marker.type = Marker::CYLINDER;
	new_marker.id = 0;
	transform.setOrigin(tf::Vector3(0, 0, 0));

	double ref_lat = 42.853452;
	double ref_lon = -83.069858;
	double latArray [9] = { 42.851358, 42.851383, 42.852443, 42.852021, 42.851525, 0, 42.851344, 42.850836, 42.849644};
	double longArray [9] = { -83.069485, -83.069007, -83.068013, -83.066888, -83.067044, 0, -83.066344, -83.066440, -83.066060};


	double currentLat;
	double currentLong;
	UTMCoords refUTM(LatLon(ref_lat, ref_lon, 0));
	double newX, newY, newZ;

	for(int aa = 0; aa < 9; aa++){


		if(aa == 0){

			new_marker.color.a = 1;
			new_marker.color.r = 1.0;
			new_marker.color.g = 1.0;
			new_marker.color.b = 1.0;
			new_marker.scale.x = 1.0;
			new_marker.scale.y = 1.0;
			new_marker.scale.z = 1.0;

			new_marker.pose.position.x = 1.4;
			new_marker.pose.position.y = 7.2;
			new_marker.pose.position.z = 0.5;

			new_marker.pose.orientation.x = 0;
			new_marker.pose.orientation.y = 0;
			new_marker.pose.orientation.z = 0;
			new_marker.pose.orientation.w = 1;

			new_marker.scale.x = 1.0;
			new_marker.scale.y = 1.0;
			new_marker.scale.z = 1.0;

			marker_msg.markers.push_back(new_marker);
			new_marker.header.frame_id = "/map";
			new_marker.header.stamp = ros::Time::now();
			new_marker.action = Marker::ADD;
			new_marker.type = Marker::CYLINDER;
			new_marker.id++;
			new_marker.color.a = 1;
			new_marker.color.r = 1.0;
			new_marker.color.g = 1.0;
			new_marker.color.b = 1.0;

			new_marker.pose.position.x = -5.0;
			new_marker.pose.position.y = 6.4;
			new_marker.pose.position.z = 0.5;

			new_marker.pose.orientation.x = 0;
			new_marker.pose.orientation.y = 0;
			new_marker.pose.orientation.z = 0;
			new_marker.pose.orientation.w = 1;
			new_marker.scale.x = 1.0;
			new_marker.scale.y = 1.0;
			new_marker.scale.z = 1.0;

			marker_msg.markers.push_back(new_marker);
			new_marker.header.frame_id = "/map";
			new_marker.header.stamp = ros::Time::now();
			new_marker.action = Marker::ADD;
			new_marker.type = Marker::CYLINDER;
			new_marker.id++;
			new_marker.color.a = 1;
			new_marker.color.r = 1.0;
			new_marker.color.g = 1.0;
			new_marker.color.b = 1.0;

			new_marker.pose.position.x = -17.0;
			new_marker.pose.position.y = 5.4;
			new_marker.pose.position.z = 0.5;
			new_marker.pose.orientation.x = 0;
			new_marker.pose.orientation.y = 0;
			new_marker.pose.orientation.z = 0;
			new_marker.pose.orientation.w = 1;

			marker_msg.markers.push_back(new_marker);
			ROS_INFO_STREAM(marker_msg);
		}


		//new_marker.id++;

		currentMark.pose.position.x = marker_msg.markers[0].pose.position.x;
		currentMark.pose.position.y = marker_msg.markers[0].pose.position.y;
		currentMark.pose.position.z = 0;
		currentMark.header = new_marker.header;
		newX = new_marker.pose.position.x;
		newY = new_marker.pose.position.y;
		newZ = new_marker.pose.position.z;

	}
	currentWay = 0;
		//printf("this is x: %G \n y: %G \n z: %G \n", newX, newY, newZ);
}
/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
*/

void timerCallback(const ros::TimerEvent& event)
{

//	static tf::TransformBroadcaster broadcaster;
//	transform.stamp_ = event.current_real;
//	broadcaster.sendTransform(transform);
	for (int i=0; i<marker_msg.markers.size(); i++)
	{
		marker_msg.markers[i].header.stamp = event.current_expected;
	}
	//if(onetimer){
	//	onetimer = false;
	//publishMarkerArray();
	//pub_Waypoints.publish(marker_msg);
	//}
	//pub_currentPoint.publish(currentWays);
	//pub_Waypoints.publish(wayOrray);
	pub_markers.publish(marker_msg);
	ROS_INFO_STREAM(marker_msg);
}

void recvCurrentPos(const nav_msgs::Odometry::ConstPtr& msg)
{
	pub_currentPoint.publish(currentWays);
	xTarget = currentMark.pose.position.x;
	yTarget = currentMark.pose.position.y;
	epsilon = 1;
	xCurrent = msg->pose.pose.position.x;
	yCurrent = msg->pose.pose.position.y;
	//pub_Waypoints.publish(currentMark);
	//ROS_INFO("marker %d at x: %G and y: %G \n", currentWay, xTarget, yTarget);

	// find the difference
	x_dif = pow((xTarget -xCurrent), 2);
	y_dif = pow((yTarget - yCurrent), 2);
	 	 //use differnece to find distance
	distToMark = sqrt(x_dif + y_dif);

	//if pint hit, incremnt x (tracks current way point)
	if (distToMark <= 0.8)
	{
		currentWay++;
		pub_currentPoint.publish(currentWays);
		currentMark.pose.position.x = marker_msg.markers[currentWay].pose.position.x;
		currentMark.pose.position.y = marker_msg.markers[currentWay].pose.position.y;

		//ROS_INFO("marker %d at x: %G and y: %G \n", currentWay, xTarget, yTarget);
		pub_Waypoints.publish(currentMark);
		//currentWays.data = currentWay;
		//pub_currentPoint.publish(currentWays);
	}



}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Customer");
	ros::NodeHandle n;

	int number_of_customers = 1;
//	transform.frame_id_ = "/map";
//	transform.child_frame_id_ = "/base_footprint";
	ros::Timer timer = n.createTimer(ros::Duration(.2),timerCallback);
	publishMarkerArray();

	//initialize the publishers
	pub_markers = n.advertise<visualization_msgs::MarkerArray>("/marker_array", 1);		//for rviz
	pub_Waypoints = n.advertise<geometry_msgs::PoseStamped>("/marker_waypoints", 1);	//for control alg.
	pub_currentPoint = n.advertise<std_msgs::UInt16>("/current_point", 1);

	//initialize the subscribers
	ros::Subscriber sub_waypoints = n.subscribe("/odom", 1, recvCurrentPos);


	ros::spin();
}


