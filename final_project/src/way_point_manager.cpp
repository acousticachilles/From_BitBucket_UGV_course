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

#include <ugv_course_libs/gps_conv.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
using namespace visualization_msgs;



ros::Publisher pub_Waypoints;
//ros::Publisher pub_markers;
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


			if(aa > 50){
				new_marker.color.a = 0;
				new_marker.color.r = 1.0;
				new_marker.color.g = 0.0;
				new_marker.color.b = 0.0;
				new_marker.pose.position.x = 270.0;
				new_marker.pose.position.y = -190;
				new_marker.pose.position.z = 0;
			}
			else if(aa == 0){

				new_marker.color.a = 1;
				new_marker.color.r = 1.0;
				new_marker.color.g = 1.0;
				new_marker.color.b = 1.0;

				new_marker.pose.position.x = -5.0;
				new_marker.pose.position.y = -2.0;
				new_marker.pose.position.z = 0.5;

			}
			else if(aa == 10){
				UTMCoords currentUTM(LatLon(latArray[aa], longArray[aa], 0));
				tf::Vector3 rel_pos_utm = currentUTM - refUTM;
				new_marker.color.a = 1;
				new_marker.color.r = 1.0;
				new_marker.color.g = 1.0;
				new_marker.color.b = 0.0;

				new_marker.pose.position.x = rel_pos_utm.x();
				new_marker.pose.position.y = rel_pos_utm.y();
				new_marker.pose.position.z = rel_pos_utm.z();

			}
			new_marker.scale.x = 2.0;
			new_marker.scale.y = 2.0;
			new_marker.scale.z = 1.8;

			marker_msg.markers.push_back(new_marker);
			new_marker.type = Marker::CYLINDER;
			new_marker.id++;

			currentMark.pose.position.x = new_marker.pose.position.x;
			currentMark.pose.position.y = new_marker.pose.position.y;
			currentMark.pose.position.z = 0;
			currentMark.header = new_marker.header;
			newX = new_marker.pose.position.x;
			newY = new_marker.pose.position.y;
			newZ = new_marker.pose.position.z;

	}
	currentMarker.data.resize(2);
	currentMarker.data[0] = marker_msg.markers[0].pose.position.x;
	currentMarker.data[1] = marker_msg.markers[0].pose.position.y;
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

	//static tf::TransformBroadcaster broadcaster;
	//transform.stamp_ = event.current_real;
	//broadcaster.sendTransform(transform);
	for (int i=0; i<marker_msg.markers.size(); i++){
		marker_msg.markers[i].header.stamp = ros::Time::now();
	}
	//if(onetimer){
	//	onetimer = false;
		publishMarkerArray();
		pub_Waypoints.publish(currentMark);
	//}

	//pub_Waypoints.publish(wayOrray);
	//pub_markers.publish(marker_msg);

}

void recvCurrentPos(const std_msgs::Float64MultiArray::ConstPtr& currentPos){
	double xTarget = currentMarker.data[0];
	double yTarget = currentMarker.data[1];
	double epsilon = 1;
	double xCurrent = currentPos->data[0];
	double yCurrent = currentPos->data[1];
	pub_Waypoints.publish(currentMark);
	/*if( ( fabs(xTarget - xCurrent) < 1) && ( fabs(yTarget - yCurrent) < 1)){
		currentWay++;
		currentMarker.data[0] = marker_msg.markers[currentWay].pose.position.x;
		currentMarker.data[1] = marker_msg.markers[currentWay].pose.position.y;
		//ROS_INFO("marker %d at x: %G and y: %G \n", currentWay, xTarget, yTarget);
		pub_Waypoints.publish(currentMarker);
		currentWays.data = currentWay;
		pub_currentPoint.publish(currentWays);
	}
	*/
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Customer");
	ros::NodeHandle n;

	int number_of_customers = 1;
	transform.frame_id_ = "/world";
	transform.child_frame_id_ = "markering";
	ros::Timer timer = n.createTimer(ros::Duration(.1),timerCallback);


	for(int i = 0; i < number_of_customers; i++){
		//initialize the publishers
		//pub_markers = n.advertise<visualization_msgs::MarkerArray>("/marker_array", 1);		//for rviz
		pub_Waypoints = n.advertise<geometry_msgs::PoseStamped>("/marker_waypoints", 1);	//for control alg.
		pub_currentPoint = n.advertise<std_msgs::UInt16>("/current_point", 1);

		//initialize the subscribers
		ros::Subscriber sub_waypoints = n.subscribe("/current_position", 1, recvCurrentPos);
	}



	ros::spin();
}


