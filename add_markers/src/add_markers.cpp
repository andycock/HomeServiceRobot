#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "nav_msgs/Odometry.h"

using namespace std;

double robot_x;
double robot_y;
nav_msgs::Odometry pose_msg;

void od_call(const nav_msgs::Odometry::ConstPtr& odom_msg){
  pose_msg = *odom_msg;
  robot_x = pose_msg.pose.pose.position.x;
  robot_y = pose_msg.pose.pose.position.y;
}


int main( int argc, char** argv )
{
  bool robotOnPickUp = false;
  bool robotOnKickOff = false;

  double pickUp_x = 3.0;
  double pickUp_y = 0.5;
  double kickOff_x = 1.0;
  double kickOff_y = 3.0;
  
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  ros::Subscriber odom_subscriber;
  odom_subscriber = n.subscribe("/odom", 10, od_call);
  
    
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickUp_x;
  marker.pose.position.y = pickUp_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
   ros::Duration(5.0).sleep();

   marker.action = visualization_msgs::Marker::DELETE;
   // Publish the marker
     while (marker_pub.getNumSubscribers() < 1)
     {
       if (!ros::ok())
       {
         return 0;
       }
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }
     marker_pub.publish(marker);

   ros::Duration(5.0).sleep();
   marker.pose.position.x = 1.0;
   marker.pose.position.y = 3.0;
   marker.action = visualization_msgs::Marker::ADD;
   // Publish the marker
     while (marker_pub.getNumSubscribers() < 1)
     {
       if (!ros::ok())
       {
         return 0;
       }
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }
     marker_pub.publish(marker);


  while(ros::ok())
  {
    if( abs(robot_x - kickOff_x) < 0.1 && abs(robot_y - kickOff_y) < 0.1)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }
      marker_pub.publish(marker);
    }

  }


  return 0;
}