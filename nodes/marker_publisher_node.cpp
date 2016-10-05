// ROS
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "labrom_control/pid_simple.h"
#include "visualization_msgs/Marker.h"

geometry_msgs::Vector3 linear_position;

// Marker
visualization_msgs::Marker marker;

void quadrotorPositionCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  linear_position = *msg;
}

void initializeMarker()
{
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 1;
  marker.pose.position.y = 1;
  marker.pose.position.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
}

int main(int argc, char **argv)
{
  // Initialize ROS machine
  ros::init(argc, argv, "marker");

  // Nodes
  ros::NodeHandle node;

  // Publisher
  initializeMarker();
  ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscribers
  ros::Subscriber sub_quadrotor_linear_position = node.subscribe("linear_position", 1, quadrotorPositionCallback);

  // Loop rate
  int rate = 20;
  ros::Rate loop_rate(rate);

  controllers::pid::Simple cntlr("height_controller", 20, 0, 10, 0);

  while (ros::ok())
  {
    // Publish visualization marker
    marker.pose.position.x = linear_position.x;
    marker.pose.position.y = linear_position.y;
    marker.pose.position.z = linear_position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    vis_pub.publish(marker);

    // ROS stuff
    ros::spinOnce();
    loop_rate.sleep();
  }
}
