// ROS
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Vector3.h"

// Libraries
#include "dynamic_model/quadrotor_pelican.h"

dynamic_model::QuadrotorPelican pelican;

void cmdMotorSpeedCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  pelican.setMotorSpeed(msg);
}

int main(int argc, char **argv)
{
  // Initialize ROS machine
  ros::init(argc, argv, "pelican");

  // Nodes
  ros::NodeHandle node;

  // Publishers
  ros::Publisher pub_lin_pos = node.advertise<geometry_msgs::Vector3>("linear_position", 1);
  ros::Publisher pub_lin_vel = node.advertise<geometry_msgs::Vector3>("linear_velocity", 1);
  ros::Publisher pub_lin_acc = node.advertise<geometry_msgs::Vector3>("linear_acceleration", 1);

  ros::Publisher pub_ang_pos = node.advertise<geometry_msgs::Vector3>("angular_position", 1);
  ros::Publisher pub_ang_vel = node.advertise<geometry_msgs::Vector3>("angular_velocity", 1);
  ros::Publisher pub_ang_acc = node.advertise<geometry_msgs::Vector3>("angular_acceleration", 1);

  // Subscribers
  ros::Subscriber sub_cmd_motor_speed = node.subscribe("cmd_motor_speed", 1, cmdMotorSpeedCallback);

  // Loop rate
  int rate = 1000;
  ros::Rate loop_rate(rate);

  // Dynamic model instance
  pelican = dynamic_model::QuadrotorPelican();

  while (ros::ok())
  {
    pelican.calculateState(rate);
    // Publish
    pub_lin_pos.publish(pelican.position_);
    pub_lin_vel.publish(pelican.linear_velocity_);
    pub_lin_acc.publish(pelican.linear_acceleration_);
    pub_ang_pos.publish(pelican.angles_);
    pub_ang_vel.publish(pelican.angular_velocity_);
    pub_ang_acc.publish(pelican.angular_acceleration_);

    // ROS routines
    ros::spinOnce();
    loop_rate.sleep();
  }
}
