// ROS
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Vector3.h"

// Libraries
#include "dynamic_model/quadrotor_pelican.h"
#include "dynamic_model/SetVector3.h"

dynamic_model::QuadrotorPelican pelican;

ros::Publisher pub_motor_speed;

void cmdMotorSpeedCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  pelican.setMotorSpeed(msg);
}

bool srvSetPos(dynamic_model::SetVector3::Request &req,
               dynamic_model::SetVector3::Response &res)
{
  pelican.position_.x = req.x;
  pelican.position_.y = req.y;
  pelican.position_.z = req.z;

  pelican.linear_velocity_.x = 0;
  pelican.linear_velocity_.y = 0;
  pelican.linear_velocity_.z = 0;

  return true;
}

bool srvSetAng(dynamic_model::SetVector3::Request &req,
               dynamic_model::SetVector3::Response &res)
{
  pelican.angles_.x = req.x;
  pelican.angles_.y = req.y;
  pelican.angles_.z = req.z;

  pelican.angular_velocity_.x = 0;
  pelican.angular_velocity_.y = 0;
  pelican.angular_velocity_.z = 0;

  return true;
}

int main(int argc, char **argv)
{
  // Initialize ROS machine
  ros::init(argc, argv, "pelican");

  // Nodes
  ros::NodeHandle node;

  // Services
  ros::ServiceServer srv_set_pos = node.advertiseService("srv_set_pos", srvSetPos);
  ros::ServiceServer srv_set_ang = node.advertiseService("srv_set_ang", srvSetAng);

  // Publishers
  ros::Publisher pub_lin_pos = node.advertise<geometry_msgs::Vector3>("linear_position", 1);
  ros::Publisher pub_lin_vel = node.advertise<geometry_msgs::Vector3>("linear_velocity", 1);
  ros::Publisher pub_lin_acc = node.advertise<geometry_msgs::Vector3>("linear_acceleration", 1);

  ros::Publisher pub_ang_pos = node.advertise<geometry_msgs::Vector3>("angular_position", 1);
  ros::Publisher pub_ang_vel = node.advertise<geometry_msgs::Vector3>("angular_velocity", 1);
  ros::Publisher pub_ang_acc = node.advertise<geometry_msgs::Vector3>("angular_acceleration", 1);

  ros::Publisher pub_motor_speed = node.advertise<std_msgs::Int32MultiArray>("motor_speed", 1);

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

    std_msgs::Int32MultiArray motor_speed_array;
    motor_speed_array.data.clear();
    for (int i = 0; i < 4; i++)
    {
      motor_speed_array.data.push_back(pelican.w_[i]);
    }
    pub_motor_speed.publish(motor_speed_array);

    // ROS routines
    ros::spinOnce();
    loop_rate.sleep();
  }
}
