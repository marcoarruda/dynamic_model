// ROS
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "labrom_control/pid_simple.h"

geometry_msgs::Vector3 linear_position;
geometry_msgs::Vector3 linear_velocity;

void quadrotorPositionCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  linear_position = *msg;
}
void quadrotorVelocityCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  linear_velocity = *msg;
}

int main(int argc, char **argv)
{
  // Initialize ROS machine
  ros::init(argc, argv, "controller");

  // Nodes
  ros::NodeHandle node;

  // Publishers
  std_msgs::Int32MultiArray motor_speed_array;
  ros::Publisher pub_motor_speed = node.advertise<std_msgs::Int32MultiArray>("cmd_motor_speed", 1);

  // Subscribers
  ros::Subscriber sub_quadrotor_linear_position = node.subscribe("linear_position", 1, quadrotorPositionCallback);
  ros::Subscriber sub_quadrotor_linear_velocity = node.subscribe("linear_velocity", 1, quadrotorVelocityCallback);

  // Loop rate
  int rate = 50;
  ros::Rate loop_rate(rate);

  controllers::pid::Simple cntlr("height_controller", 20, 0, 10, 0);

  while (ros::ok())
  {
    int motor_speed = 65 + cntlr.LoopOnce(0, linear_position.z, 0, linear_velocity.z);
    // ... push values to array
    motor_speed_array.data.clear();
    for (int i = 0; i < 4; i++)
    {
      motor_speed_array.data.push_back(motor_speed);
    }
    // ... and publish
    pub_motor_speed.publish(motor_speed_array);

    // ROS stuff
    ros::spinOnce();
    loop_rate.sleep();
  }
}
