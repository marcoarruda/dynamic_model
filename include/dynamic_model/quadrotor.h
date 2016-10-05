#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Vector3.h"

namespace dynamic_model
{
class Quadrotor
{
public:
  // linear state
  geometry_msgs::Vector3 position_;
  geometry_msgs::Vector3 linear_velocity_;
  geometry_msgs::Vector3 linear_acceleration_;

  // angular state
  geometry_msgs::Vector3 angles_;
  geometry_msgs::Vector3 angular_velocity_;
  geometry_msgs::Vector3 angular_acceleration_;

  // rotor speed
  float w_[4];

  Quadrotor();
  Quadrotor(float b, float d, float jr, float l, float m, geometry_msgs::Vector3 i);
  void setMotorSpeed(const std_msgs::Int32MultiArray::ConstPtr &msg);
  void calculateState(int update_rate);

private:
  int update_rate_;
  // gravity
  float g_;
  // thrust factor and drag factor
  float b_, d_;
  // rotor inertia
  float jr_;
  // lever
  float l_;
  // mass (Kg)
  float m_;
  // inertia moment (Kg.m^2)
  geometry_msgs::Vector3 i_;

protected:
  void setProperties(float b, float d, float jr, float l, float m, geometry_msgs::Vector3 i);
  void initializeState();
};
}
