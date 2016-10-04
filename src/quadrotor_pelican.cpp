#include "dynamic_model/quadrotor_pelican.h"

namespace dynamic_model
{
QuadrotorPelican::QuadrotorPelican() : Quadrotor::Quadrotor()
{
  float b = 0.001;
  float d = 0.001;
  float jr = 1;
  float l = 0.3;
  float m = 2;
  geometry_msgs::Vector3 i;
  i.x = 0.1;
  i.y = 0.1;
  i.z = 0.2;
  
  setProperties(b, d, jr, l, m, i);
}
}