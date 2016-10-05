#include "dynamic_model/quadrotor.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace dynamic_model
{
Quadrotor::Quadrotor()
{
    g_ = 9.81;
    initializeState();
};
Quadrotor::Quadrotor(float b, float d, float jr, float l, float m, geometry_msgs::Vector3 i)
{
    b_ = b;
    d_ = d;
    jr_ = jr;
    l_ = l;
    m_ = m;
    i_.x = i.x;
    i_.y = i.y;
    i_.z = i.z;
    initializeState();
};
void Quadrotor::initializeState()
{
    position_.x = 0;
    position_.y = 0;
    position_.z = 0;

    linear_velocity_.x = 0;
    linear_velocity_.y = 0;
    linear_velocity_.z = 0;

    linear_acceleration_.x = 0;
    linear_acceleration_.y = 0;
    linear_acceleration_.z = 0;

    angles_.x = 0;
    angles_.y = 0;
    angles_.z = 0;

    angular_velocity_.x = 0;
    angular_velocity_.y = 0;
    angular_velocity_.z = 0;

    angular_acceleration_.x = 0;
    angular_acceleration_.y = 0;
    angular_acceleration_.z = 0;
};
void Quadrotor::setProperties(float b, float d, float jr, float l, float m, geometry_msgs::Vector3 i)
{
    b_ = b;
    d_ = d;
    jr_ = jr;
    l_ = l;
    m_ = m;
    i_ = i;
};
void Quadrotor::setMotorSpeed(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < 4; i++)
    {
        if (msg->data[i] > 200)
        {
            w_[i] = 200;
        }
        else if (msg->data[i] < 0)
        {
            w_[i] = 0;
        }
        else
        {
            w_[i] = msg->data[i];
        }
    }
};
void Quadrotor::calculateState(int update_rate)
{
    position_.x += linear_velocity_.x / update_rate;
    position_.y += linear_velocity_.y / update_rate;
    position_.z += linear_velocity_.z / update_rate;

    linear_velocity_.x += linear_acceleration_.x / update_rate;
    linear_velocity_.y += linear_acceleration_.y / update_rate;
    linear_velocity_.z += linear_acceleration_.z / update_rate;

    angles_.x += angular_velocity_.x / update_rate;
    angles_.y += angular_velocity_.y / update_rate;
    angles_.z += angular_velocity_.z / update_rate;

    angular_velocity_.x += angular_acceleration_.x / update_rate;
    angular_velocity_.y += angular_acceleration_.y / update_rate;
    angular_velocity_.z += angular_acceleration_.z / update_rate;

    float w1 = w_[0], w2 = w_[1], w3 = w_[2], w4 = w_[3];
    float u1 = b_ * (pow(w1, 2) + pow(w2, 2) + pow(w3, 2) + pow(w4, 2));
    float u2 = b_ * (pow(w4, 2) - pow(w2, 2));
    float u3 = b_ * (pow(w3, 2) - pow(w1, 2));
    float u4 = d_ * (pow(w2, 2) + pow(w4, 2) - pow(w1, 2) - pow(w3, 2));
    float w = w2 + w4 - w1 - w3;

    linear_acceleration_.x = (u1 / m_) * (cos(angles_.x) * sin(angles_.y) * cos(angles_.z) + sin(angles_.x) * sin(angles_.z));
    linear_acceleration_.y = (u1 / m_) * (cos(angles_.x) * sin(angles_.y) * sin(angles_.z) - sin(angles_.x) * cos(angles_.z));
    linear_acceleration_.z = -g_ + (u1 / m_) * (cos(angles_.x) * cos(angles_.y));

    angular_acceleration_.x = (angular_velocity_.y * angular_velocity_.z * ((i_.y - i_.z) / i_.x)) - ((jr_ / i_.x) * angular_velocity_.y * w) + ((l_ / i_.x) * u2);
    angular_acceleration_.y = (angular_velocity_.x * angular_velocity_.z * ((i_.z - i_.x) / i_.y)) - ((jr_ / i_.y) * angular_velocity_.x * w) + ((l_ / i_.y) * u3);
    angular_acceleration_.z = (angular_velocity_.x * angular_velocity_.y * ((i_.x - i_.y) / i_.z)) + ((1 / i_.z) * u4);
};
}