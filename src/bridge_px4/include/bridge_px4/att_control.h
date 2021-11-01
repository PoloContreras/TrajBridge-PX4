
#ifndef __ATT_CONTROL_H__
#define __ATT_CONTROL_H__

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Dense>
#include <numeric>
#include <math.h>
#include <stdio.h>

using namespace Eigen;
using namespace std;
using namespace std_msgs;

using OneD = vector<double>;
using TwoD = vector<vector<double>>;
using ThreeD = vector<vector<vector<double>>>;


class Att_Control
{
public:
  // Constructor
  Att_Control();
  virtual ~Att_Control();

  void controller();

protected:
  ros::NodeHandle nh;

private:

  float max_thrust_;

  // ROS variables
  ros::Subscriber    pose_curr_sub;
  ros::Subscriber    vel_curr_sub;
  ros::Subscriber    force_sub;
  ros::Subscriber    yaw_sub;
  ros::Publisher     att_sp_pub;

  Matrix<float,17,1> x_curr;  // Current State

  Matrix<float,3,1>  force_sp;
  Matrix<float,3,1>  K_force_sp;
  Matrix<float,3,1>  L_force_sp;
  float              roll_sp;
  float              pitch_sp;
  float              yaw_sp;
  float              qw;
  float              qx;
  float              qy;
  float              qz;
  Matrix<float,4,1>  q_sp;
  Matrix<float,4,1>  q_err;
  float              tau;
  float              sign;

  // Counter and Time Variables
  ros::Time          force_sp_time;
  double             expired_time;
  bool               att_control_switch;

  // Quad Setpoints
  mavros_msgs::AttitudeTarget att_sp_out;      // Setpoint Attitude (body rate, orientation, thrust) Out

  // Function(s)
  void pose_curr_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void vel_curr_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void force_sp_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void yaw_sp_cb(const std_msgs::Float64::ConstPtr& msg);

};

#endif