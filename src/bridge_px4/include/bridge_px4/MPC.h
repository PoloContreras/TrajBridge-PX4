
#ifndef __MPC_H__
#define __MPC_H__

#include "ros/ros.h"

extern "C" {
    #include "cpg_solve.h"
    #include "cpg_workspace.h"
    #include "workspace.h"
}

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <numeric>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace Eigen;
using namespace std;
using namespace std_msgs;

using OneD = vector<double>;
using TwoD = vector<vector<double>>;
using ThreeD = vector<vector<vector<double>>>;


class MPC
{
public:
  // Constructor
  MPC();
  virtual ~MPC();

  void controller(ros::Time t_start);

protected:
  ros::NodeHandle nh;

private:
  // Limit Check variables
  float x_box_lim_;
  float y_box_lim_;
  float z_box_lim_;
  float mass_;
  float setpoint_mode_;
  Matrix<float,10,1> del_slim;
  Matrix<float,7,1> err_lim;

  static const string states[];

  // ROS variables
  ros::Subscriber    pose_curr_sub;
  ros::Subscriber    vel_curr_sub;
  ros::Publisher     force_sp_pub;
  ros::Publisher     yaw_sp_pub;
  ros::Publisher     stats_pub;

  Matrix<float,17,1> x_bar;
  Matrix<float,17,1> x_curr;  // Current State
  Matrix<float,17,1> del_x;   // Current State Error Relative to Nominal

  vector<double> stats;    // [overall node time, spin time, read time, init time, ASA time, osqp time, publish time, number of iterations]

  float amp_x;
  float amp_y;
  float amp_z;
  float om_x;
  float om_y;
  float om_z;
  float ph_x;
  float ph_y;
  float ph_z;
  float off_x;
  float off_y;
  float off_z;

  float yaw_sp;
  float F_max;

  // Counter and Time Variables
  double t_dt;
  int k_main;
  bool main_switch;
  bool safety_switch;

  // Quad Setpoints
  geometry_msgs::Vector3Stamped force_sp_out;      // Setpoint Attitude (body rate, orientation, thrust) Out
  std_msgs::Float64 yaw_sp_out;
  std_msgs::Float64MultiArray stats_out;

  // Function(s)
  bool setpoint_update();
  bool limit_check();
  void pose_curr_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void vel_curr_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

};

#endif