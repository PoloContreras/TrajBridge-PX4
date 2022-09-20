#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher tracker_pub = n.advertise<geometry_msgs::Pose2D>("tracker", 1000);

  ros::Rate loop_rate(10);

  int count = 0;

  float trajectory[3][11] = {{0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0},
                            {0.0,-1.45,-2.68,-3.80,-5.07,-6.54,-7.96,-9.16,-10.29,-11.60,-13.08},
                            {0.0,0.34,1.19,2.19,2.97,3.21,2.76,1.87,00.88,00.17,00.05}};

  ros::Time t_start = ros::Time::now(); 

  while (ros::ok())
  {
    ros::Time t_now = ros::Time::now();

    geometry_msgs::Pose2D msg;

    /* msg.x = trajectory[1][count]; //Position sourced from the list of points in the prerecorded trajectory
    msg.y = trajectory[2][count]; */

    msg.x = 0.0; //Target location is always at the point of origin (specified by hard-coded latitude and longitude)
    msg.y = 0.0;

    tracker_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    if ((t_now - t_start > ros::Duration(trajectory[0][count])) && (count < 10)) {
        ++count;
    }
  }

  return 0;
}