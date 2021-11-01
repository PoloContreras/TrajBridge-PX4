#include <bridge_px4/att_control.h>

Att_Control::Att_Control()
:   max_thrust_(21.1546)
{
    ros::param::get("~max_thrust", max_thrust_);

    // ROS Initialization
    pose_curr_sub = nh.subscribe("mavros/local_position/pose",1,&Att_Control::pose_curr_cb,this);
    vel_curr_sub = nh.subscribe("mavros/local_position/velocity_local",1,&Att_Control::vel_curr_cb,this);    
    force_sub = nh.subscribe("setpoint/force",1,&Att_Control::force_sp_cb,this);
    yaw_sub = nh.subscribe("setpoint/yaw",1,&Att_Control::yaw_sp_cb,this);
    att_sp_pub  = nh.advertise<mavros_msgs::AttitudeTarget>("setpoint/attitude",1);

    // Initialize Remainder of Parameters
    att_sp_out.header.frame_id = "map";
    att_sp_out.type_mask = att_sp_out.IGNORE_ATTITUDE;

    att_control_switch = false;
    yaw_sp = 0.0;
    tau = 0.5;

}

Att_Control::~Att_Control()
{
  ROS_WARN("Terminating Attitude Controller");
}

void Att_Control::pose_curr_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    x_curr(0,0) = msg->pose.position.x;
    x_curr(1,0) = msg->pose.position.y;
    x_curr(2,0) = msg->pose.position.z;
    x_curr(6,0) = msg->pose.orientation.w;
    x_curr(7,0) = msg->pose.orientation.x;
    x_curr(8,0) = msg->pose.orientation.y;
    x_curr(9,0) = msg->pose.orientation.z;
}

void Att_Control::vel_curr_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    x_curr(3,0) = msg->twist.linear.x;
    x_curr(4,0) = msg->twist.linear.y;
    x_curr(5,0) = msg->twist.linear.z;
}

void Att_Control::force_sp_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    force_sp(0,0) = msg->vector.x;
    force_sp(1,0) = msg->vector.y;
    force_sp(2,0) = msg->vector.z;
    force_sp_time = msg->header.stamp;
    // generally enable attitude controller when first force setpoint arrives
    att_control_switch = true;
}

void Att_Control::yaw_sp_cb(const std_msgs::Float64::ConstPtr& msg){
    //yaw_sp = msg->data;
}

void Att_Control::controller()
{
    ros::Time t_now = ros::Time::now();
    expired_time = t_now.toSec() - force_sp_time.toSec();

    if ((expired_time < 0.04) && (att_control_switch == true)) { 

        K_force_sp(0,0) =  cos(yaw_sp)*force_sp(0,0) + sin(yaw_sp)*force_sp(1,0);
        K_force_sp(1,0) = -sin(yaw_sp)*force_sp(0,0) + cos(yaw_sp)*force_sp(1,0);
        K_force_sp(2,0) =  force_sp(2,0);

        pitch_sp = atan2(K_force_sp(0,0), K_force_sp(2,0));

        L_force_sp(0,0) =  cos(pitch_sp)*K_force_sp(0,0) - sin(pitch_sp)*K_force_sp(2,0);
        L_force_sp(1,0) =  K_force_sp(1,0);
        L_force_sp(2,0) =  sin(pitch_sp)*K_force_sp(0,0) + cos(pitch_sp)*K_force_sp(2,0);

        roll_sp = atan2(-L_force_sp(1,0), L_force_sp(2,0));

        q_sp(0,0) =  cos(roll_sp/2)*cos(pitch_sp/2)*cos(yaw_sp/2) + sin(roll_sp/2)*sin(pitch_sp/2)*sin(yaw_sp/2);
        q_sp(1,0) = -cos(roll_sp/2)*sin(pitch_sp/2)*sin(yaw_sp/2) + sin(roll_sp/2)*cos(pitch_sp/2)*cos(yaw_sp/2);
        q_sp(2,0) =  cos(roll_sp/2)*sin(pitch_sp/2)*cos(yaw_sp/2) + sin(roll_sp/2)*cos(pitch_sp/2)*sin(yaw_sp/2);
        q_sp(3,0) =  cos(roll_sp/2)*cos(pitch_sp/2)*sin(yaw_sp/2) - sin(roll_sp/2)*sin(pitch_sp/2)*cos(yaw_sp/2);

        qw = x_curr(6,0);
        qx = x_curr(7,0);
        qy = x_curr(8,0);
        qz = x_curr(9,0);

        q_err(0,0) =  qw*q_sp(0,0) + qx*q_sp(1,0) + qy*q_sp(2,0) + qz*q_sp(3,0);
        q_err(1,0) = -qx*q_sp(0,0) + qw*q_sp(1,0) + qz*q_sp(2,0) - qy*q_sp(3,0);
        q_err(2,0) = -qy*q_sp(0,0) - qz*q_sp(1,0) + qw*q_sp(2,0) + qx*q_sp(3,0);
        q_err(3,0) = -qz*q_sp(0,0) + qy*q_sp(1,0) - qx*q_sp(2,0) + qw*q_sp(3,0);

        if (q_err(0,0) >= 0) {
            sign = 1;
        } else {
            sign = -1;
        }

        // compute setpoints
        att_sp_out.thrust = sqrt(force_sp(0,0)*force_sp(0,0) + force_sp(1,0)*force_sp(1,0) + force_sp(2,0)*force_sp(2,0)) / max_thrust_;
        att_sp_out.body_rate.x = sign*(2/tau)*q_err(1,0);
        att_sp_out.body_rate.y = sign*(2/tau)*q_err(2,0);
        att_sp_out.body_rate.z = sign*(2/tau)*q_err(3,0);

        att_sp_out.header.stamp = ros::Time::now();
        att_sp_out.header.seq = -1;

        att_sp_pub.publish(att_sp_out);

    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "att_node");

    Att_Control att_control;

    ros::Rate rate(200);
    while(ros::ok())
    {
        ros::spinOnce();
        att_control.controller();
        rate.sleep();
    }
    return 0;
}
