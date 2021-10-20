#include <bridge_px4/MPC.h>

//ifstream in("/home/max/Documents/codegen/drone/WPs_jump.csv");
ifstream in("/home/max/Documents/codegen/drone/hover.csv");
//ifstream in("/home/carl/Desktop/CPG/TrajBridge-PX4/WPs_jump.csv");
//ifstream in("/home/carl/Desktop/CPG/TrajBridge-PX4/hover.csv");

const string MPC::states[] = {
    "px","py","pz",
    "vx","vy","vz",
    "qw","qx","qy","qz"};

MPC::MPC()
:   x_box_lim_(3.0),y_box_lim_(1.75),z_box_lim_(1.0),
    mass_(0.53),setpoint_mode_(1)
{
    ros::param::get("~x_box_lim", x_box_lim_);
    ros::param::get("~y_box_lim", y_box_lim_);
    ros::param::get("~z_box_lim", z_box_lim_);
    ros::param::get("~mass", mass_);
    ros::param::get("~setpoint_mode", setpoint_mode_);

    // ROS Initialization
    pose_curr_sub = nh.subscribe("mavros/local_position/pose",1,&MPC::pose_curr_cb,this);
    vel_curr_sub = nh.subscribe("mavros/local_position/velocity_local",1,&MPC::vel_curr_cb,this);    
    force_sp_pub = nh.advertise<geometry_msgs::Vector3Stamped>("setpoint/force",1);
    yaw_sp_pub = nh.advertise<std_msgs::Float64>("setpoint/yaw",1);
    stats_pub = nh.advertise<std_msgs::Float64MultiArray>("stats",1); 

    // Initialize Limits Vector
    /*
    del_slim(0,0) = del_slim(1,0) = pxy_slim_;
    del_slim(2,0) = pz_slim_;
    del_slim(3,0) = del_slim(4,0) = del_slim(5,0) = v_slim_;
    del_slim(6,0) = del_slim(7,0) = del_slim(8,0) = del_slim(9,0) = q_slim_;
    */

    /*
    cout << "pos_xy safety limit: " << pxy_slim_ << endl;
    cout << "pos_z safety limit: " << pz_slim_ << endl;
    cout << "vel safety limit: " << v_slim_ << endl;
    cout << "quat safety limit: " << q_slim_ << endl;
    */

    /*
    err_lim(0,0) = err_lim(1,0) = err_lim(2,0) = ep_lim_;
    err_lim(3,0) = err_lim(4,0) = err_lim(5,0) = err_lim(6,0) = eq_lim_;
    */

    k_main = 0;
    main_switch = false;
    safety_switch = true;

    F_max = 1.5*mass_*9.81;
    update_fv_min(-0.5*mass_*9.81);
    update_fv_max(F_max*0.98481 - mass_*9.81);
    for (int i = 0; i < 10; i++) {
        update_G(i, 0.12468*mass_*9.81);
    }

    set_OSQP_check_termination(5);
    set_OSQP_max_iter(15);

    stats_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
    stats_out.layout.dim[0].size = 8;
    stats_out.layout.dim[0].stride = 1;
    stats_out.layout.dim[0].label = "stats";

    if (setpoint_mode_ == 2) {

        amp_x = 1.0;
        amp_y = 1.0;
        amp_z = 0.0;

        om_x = 1.57/(2*30);
        om_y = om_x;
        om_z = om_x;

        ph_x = 1.57;
        ph_y = 0.0;
        ph_z = 0.0;

        off_x = -amp_x;
        off_y = 0.0;
        off_z = 0.5;

    } else if (setpoint_mode_ == 3) {

        amp_x = 2.0;
        amp_y = 1.0;
        amp_z = 0.5;

        om_x = 1.57/(2*30);
        om_y = 1.2*om_x;
        om_z = 1.4*om_x;

        ph_x = 0.0;
        ph_y = 0.0;
        ph_z = 1.57;

        off_x = 0.0;
        off_y = 0.0;
        off_z = 0.5;

    }

}

MPC::~MPC()
{
  ROS_WARN("Terminating MPC");
}

void MPC::pose_curr_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    x_curr(0,0) = msg->pose.position.x;
    x_curr(1,0) = msg->pose.position.y;
    x_curr(2,0) = msg->pose.position.z;
    x_curr(6,0) = msg->pose.orientation.w;
    x_curr(7,0) = msg->pose.orientation.x;
    x_curr(8,0) = msg->pose.orientation.y;
    x_curr(9,0) = msg->pose.orientation.z;
    // hand over to MPC when at least 0.5 m altitude
    if (x_curr(2,0) > 0.5) {
        main_switch = true;
    }
}

void MPC::vel_curr_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    x_curr(3,0) = msg->twist.linear.x;
    x_curr(4,0) = msg->twist.linear.y;
    x_curr(5,0) = msg->twist.linear.z;
}

bool MPC::setpoint_update()
{

    if (setpoint_mode_ == 1) {

        if (in) {

            if ((k_main % 90) == 0) {
                string line;
                if (getline(in, line)) {
                    stringstream s(line);
                    string field;
                    for (int j = 0; j < 6; j++) {
                        getline(s, field, ',');
                        x_bar(j, 0) = stod(field);
                    }
                    getline(s, field, ',');
                    yaw_sp = stod(field);
                }
            }

            k_main += 1;

            return true;
    
        } else {

            return false;

        }

    } else {

        if (k_main <= 30*20) {

            x_bar(0, 0) = amp_x*sin(om_x*k_main + ph_x) + off_x;
            x_bar(1, 0) = amp_y*sin(om_y*k_main + ph_y) + off_y;
            x_bar(2, 0) = amp_z*sin(om_z*k_main + ph_z) + off_z;

            x_bar(3, 0) = amp_x*om_x*cos(om_x*k_main + ph_x);
            x_bar(4, 0) = amp_y*om_y*cos(om_y*k_main + ph_y);
            x_bar(5, 0) = amp_z*om_z*cos(om_z*k_main + ph_z);

            x_bar(6, 0) = 0.0;

            k_main += 1;

            return true;

        } else {

            return false;

        }

    }

    return false;
}

bool MPC::limit_check()
{

    if (abs(x_curr(0,0)) > x_box_lim_) {
        cout << "Limit Triggered by x position at " << k_main/30 << "s." << endl;
        cout << "Limit was: " << x_box_lim_ << ", but I had: " << x_curr(0,0) << endl;
        return false;
    }

    if (abs(x_curr(1,0)) > y_box_lim_) {
        cout << "Limit Triggered by y position at " << k_main/30 << "s." << endl;
        cout << "Limit was: " << y_box_lim_ << ", but I had: " << x_curr(1,0) << endl;
        return false;
    }

    if (abs(x_curr(2,0)-1.0) > z_box_lim_) {
        cout << "Limit Triggered by z position at " << k_main/30 << "s." << endl;
        cout << "Limit was: " << z_box_lim_ << ", but I had: " << x_curr(2,0)-1.0 << endl;
        return false;
    }

    return true;
}

void MPC::controller(ros::Time t_start)
{

    if ((main_switch == true) && (safety_switch == true)) {

        ros::Time t_end_spin = ros::Time::now();

        if (setpoint_update() == true) {
            
            ros::Time t_end_read = ros::Time::now();

            del_x = x_curr - x_bar;

            for (int i = 0; i < 6; i++) {
                update_x_init(i, del_x(i,0));
            }

            ros::Time t_end_init = ros::Time::now();

            solve();

            ros::Time t_end_ASA = ros::Time::now();

            for (int i = 0; i < 3; i++) {
                update_u_prev(i, CPG_Result.U[3+i]);
            }
            
            if (limit_check() == true) {

                // Publish force setpoint
                force_sp_out.vector.x = CPG_Result.U[3];
                force_sp_out.vector.y = CPG_Result.U[4];
                force_sp_out.vector.z = CPG_Result.U[5] + mass_*9.81;

                force_sp_out.header.stamp = ros::Time::now();
                force_sp_out.header.seq = k_main;

                force_sp_pub.publish(force_sp_out);

                // Publish yaw setpoint
                yaw_sp_out.data = yaw_sp;
                yaw_sp_pub.publish(yaw_sp_out);

                ros::Time t_end_publish = ros::Time::now();
                
                stats = {t_end_publish.toSec()   - t_start.toSec(),      // whole node time
                         t_end_spin.toSec()      - t_start.toSec(),      // spin time
                         t_end_read.toSec()      - t_end_spin.toSec(),   // csv read time
                         t_end_init.toSec()      - t_end_read.toSec(),   // param init time
                         t_end_ASA.toSec()       - t_end_init.toSec(),   // ASA time
                         CPG_Result.osqp_solve_time,                     // osqp solve time
                         t_end_publish.toSec()   - t_end_ASA.toSec(),    // publish time
                         (float) workspace.info->iter};                  // number of iterations
                
                stats_out.data.clear();
                stats_out.data.insert(stats_out.data.end(), stats.begin(), stats.end());

                stats_pub.publish(stats_out);

            } else {

                safety_switch = false;

            }
        }
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPC_node");

    MPC mpc;

    ros::Rate rate(30);
    while(ros::ok())
    {
        ros::Time t_start = ros::Time::now();
        ros::spinOnce();
        mpc.controller(t_start);
        rate.sleep();
    }
    return 0;
}
