#include <bridge_iroad/iroad_auto.h>

Auto_IRoad::Auto_IRoad():
  accel_id(5),
  steer_id(0),
  ct_input_id(2),
  PRNDL_vr_id(1),
  PRNDL_ct_id(2),
  pk_brake_id(0),
  auto_id(3),
  steer_scale(350)
{
  nh.param("accel_id", accel_id, accel_id);
  nh.param("steer_id", steer_id, steer_id);
  nh.param("ct_input_id", ct_input_id, ct_input_id);
  nh.param("PRNDL_vr_id", PRNDL_vr_id, PRNDL_vr_id);
  nh.param("PRNDL_ct_id", PRNDL_ct_id, PRNDL_ct_id);
  nh.param("auto_id", auto_id, auto_id);
  nh.param("pk_brake_id", pk_brake_id, pk_brake_id);

  nh.param("steer_scale", steer_scale, steer_scale);

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Auto_IRoad::joy_cb, this); 
  pose_curr_sub = nh.subscribe("mavros/local_position/pose",1,&Auto_IRoad::pose_curr_cb,this);

  double hz = 100.0;
  udpLoop = nh.createTimer(ros::Duration(1.0/hz),&Auto_IRoad::udp_cb, this);

  // Creating socket file descriptor
  if ( (socket_desc = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  // Filling server/client information
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr = inet_addr("192.168.140.2");

  client_addr.sin_family = AF_INET;
  client_addr.sin_port = htons(PORT);
  client_addr.sin_addr.s_addr = inet_addr("192.168.140.3");

  if ( bind(socket_desc, (const struct sockaddr *)&server_addr,sizeof(server_addr)) < 0 ) {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }
      
}

void Auto_IRoad::joy_cb(const sensor_msgs::Joy::ConstPtr& joy)
{
  accel_cmd = (float) -(joy->axes[accel_id]-1.0)/2.0;
  steer_cmd = (float) steer_scale*joy->axes[steer_id];
  ct_input_cmd = (float) -(joy->axes[ct_input_id]-1.0)/2.0;

  PRNDL_vr_cmd = (float) joy->buttons[PRNDL_vr_id];
  PRNDL_ct_cmd = (float) joy->buttons[PRNDL_ct_id];
  pk_brake_cmd = (float) joy->buttons[pk_brake_id];

  auto_check = joy->buttons[auto_id]!=0;

  cout << "accel: " << accel_cmd << endl;
  cout << "steer: " << steer_cmd << endl;
  cout << "ct_input: " << ct_input_cmd << endl;

  cout << "PRNDL_vr: " << PRNDL_vr_cmd << endl;
  cout << "PRNDL_ct: " << PRNDL_ct_cmd << endl;
  cout << "pk_brake: " << pk_brake_cmd << endl;
  cout << endl;
}

void Auto_IRoad::udp_cb(const ros::TimerEvent& event) {

  if (auto_check == 1) {
    cmd_struct.accel = 0.1;
    cmd_struct.steer = 0.0;
    cmd_struct.ct_input = 0.0;

    cmd_struct.PRNDL_vr = 0.0;
    cmd_struct.PRNDL_ct = 0.0;
    cmd_struct.pk_brake = 0.0;
  } else{
    cmd_struct.accel = accel_cmd;
    cmd_struct.steer = steer_cmd;
    cmd_struct.ct_input = ct_input_cmd;

    cmd_struct.PRNDL_vr = PRNDL_vr_cmd;
    cmd_struct.PRNDL_ct = PRNDL_ct_cmd;
    cmd_struct.pk_brake = pk_brake_cmd;
  }

  // Send Packet    
  sendto(socket_desc, &cmd_struct, sizeof(cmd_struct),MSG_CONFIRM, (const struct sockaddr *)&client_addr,sizeof(client_addr));
  
  //close(sockfd);

  //cout << "hoho" << endl;
}

void Auto_IRoad::pose_curr_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    x_curr(0,0) = msg->pose.position.x;
    x_curr(1,0) = msg->pose.position.y;
    x_curr(2,0) = msg->pose.position.z;
    x_curr(6,0) = msg->pose.orientation.w;
    x_curr(7,0) = msg->pose.orientation.x;
    x_curr(8,0) = msg->pose.orientation.y;
    x_curr(9,0) = msg->pose.orientation.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Auto_IRoad");
  Auto_IRoad Auto_IRoad;

  ros::spin();
  return 0;
}
