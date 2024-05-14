#include "vehicle_control_node.hpp"

VehicleControl::VehicleControl()
          : Node("VehicleControl", rclcpp::NodeOptions()
                                 .allow_undeclared_parameters(true)
                                 .automatically_declare_parameters_from_overrides(true))
{
  /**************/
  /* ROS2 Topic */
  /**************/
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string LaneTopicName;
  int LaneQueueSize;
  std::string XavPubTopicName;
  int XavPubQueueSize;

  /**************/
  /* QoS Option */
  /**************/
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos.best_effort();

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/xavier_to_lane/topic", XavSubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("subscribers/xavier_to_lane/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("subscribers/lane_to_xavier/topic", LaneTopicName, std::string("lane2xav_msg"));
  this->get_parameter_or("subscribers/lane_to_xavier/queue_size", LaneQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  XavSubscriber_ = this->create_subscription<ros2_msg::msg::Xav2lane>(XavSubTopicName, XavSubQueueSize, std::bind(&VehicleControl::XavSubCallback, this, std::placeholders::_1));
  LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>(LaneTopicName, LaneQueueSize, std::bind(&VehicleControl::LaneSubCallback, this, std::placeholders::_1));
  subscriber_ = this->create_subscription<ros2_msg::msg::Lrc2ocr>("lrc2ocr_msg", qos_profile, std::bind(&VehicleControl::LrcCallback, this, std::placeholders::_1)
  );
  VelocitySubscriber = this->create_subscription<std_msgs::msg::Float32>("velocity_info",1,std::bind(&VehicleControl::velocity_callback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  SteerPublisher_ = this->create_publisher<std_msgs::msg::Float32>("steer",XavPubQueueSize);
  publisher_ = this->create_publisher<ros2_msg::msg::Ocr2lrc>("ocr2lrc_msg", qos_profile);
  ControlPublisher = this->create_publisher<std_msgs::msg::Float32>("velocity",1);
  timer_ = this->create_wall_timer(10ms, std::bind(&VehicleControl::SetSpeed, this));

  /********** PID control ***********/

  lane_coef_.coef.resize(3);
  poly_coef_.coef.resize(3);
  e_values_.resize(2);
  left = Mat::zeros(3, 1, CV_32F);
  right = Mat::zeros(3, 1, CV_32F);
  center = Mat::zeros(3, 1, CV_32F);

  /* Lateral Control coefficient */
  this->get_parameter_or("params/K", K_, 0.15f);

  this->get_parameter_or("params/a/a", a_[0], 0.);
  this->get_parameter_or("params/a/b", a_[1], -0.37169);
  this->get_parameter_or("params/a/c", a_[2], 1.2602);
  this->get_parameter_or("params/a/d", a_[3], -1.5161);
  this->get_parameter_or("params/a/e", a_[4], 0.70696);

  this->get_parameter_or("params/b/a", b_[0], 0.);
  this->get_parameter_or("params/b/b", b_[1], -1.7536);
  this->get_parameter_or("params/b/c", b_[2], 5.0931);
  this->get_parameter_or("params/b/d", b_[3], -4.9047);
  this->get_parameter_or("params/b/e", b_[4], 1.6722);

  LoadParams();
}   


/********** Throttle Control **********/
void VehicleControl::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) 
{
  cur_vel_ = msg->data; // update current velocity
}

void VehicleControl::LrcCallback(const ros2_msg::msg::Lrc2ocr::SharedPtr msg) 
{
  Index_ = msg->index;
  tx_dist_ = msg->cur_dist;
  tx_tdist_ = msg->tar_dist;
  tx_vel_ = msg->tar_vel;
  emergency_stop_ = msg->emergency_flag;
 // cur_vel_ = msg->cur_vel;
 // SetSpeed();
 // publisher_->publish(pub_msg_);
}

void VehicleControl::SetSpeed()
{
  std::chrono::high_resolution_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
  static std::chrono::high_resolution_clock::time_point prev_time;
  float dt_ = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - prev_time).count() / 1000000;
  dt_ = 0.01;
  Throttle_PID(dt_, tx_vel_, cur_vel_);
  prev_time = cur_time;
  publisher_->publish(pub_msg_);
}

void VehicleControl::Throttle_PID(double dt_, float tar_vel, float current_vel)
{
  static float err, P_err, I_err;
  static float prev_u_b, prev_u_k, prev_u, A_err;
  static float dist_err, prev_dist_err, P_dist_err, D_dist_err;
  float u = 0.f, u_k = 0.f, u_b = 0.f;
  float u_dist = 0.f, u_dist_k = 0.f;
  float ref_vel = 0.f;
  pub_msg_.cur_vel = current_vel;

  if(Index_ != 10) {
    dist_err = tx_dist_ - tx_tdist_;    
    P_dist_err = Kp_dist_ * dist_err;
    D_dist_err = (Kd_dist_ * ((dist_err - prev_dist_err) / dt_ )); 
    u_dist = P_dist_err + D_dist_err + tar_vel;

    // sat(u(k))  saturation start 
    if(u_dist > 90.0) u_dist_k = 90.0;
    else if(u_dist <= 0) u_dist_k = 0;
    else u_dist_k = u_dist;

    ref_vel = u_dist_k;
  } else {
    ref_vel = tar_vel;
  }

  pub_msg_.ref_vel = ref_vel;
  
  if(ref_vel >= 0 && emergency_stop_ == false) {
    err = ref_vel - current_vel;
    P_err = Kp_throttle_ * err;
    I_err += Ki_throttle_ * err * dt_;
    A_err += Ka_throttle_ * ((prev_u_k - prev_u) / dt_);

    if(tar_vel <= 0) {
      P_err = 0;
      I_err = 0;
      A_err = 0;
    }
  
    u = P_err + I_err + A_err + ref_vel * Kf_throttle_;

    if(u > 1.0) u_k = 1.0f;
    else if(u <= -1.0) u_k = -1.0f;
    else u_k = u;

    pub_msg_.u_k = u_k;
    control_msg_.data = u_k;
    ControlPublisher->publish(control_msg_);
    prev_u_k = u_k;
    prev_u = u;
    prev_dist_err = dist_err;
  }
  
  else
  {
//    err = ref_vel - current_vel;
//    P_err = Kp_brake_ * err;
//    I_err += Ki_brake_ * err * dt_;
//    A_err += Ka_brake_ * ((prev_u_b - prev_u) / dt_);
//
//    if(tar_vel <= 0){
//      P_err = 0;
//      I_err = 0;
//      A_err = 0;
//    }
//  
//    u = P_err + I_err + A_err + ref_vel * Kf_brake_;
//
//    if(u > -2.0) u_b = -2.0;
//    else if(u <= 0) u_b = -1.0;
//    else u_b = u;
//  
//    pub_msg_.u_k = u_b;
//    prev_u_b = u_b;
//    prev_u = u;
//    prev_dist_err = dist_err;
    u_k = -1.0f;
    pub_msg_.u_k = u_k;
    control_msg_.data = u_k;
    ControlPublisher->publish(control_msg_);
  }
}
/********** Throttle Control **********/


/********** Steer Control **********/
void VehicleControl::LoadParams(void)
{
  this->get_parameter_or("LaneKeeping/eL_height",eL_height_, 0.2f);
  this->get_parameter_or("LaneKeeping/e1_height",e1_height_, 0.6667f);
  this->get_parameter_or("LaneKeeping/lp",lp_, 609.3f);
  this->get_parameter_or("LaneKeeping/steer_angle",SteerAngle_, 0.0f);
}

void VehicleControl::XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg)
{
  cur_vel_ = msg->cur_vel;
  get_steer_coef(cur_vel_);
}

void VehicleControl::LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg)
{
  poly_coef_.coef = msg->coef;
  left.at<float>(2,0) = poly_coef_.coef[0].a;
  left.at<float>(1,0) = poly_coef_.coef[0].b;
  left.at<float>(0,0) = poly_coef_.coef[0].c;

  right.at<float>(2,0) = poly_coef_.coef[1].a;
  right.at<float>(1,0) = poly_coef_.coef[1].b;
  right.at<float>(0,0) = poly_coef_.coef[1].c;

  center.at<float>(2,0) = poly_coef_.coef[2].a;
  center.at<float>(1,0) = poly_coef_.coef[2].b;
  center.at<float>(0,0) = poly_coef_.coef[2].c;

  controlSteer(left, right, center);
  SteerPublisher_->publish(steer_);
}

void VehicleControl::get_steer_coef(float vel)
{
  float value;
  if (vel > 1.2f)
    value = 1.2f;
  else
    value = vel;

  if (value < 0.65f){
    K1_ = K2_ =  K_;
  }
  else{
    K1_ = (a_[0] * pow(value, 4)) + (a_[1] * pow(value, 3)) + (a_[2] * pow(value, 2)) + (a_[3] * value) + a_[4];
    K2_ = (b_[0] * pow(value, 4)) + (b_[1] * pow(value, 3)) + (b_[2] * pow(value, 2)) + (b_[3] * value) + b_[4];
  }

}

void VehicleControl::controlSteer(Mat left, Mat right, Mat center) 
{
  float car_position = 320.0; //width of image / 2
  float l1 = 0.0f, l2 = 0.0f;
  float i = 480.0 * eL_height_;
  float j = 480.0 * e1_height_;

  if (!left.empty() && !right.empty()) {

    lane_coef_.coef[0].a = left.at<float>(2, 0);
    lane_coef_.coef[0].b = left.at<float>(1, 0);
    lane_coef_.coef[0].c = left.at<float>(0, 0);

    lane_coef_.coef[1].a = right.at<float>(2, 0);
    lane_coef_.coef[1].b = right.at<float>(1, 0);
    lane_coef_.coef[1].c = right.at<float>(0, 0);

    lane_coef_.coef[2].a = center.at<float>(2, 0);
    lane_coef_.coef[2].b = center.at<float>(1, 0);
    lane_coef_.coef[2].c = center.at<float>(0, 0);

    l1 =  j - i;
    l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);

    e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //e1

    SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
    steer_.data = SteerAngle_;
//    cout << SteerAngle_  << '\n';
  }
 }

/********** Steer Control **********/

int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
std::shared_ptr<rclcpp::Node> node = std::make_shared<VehicleControl>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}

