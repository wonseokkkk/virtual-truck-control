// C++
#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <algorithm>
#include <limits>
#include <random>
#include <condition_variable>
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/xav2lane.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_msg/msg/ocr2lrc.hpp"
#include "ros2_msg/msg/lrc2ocr.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;
class VehicleControl : public rclcpp::Node
{
public:
    VehicleControl();

    /********** Throttle Control **********/
    int Index_;
    float tx_vel_ = 0.0;
    float tx_dist_;
    float tx_tdist_;
    float cur_vel_ = 0.0;
    float est_vel_;
    float preceding_truck_vel_;
    float output_;

    bool emergency_stop_ = false;
    
    ros2_msg::msg::Ocr2lrc pub_msg_;
    std_msgs::msg::Float32 control_msg_;
    float Kp_dist_ = 0.8; // 2.0; //0.8;
    float Kd_dist_ = 0.03; //0.05;
    float Kp_throttle_ = 0.3; // 2.0; //0.8;
    float Ki_throttle_ = 1.5; // 0.4; //10.0;
    float Ka_throttle_ = 0.01;
    float Kf_throttle_ = 1.0;  // feed forward const.
    float Kp_brake_;
    float Ki_brake_;
    float Ka_brake_;
    float Kf_brake_;
    //float dt_ = 0.1;

    /********** Steer Control **********/
    void get_steer_coef(float vel);
    float K1_, K2_;
    void controlSteer(Mat left, Mat right, Mat center);
    ros2_msg::msg::Lane2xav lane_coef_, poly_coef_;
    Mat left;
    Mat right;
    Mat center;
    std_msgs::msg::Float32 steer_;

private:
    /********** Throttle Control **********/
    void LrcCallback(const ros2_msg::msg::Lrc2ocr::SharedPtr msg);
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void SetSpeed();
    void Throttle_PID(double dt_, float tar_vel, float current_vel);
    void Brake_PID(double dt_, float tar_vel, float current_vel);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_msg::msg::Ocr2lrc>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ControlPublisher;
    rclcpp::Subscription<ros2_msg::msg::Lrc2ocr>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber;
    void LoadParams(void);
    
    /********** Steer Control **********/
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr SteerPublisher_;
    rclcpp::Subscription<ros2_msg::msg::Xav2lane>::SharedPtr XavSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr LaneSubscriber_;

    void XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg);
    void LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);


    float SteerAngle_;
    float eL_height_, e1_height_, lp_;
    float K_;
    double a_[5], b_[5];
    vector<float> e_values_;
};

