#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <math.h>
#include <memory>
#include <tuple>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>

#define pi 3.1417
 
using Eigen::MatrixXd;
using namespace std::chrono_literals;
using std::placeholders::_1;

enum SceneType {
  Simulation = 1,
  Cyberworld = 2
};



//std::chrono::nanoseconds fifty_milisec = 5000000;
class TurnController : public rclcpp::Node {
public:

  TurnController(enum SceneType scene_number): Node("turn_controller"), scene_number_(scene_number) {
    // std::string  KP_str = argv[1];
    // std::string  KI_str = argv[2];
    // std::string  KD_str = argv[3];
    // ---- 1. publisher to cmd_vel
      publisher_1_twist =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    //------- 2. timer_1 related -----------//
    timer_1_ = this->create_wall_timer(
        1000ms, std::bind(&TurnController::timer1_callback, this));//nothing about 1 sec
    //------- 3. Odom related  -----------//
    callback_group_3_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options3_odom;
    options3_odom.callback_group = callback_group_3_odom;
    subscription_3_odom = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
        //"/rosbot_xl_base_controller/odom", 10,
        std::bind(&TurnController::odom_callback, this,
                  std::placeholders::_1), options3_odom);

    //ref_points->push_back(std::make_tuple(0,0));
    double cur_ref_phi = 0;
    double cur_ref_x = 0;
    double cur_ref_y = 0;
    //RCLCPP_DEBUG(this->get_logger(), "initialize ref_point (x,y) = %f,%f ", cur_ref_x, cur_ref_y);


     //PID parameter   best Kp=4.0 
    switch(this->scene_number_){
    case Simulation:
        this->Kp = 1;
        this->Ki = 0.0;
        this->Kd = 0.0;
        this->Kp_angle = 2.0;//std::stod(KP_str);  (2.0,0.01,0.1) best
        this->Ki_angle = 0.01;//std::stod(KI_str); 
        this->Kd_angle = 0.1;//std::stod(KD_str); 
    break;
    case Cyberworld:
        this->Kp = 1;
        this->Ki = 0.0;
        this->Kd = 0.0;
        this->Kp_angle = 0.65;//std::stod(KP_str);  (1.5,0.001,0.5) best
        this->Ki_angle = 0.001;//std::stod(KI_str); 
        this->Kd_angle = 0.01;//std::stod(KD_str); 
    break;
    }

    this->Hz = 10.0;
    this->dt = 0.1;
    this->hz_inverse_us = 100000;//10 Hz = 0.1 sec = 100,000 microsec 
    this->pid_reset();
  }

private:
    //int max_iter = 20;

  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer Callback ");
    timer_1_->cancel();
    while( !this->is_odom_started){
        sleep(1);
    }
    double x,y;
    if(this->scene_number_ == Cyberworld){
        //this->ref_points_cyberworld.push_back(std::make_tuple(this->current_pos_.x, this->current_pos_.y,1));
        int i=0;
        x = this->current_pos_.x;
        y = this->current_pos_.y;
        for(auto it = waypoints_cyberworld.begin(); it!= waypoints_cyberworld.end(); it++){
            i++;
            double dx = std::get<0>(*it);
            double dy = std::get<1>(*it);
            x += dx;
            y += dy;
            this->ref_points_cyberworld.push_back(std::make_tuple(x,y,1));  
        
        }     
        this->offset_yawrad_ = this->current_yaw_rad_;   
    }
    //assert(false);
    std::thread{std::bind(&TurnController::execute, this)}.detach();
  }

  void pid_reset(){
        this->integral_x_pos = 0;
        this->integral_y_pos = 0;
        this->integral_theta = 0; 
        this->last_error_signal = std::make_tuple(0,0,0);
  }
  // PID FUNCTIONS

  std::tuple<double,double,double> pid_controller(std::tuple<double,double,double> &error_signal){
    double dtheta_pos = std::get<0>(error_signal); 
    double dx_pos = std::get<1>(error_signal);
    double dy_pos = std::get<2>(error_signal);  
    double omega = dtheta_pos/this->dt;//-current_speed_.angular.z; 
    double vx = dx_pos/this->dt;//-current_speed_.linear.x;// 
    double vy = dy_pos/this->dt; //-current_speed_.linear.y;// 
    this->integral_x_pos += dx_pos*this->dt;
    this->integral_y_pos += dy_pos*this->dt;
    this->integral_theta += dtheta_pos*this->dt;
    double proportion_signal_x = this->Kp*dx_pos;
    double proportion_signal_y = this->Kp*dy_pos;
    double proportion_signal_theta = this->Kp_angle*dtheta_pos;
    double integral_signal_x = this->Ki*this->integral_x_pos;
    double integral_signal_y = this->Ki*this->integral_y_pos;
    double integral_signal_theta = this->Ki_angle*this->integral_theta;
    double derivative_signal_x = this->Kd*(dx_pos - std::get<1>(this->last_error_signal))/this->dt;//this->Kd * vx;
    double derivative_signal_y = this->Kd*(dy_pos - std::get<2>(this->last_error_signal))/this->dt;//this->Kd * vy;
    double derivative_signal_theta = this->Kd_angle*(dtheta_pos - std::get<0>(this->last_error_signal))/this->dt;//this->Kd_angle * omega;
    omega = normalize_angle(proportion_signal_theta + integral_signal_theta + derivative_signal_theta);
    vx = proportion_signal_x + integral_signal_x + derivative_signal_x;
    vy = proportion_signal_y + integral_signal_y + derivative_signal_y;
    this->last_error_signal = error_signal;
    std::tuple<double, double, double> controller_signal = std::make_tuple(omega, vx, vy);
    return controller_signal;
  }

  std::tuple<double,double,double> pid_plant_process(std::tuple<double,double,double> &controller_signal){
    auto message = std_msgs::msg::Float32MultiArray();
    double w = std::get<0>(controller_signal);
    double delta_x = std::get<1>(controller_signal);
    double delta_y = std::get<2>(controller_signal);
    MatrixXd vb = velocity2twist(w, delta_x, delta_y);
    std::vector<float> u_vector = twist2wheels(vb);
    message.data = u_vector;
    wheels2ling(message);
    std::tuple<double, double, double> output_signal = std::make_tuple(current_yaw_rad_,current_pos_.x, current_pos_.y);//(theta_pos,x_pos,y_pos)
    return output_signal;    
  }

  std::tuple<double,double,double> pid_error_rotating(std::tuple<double,double,double>& output_signal,double xf, double yf){       
        double theta_pos = std::get<0>(output_signal);
        double x_pos = std::get<1>(output_signal);
        double y_pos = std::get<2>(output_signal);
        double thetag= atan2(yf - y_pos,xf - x_pos);
        double theta_error = normalize_angle(thetag - theta_pos);
        std::tuple<double,double,double> error_signal = std::make_tuple(theta_error, 0, 0);
        RCLCPP_DEBUG(get_logger(), "rotating output |Goal_x:%f,Goal_y:%f|thetag:%f|Current_x:%f,Current_y:%f|current_yaw:%f|angular error:%f "
        ,xf,yf, thetag,x_pos, y_pos, theta_pos, theta_error);
        return error_signal;
  }



bool pid_simulate_rotating(double x_goal, double y_goal, double tolerance, double angle_tolerance){
    //double theta_goal = normalize_angle(theta_goal_radian);
    RCLCPP_DEBUG(get_logger(), "x_goal %f, y_goal %f",x_goal,y_goal);
   
    double distance_error_norm = 1000; // some large number
    double error_angle = 1000;//some large number
    int number_of_secs = 20;
    int time_to_move = this->Hz * number_of_secs;
    //bool historical_distance_error[10] = {false;false;false;false;false;false;false;false;false;false};
    int within_tolerance_counter = 0;
    bool is_stabilized_success = false;
    
    for(int i =0; i< time_to_move; i++){
    //while(distance_error_norm > tolerance || fabs(normalize_angle(error_angle)) > angle_tolerance){
        std::tuple<double, double, double> output_signal = std::make_tuple(current_yaw_rad_,current_pos_.x, current_pos_.y);//(theta_pos,x_pos,y_pos)
        std::tuple<double, double, double> error = pid_error_rotating(output_signal, x_goal, y_goal);
        error_angle= std::get<0>(error); 
        double error_x= std::get<1>(error);
        double error_y= std::get<2>(error); 
        RCLCPP_DEBUG(this->get_logger(), "error_angle= %f, error_x= %f,error_y=%f",error_angle,error_x,error_y); 
        std::tuple<double, double, double> res = pid_controller(error);//(omega, vx, vy)
        output_signal = pid_plant_process(res);//(theta_pos,x_pos,y_pos)
        distance_error_norm = sqrt(error_x*error_x+error_y*error_y);
        
        if( fabs(normalize_angle(error_angle)) <= angle_tolerance){
            RCLCPP_DEBUG(this->get_logger(), "rotating distance_error_norm= %f, error_angle= %f, angle in tolerence %f", 
            distance_error_norm ,error_angle, angle_tolerance);
            within_tolerance_counter++;
            if(within_tolerance_counter >= 10) {
                is_stabilized_success = true;
                break;
            }
        }else{
            RCLCPP_DEBUG(this->get_logger(), "rotating distance_error_norm= %f, error_angle= %f ", distance_error_norm ,error_angle);
            within_tolerance_counter = 0;
        }  
        usleep(hz_inverse_us);
    }
    this->pid_reset();
    return is_stabilized_success;
  }

  void execute() {
    auto message = std_msgs::msg::Float32MultiArray();
    double distance_error_tolerance = 0.01; 
    double angle_tolerance = 0.01;
    long int total_elapsed_time = 0;
    bool all_success = true, all_success_rotating = true;
    std::list<std::tuple<double,double,int>> *ref_points;
    switch(this->scene_number_){
    case Simulation:
        ref_points = &this->ref_points_simulation;        
        RCLCPP_DEBUG(this->get_logger(), "Simulation Scence");
    break;
    case Cyberworld:
      ref_points = &this->ref_points_cyberworld;   
      RCLCPP_DEBUG(this->get_logger(), "Cyberworld Scence");
    break;
    }
   for(auto it2 =ref_points->begin(); it2 != ref_points->end(); it2++){        
        double xg = std::get<0>(*it2);
        double yg = std::get<1>(*it2);
        int w_name = std::get<2>(*it2);
 
        std::string result_pid;
        RCLCPP_DEBUG(this->get_logger(), "next ref_points w%d  (%f,%f)",
        w_name, xg, yg);
      

        std::string result_pid_rotating;
        RCLCPP_DEBUG(this->get_logger(), "start face ref_points w%d  (%f,%f), current_yaw_rad %f: %s",
        w_name, xg, yg, this->current_yaw_rad_, result_pid_rotating.c_str());
        auto beg_rotating= std::chrono::high_resolution_clock::now();
        bool success_rotating = pid_simulate_rotating(xg,yg, distance_error_tolerance,angle_tolerance);
        auto end_rotating = std::chrono::high_resolution_clock::now();
        auto duration_rotating = std::chrono::duration_cast<std::chrono::microseconds>(end_rotating - beg_rotating);
        all_success_rotating = all_success_rotating && success_rotating;
        if(success_rotating){
            result_pid_rotating = "success";
        }else{
            result_pid_rotating = "failed";
        }
        ling.angular.z = 0;
        ling.linear.x = 0;
        ling.linear.y = 0;
        this->move_robot(ling);
        RCLCPP_DEBUG(this->get_logger(), "end face ref_points w%d  (%f,%f), current_yaw_rad %f: %s, elasped time %ld",
        w_name, xg, yg, this->current_yaw_rad_, result_pid_rotating.c_str(),duration_rotating.count());
        total_elapsed_time += duration_rotating.count();    
        usleep(hz_inverse_us);
        sleep(1);
    }
    char all_success_char = all_success? 'Y':'N';
    RCLCPP_DEBUG(get_logger(), "Summary Kp_angle %f, Ki_angle %f, Kd_angle %f total elapsed time %ld, all successes? %c",
    this->Kp_angle, this->Ki_angle, this->Kd_angle, total_elapsed_time, all_success_char); 
    rclcpp::shutdown();
  }

  MatrixXd velocity2twist(double dphi, double dx, double dy){
    RCLCPP_DEBUG(get_logger(), "velocity2twist current_yaw_rad_ %f",current_yaw_rad_);  
    MatrixXd R(3,3);
    R(0,0) = 1; R(0,1) = 0; R(0,2) = 0;
    R(1,0) = 0; R(1,1) = cos(current_yaw_rad_); R(1,2) =  sin(current_yaw_rad_); 
    R(2,0) = 0; R(2,1) = -sin(current_yaw_rad_); R(2,2) =  cos(current_yaw_rad_);        
    MatrixXd v(3,1);
    v(0,0) = dphi;
    v(1,0) = dx;
    v(2,0) = dy;

    MatrixXd twist = R*v; 
    return twist;
  }

 std::vector<float> twist2wheels(MatrixXd twist){
    std::vector<float> u_vector;

    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0,0));
    u_vector.push_back(u(1,0));
    u_vector.push_back(u(2,0));
    u_vector.push_back(u(3,0));
    return u_vector;
  
  }
  std::vector<float> twist2wheels(double wz, double vx, double vy){
    std::vector<float> u_vector;
    MatrixXd twist(3,1);
    twist(0,0) = wz;
    twist(1,0) = vx;
    twist(2,0) = vy;
    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0,0));
    u_vector.push_back(u(1,0));
    u_vector.push_back(u(2,0));
    u_vector.push_back(u(3,0));
    return u_vector;
  
  }

  double normalize_angle(double angle_radian){
    double sign_multiplier, normalized_angle;

    if(angle_radian >= -pi && angle_radian <= pi)
        return angle_radian;

    if(angle_radian < -pi)
        sign_multiplier = 1;

    if(angle_radian > pi)
        sign_multiplier = -1;

    normalized_angle = angle_radian;
    for (int i=0; i<20; i++){
        normalized_angle += sign_multiplier*2*pi;
        if( normalized_angle >= -pi && normalized_angle <= pi)
            return normalized_angle;
    }
    return -100;  
  }



MatrixXd KinematicLeastSquareNormalEq(MatrixXd & u){
    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd HTH_inv = (H.transpose()* H).inverse();
    MatrixXd HTHinv_least_square  = HTH_inv * H.transpose();
    MatrixXd twist = HTHinv_least_square * u;
    return twist;
}

  void wheels2ling(const std_msgs::msg::Float32MultiArray msg)
  {
    MatrixXd u(4,1);
    
     RCLCPP_DEBUG(this->get_logger(), "wheel_speed topic callback");
    for(unsigned int i = 0; i < msg.data.size(); i++){
    u(i,0) = msg.data[i];
    RCLCPP_DEBUG(this->get_logger(), "I heard: '%f'",u(i,0));
    }
    MatrixXd twist = KinematicLeastSquareNormalEq(u);
    RCLCPP_DEBUG(this->get_logger(), "twist wz: %f vx ,%f, vy %f",twist(0,0),twist(1,0),twist(2,0));
    ling.angular.z = twist(0,0);
    ling.linear.x = twist(1,0);
    ling.linear.y = twist(2,0);

    this->move_robot(ling);
  }



void move_robot(geometry_msgs::msg::Twist &msg) {
    publisher_1_twist->publish(msg);
}
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1_twist;
  geometry_msgs::msg::Twist ling;
  //------- 3. Odom related  Functions -----------//  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    is_odom_started = true;
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_raw_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_yaw_rad_ = current_yaw_rad_raw_ - offset_yawrad_;
    
    current_speed_ = msg->twist.twist;
    //current_speed_y = msg->twist.twist.linear.y;
    //current_angular_velocity = msg->twist.twist.angular.z;

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f','%f',%f",
                 current_pos_.x, current_pos_.y, current_yaw_rad_raw_, offset_yawrad_, current_yaw_rad_);
  }
  double yaw_theta_from_quaternion(double qx, double qy, double qz, double qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }


  //------- 3. Odom related private variables  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_3_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_3_odom;
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  geometry_msgs::msg::Twist current_speed_;
  double current_yaw_rad_, current_yaw_rad_raw_, offset_yawrad_=0;
  bool is_odom_started = false;
  //--------  Kinematic related private variables --------// 
  double l = 0.500/2;
  double r = 0.254/2;
  double w = 0.548/2;
  //------- Pid variables -------//
  double Kp, Ki, Kd, Kp_angle, Ki_angle, Kd_angle, integral_x_pos, integral_y_pos, integral_theta, Hz, dt;
  int hz_inverse_us;   
  std::tuple<double,double,double> last_error_signal;

//   std::list<std::tuple<double, double, double>> waypoints {std::make_tuple(0,1,-1),std::make_tuple(0,1,1),
//                                 std::make_tuple(0,1,1),std::make_tuple(1.5708, 1, -1),std::make_tuple(-3.1415, -1, -1),
//                                 std::make_tuple(0.0, -1, 1),std::make_tuple(0.0, -1, 1),std::make_tuple(0.0, -1, -1)};
  std::list<std::tuple<double,double,int>> ref_points_simulation { //(face_x, face_y, x, y)
  std::make_tuple(0.527354894562477,-1.306332441241303,1),
  std::make_tuple(1.4148843902023136,-0.3760951047490883,2),
  std::make_tuple(0.7640531990921009,0.3725023199809377,3)};
  //std::list<std::tuple<double, double, double>> ref_points;
  std::list<std::tuple<double,double,int>> ref_points_cyberworld;
  std::list<std::tuple<double,double,int>> waypoints_cyberworld{ 
  std::make_tuple(0.9482432341447602, -0.50,1),//Husarian logo
  std::make_tuple(-0.55, -0.85,2)};// The construct's logo

  std::tuple<double,double> start_position_cyberworld;

 
  rclcpp::TimerBase::SharedPtr timer_1_;
  int timer1_counter;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  enum SceneType scene_number_;
};



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  enum SceneType scene_number = Simulation;
    // Check if a scene number argument is provided

  if (argc > 1) {
    scene_number = static_cast<enum SceneType>(std::atoi(argv[1]));
    switch (scene_number) {
    case Simulation:
    std::cout<<"Simulation scene"<<std::endl;
    break;

    case  Cyberworld:
    std::cout<<"Cyberworld scene"<<std::endl;
    break;
    }
  }
  auto turn_controller_node = std::make_shared<TurnController>(scene_number);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(turn_controller_node);
  executor.spin();



  //rclcpp::spin(std::make_shared<TurnController>());
  rclcpp::shutdown();
  return 0;
}