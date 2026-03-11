#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "zlac706_driver.h"

using namespace std::chrono_literals;

std::string node_name = "wheels_driver";

constexpr int wait_time{100};

// This driver is for ZLAC706 
class ZlacNode : public rclcpp::Node {
public:
  ZlacNode(const std::string &port) : Node(node_name), port_(port) {

    // Serial Parameters only for ZLAC706
    this->declare_parameter<int>("rate", 10);
    this->declare_parameter<int>("can_id", 666);
    this->declare_parameter<int>("baudrate", 57600);

    // Wheels configuration parameters
    this->declare_parameter<bool>("wheelR_is_backward", false);
    this->declare_parameter<bool>("wheelL_is_backward", true);
    this->declare_parameter<double>("wheels_separation", 0.4f);
    this->declare_parameter<double>("wheel_radius", 0.1f);

    // Acceleration and deceleration time for velocity control mode. 
    // The driver can receive accel for left and right wheel separately.
    this->declare_parameter<int>("accel_time_ms", 10);
    this->declare_parameter<int>("decel_time_ms", 10);  

    // Driver Movement Lock
    // If "unlock_driver" == true, after "time_disabled_driver_s":
    //     The wheels are going to be unlocked and current is going to be limited to 3A.
    // If If "unlock_driver" == false, after "time_disabled_driver_s":
    //     The wheels are going to be locked and current is going to be limited to 3A.
    this->declare_parameter<bool>("unlock_driver", true); 
    this->declare_parameter<double>("time_disabled_driver_s", 3.0f);

    // PID gains for velocity control mode
    this->declare_parameter<std::vector<int64_t>>("left_velocity_gains",std::vector<int64_t>{80, 30, 15});
    this->declare_parameter<std::vector<int64_t>>("right_velocity_gains",std::vector<int64_t>{80, 30, 15});

    // Resolution Mode
    // TRUE == wait_time RPM
    // FALSE == 1.0 RPM
    this->declare_parameter<bool>("resolution_mode", false);

    // Serial Parameters only for ZLAC706
    rate_ = this->get_parameter("rate").as_int();
    can_id_ = this->get_parameter("can_id").as_int();
    baudrate_ = this->get_parameter("baudrate").as_int();
    
    // Wheels configuration parameters
    wheelR_is_backward_ = this->get_parameter("wheelR_is_backward").as_bool();
    wheelL_is_backward_ = this->get_parameter("wheelL_is_backward").as_bool();
    wheels_separation_ = this->get_parameter("wheels_separation").as_double();
    wheels_radius_ = this->get_parameter("wheel_radius").as_double();

    // Acceleration and deceleration time for velocity control mode
    accel_time_ms_ = this->get_parameter("accel_time_ms").as_int();
    decel_time_ms_ = this->get_parameter("decel_time_ms").as_int();

    //Driver Movement Lock
    unlock_driver_ = this->get_parameter("unlock_driver").as_bool();
    time_disabled_driver_s_ = get_parameter("time_disabled_driver_s").as_double();

    // PID gains for velocity control mode
    auto v_l = this->get_parameter("left_velocity_gains").as_integer_array();
    auto v_r = this->get_parameter("right_velocity_gains").as_integer_array(); 

    velocity_gains_.left.kp = static_cast<int16_t>(v_l[0]);
    velocity_gains_.left.ki = static_cast<int16_t>(v_l[1]);
    velocity_gains_.left.kf = static_cast<int16_t>(v_l[2]);
    velocity_gains_.right.kp = static_cast<int16_t>(v_r[0]);
    velocity_gains_.right.ki = static_cast<int16_t>(v_r[1]);
    velocity_gains_.right.kf = static_cast<int16_t>(v_r[2]);

    // Resolution Mode
    bool resolution_mode_flag = this->get_parameter("resolution_mode").as_bool();

    // -------------------------------------- Timers to Manage the Driver Control ----------------------------------------
    warning_timer_ = this->create_wall_timer(1000ms, std::bind(&ZlacNode::warning_handler, this));
    wheel_ticks_timer_ = this->create_wall_timer(20ms, std::bind(&ZlacNode::wheel_ticks_timer, this));

    movement_lock_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(time_disabled_driver_s_)), std::bind(&ZlacNode::movement_lock_timer, this));
    movement_lock_timer_->cancel();
    
    // ------------------------------------------- Publishers and Subscribers --------------------------------------------
    pub_left_data_ = this->create_publisher<std_msgs::msg::Float64>("wheel/left_data", 10); //Recommendation: Send both in one message
    pub_right_data_ = this->create_publisher<std_msgs::msg::Float64>("wheel/right_data", 10);
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_safe", 10, std::bind(&ZlacNode::command_vel_CB, this, std::placeholders::_1));

    // ----------------------------------------- Initialization of Motor Control -----------------------------------------
    
    // Start motor connection
    motors_ = ZLAC706();
    motors_.port = port_;
    motors_.baudrate = baudrate_;

    // Get devices
    std::vector<std::string> devs;
    motors_.getDevices(devs);
    
    // Start Serial
    serial_.open(motors_.port, motors_.baudrate);
    //serial_.setCallback(received());
    
    // ------------------------------
    motors_.getParams();
    serial_.write(motors_.gParams_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT

    if (can_id_ != motors_.can_id) {
        RCLCPP_WARN(rclcpp::get_logger(node_name), "CAN Device not recognized");
        RCLCPP_INFO(rclcpp::get_logger(node_name), "Can ID is %d", motors_.can_id);
        rclcpp::shutdown();
    }
    
    motors_.setSpeedMode();
    serial_.write(motors_.spm_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));// < ------------------------ CHECK IT



    // Set the acceleration and deceleration time for velocity control mode
    motors_.configAcc(accel_time_ms_, decel_time_ms_);
    serial_.write(motors_.cfg_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT



    // Set the PID gains for velocity control mode
    motors_.setGain(3, velocity_gains_.left.kp);
    serial_.write(motors_.kiG_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT

    motors_.setGain(2, velocity_gains_.left.kf);
    serial_.write(motors_.kiG_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT

    motors_.setGain(1, velocity_gains_.left.ki);
    serial_.write(motors_.kiG_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT



    //Set speed to 0 m/s
    motors_.setSpeed(0);
    serial_.write(motors_.sSpeed_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT



    //Start the motor
    motors_.start();
    serial_.write(motors_.str_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT

    RCLCPP_INFO(rclcpp::get_logger(node_name), "Node initialized");
  }

  ~ZlacNode() override {
    try {
      motors_.configAcc(2000, 2000);
      serial_.write(motors_.cfg_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(wait_time)); // < ------------------------ CHECK IT

      motors_.setSpeed(0);
      serial_.write(motors_.sSpeed_msg);
      
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "Error launching emergency stop when the node was destructed");
    }
  }

private:
  // Error and troubleshooting
  rclcpp::TimerBase::SharedPtr warning_timer_;

  // Encoder - Wheels Ticks
  rclcpp::TimerBase::SharedPtr wheel_ticks_timer_;
  rclcpp::TimerBase::SharedPtr movement_lock_timer_;

  // Publishers and Subscribers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_data_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_data_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  // Params Callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_handler_;

  // Serial Parameters only for ZLAC706
  int rate_;
  int can_id_;
  int baudrate_;
  CallbackAsyncSerial serial_;
  ZLAC706 motors_;

  bool wheelR_is_backward_;
  bool wheelL_is_backward_;
  double wheels_separation_;
  double wheels_radius_;
  int accel_time_ms_;
  int decel_time_ms_;
  bool unlock_driver_;
  double time_disabled_driver_s_;
  VelocityGains velocity_gains_;  
  const std::string port_;

  void received(const char *data, unsigned int len){
    std::vector<char> v(data, data + len);
    motors_.received(data, len);
  }

  void warning_handler(){

    // Debug message
    // RCLCPP_INFO(get_logger(), "[WARN_MON] enc(L=%ld R=%ld) rpm(L=%.1f R=%.1f) I(L=%.2fA R=%.2fA) T(L=%.1fC R=%.1fC)",
    //   static_cast<long>(count_left), static_cast<long>(count_right),
    //   static_cast<double>(rpm_left), static_cast<double>(rpm_right),
    //   static_cast<double>(current_left), static_cast<double>(current_right),
    //   static_cast<double>(temp_left), static_cast<double>(temp_right)
    // );

    // If more than one publisher   
    const size_t pubs = count_publishers("cmd_vel_safe");
    if (pubs > 1) {
      RCLCPP_FATAL(rclcpp::get_logger(node_name), "More than one publisher on cmd_vel_safe (%zu). Stopping robot and shutting down.", pubs);
      //Set speed to 0 m/s
      motors_.setSpeed(0);
      serial_.write(motors_.sSpeed_msg);

      rclcpp::sleep_for(std::chrono::milliseconds(wait_time));
      // Destroy node
      rclcpp::shutdown();
    }
    else if (pubs == 0){
      RCLCPP_WARN(rclcpp::get_logger(node_name), "No publishers on cmd_vel_safe (%zu). Stopping robot and locking wheels for security.", pubs);
          //Set speed to 0 m/s
      motors_.setSpeed(0);
      serial_.write(motors_.sSpeed_msg);

      rclcpp::sleep_for(std::chrono::milliseconds(wait_time));
    }
  }

  void wheel_ticks_timer(){

    std_msgs::msg::Float64 msg_left_, msg_right_;
    const int32_t encoder_ticks = motors_.ticks;

    msg_left_.data  = static_cast<double>(encoder_ticks);
    msg_right_.data = static_cast<double>(encoder_ticks);

    pub_left_data_->publish(msg_left_);
    pub_right_data_->publish(msg_right_);
    
  }
  
  void command_vel_CB(const geometry_msgs::msg::Twist::SharedPtr msg){
    // Twist to Rpm 
    static bool flag = false;
    const double vx = msg->linear.x;
    const double wz = msg->angular.z;

    RCLCPP_INFO(rclcpp::get_logger(node_name), "Angular Speed:  %.1f, Linear Speed:  %.1f", wz, vx);

    if(vx == 0.0 && wz == 0.0){
      if(!flag){
        flag = true;
        movement_lock_timer_->reset();
      }
    }
    else{
      if(flag){
        movement_lock_timer_->cancel();
        motors_.start();
        serial_.write(motors_.str_msg);
        flag=false;
      }
    }

    auto [rpm_l, rpm_r] = twist_to_rpm(vx, wz);
    RCLCPP_INFO(rclcpp::get_logger(node_name), "Publish Speed RPM LEFT:  %.1f, RIGHT:  %.1f", rpm_l, rpm_r);

    //Set speed to 0 m/s
    motors_.setSpeed(rpm_l);
    serial_.write(motors_.sSpeed_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    //motors_.setSpeed(rpm_r);
    //serial_.write(motors_.sSpeed_msg);

    //rclcpp::sleep_for(std::chrono::milliseconds(wait_time));
  }

  void movement_lock_timer(){
    
    if(!unlock_driver_){
      motors_.start();
      serial_.write(motors_.str_msg);
      RCLCPP_DEBUG(rclcpp::get_logger(node_name), "Driver locked, not free wheels enable");
    }
    else{
      motors_.stop();
      serial_.write(motors_.str_msg);
      RCLCPP_DEBUG(rclcpp::get_logger(node_name), "Driver unlocked");
    }
    movement_lock_timer_->cancel();
  }

  std::pair<float,float> twist_to_rpm(double vx, double wz){
    const double w_l = (vx - (wz * wheels_separation_ / 2.0)) / wheels_radius_;
    const double w_r = (vx + (wz * wheels_separation_ / 2.0)) / wheels_radius_;

    double rpm_l = w_l * 60.0 / (2.0 * M_PI);
    double rpm_r = w_r * 60.0 / (2.0 * M_PI);

    if (wheelL_is_backward_) rpm_l = -rpm_l;
    if (wheelR_is_backward_) rpm_r = -rpm_r;

    rpm_l = static_cast<float>(rpm_l);
    rpm_r = static_cast<float>(rpm_r);

    return {rpm_l, rpm_r};
  }
};


int main(int argc, char * argv[]) {
  auto logger = rclcpp::get_logger(node_name);

  std::string port = "";

  if (argc >= 2) {
    port = argv[1];
    // Validate if port exists
    if (access(port.c_str(), F_OK) != 0) {
      RCLCPP_ERROR(logger, "Port %s does not exist", port.c_str());
      return 1;
    }
    RCLCPP_INFO(logger, "Port: %s", port.c_str());
  }else{
    RCLCPP_ERROR(logger, "No argument given. Usage: \n$ ros2 run <package_name> <node_name> <port>");
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZlacNode>(port));
  rclcpp::shutdown();
  return 0;
}
