#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "AsyncSerial.h"
#include "zlac706.h"

#include <signal.h>
#include <sstream>

#include "std_msgs/msg/int32.hpp"
#include "controller_msg_srv/msg/wheel.hpp"
#include "controller_msg_srv/srv/setgain.hpp"
#include "controller_msg_srv/srv/getgain.hpp"

//Si se quiere usar la librería del mismo proyecto. No comentar la linea sig.
//#include "zlac706_driver_control/WheelDriver.h"

//Si se quiere usar los servicios de otra carpeta "robot_core"
//Descomentar la linea sig. y comentar el include anterior

//#include "robot_msgs/WheelDriver.h"
#include "controller_msg_srv/srv/wheeldriver.hpp"

/*
Programa principal (por llanta), encargado de asignar los rpm a
cada llanta por su subscritor y monitorea el voltaje de la pila
y publica su valor, asi como los ticks del encoder
*/
//////////////////////////////////////////////////////////////
using std::string;
using std::vector;
using std::cout;
using std::endl;

CallbackAsyncSerial serial;
ZLAC706 wheel;

std::chrono::seconds duration_sec(2);
std::chrono::milliseconds duration_ms(10);

int speed = 0;
const uint8_t acc = 20, decc = 20;
bool isOccupied =false;

//prototipos de funciones
void received(const char *data, unsigned int len);
void setSpeed(const std_msgs::msg::Int32 & msg);
void shutdown_rutine(int sig);

//void servicio_callback(controller_msg_srv::srv::Wheeldriver::Request &req_msg,controller_msg_srv::srv::Wheeldriver::Response &res_msg);
bool servicio_callback(const std::shared_ptr<controller_msg_srv::srv::Wheeldriver::Request> req_msg,std::shared_ptr<controller_msg_srv::srv::Wheeldriver::Response>res_msg);
bool setGainCallback(const std::shared_ptr<controller_msg_srv::srv::Setgain::Request> req_msg,std::shared_ptr<controller_msg_srv::srv::Setgain::Response>res_msg);
bool getGainCallback(const std::shared_ptr<controller_msg_srv::srv::Getgain::Request> req_msg,std::shared_ptr<controller_msg_srv::srv::Getgain::Response>res_msg);
//bool setGainCallback(controller_msg_srv::srv::Setgain::Request &req_msg,controller_msg_srv::srv::Setgain::Response &res_msg);
//bool getGainCallback(controller_msg_srv::srv::Getgain::Request &req_msg,controller_msg_srv::srv::Getgain::Response &res_msg);
//-----------------------

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("zlac706_robot");
  //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");
  
  //ros::init(argc, argv, "zlac706_robot"); //iniciamos el nodo
  //ros::NodeHandle n("~");
  signal(SIGINT, shutdown_rutine);

  int rate = 10;
  int can_id = 0;
  string port_r;
  string publish_topic, subscrib_topic;
  string publish_voltage_topic="voltage";

  node->declare_parameter("rate", 10);
  node->declare_parameter("port", "/dev/ttyUSB0");
  node->declare_parameter("can_id", 0);
  node->declare_parameter("baudrate", 57600);

  rate = node->get_parameter("rate").as_int();
  wheel.port = node->get_parameter("port").as_string();
  can_id = node->get_parameter("can_id").as_int();
  wheel.baudrate = node->get_parameter("baudrate").as_int();
/*
  //Alojamos los parametros en variables locales para su uso//
  n.param("rate", rate, int(10));
  n.param("port", wheel.port, string("/dev/ttyUSB0"));
  n.param("can_id", can_id, int(0));
  n.param("baudrate", wheel.baudrate, int(57600));
  //---------------------------------------------------------

  string node_name=ros::this_node::getName();

  ROS_INFO("Iniciando %s",node_name.c_str());
*/
  std::string node_name = node->get_name();

  vector<string> devs;
  wheel.getDevices(devs);
  bool isConnect = false;
  //////////////////////////////////////////////////////////////
  //Descomentar esta parte si se queire probar en fisico
  //////////////////////////////////////////////////////////////

  serial.open(wheel.port, wheel.baudrate);
  serial.setCallback(received);

  wheel.getParams();
  serial.write(wheel.gParams_msg);

  //std::chrono::seconds duration_sec(2);
  rclcpp::sleep_for(duration_sec);
  //ros::Duration(2).sleep();
  if (can_id == wheel.can_id)
  {
    isConnect = true;
  }

  if (!isConnect)
  {
    //ROS_ERROR("%s: No se encontro el dispositivo %d", node_name.c_str(),wheel.can_id);
    RCLCPP_ERROR(node->get_logger(), "No se encontro el dispositivo %d",wheel.can_id);
    return 0;
  }

  //ROS_INFO("%s: Dispositivo %d conectado en el puerto: %s",node_name.c_str(), wheel.can_id, wheel.port.c_str());
  //ROS_INFO("Kp:  %d, Ki: %d, Kd: %d",wheel.kp, wheel.ki, wheel.kd);


  //-----PUBLISHERS----//
  // auto wheel_pub = node->create_publisher<controller_msg_srv::msg::Wheel>(node_name+"/data", 10); 
  //-------------------//

  //-----SUCRIBERS-----//
  // auto wheel_sub = node->create_subscription<std_msgs::msg::Int32>(node_name+"/control_speed", 10, setSpeed);
  //auto wheel_sub = node->create_subscription<nav_msgs::msg::Odometry>("wheel/odom", 10, odom_callback);

  //-------------------//
  //-----SERVICES------//
  //ros::ServiceServer srv_rueda = n.advertiseService("action/driver", servicio_callback);
  
  //auto srv_rueda = node->create_service<controller_msg_srv::srv::Wheeldriver>("action/driver", servicio_callback);

  rclcpp::Service<controller_msg_srv::srv::Wheeldriver>::SharedPtr srv_rueda = node->create_service<controller_msg_srv::srv::Wheeldriver>("action/driver",  &servicio_callback);
  rclcpp::Service<controller_msg_srv::srv::Setgain>::SharedPtr srv_set_gain = node->create_service<controller_msg_srv::srv::Setgain>("gain/set",  &setGainCallback);
  rclcpp::Service<controller_msg_srv::srv::Getgain>::SharedPtr srv_get_gain = node->create_service<controller_msg_srv::srv::Getgain>("gain/get",  &getGainCallback);
  //ros::ServiceServer srv_set_gain = n.advertiseService("gain/set", setGainCallback);
  //ros::ServiceServer srv_get_gain = n.advertiseService("gain/get", getGainCallback);

  //-------------------//
  rclcpp::Rate loop_rate(rate);
  controller_msg_srv::msg::Wheel msg;

  //Set speedMode
  wheel.setSpeedMode();
  serial.write(wheel.spm_msg);
  
  rclcpp::sleep_for(duration_ms);
  //ros::Duration(0.010).sleep();

  wheel.configAcc(20, 20);
  serial.write(wheel.cfg_msg);
  //std::chrono::milliseconds duration_ms(10);
  rclcpp::sleep_for(duration_ms);
  //ros::Duration(0.010).sleep();

  //Set speed to 0 m/s
  wheel.setSpeed(0);
  serial.write(wheel.sSpeed_msg);
  //std::chrono::milliseconds duration_ms(10);
  rclcpp::sleep_for(duration_ms);
  //ros::Duration(0.010).sleep();

  //Start motor
  wheel.start();
  serial.write(wheel.str_msg);
  //std::chrono::milliseconds duration_ms(10);
  rclcpp::sleep_for(duration_ms);
  //ros::Duration(0.010).sleep();

  //ROS_INFO("Driver %s configurado", node_name.c_str());

  while (rclcpp::ok())
  {

    if(!isOccupied){
      wheel.keepRunning();
      serial.write(wheel.alv_msg);
      //std::chrono::milliseconds duration_ms(10);
      rclcpp::sleep_for(duration_ms);
      //ros::Duration(0.010).sleep();
    }

    //////////////////////////////////////////////////
    msg.ticks = wheel.ticks;
    msg.speed = wheel.speed;
    msg.status = wheel.status;
    msg.voltage = wheel.voltage;
    wheel_pub->publish(msg);

    //////////////////////////////////////////////////
    //ros::spinOnce();
    rclcpp::spin_some(node);

    loop_rate.sleep();
  }

  return 0;
}

void received(const char *data, unsigned int len)
{
  vector<char> v(data, data + len);
  wheel.received(data, len);
}


//void setSpeed(const std_msgs::msg::Int32::ConstPtr &msg)
void setSpeed(const std_msgs::msg::Int32 & msg)
{
  int speed = msg.data;
  if (speed>0)speed+=1;
  if (speed<0)speed-=1;
  wheel.setSpeed(speed);
  serial.write(wheel.sSpeed_msg);
}

void shutdown_rutine(int sig)
{
  wheel.setSpeed(0);
  serial.write(wheel.sSpeed_msg);
  //std::chrono::milliseconds duration_ms(10);
  rclcpp::sleep_for(duration_ms);
  //ros::Duration(0.010).sleep();
  serial.close();
  //std::chrono::milliseconds duration_ms(10);
  rclcpp::sleep_for(duration_ms);
  //ros::Duration(0.010).sleep();
  //ROS_INFO("Puerto serial cerrado");
  rclcpp::shutdown();
}

bool servicio_callback(const std::shared_ptr<controller_msg_srv::srv::Wheeldriver::Request> req_msg,     // CHANGE
          std::shared_ptr<controller_msg_srv::srv::Wheeldriver::Response>       res_msg)  // CHANGE
{
    
    isOccupied=true;
    if(req_msg->todo=="stop"){
      wheel.stop();
      serial.write(wheel.stp_msg);
      res_msg->status = true;
    } else if (req_msg->todo=="start"){
      wheel.start();
      serial.write(wheel.str_msg);
      res_msg->status = true;
    }else if (req_msg->todo=="clear"){
      wheel.clearAlarms();
      serial.write(wheel.alm_msg);
      res_msg->status = true;
    }else{
      res_msg->status = false;
    }
    isOccupied=false;
    return true;
}

bool setGainCallback(const std::shared_ptr<controller_msg_srv::srv::Setgain::Request> req_msg,     // CHANGE
          std::shared_ptr<controller_msg_srv::srv::Setgain::Response>       res_msg)  // CHANGE
{    
    isOccupied=true;
    if (req_msg->name=="ki")
    {
      wheel.setGain(1,req_msg->value);
      serial.write(wheel.kiG_msg);
    }
    else if (req_msg->name=="kd")
    {
      wheel.setGain(2,req_msg->value);
      serial.write(wheel.kdG_msg);
    }
    else
    {
      wheel.setGain(3,req_msg->value);
      serial.write(wheel.kpG_msg);
    }
    //std::chrono::milliseconds duration_ms(10);
    rclcpp::sleep_for(duration_ms);
    //ros::Duration(0.010).sleep();

    wheel.getParams();
    serial.write(wheel.gParams_msg);
    //std::chrono::seconds duration_sec(2);
    rclcpp::sleep_for(duration_sec);
    //ros::Duration(2).sleep();
    isOccupied=false;

    if (req_msg->name=="ki")
    {
      res_msg->status = wheel.ki==req_msg->value;
    }
    else if (req_msg->name=="kd")
    {
      res_msg->status = wheel.kd==req_msg->value;
    }
    else
    {
      res_msg->status = wheel.kp==req_msg->value;
    }

    //ROS_INFO("Kp:  %d, Ki: %d, Kd: %d",wheel.kp, wheel.ki, wheel.kd);

    return true;
}

bool getGainCallback(const std::shared_ptr<controller_msg_srv::srv::Getgain::Request> req_msg,     // CHANGE
          std::shared_ptr<controller_msg_srv::srv::Getgain::Response>       res_msg)  // CHANGE
{    
//bool getGainCallback(controller_msg_srv::srv::Getgain::Request &req_msg,controller_msg_srv::srv::Getgain::Response &res_msg){

    isOccupied=true;
    wheel.getParams();
    serial.write(wheel.gParams_msg);
    //std::chrono::seconds duration_sec(2);
    rclcpp::sleep_for(duration_sec);
    //ros::Duration(2).sleep();
    isOccupied=false;

    res_msg->kp =wheel.kp;
    res_msg->ki = wheel.ki;
    res_msg->kd = wheel.kd;
    return true;
}
