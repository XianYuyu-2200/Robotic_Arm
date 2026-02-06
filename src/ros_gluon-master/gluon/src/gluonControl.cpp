/*
*执行器位置控制
*/

#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

#define STEERING_GEAR_RATIO 36

#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)

using namespace std;
using std::placeholders::_1;

class GluonControlNode : public rclcpp::Node
{
public:
  GluonControlNode() : Node("gluonControl")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&GluonControlNode::jointStatesCallback, this, _1));
    
    // Initialize Controller
    pController = ActuatorController::initController();
    Actuator::ErrorsDefine ec;
    uIDArray = pController->lookupActuators(ec);
    
    if (uIDArray.size() > 0)
    {
      for(size_t k = 0; k < uIDArray.size(); k++) {
        ActuatorController::UnifiedID actuator = uIDArray.at(k);
        RCLCPP_INFO(this->get_logger(), "actuator ID %d, ipAddr %s", actuator.actuatorID, actuator.ipAddress.c_str());
        pController->enableActuator(actuator.actuatorID, actuator.ipAddress);
        pController->activateActuatorMode(actuator.actuatorID, Actuator::Mode_Profile_Pos);
        
        cout << "set position to 0" << endl;
        pController->setPosition(actuator.actuatorID, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Connected error code: %x", ec);
    }
    
    // Create a timer to run the control loop at 50Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&GluonControlNode::controlLoop, this));
  }

private:
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    s_jointStatesList.push(*msg);
    if (s_jointStatesList.size() >= 10) {
      s_jointStatesList.pop();
      RCLCPP_WARN(this->get_logger(), "jointStatesCallback, pop");
    }
  }

  void controlLoop()
  {
    if (!s_jointStatesList.empty() && uIDArray.size() > 0) {
      sensor_msgs::msg::JointState js_data = s_jointStatesList.front();
      s_jointStatesList.pop();
      
      size_t count = std::min((size_t)6, std::min(js_data.position.size(), uIDArray.size()));

      for(size_t i = 0; i < count; i++) {
        ActuatorController::UnifiedID actuator = uIDArray.at(i);
        RCLCPP_INFO(this->get_logger(), "jointstate_%d, %s, %f", (int)i+1, js_data.name[i].c_str(), js_data.position[i]);
        pController->setPosition(actuator.actuatorID, RAD_TO_POS(js_data.position[i]));
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  ActuatorController * pController;
  std::vector<ActuatorController::UnifiedID> uIDArray;
  std::queue<sensor_msgs::msg::JointState> s_jointStatesList;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GluonControlNode>());
  rclcpp::shutdown();
  return 0;
}
