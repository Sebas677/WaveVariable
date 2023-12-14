#ifndef OMEGA_FORCE_DRIVER_HPP
#define OMEGA_FORCE_DRIVER_HPP

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "ohrc_bilateral_ctrl_tutorial/method.h"
#include "omega_haptic_device/omega_driver.hpp"

class OmegaForceController : public OmegaDriver {
  ros::Subscriber subForce, subGripperForce, subFollowerEnergy;
  ros::Publisher pubEnergy;
  std::mutex mtx;

  geometry_msgs::Wrench _inputForce;
  double _inputGripperForce = 0.0;

    //IS IT REQUIRED TO UNCOMMENT THIS LINES IN ORDER TO SUBSCRIBE?

  //OmegaCommand(ros::NodeHandle n, std::string deviceName) {
    //obtVs = n.subscribe<geometry_msgs::Point>("/Vs_feed", 3,  &OmegaForceController::getVsCallback, this);
  //   subForce = n.subscribe<geometry_msgs::Wrench>(deviceName + "/cmd_force", 2, &OmegaCommand::cbForce, this, ros::TransportHints().tcpNoDelay(true));
  //   subGripperForce = n.subscribe<std_msgs::Float32>(deviceName + "/cmd_gripper_force", 2, &OmegaCommand::cbGripperForce, this);

  //   pubWave = n.advertise<std_msgs::Float32MultiArray>(deviceName + "/wave_variable", 2);
  //}

  void cbForce(const geometry_msgs::Wrench::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx);
    _inputForce = *msg;
  }

  void cbGripperForce(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx);
    _inputGripperForce = msg->data;
  }

  // void cbEnergy(const std_msgs::Float32::ConstPtr& msg) {
  //   std::lock_guard<std::mutex> lock(mtx3);
  //   _followerEnergy = msg->data;
  // }

  // std::vector<std::unique_ptr<OmegaCommand>> omageCmd;

  bool sendCommandToOmega(const ohrc_msgs::State& omega) override;



public:
virtual void modifyOmegaCommand(const ohrc_msgs::State& omega, geometry_msgs::Wrench& inputForce, double& inputGripperForce){};
  OmegaForceController(int i);
  ~OmegaForceController(){};
};

#endif  // OMEGA_FORCE_DRIVER_HPP