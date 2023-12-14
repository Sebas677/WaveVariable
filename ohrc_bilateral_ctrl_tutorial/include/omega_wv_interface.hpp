#ifndef OMEGA_WV_INTERFACE_HPP
#define OMEGA_WV_INTERFACE_HPP

#include "ohrc_teleoperation/omega_interface.hpp"
#include <list>
#include <queue>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include "ohrc_bilateral_ctrl_tutorial/method.h"


class OmegaWVInterface : public OmegaInterface {
  ros::Publisher pubOmegaVs, pubImpedance;
  ros::Subscriber obtUm, obtImp;
protected:
  void modifyTargetState(ohrc_msgs::State& state) override;
  // void feedback(const KDL::Frame& targetPos, const KDL::Twist& targetTwist) override;
  
public:
  using OmegaInterface::OmegaInterface;
  std::vector<geometry_msgs::Vector3> accumUm,accumImp;
  geometry_msgs::Vector3 inputUm,oldestUm,inputImp, oldestImp,lastVal,acVs;

  void getMethodCallback(const boost::shared_ptr<const ohrc_bilateral_ctrl_tutorial::method_<std::allocator<void> > >& data);
  void setParam();

  struct Parameters {
    double b;
    geometry_msgs::Vector3 wv_b;
    double t_D;
    double damp;
    double f_gain;
    int forces[3];
  };
  Parameters retrieveParameters();
  //double t_D;
  // OmegaWVInterface();
};



#endif  // OMEGA_WV_INTERFACE_HPP