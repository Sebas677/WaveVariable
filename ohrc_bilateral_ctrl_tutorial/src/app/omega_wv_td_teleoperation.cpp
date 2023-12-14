#include "ohrc_control/single_interface.hpp"
#include "omega_wv_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "omega_wv_teleoperation_td_node");
  SingleInterface<OmegaWVInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}