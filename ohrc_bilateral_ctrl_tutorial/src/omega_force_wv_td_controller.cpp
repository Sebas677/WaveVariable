#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "omega_force_wv_driver.hpp"
#include <list>
#include <queue>
#include "ohrc_bilateral_ctrl_tutorial/method.h"

class OmegaForceWVController : public OmegaForceController {
public:
  void modifyOmegaCommand(const ohrc_msgs::State& omega, geometry_msgs::Wrench& inputForce, double& inputGripperForce) override;
  ros::Publisher pubForwardWave, pubExp;
  ros::Subscriber obtVs,obtMethod;
  OmegaForceWVController(int id);
  geometry_msgs::Vector3 inputVs, oldestVs,oldestImp,lastVal, wv_b, inputImp;
  void getMethodCallback(const boost::shared_ptr<const ohrc_bilateral_ctrl_tutorial::method_<std::allocator<void> > >& data);
  double t_D,b,f_gain,damp;
  std::vector<geometry_msgs::Vector3> accumVs,accumImp;
  bool flag= true;
};


OmegaForceWVController::OmegaForceWVController(int id) : OmegaForceController(id) {
  obtMethod = n.subscribe<ohrc_bilateral_ctrl_tutorial::method>(
      "/backwardWave/Toroboarm/Filtered", 6,
      [this](const boost::shared_ptr<const ohrc_bilateral_ctrl_tutorial::method_<std::allocator<void>>>& data)
      {
          this->getMethodCallback(data);
      });
  //pubForwardWave = n.advertise<geometry_msgs::Vector3>("/forwardWave/Omega_filtered", 3);
  pubForwardWave = n.advertise<ohrc_bilateral_ctrl_tutorial::method>("/forwardWave/Omega/Filtered", 6);
  pubExp = n.advertise<geometry_msgs::Vector3>("/forwardWave/Omega", 3);
  if (n.getParam("b", b)) {
    ROS_WARN_STREAM("wv_b from omega_force_wv_td_controller: " << b);
  }
  if (n.getParam("wv_b/x", wv_b.x)) {
    ROS_WARN_STREAM("wv_b_x from omega_force_wv_td_controller: " << wv_b.x);
  }
  if (n.getParam("wv_b/y", wv_b.y)) {
    ROS_WARN_STREAM("wv_b_y from omega_force_wv_td_controller: " << wv_b.y);
  }
  if (n.getParam("wv_b/z", wv_b.z)) {
    ROS_WARN_STREAM("wv_b_z from omega_force_wv_td_controller: " << wv_b.z);
  }
  if (n.getParam("t_delay", t_D)) {
    ROS_WARN_STREAM("t_delay from omega_force_wv_td_controller: " << t_D);
  }
  if (n.getParam("f_gain", f_gain)) {
    ROS_WARN_STREAM("force gain from omega_force_wv_td_controller: " << f_gain);
  }
  if (n.getParam("damp", damp)) {
    ROS_WARN_STREAM("damp for filter: " <<damp);
  }
}

void OmegaForceWVController::getMethodCallback(const boost::shared_ptr<const ohrc_bilateral_ctrl_tutorial::method_<std::allocator<void> > >& data) {
  inputVs = data->wv;
  inputImp = data->impedance;
  accumVs.push_back(inputVs);
  accumImp.push_back(inputImp);

  if (flag) {
    // Retrieve and use the oldest value
    oldestVs = accumVs.front();
    oldestImp = accumImp.front();
    if (accumVs.size() >= t_D) {
      accumVs.erase(accumVs.begin());
      accumImp.erase(accumImp.begin());
    }
  } else {
    accumVs.clear();
    //accumImp.clear();
  }
}

geometry_msgs::Vector3 setForce;
geometry_msgs::Vector3 forceInput(const ohrc_msgs::State& omega, geometry_msgs::Vector3 b_mod, double f_gain, geometry_msgs::Vector3 Vs){
  setForce.x = f_gain*((b_mod.x * omega.twist.linear.x) + (Vs.x * sqrt(2 * b_mod.x) ));
  setForce.y = f_gain*((b_mod.y * omega.twist.linear.y) + (Vs.y * sqrt(2 * b_mod.y) ));
  setForce.z = f_gain*((b_mod.z * omega.twist.linear.z) + (Vs.z * sqrt(2 * b_mod.z) ));
  return setForce;
}

geometry_msgs::Vector3 modWave;
geometry_msgs::Vector3 writeWave(const ohrc_msgs::State& omega, geometry_msgs::Vector3 b_mod, geometry_msgs::Vector3 forceSet){
  //
  modWave.x = (forceSet.x + b_mod.x * omega.twist.linear.x) / sqrt(2 * b_mod.x) ;
  //
  modWave.y = (forceSet.y + b_mod.y * omega.twist.linear.y) / sqrt(2 * b_mod.y) ;
  //
  modWave.z = (forceSet.z + b_mod.z * omega.twist.linear.z) / sqrt(2 * b_mod.z) ;

  return modWave;
}



geometry_msgs::Vector3 imp,error, p, prevImp,prevGain,adaptGain, errorMag;
bool initialFlag = true;
geometry_msgs::Vector3 getError(geometry_msgs::Vector3 resVs,geometry_msgs::Vector3 resUm, geometry_msgs::Vector3 bg, geometry_msgs::Vector3 wb ){

  if (initialFlag ==true){
    prevImp = bg;
    prevGain.x = 1;
    prevGain.y = 1;
    prevGain.z = 1;
    initialFlag = false;
  }
  std::cout<<prevImp<<std::endl;
  //RESPECTING LEADER AND FOLLOWER SYSTEM
  error.x = resUm.x - resVs.x;
  error.y = resUm.y - resVs.y;
  error.z = resUm.z - resVs.z;

  errorMag.x = abs(error.x);
  errorMag.y = abs(error.y);
  errorMag.z = abs(error.z);

  const double ERROR_THRESHOLD = 0.05;
  const double ADAPT_GAIN_RATE = 0.01; //the lower the better
  const double IMPEDANCE_LOW_THRESHOLD = 1;
  const double IMPEDANCE_UPPER_THRESHOLD = 35.0;

    // For X dimension
  if (errorMag.x > ERROR_THRESHOLD && error.x > ERROR_THRESHOLD) {
      std::cout << "BACK (X)" << std::endl;

  } else if (errorMag.x > ERROR_THRESHOLD && error.x < -ERROR_THRESHOLD) {
      std::cout << "FORWARD (X)" << std::endl;

  } else if (errorMag.x < ERROR_THRESHOLD && -errorMag.x > -ERROR_THRESHOLD) {
      std::cout << "CENTER (X)" << std::endl;
  }
  // For Y dimension
  if (errorMag.y > ERROR_THRESHOLD && error.y > ERROR_THRESHOLD) {
      std::cout << "RIGHT (Y)" << std::endl;
  } else if (errorMag.y > ERROR_THRESHOLD && error.y < -ERROR_THRESHOLD) {
      std::cout << "LEFT (Y)" << std::endl;
  } else if (errorMag.y < ERROR_THRESHOLD && -errorMag.y > -ERROR_THRESHOLD) {
      std::cout << "CENTER (Y)" << std::endl;
  }
  //For Z dimension
  if (errorMag.z > ERROR_THRESHOLD && error.z > ERROR_THRESHOLD) {
    std::cout << "UP (Z)" << std::endl;
  } 
  else if (errorMag.z > ERROR_THRESHOLD && error.z < -ERROR_THRESHOLD) {
    std::cout << "DOWN (Z)" << std::endl;
  } 
  else if ( errorMag.z < ERROR_THRESHOLD && -errorMag.z > -ERROR_THRESHOLD){
    std::cout << "CENTER (Z)" << std::endl;
  }


  adaptGain.x = 0.02;
  adaptGain.y = 0.02;
  adaptGain.z = 0.02;

  prevGain.x = adaptGain.x;
  prevGain.y = adaptGain.y;
  prevGain.z = adaptGain.z;

  p.x = adaptGain.x * errorMag.x;
  p.y = adaptGain.y * errorMag.y;  
  p.z = adaptGain.z * errorMag.z;


  if (errorMag.x > ERROR_THRESHOLD && error.x > ERROR_THRESHOLD || errorMag.y > ERROR_THRESHOLD && error.y > ERROR_THRESHOLD ||errorMag.z > ERROR_THRESHOLD && error.z > ERROR_THRESHOLD) {

      imp.x = prevImp.x - p.x;
      imp.y = prevImp.y - p.y;
      imp.z = prevImp.z - p.z;

  } else if (errorMag.x > ERROR_THRESHOLD && error.x < -ERROR_THRESHOLD || errorMag.y > ERROR_THRESHOLD && error.y < -ERROR_THRESHOLD || errorMag.z > ERROR_THRESHOLD && error.z < -ERROR_THRESHOLD) {

      imp.x = prevImp.x - p.x;
      imp.y = prevImp.y - p.y;
      imp.z = prevImp.z - p.z;

  } else if (errorMag.x < ERROR_THRESHOLD && -errorMag.x > -ERROR_THRESHOLD || errorMag.y < ERROR_THRESHOLD && -errorMag.y > -ERROR_THRESHOLD||errorMag.z < ERROR_THRESHOLD && -errorMag.z > -ERROR_THRESHOLD) {
      
      adaptGain.x = 0.01;
      adaptGain.y = 0.01;
      adaptGain.z = 0.01;

      prevGain.x = adaptGain.x;
      prevGain.y = adaptGain.y;
      prevGain.z = adaptGain.z;
      
      p.x = adaptGain.x * errorMag.x;
      p.y = adaptGain.y * errorMag.y;
      p.z = adaptGain.z * errorMag.z;

      imp.x = prevImp.x + p.x;
      imp.y = prevImp.y + p.y;
      imp.z = prevImp.z + p.z;
  }

  double impAvg = (imp.x + imp.y + imp.z ) / 3;

  if (abs(imp.x) >= IMPEDANCE_UPPER_THRESHOLD) {
    imp.x = imp.x - impAvg;
    std::cout << "over imp x" << std::endl;
  }
  if (abs(imp.y) >= IMPEDANCE_UPPER_THRESHOLD) {
    imp.y = imp.y - impAvg;
    std::cout << "over imp y" << std::endl;
  }
  if (abs(imp.z) >= IMPEDANCE_UPPER_THRESHOLD) {
    imp.z = imp.z - impAvg;
    std::cout << "over imp z" << std::endl;
  }

  if (abs(imp.x) < IMPEDANCE_LOW_THRESHOLD) {
    imp.x += 1;
    std::cout << "abs of imp x" << std::endl;
  }
  if (abs(imp.y) < IMPEDANCE_LOW_THRESHOLD) {
    imp.y += 1;
    std::cout << "abs of imp y" << std::endl;
  }
  if (abs(imp.z) < IMPEDANCE_LOW_THRESHOLD) {
    imp.z += 1;
    std::cout << "abs of imp z" << std::endl;
  }
  prevImp= imp;
  return imp;
}


bool ema_flag=false;
ohrc_bilateral_ctrl_tutorial::method data;
geometry_msgs::Vector3 Um_wave, inputForce_omega, Yprev, Yout, imp_send, bg_WV;
void OmegaForceWVController::modifyOmegaCommand(const ohrc_msgs::State& omega, geometry_msgs::Wrench& inputForce, double& inputGripperForce) {
  Yprev = Um_wave;
  //bg_WV = wv_b;
  if (omega.gripper.button == true) {    
    modWave=omega.wave; //important to initialize the values!!!!
    setForce = omega.wrench.force;

    if (oldestVs.x == 0 && oldestVs.y == 0 && oldestVs.z == 0){
      bg_WV = wv_b;
    }
    else{
      bg_WV = oldestImp;
    }
    inputForce_omega = forceInput(omega,bg_WV,f_gain,oldestVs);
    Um_wave = writeWave(omega, bg_WV, inputForce_omega);
    //_________________________________________EXPONENTIAL FIRST ORDER FILTER
    Yout.x = damp * Yprev.x + (1-damp) * Um_wave.x;
    Yout.y = damp * Yprev.y + (1-damp) * Um_wave.y;
    Yout.z = damp * Yprev.z + (1-damp) * Um_wave.z;
    Yprev = Yout;
    //
    imp_send= getError(oldestVs, Um_wave, bg_WV, wv_b);
    //imp_send = bg_WV;

    data.wv = Yout;
    data.impedance = imp_send;
    pubForwardWave.publish(data);
    //pubExp.publish(Um_wave);

    inputForce.force.x=inputForce_omega.x;
    inputForce.force.y=inputForce_omega.y;
    inputForce.force.z=inputForce_omega.z;

    flag=true;
    
  }
  
  if (omega.gripper.button == false){

    geometry_msgs::Vector3* variables[] = {&inputForce.force, &Um_wave,
                                        &oldestVs, &Yprev, &Yout};

    for (auto variable : variables) {
        variable->x = 0;
        variable->y = 0;
        variable->z = 0;
    }

    prevImp = bg_WV;
    imp = bg_WV;
    imp_send = bg_WV;
    
    flag=false;
  }
}

void* cyclic_Task(void* arg) {
  int id = *((int*)arg);
  OmegaForceWVController OmegaForceWVController(id);
  if (OmegaForceWVController.control() < 0)
    ROS_ERROR("Failed to something");

  return nullptr;
}

//////////////////////////////////////////////
int main(int argc, char** argv) {
  ros::init(argc, argv, "omega_force_driver");
  ros::NodeHandle n;

  int nDevice = dhdGetDeviceCount();
  ROS_ERROR_STREAM("Start Omega WV initialization: " << nDevice << " device(s) detected");

  std::vector<pthread_t> thread_cyclic_loop(nDevice);
  for (int i = 0; i < nDevice; i++) {
    int device = i;
    pthread_create(&thread_cyclic_loop[i], NULL, cyclic_Task, &device);
    // dhdStartThread(cyclic_Task, &device, DHD_THREAD_PRIORITY_HIGH);
  }
  // loop in the real-time thread

  for (int i = 0; i < nDevice; i++)
    pthread_join(thread_cyclic_loop[i], nullptr);

  return 0;
}