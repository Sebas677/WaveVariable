#include "omega_wv_interface.hpp"


OmegaWVInterface::Parameters  OmegaWVInterface::retrieveParameters() {
  Parameters params;
  // Retrieve time delay and impedance parameters
  if (!n.getParam("b", params.b)) {
    ROS_ERROR_STREAM("wv_b from omega_wv_td_interface failed to retrieve ");
  }
  //______________________________________________________________
  if (!n.getParam("wv_b/x", params.wv_b.x)) {
    ROS_ERROR_STREAM("wv_b_x from omega_wv_td_interface failed to retrieve ");
  }
  if (!n.getParam("wv_b/y", params.wv_b.y)) {
    ROS_ERROR_STREAM("wv_b_y from omega_wv_td_interface failed to retrieve ");
  }
  if (!n.getParam("wv_b/z", params.wv_b.z)) {
    ROS_ERROR_STREAM("wv_b_z from omega_wv_td_interface failed to retrieve");
  }
  //______________________________________________________________
  if (!n.getParam("t_delay", params.t_D)){
    ROS_ERROR_STREAM("Failed to retrieve time delay FACTOR from config file");
  }
  //______________________________________________________________
  if (!n.getParam("f_gain", params.f_gain)) {
    ROS_ERROR_STREAM("Failed to get force gain ");
  }
  if (!n.getParam("damp", params.damp)) {
    ROS_WARN_STREAM("failed to retrieve damp val");
  }
  //______________________________________________________________
  // Retrieve FORCE PARAM DATA FOR TESTING
  if (!n.getParam("forces/transform", params.forces[0])){
    ROS_ERROR_STREAM("Failed to retrieve force transformation param" << params.forces[0]);
  }
  if (!n.getParam("forces/vel", params.forces[1])){
    ROS_ERROR_STREAM("Failed to retrieve force for Vs vel param" << params.forces[1]);
  }
  if (!n.getParam("forces/vs", params.forces[2])){
    ROS_ERROR_STREAM("Failed to retrieve Vs force param" << params.forces[2]);
  }
  return params;
}


bool flag_S = true;

void OmegaWVInterface::getMethodCallback(const boost::shared_ptr<const ohrc_bilateral_ctrl_tutorial::method_<std::allocator<void> > >& data) {
  Parameters params = retrieveParameters();
  double t_D = params.t_D;
  inputUm = data->wv;
  inputImp = data->impedance;
  accumUm.push_back(inputUm);
  accumImp.push_back(inputImp);

  if (flag_S) {
    oldestUm = accumUm.front();
    oldestImp = accumImp.front();
    if (accumUm.size() >= t_D) {
      // Retrieve and use the oldest value

      accumUm.erase(accumUm.begin());
      accumImp.erase(accumImp.begin());
    }
  }
  else {
    accumUm.clear();
    //accumImp.clear();
  }
}

geometry_msgs::Vector3 toroVel_linear;
geometry_msgs::Vector3 transform(geometry_msgs::Vector3 wave, Vector3d  force,geometry_msgs::Vector3 b) {
  //linear velocity
  toroVel_linear.x = (wave.x * sqrt(2 * b.x) - force[0])/b.x;
  toroVel_linear.y = (wave.y * sqrt(2 * b.y) - force[1])/b.y;
  toroVel_linear.z = (wave.z * sqrt(2 * b.z) - force[2])/b.z;
  return toroVel_linear;
}

geometry_msgs::Point point_pq;
geometry_msgs::Point integrate(geometry_msgs::Vector3 toroVel_linear,double dt,geometry_msgs::Vector3 b, geometry_msgs::Vector3 vs, geometry_msgs::Vector3 um) {
  //_______________________________________________integral part operation
  //
  point_pq.x += (1/(sqrt(2*b.x))) * ((um.x*dt)-(vs.x* dt)) ;
  //
  point_pq.y += (1/(sqrt(2*b.y))) * ((um.y*dt)-(vs.y* dt)) ;
  //
  point_pq.z += (1/(sqrt(2*b.z))) * ((um.z*dt)-(vs.z* dt)) ;

  //point_pq.x += (1/(sqrt(2*b.x))) * (toroVel_linear.x * dt);
  // point_pq.y += (1/(sqrt(2*b.y))) * (toroVel_linear.y * dt);
  //point_pq.z += (1/(sqrt(2*b.z))) * (toroVel_linear.z * dt);

  return point_pq;
}

geometry_msgs::Vector3 mid_linear;
geometry_msgs::Vector3 vel_Vs(geometry_msgs::Vector3 wave, Vector3d  force, geometry_msgs::Vector3 b) {
  //FOLLOWING THE SCHEME, THE /b IS NOT USED FOR VELOCITY
  mid_linear.x = (wave.x * sqrt(2 * b.x) - force[0]) ;
  mid_linear.y = (wave.y * sqrt(2 * b.y) - force[1]) ;
  mid_linear.z = (wave.z * sqrt(2 * b.z) - force[2]) ;
  return mid_linear;
}
geometry_msgs::Vector3 obtWV_vs;
geometry_msgs::Vector3 getVs(geometry_msgs::Vector3 mid_vel_linear, Vector3d force,geometry_msgs::Vector3 b){
  //_______________________________________________________ changed used equation
  obtWV_vs.x=(force[0] - mid_vel_linear.x) / sqrt(2 * b.x) ;
  obtWV_vs.y=(force[1] - mid_vel_linear.y) / sqrt(2 * b.y) ;
  obtWV_vs.z=(force[2] - mid_vel_linear.z) / sqrt(2 * b.z) ;
  return obtWV_vs;
}


//????????????????????? flag to set subscriber
bool Pub_Flag = true;
int force[3];
double b,t_D,f_gain, damp;
geometry_msgs::Vector3 wv_b;
//?????????????????????
void OmegaWVInterface::setParam(){
  if (Pub_Flag)
    pubImpedance = n.advertise<ohrc_bilateral_ctrl_tutorial::method>("/backwardWave/Toroboarm/Filtered", 6);
    obtUm = n.subscribe<ohrc_bilateral_ctrl_tutorial::method>(
    "/forwardWave/Omega/Filtered", 6,
    [this](const boost::shared_ptr<const ohrc_bilateral_ctrl_tutorial::method_<std::allocator<void>>>& data)
    {
        this->getMethodCallback(data);
    });

  Pub_Flag = false;
    //__________________________________________RETRIEVE PARAMS FROM FUNCTION
  Parameters params = retrieveParameters();
  b = params.b;
  wv_b = params.wv_b;
  t_D = params.t_D;
  damp = params.damp;
  f_gain = params.f_gain;
  force[3];
  force[0] = params.forces[0];
  force[1] = params.forces[1];
  force[2] = params.forces[2];
}

geometry_msgs::Vector3 res_Vs, vel_toPos,imp_send, prev_Vel, bg_WV,Yprev, Yout;
geometry_msgs::Point prev_Pos;
ohrc_bilateral_ctrl_tutorial::method data;
Vector3d eefForce_eef, eefForce_base, eefForce_omega;
Affine3d T_cur, T_des;
void OmegaWVInterface::modifyTargetState(ohrc_msgs::State& state) {

  setParam(); 

  state.enabled = state.gripper.button;

  point_pq=prev_Pos; //VERY IMPORTANT DECLARATION
  toroVel_linear=prev_Vel; //VERY IMPORTANT DECLARATION 2

  //____________________________________________declaration of force related variables
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);
  tf::transformKDLToEigen(frame, T_cur); 
  //_______________________________________________________________________FORCE DECLARATIONS//
  eefForce_eef = tf2::fromMsg(controller->getForceEef().wrench).head(3);                    //
  //USE GAIN TO REDUCE EXPONENTIAL INCREASE OF FORCE                                       //
  for (int i = 0; i < eefForce_eef.size(); ++i) {                                         //
    eefForce_eef[i] *= f_gain;                                                           //
  }                                                                                     //
  eefForce_base = T_cur.rotation() * eefForce_eef;                                     //
  eefForce_omega = T_state_base.rotation().transpose() * eefForce_base; //CORRECT!    //
  std::vector<Vector3d> force_array{eefForce_eef, eefForce_base, eefForce_omega};    //
  //______________________________________________________________FORCE DECLARATIONS//
  bg_WV = wv_b;

  if (state.enabled==true){
    if (oldestUm.x == 0 && oldestUm.y == 0 && oldestUm.z == 0){
      wv_b = wv_b;
    }
    else{
      wv_b = oldestImp;
    }
    //__________________________overwrite state VARIABLES
    state.wave = oldestUm;
    state.twist.linear = transform(state.wave, force_array[force[0]], wv_b);

    state.pose.position = integrate(state.twist.linear, this->dt, wv_b, res_Vs, state.wave);
    
    state.twist = geometry_msgs::Twist();
    state.pose.orientation= geometry_msgs::Quaternion();

    vel_toPos = vel_Vs(state.wave, force_array[force[1]],wv_b);
    res_Vs = getVs(vel_toPos, force_array[force[2]], wv_b);

    imp_send = wv_b;
    //_________________________________________EXPONENTIAL FIRST ORDER FILTER
    Yout.x = damp * Yprev.x + (1-damp) * res_Vs.x;
    Yout.y = damp * Yprev.y + (1-damp) * res_Vs.y;
    Yout.z = damp * Yprev.z + (1-damp) * res_Vs.z;
    Yprev = Yout;

    data.wv = Yout;
    data.impedance = imp_send;

    pubImpedance.publish (data);

    flag_S=true;
  }

  prev_Pos = state.pose.position;
  
  if (state.enabled ==false){

    //____________________array of pointers to set to 0  all non state variables
    geometry_msgs::Vector3* variables[] = {&mid_linear, &obtWV_vs,
                                        &vel_toPos, &res_Vs, &oldestUm, &toroVel_linear};

    for (auto variable : variables) {
        variable->x = 0;
        variable->y = 0;
        variable->z = 0;
    }

    point_pq= prev_Pos;
    toroVel_linear  = geometry_msgs::Vector3();
    //point_pq = geometry_msgs::Point();
    //___________________________________________________________
    
    //________________bool to clear data from delay
    flag_S=false;
  }
  prev_Vel = state.twist.linear;

}