# ohrc_bilateral_ctrl_tutorial

## The functionality of the package depends on the following, please follow the corresponding README: ###

-[OpenHRC].

-[omega_haptic_device] 

-[toroboarm_robot] 


## In order to install this package follow the next instructions in your ROS workspace: ###


```
cd ~ 
cd (your_workspace)/src
git clone https://github.com/itadera/ohrc_bilateral_ctrl_tutorial.git
cd ..
:~/(your_workspace)$ source devel/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release

```

## Your .../src folder should have: ###

```
ohrc_bilateral_ctrl_tutorial |  omega_hatptic_device |  OpenHRC |  toroboarm_robot

```

## In order to test the package you can use the following ROS launch: ###

```
roslaunch ohrc_bilateral_ctrl_tutorial omega_wv_test_delay_teleoperation.launch 
```

<div align="center">
  <img src="https://github.com/itadera/ohrc_bilateral_ctrl_tutorial/assets/73600923/223cd5ac-19f5-4a30-8076-83b55a9eaa6d" width="200" height="200">
</div>




### **This repository intends the development of a WAVE VARIABLE CONTROLLER**

In the following bullet points are listed the state of the art papers used as resources:

+[Bilateral Control by Transmitting Force Information with Application to
Time-delay Systems and Human Motion Reproduction](https://www.jstage.jst.go.jp/article/ieejjia/10/2/10_20004757/_pdf/-char/en).

+[Passivity-based bilateral shared variable impedance control for teleoperation compliant assembly](https://www.sciencedirect.com/science/article/pii/S0957415823001137?ref=pdf_download&fr=RR-2&rr=804eb1858b3fe360).
