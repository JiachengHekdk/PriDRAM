<h1 align="center">
  <span style="font-size: 40px;">DR-PAM</span> <!-- 将字体大小从30px调整到40px -->
  <br> <!-- 在描述文字之前的换行保持不变 -->
  Priority-Based Deadlock Recovery for Distributed Swarm Obstacle Avoidance in Cluttered Environments
</h1>



The framework of DR-PAM is as follow:

![image-20240806011806923](C:\Users\何嘉诚\AppData\Roaming\Typora\typora-user-images\image-20240806011806923.png)

![image-20240806011850026](C:\Users\何嘉诚\AppData\Roaming\Typora\typora-user-images\image-20240806011850026.png)





An introduction video can be seen on  [Bilibili](https://www.bilibili.com/video/BV19DYAe3EVi/?spm_id_from=333.999.list.card_archive.click&vd_source=2421d86d062a73ffa2c05fb6487e7fcf)


## Quick Start

### Installation

```
Installation
This work is implemented in C++17 and tested on ROS Melodic and Noetic.

Install ROS

Install eigen_quad_prog and run sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen.
```

### Preparation

(1) Clone and build the package

```
cd catkin_ws/src
https://github.com/JiachengHekdk/DR-PAM.git
cd .. 
catkin_make  
source devel/setup.bash  
```

(2) Change the parameters of config_5deadlock.yaml according to your settings and run the demo:

```yaml
rosrun dr_pam run_dr_pam
```

### Contact

Jiacheng he (22232021@zju.edu.cn)
