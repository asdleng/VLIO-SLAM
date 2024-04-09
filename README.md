# VLIO-SLAM

一个用于地面车辆的SLAM系统，融合IMU、激光雷达、底盘车速和GPS，采用基于迭代卡尔曼滤波的Vehicle-LiDAR-Inertial-Odometry前端，后端基于ScanContext实现回环检测，添加GPS因子，完成位姿图优化。

![img](http://szari.tpddns.cn:8088/asdleng/VLIO-SLAM/src/branch/master/PGO/data/tongji.png)

## Prerequisites

- Ubuntu 18.04 and ROS Melodic
- Eigen
- GTSAM

## Build

```
cd YOUR_WORKSPACE/src
git clone ssh://git@szari.tpddns.cn:8089/asdleng/VLIO-SLAM.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES='can_msg'
catkin_make -DCATKIN_WHITELIST_PACKAGES='livox_ros_driver'
catkin_make -DCATKIN_WHITELIST_PACKAGES='send_chassisdata'
catkin_make -DCATKIN_WHITELIST_PACKAGES='fast_lio_vehicle'
catkin_make -DCATKIN_WHITELIST_PACKAGES='aloam_velodyne'
catkin_make -DCATKIN_WHITELIST_PACKAGES='trans_form_score'
```

## Start

### Frontend

下载[同济大学数据集](https://pan.baidu.com/s/1cEF_t9K2zv6DVMBgwVkCvA?dp-logid=71905100281487500002&pwd=u0nl#/home/%2F/%2F)，运行

```
roslaunch fast_lio_vehicle mapping_tongji_cycle.launch
```

或者使用Autoad录制的bag

```
roslaunch fast_lio_vehicle mapping_autoad.launch
```

然后播放bag

### Backend

运行

```
roslaunch aloam_velodyne fastlio_velodyne_VLP_16.launch
```

## Relocalization

**22/10/12更新：**

![img](http://szari.tpddns.cn:8088/asdleng/VLIO-SLAM/src/branch/master/PGO/data/relocalization.gif)

新增重定位/全局定位模块

- Backend运行结束后会记录下优化后的关键帧及其位姿，位于`PGO/keyframes/Scans`下
- Backend运行结束后会记录下优化后的点云地图，即`PGO/data/map.pcd`
- relocalization模块运行时基于关键帧及其位姿进行计算，为NDT定位提供`/initialpose`话题

注意：ndt_localization须使用相同的点云地图

运行

```
roslaunch aloam_velodyne relocalization.launch
```

## Acknowledgements

该仓库来自 [FAST-LIO](https://github.com/hku-mars/FAST_LIO) 与 [FAST-LIO-LC](https://github.com/yanliang-wang/FAST_LIO_LC)的内容融合，添加了一个地面观测和车速观测
