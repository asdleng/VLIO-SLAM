#include <ros/ros.h>
#include <Eigen/Core>
#include <fast_lio_vehicle/dyna.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <ctime>
#include <random>

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include<ctime>

int is_carla;
int chassis_rate;
int lidar_rate;
std::string chassis_topic;
std::string raw_carla_lidar_topic;
std::string raw_lidar_topic;
std::string lidar_topic;
std::string carla_chassis_topic;
fast_lio_vehicle::dyna dyna;
fast_lio_vehicle::dyna last_dyna;
ros::Publisher pub_dyna;
ros::Publisher  pub_lidar;
ros::Subscriber sub_chassis;
ros::Subscriber sub_point_cloud;
ros::Subscriber sub_chassis_carla;
int carla_chassis_count = 0;
int carla_chassis_num = 1;
int carla_lidar_count = 0;
int carla_lidar_num = 1;
// 雷神的数据结构
 struct PointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (uint16_t, ring, ring)
                                          (double, time, time)
                                          )
// velodyne数据结构
namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    float time;                     // 时间
    uint16_t ring;                  // 点所属的圈数
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace velodyne_ros

// 注册velodyne_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))
                                       
bool first_frame=true;
double last_frame_time;

bool first_dyna = true;
bool second_dyna = true;
double last_dyna_time;
double anchor_time;
int dyna_cout=0;

void carla_lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    carla_lidar_count++;
    if(carla_lidar_count==carla_lidar_num){
        carla_lidar_count = 0;
        pub_lidar.publish(*msg);
    }
}
void carla_chassis_cbk(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg){
    if(first_dyna){
        last_dyna.stamp = msg->header.stamp;
        last_dyna.vel = msg->velocity;
        last_dyna.alpha = msg->control.steer*1.0; // 1.062
        first_dyna = false;
        return; 
    }
    if(second_dyna){
        last_dyna.stamp = msg->header.stamp;
        last_dyna.vel = msg->velocity;
        last_dyna.alpha = msg->control.steer*1.0; // 1.062
        second_dyna = false;
        return; 
    }
    dyna.vel = msg->velocity;
    dyna.stamp = msg->header.stamp;
    dyna.alpha = -msg->control.steer*1.0; // 1.062
    double dt = (dyna.stamp-last_dyna.stamp).toSec();
    last_dyna = dyna;
    carla_chassis_count++;
    if(dyna.stamp.toSec()!=0&&carla_chassis_count==carla_chassis_num){
        carla_chassis_count=0;
        // 增加误差
        double vel_std = 0.01;
        double alpha_std = 0.01;
        dyna.vel += vel_std * (double(rand())/RAND_MAX-0.5)*2;
        dyna.alpha += alpha_std * (double(rand())/RAND_MAX-0.5)*2;
        // std::cout<<(double(rand())/RAND_MAX-0.5)*2<<std::endl;
        pub_dyna.publish(dyna);
    }
    
}
void lidar_cbk(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    if(first_frame){
        last_frame_time = laserCloudMsg->header.stamp.toSec()*1e6;
        first_frame=false;
        return;
    }
    pcl::PointCloud<PointXYZIRT>::Ptr ls_pl=boost::make_shared <pcl::PointCloud<PointXYZIRT>> ();
    pcl::PointCloud<velodyne_ros::Point> velodyne_pl;
    sensor_msgs::PointCloud2 out_pl;
    pcl::fromROSMsg(*laserCloudMsg, *ls_pl);
    std::sort(ls_pl->begin(), ls_pl->end(),
 [](PointXYZIRT pt1, PointXYZIRT pt2) {return pt1.time < pt2.time; });
    double head_time = ls_pl->begin()->time*1e6;
    double end_time = ls_pl->at(ls_pl->size()-1).time*1e6;
    double diff_time = end_time-head_time;
    double real_diff_time = laserCloudMsg->header.stamp.toSec()*1e6-last_frame_time;
    // ROS_WARN("%f",diff_time);
    // ROS_WARN("%f",real_diff_time);
        velodyne_pl.clear();
    for(double i=0;i<ls_pl->size();i++){
        velodyne_ros::Point p;
        p.x = ls_pl->at(i).x;
        p.y = ls_pl->at(i).y;
        p.z = ls_pl->at(i).z;
        double pro = (ls_pl->points[i].time*1e6-head_time)/diff_time;
        p.time = pro*real_diff_time;

        // ROS_WARN("%f", last_frame_time);
        // ROS_WARN("%f", add);
        // ROS_WARN("%f", p.time);

        p.ring = ls_pl->at(i).ring;
        p.intensity = ls_pl->at(i).intensity;
        velodyne_pl.push_back(p);
    }
    last_frame_time =laserCloudMsg->header.stamp.toSec()*1e6;

    // ROS_WARN("%s","===============");
    // ROS_WARN("%f",laserCloudMsg->header.stamp.toSec());
    // ROS_WARN("%f",velodyne_pl.points.begin()->time);
    // ROS_WARN("%f",velodyne_pl.points.at(ls_pl->size()/2-1).time);
    // ROS_WARN("%f",velodyne_pl.points.at(ls_pl->size()-1).time);
    // ROS_WARN("%s","===============");

    pcl::toROSMsg(velodyne_pl,out_pl);
    out_pl.header = laserCloudMsg->header;
    out_pl.header.frame_id="velodyne";
    pub_lidar.publish(out_pl);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyna_convert");
    ros::NodeHandle nh;
    srand((unsigned)time(NULL));
    nh.param<int>("common/is_carla",is_carla,0);
    nh.param<std::string>("common/chassis_topic", chassis_topic, "/Chassis_control_for_pntcld");
    nh.param<std::string>("common/raw_carla_lid_topic",raw_carla_lidar_topic,"/carla/ego_vehicle/lidar/lidar1/point_cloud");
    nh.param<std::string>("common/lid_topic", lidar_topic, "/point_cloud");
    nh.param<int>("common/chassis_rate", chassis_rate, 100);
    nh.param<int>("common/lidar_rate", lidar_rate, 100);
    
    nh.param<std::string>("common/carla_chassis_topic",carla_chassis_topic, "/carla/ego_vehicle/vehicle_status");
    std::cout<<"+++++++++++++++++++++++++++="<<std::endl;
    std::cout<<is_carla<<std::endl;
    if(is_carla==0){
        std::cout<<"不是carla"<<std::endl;
    }
    else if(is_carla==1){
        std::cout<<"是carla"<<std::endl;
        carla_chassis_num = 100/chassis_rate;
        carla_lidar_num = 100/lidar_rate;
        srand((unsigned)time(NULL));
        carla_chassis_count = rand()%carla_chassis_num;
        carla_lidar_count = rand()%carla_lidar_num;
        std::cout<<"设定lidar频率为："<<lidar_rate<<"Hz"<<std::endl;
        std::cout<<"设定底盘频率为："<<chassis_rate<<"Hz"<<std::endl;
    }
    //sub_chassis =  nh.subscribe(chassis_topic, 200000, chassis_cbk);
    sub_chassis_carla = nh.subscribe(carla_chassis_topic,200000,carla_chassis_cbk);
    sub_point_cloud = nh.subscribe(raw_carla_lidar_topic,200000,carla_lidar_cbk);
    pub_lidar = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic,10000);
    pub_dyna = nh.advertise<fast_lio_vehicle::dyna>("/dyna",100000);
    
    ros::spin();
}