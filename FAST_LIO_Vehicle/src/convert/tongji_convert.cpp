#include <ros/ros.h>
#include <Eigen/Core>
#include <fast_lio_vehicle/dyna.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <send_chassisdata/chassis_data.h>

int is_carla;
std::string chassis_topic;
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
bool first_frame=true;
double last_frame_time;
bool first_dyna = true;
bool second_dyna = true;
double last_dyna_time;
double anchor_time;
int dyna_cout=0;
double dyna_delay;
double dyna_offset;
double vel_scale;
std::vector<double> filter_vel(4);
std::vector<double> filter_alpha(4);
std::vector<double> filter_dvel(4);
std::vector<double> filter_dalpha(4);
std::vector<double> filter_ddvel(4);
double filted;
double MovingRMS(std::vector<double> &filter, const double & x){
    filted = filter.front();
    for(int i=0;i<filter.size()-1;i++){
        filter[i] = filter[i+1];
    }
    filter.back() = x;
    double sum = 0;
    for(int i=0;i<filter.size();i++){
        sum+= filter[i]*filter[i];        
    }
    sum = sum/filter.size();
    if(x>0){
        return sqrt(sum);
    }
    else{
        return -sqrt(sum);
    }
    
}
void filter_rollback(std::vector<double> &filter){
    for(int i=0;i<filter.size()-1;i++){
        filter[i+1] = filter[i];
    }
    filter.front() = filted;
}
void chassis_cbk(const send_chassisdata::chassis_data::ConstPtr& msg){
    // if(first_dyna){
    //     last_dyna.stamp = msg->header.stamp;
    //     last_dyna.vel = MovingRMS(filter_vel,msg->Velocity);
    //     last_dyna.alpha = MovingRMS(filter_alpha, -msg->SteeringAngle/10);
    //     first_dyna = false;
    //     return; 
    // }
    // if(second_dyna){
    //     filter_rollback(filter_vel);
    //     last_dyna.stamp = msg->header.stamp;
    //     last_dyna.vel = MovingRMS(filter_vel,msg->Velocity);
    //     last_dyna.alpha = MovingRMS(filter_alpha, -msg->SteeringAngle/10);
    //     second_dyna = false;
    //     return; 
    // }
    auto delay = ros::Duration(dyna_delay);
    dyna.stamp = msg->header.stamp-delay;
    dyna.alpha = -msg->SteeringAngle/10;
    dyna.vel = (msg->Velocity/3.6*vel_scale-dyna_offset);
    // last_dyna = dyna;
    pub_dyna.publish(dyna);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyna_convert");
    ros::NodeHandle nh;
    nh.param<std::string>("common/chassis_topic", chassis_topic, "/chassis_msgs");
    nh.param<double>("mapping/dyna_time_delay",dyna_delay,0.0);
    nh.param<double>("mapping/dyna_offset",dyna_offset,0.0);
    nh.param<double>("mapping/vel_scale",vel_scale,1.0);
    //sub_chassis =  nh.subscribe(chassis_topic, 200000, chassis_cbk);
    sub_chassis_carla = nh.subscribe(chassis_topic,200000,chassis_cbk);
    //sub_point_cloud = nh.subscribe(raw_lidar_topic,200000,lidar_cbk);
    //pub_lidar = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic,10000);
    pub_dyna = nh.advertise<fast_lio_vehicle::dyna>("/dyna",100000);

    ros::spin();
}