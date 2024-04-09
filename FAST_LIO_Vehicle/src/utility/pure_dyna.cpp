#include <ros/ros.h>
#include <Eigen/Core>
#include <fast_lio_vehicle/dyna.h>
#include <sensor_msgs/PointCloud2.h>
//#include <preprocess.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <parameters.h>
#include <common_lib.h>
#include <sensor_msgs/Imu.h>


#include<cstdlib>

#include<ctime>

using namespace Eigen;
using namespace std;
nav_msgs::Path tra;
nav_msgs::Path tra_imu;
nav_msgs::Path tra_gps;

fast_lio_vehicle::dyna last_point;
sensor_msgs::Imu last_imu_point;
sensor_msgs::Imu first_imu_point;
nav_msgs::Odometry first_gps_point;

geometry_msgs::PoseStamped last_tmp;
geometry_msgs::PoseStamped last_imu_tmp;

Vector3f last_imu_vel;
ros::Publisher tra_pub;
ros::Publisher tra_imu_pub;
ros::Publisher tra_gps_pub;

bool is_first=true;
bool is_imu_first = true;
bool is_gps_first = true;

int j=0;
int k=0;
int n=0; 

Matrix3f rot_gps;
Matrix3f cor_rot;

double calbeta(double vel,double alpha){
  return (1-m/2/l*l_f/l_r/k_r*vel*vel)/(1+K*vel*vel)*l_r/l*alpha;
}
double calomega(double vel, double alpha){
    if(vel==0||alpha==0){
        return 0;
    }
    return vel/(1+K*vel*vel)/l*alpha;
}
void chassis_cbk(const fast_lio_vehicle::dyna::ConstPtr& msg){
    if(is_first){
        is_first=false;
        last_point = *msg;
        geometry_msgs::PoseStamped tmp;
        last_tmp = tmp;
        tra.poses.push_back(tmp);
        tra.header.frame_id = "/camera_init";
        return;
    }
    double beta = calbeta(msg->vel,msg->alpha);
    double dt = (msg->stamp-last_point.stamp).toSec();

    Quaternionf q_0(last_tmp.pose.orientation.w,
    last_tmp.pose.orientation.x,
    last_tmp.pose.orientation.y,
    last_tmp.pose.orientation.z);
    Matrix3f rot = q_0.toRotationMatrix();

    Vector3f vel, omega;
    vel << msg->vel*1, 0, 0;
    omega << 0, 0, calomega(msg->vel,msg->alpha);
    
    vel = rot*vel;
    rot = rot*Exp(omega,dt);
    // cout<<"观测："<<endl;
    // cout<<"--时间间隔(s):"<<dt<<endl;
	// cout<<"--前轮转角(rad):"<<msg->alpha<<endl;
	// cout<<"--车速(m/s):"<<msg->vel<<endl;
	// cout<<"                       计算："<<endl;
	// cout<<"                       --角速度(rad/s)："<<omega.transpose()<<endl;
	// cout<<"                       --质心侧偏角(rad)："<<beta<<endl;
	// cout<<"                       --地面速度(x y z)(m/s):"<<vel.transpose()<<endl;
    geometry_msgs::PoseStamped tmp;
    tmp.pose.position.x = last_tmp.pose.position.x+vel(0)*dt;
    tmp.pose.position.y = last_tmp.pose.position.y+vel(1)*dt;
    tmp.pose.position.z = last_tmp.pose.position.z+vel(2)*dt;
    Quaternionf q(rot);
    tmp.pose.orientation.x = q.x();
    tmp.pose.orientation.y = q.y();
    tmp.pose.orientation.z = q.z();
    tmp.pose.orientation.w = q.w();
    
    last_tmp = tmp;
    last_point = *msg;
    if (j % 30 == 0){
        tra.poses.push_back(tmp);
        tra_pub.publish(tra);
    }
    j++;
}
void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg){
        if(is_imu_first){
            is_imu_first=false;
            last_imu_point = *msg;
            
            last_imu_vel<<0,0,0;
            geometry_msgs::PoseStamped tmp;
            last_imu_tmp = tmp;
            tra_imu.poses.push_back(tmp);
            tra_imu.header.frame_id = "/camera_init";
            // 给gps用
            first_imu_point = *msg;
            Quaternionf q_gps(first_imu_point.orientation.w,
            first_imu_point.orientation.x,
            first_imu_point.orientation.y,
            first_imu_point.orientation.z);
            rot_gps = q_gps.toRotationMatrix();
            AngleAxisf V1(M_PI, Vector3f(0, 0, 1));
            rot_gps = V1*rot_gps;
            return;
    }
    double dt = (msg->header.stamp - last_imu_point.header.stamp).toSec();
    
    Quaternionf q_0(last_imu_tmp.pose.orientation.w,
    last_imu_tmp.pose.orientation.x,
    last_imu_tmp.pose.orientation.y,
    last_imu_tmp.pose.orientation.z);
    Matrix3f rot = q_0.toRotationMatrix();

    Vector3f acc, vel, omega;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    omega << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    acc = rot*acc;
    acc(2) = acc(2)-9.8;
    rot = rot*Exp(omega,dt);

    vel = last_imu_vel+acc*dt;
    //cout<<"地面速度："<<vel.transpose()<<endl;
    geometry_msgs::PoseStamped tmp;
    tmp.pose.position.x = last_imu_tmp.pose.position.x+last_imu_vel(0)*dt+0.5*acc(0)*dt*dt;
    tmp.pose.position.y = last_imu_tmp.pose.position.y+last_imu_vel(1)*dt+0.5*acc(1)*dt*dt;
    tmp.pose.position.z = last_imu_tmp.pose.position.z+last_imu_vel(2)*dt+0.5*acc(2)*dt*dt;
    Quaternionf q(rot);
    tmp.pose.orientation.x = q.x();
    tmp.pose.orientation.y = q.y();
    tmp.pose.orientation.z = q.z();
    tmp.pose.orientation.w = q.w();
    last_imu_vel = vel;
    last_imu_tmp = tmp;
    last_imu_point = *msg;
    if (k % 100 == 0){
        tra_imu.poses.push_back(tmp);
        tra_imu_pub.publish(tra_imu);
    }
    k++;
}
// void gps_cbk(const nav_msgs::Odometry::ConstPtr& msg){
//     if(is_gps_first){
//             is_gps_first=false;
//             first_gps_point = *msg;
//             geometry_msgs::PoseStamped tmp;
//             tra_gps.poses.push_back(tmp);
//             tra_gps.header.frame_id = "/camera_init";
//             return;
//     }
//     geometry_msgs::PoseStamped tmp;
//     double x = msg->pose.pose.position.x - first_gps_point.pose.pose.position.x;
//     double y = msg->pose.pose.position.y - first_gps_point.pose.pose.position.y;
//     Vector3f pos_0,pos_1;
//     pos_0<<x, y, 0;
//     pos_1 = cor_rot*rot_gps*pos_0;
//     tmp.pose.position.x = pos_1(0);
//     tmp.pose.position.y = pos_1(1);
//     tmp.pose.position.z = pos_1(2);

//     if (n % 100 == 0){
//         tra_gps.poses.push_back(tmp);
//         tra_gps_pub.publish(tra_gps);
//     }
//     n++;
// }

int main(int argc, char **argv){   
    ros::init(argc, argv, "pure_dyna");
    ros::NodeHandle nh;
    std::string imu_topic;
    nh.param<std::string>("common/imu_topic", imu_topic, "/imu2");
    ros::Subscriber sub_chassis = nh.subscribe("/dyna", 2000, chassis_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 2000, imu_cbk);
    // ros::Subscriber sub_gps = nh.subscribe("/gps_info",2000, gps_cbk); 
    tra_pub = nh.advertise<nav_msgs::Path>("/pure_dyna_tra",10000);
    tra_imu_pub = nh.advertise<nav_msgs::Path>("/pure_imu_tra",10000);
    tra_gps_pub = nh.advertise<nav_msgs::Path>("/pure_gps_tra",10000);
    cor_rot.setIdentity();
    srand((unsigned)time(NULL));

    ros::spin();
    return 0;
}
