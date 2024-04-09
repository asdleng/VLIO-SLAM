/**
 * @file Relocalization.cpp
 * @author Jianghao Leng (lengjianghao2006@163.com)
 * @brief 
 * LiDAR-based relocalization based on keyframes
 * @version 0.1
 * @date 2022-10-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <aloam_velodyne/common.h>
#include <aloam_velodyne/tic_toc.h>

#include <scancontext/Scancontext.h>
#include <sstream>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trans_probal_score/trans_probal_score.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
using namespace std;
SCManager scManager;
string lidar_topic;
string ndt_score_topic;
string KeyFramePath;
string KeyFrameTxtPath;
std::vector<Pose6D> keyframePoses;
std::vector<double> keyframeTimes;
std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
Pose6D closest_pose;
pcl::PCDReader pcd_reader;
fstream readTxt;
std::deque<sensor_msgs::PointCloud2ConstPtr> pcl_buf;
std::mutex mBuf;
double scDistThres;
pcl::VoxelGrid<PointType> downSizeFilterScancontext;
geometry_msgs::PoseWithCovarianceStamped initial_guess;
trans_probal_score::trans_probal_score ndt_score;
ros::Publisher pubinitialpose;
ros::Time start;
ros::Time end;
double RestrictHeading(double heading){
	while(heading>=2*M_PI){
		heading = heading - 2*M_PI;
	}
    while(heading<0){
        heading = heading+2*M_PI;
    }
    return heading;
}
// 比较
typedef struct NODE {
    double dis;
    int shift;
    int i;
}node;
bool comp(node & a, node & b) {
    return a.dis < b.dis;//若前一个元素小于后一个元素，则返回真，否则返回假，即可自定义排序方式
}
vector<node> dis_seq;
void shiftpose(Pose6D &pose,int shift){
    auto delta_yaw = shift* 2*M_PI/scManager.PC_NUM_SECTOR;
    pose.yaw = RestrictHeading(pose.yaw+delta_yaw);
}
bool loadKeyFrames(){
    readTxt.open(KeyFrameTxtPath,ios::in);
    string line_str;
    stringstream ss;
    while(getline(readTxt, line_str)){      
        for(int i = 0; i < line_str.size(); i++) {
            if(line_str[i] == ',') line_str[i] = ' ';
        }
        ss.clear();
        ss<<line_str;
        string sub_str;
        Pose6D pose;
        ss >> sub_str;pose.x = stod(sub_str);
        ss >> sub_str;pose.y = stod(sub_str);
        ss >> sub_str;pose.z = stod(sub_str);
        ss >> sub_str;pose.roll = stod(sub_str);
        ss >> sub_str;pose.pitch = stod(sub_str);
        ss >> sub_str;pose.yaw = stod(sub_str);
        keyframePoses.push_back(pose);
    }
    for(int i=0;i<keyframePoses.size();i++){
        // cout<<keyframePoses.at(i).x<<",";
        // cout<<keyframePoses.at(i).y<<",";
        // cout<<keyframePoses.at(i).z<<",";
        // cout<<keyframePoses.at(i).roll<<",";
        // cout<<keyframePoses.at(i).pitch<<",";
        // cout<<keyframePoses.at(i).yaw<<","<<endl;
        auto one_frame_path = KeyFramePath+"/Scans/"+to_string(i)+".pcd";
        pcl::PointCloud<PointType>::Ptr pointcloud(new pcl::PointCloud<PointType>());
        pcl::io::loadPCDFile<PointType> (one_frame_path, *pointcloud);
        keyframeLaserClouds.push_back(pointcloud);
    }
}
void findkeyframe(){
        if(pcl_buf.size()>0){
            pcl::PointCloud<PointType>::Ptr cur_pcl(new pcl::PointCloud<PointType>());
            mBuf.lock();
            pcl::fromROSMsg(*pcl_buf.back(),*cur_pcl);
            pcl_buf.pop_back();
            downSizeFilterScancontext.setInputCloud(cur_pcl);
            downSizeFilterScancontext.filter(*cur_pcl);
            auto cur_sc = scManager.makeScancontext(*cur_pcl);
            mBuf.unlock();
            // cout<<"====New Scan in===="<<endl;
            double min_dis = 1.0;
            int closest_ind = -1;
            dis_seq.clear();
            for(int i=0;i<keyframeLaserClouds.size();i++){
                auto keyframe_sc = scManager.makeScancontext(*keyframeLaserClouds.at(i));
                auto dis_shift_pair = scManager.distanceBtnScanContext(cur_sc,keyframe_sc);
                double dis = dis_shift_pair.first;
                int shift = dis_shift_pair.second;
                node n ={dis,shift,i};
                dis_seq.push_back(n);
                // cout<<dis<<", ";
            }
            sort(dis_seq.begin(), dis_seq.end(),comp);
            // cout<<"closest 10 key frames:"<<endl;
            // for(int i=0;i<10;i++){
            //     cout<<dis_seq.at(i).i<<": "<<dis_seq.at(i).x<<", ";
            // }
        }
}
void pub_initial_pose(){
    initial_guess.header.frame_id = "world";
    auto q = tf::createQuaternionMsgFromRollPitchYaw(closest_pose.roll, closest_pose.pitch,closest_pose.yaw);//返回四元数
    initial_guess.pose.pose.orientation = q;
    initial_guess.pose.pose.position.x = closest_pose.x;
    initial_guess.pose.pose.position.y = closest_pose.y;
    initial_guess.pose.pose.position.z = closest_pose.z;
    pubinitialpose.publish(initial_guess);
}
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg){
    mBuf.lock();
    pcl_buf.push_back(msg);
    if(pcl_buf.size()>10){
        pcl_buf.pop_front();
    }
    mBuf.unlock();
}
void ndtScoreHandler(const trans_probal_score::trans_probal_score::ConstPtr &msg){
    ndt_score = *msg;
}
void process_relocaliztion(void){
    ros::Rate rate(10);
    bool first_frame = true;
    int try_num=0;
    bool need_pub = false;
    bool bad_flag = false;
    while(ros::ok()){
        if(pcl_buf.size()==0){
            continue;
        }
        findkeyframe();
        if(ndt_score.score<1.6&&try_num<5){
            bad_flag = true;
            if(first_frame){
                first_frame = false;

                start = ros::Time::now();
                need_pub = true;
            }
            auto now = ros::Time::now();
            if((now-start).toSec()>3){
                try_num++;
                start = ros::Time::now();
                need_pub = true;
            }
            if(need_pub){
                cout<<"Try "<<try_num+1<<": keyframe"<<dis_seq.at(try_num).i<<endl;
                closest_pose = keyframePoses.at(dis_seq.at(try_num).i);
                shiftpose(closest_pose,dis_seq.at(try_num).shift);
                pub_initial_pose();
                need_pub = false;
            }
            
        }
        else if(ndt_score.score>=1.6&&try_num<5){
            if(bad_flag){
                cout<<"FIND INITIAL POSE of NDT: keyframe"<<dis_seq.at(try_num).i<<endl;
                bad_flag = false;
            }
            first_frame = true;
            try_num = 0;
        }
        else{
            cout<<"COULD NOT FIND INITIAL POSE, PLEASE MOVE AROUND!!!!"<<endl;
            first_frame = true;
            try_num = 0;
        }
        rate.sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relocalization");
	ros::NodeHandle nh;

    nh.param<string>("KeyFramePath",KeyFramePath,"/home/i/VLIO/src/VLIO/PGO/keyframes");
    nh.param<string>("lidar_topic",lidar_topic,"/point_cloud");
    nh.param<string>("ndt_score_topic", ndt_score_topic,"/transform_probability");

    nh.param<double>("sc_dist_thres", scDistThres, 0.2);  
    downSizeFilterScancontext.setLeafSize(0.4,0.4,0.4);
    KeyFrameTxtPath = KeyFramePath+"/keyframes.txt";
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, laserCloudHandler);
    ros::Subscriber subndtscore = nh.subscribe<trans_probal_score::trans_probal_score>(ndt_score_topic, 100, ndtScoreHandler);
    pubinitialpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
    loadKeyFrames();

    thread findkeyframe{process_relocaliztion};

    ros::spin();
    return 0;
}
