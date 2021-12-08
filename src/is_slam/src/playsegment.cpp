#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <list>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/common/common.h>
#include <pcl/common/centroid.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;

int main(int argc, char** argv)
{
	if(argc != 4)
    {
        cout<<"需要传入参数：rgb图像路径 深度图像路径 位姿路径" << endl;
        ros::shutdown();//关闭节点
        return 1;
    }



	setlocale(LC_ALL, "");//防止中文乱码
	ros::init(argc, argv, "Player");//初始化节点
	ros::start();//启动节点
	
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("正在初始化data player线程...");

    string data_path="/media/chen/chen/Robot/Data/InstanceSegmentation/";
	
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_rgb=it.advertise("/ps/rgb",1);
    image_transport::Publisher pub_depth=it.advertise("/ps/depth",1);
    tf::TransformBroadcaster br;
	
	
    int N=90;//读取数量
	int Bias=0;//偏置

    //读取位姿
	ifstream fin(argv[3]);
	vector<Eigen::Isometry3d> Tv;
    for(int i=1;i<=N+Bias;i++)
	{
		double data[12] = {0};
        for ( auto& d:data )
            fin>>d;
		Eigen::Matrix3d m;
		m<<data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8];
		Eigen::Vector3d v;
		v<<data[9],data[10],data[11];
		Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();;
		Tcw.rotate(m);
		Tcw.pretranslate(v);
		Tv.push_back(Tcw);
    }
    fin.close();

    ROS_INFO_STREAM("Player节点初始化完成");


    int i=0;

	ros::Rate loop_rate(5);
	while(ros::ok())
	{
        if(i>90)
            break;
		cout<<"深度图像："<<i<<endl;
		cv::Mat color=cv::imread(data_path+"rgb/"+to_string(i+1+Bias)+".png",-1);
		cv::Mat depth=cv::imread(data_path+"depth/"+to_string(i+1+Bias)+".exr",-1);
		//Eigen::Isometry3d Twc=Tv[i+Bias].inverse();//求逆变换
        Eigen::Isometry3d Tcw=Tv[i+Bias];

        //发布坐标变换
        tf::Transform m;
	    tf::transformEigenToTF(Tcw,m);
        br.sendTransform(tf::StampedTransform(m, ros::Time(0), "world", "orb_slam2"));
		
        std_msgs::Header h;
        h.stamp=ros::Time(0);
        sensor_msgs::ImagePtr msgColor = cv_bridge::CvImage(h, "bgr8", color).toImageMsg();
        sensor_msgs::ImagePtr msgDepth = cv_bridge::CvImage(h, "32FC1", depth).toImageMsg();

        pub_rgb.publish(msgColor);
        pub_depth.publish(msgDepth);

		ros::spinOnce();
		loop_rate.sleep();
        i++;
	}

	
	ROS_INFO_STREAM("Player节点结束");
		
	return 0;
}