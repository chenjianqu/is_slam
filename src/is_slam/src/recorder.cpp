/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of is_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include  <iostream>
#include <fstream>  
#include <string>

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

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;


class CallBackClass
{
public:
	CallBackClass(string base_path_);
	void CallBack(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

	tf::TransformListener* listener;
    

protected:
	unsigned long frameCounter;
    vector<int> compression_params;

	string base_path;

};




int main(int argc, char** argv)
{
	setlocale(LC_ALL, "");//防止中文乱码
	ros::init(argc, argv, "Recorder_Node");//初始化节点
	ros::start();//启动节点
	
	ros::NodeHandle nh;
	
	CallBackClass cb(argv[1]);//初始化

	//接受RGB图和深度图
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/ps/rgb", 10);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/ps/depth", 10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
	sync.registerCallback(boost::bind(&CallBackClass::CallBack,&cb,_1,_2));//_1表示消息1，_2表示消息2
	
	ROS_INFO_STREAM("Recorder节点初始化完成");
	
	ros::spin();
	
	ROS_INFO_STREAM("Recorder节点结束");
		
	return 0;
}

CallBackClass::CallBackClass(string base_path_)
{
	frameCounter=0;
	listener=new tf::TransformListener();//tf监听器

	base_path=base_path_;

	//清空记录
	ofstream ofs(base_path+"/poses.txt", ios::trunc);
	ofs.close();


	compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);//png压缩质量
	compression_params.push_back(0); //最高质量
}


void CallBackClass::CallBack(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	//获取相机位姿
	Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();
	ros::Time timestamp= msgRGB->header.stamp;
	tf::StampedTransform m;
	try{
		//最长等待3s，直到该坐标变换出现
		listener->waitForTransform("world", "orb_slam2",timestamp, ros::Duration(1.0));
		listener->lookupTransform("world", "orb_slam2", timestamp, m);//查找坐标转换
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
    }
	tf::transformTFToEigen(m, Tcw);
	
	frameCounter++;
    ROS_INFO_STREAM("Record——帧序号："+to_string(frameCounter));


	//获取图像
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try{
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
	}
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptrD;
    try{
        cv_ptrD = cv_bridge::toCvShare(msgD);
	}
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	cv::Mat color=cv_ptrRGB->image.clone();
	cv::Mat depth=cv_ptrD->image.clone();

	cout<<depth.type()<<endl;

    //写入图像
    cv::imwrite(base_path+"/instance/"+to_string(frameCounter)+".png",color,compression_params);
    //cv::imwrite(base_path+"/depth/"+to_string(frameCounter)+".png",depth,compression_params);
	cv::imwrite(base_path+"/depth/"+to_string(frameCounter)+".exr",depth);
    //写入矩阵
	ofstream ofs(base_path+"/poses.txt", ios::app);
    Eigen::Matrix4d T(Tcw.matrix());
	ofs<<T(0,0)<<" "<<T(0,1)<<" "<<T(0,2)<<" "
        <<T(1,0)<<" "<<T(1,1)<<" "<<T(1,2)<<" "
        <<T(2,0)<<" "<<T(2,1)<<" "<<T(2,2)<<" "
        <<T(0,3)<<" "<<T(1,3)<<" "<<T(2,3)<<endl;
	ofs.close();
	
}


