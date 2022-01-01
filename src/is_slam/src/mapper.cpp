/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of is_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <list>
#include <thread>
#include <regex>

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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>


#include "is_slam/semanticmap.h"

// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;



class CallBackClass
{
public:
	CallBackClass(const string &strSettingsFile,const string &dataSaveFile);
	void CallBack(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
	void CallBackPose(const std_msgs::String::ConstPtr& msg);

	PointCloud::Ptr BuildPointCloud(cv::Mat &color,cv::Mat &depth);
	
	tf::TransformListener* listener;
	
	SemanticMap *semanticMap;
	std::thread* semanticMapThread;

protected:
	unsigned long frameCounter;

	//相机内参
	float fx,fy,cx,cy;
	float depthFactor;

	pcl::ApproximateVoxelGrid<PointT> *voxelFilter01; 
};



void split(const std::string& source, std::vector<std::string>& tokens, const string& delimiters = " ") {
    std::regex re(delimiters);
    std::copy(std::sregex_token_iterator(source.begin(), source.end(),re,-1), 
        std::sregex_token_iterator(),
        std::back_inserter(tokens));
}


double stringToNum(string str)
{
	istringstream iss(str);
	double num;
	iss >> num;
	return num;    
}



int main(int argc, char** argv)
{
	setlocale(LC_ALL, "");//防止中文乱码
	ros::init(argc, argv, "mapper_node");//初始化节点
	ros::start();//启动节点
	
	if(argc != 3)
    {
        cout<<"需要传入参数：配置文件路径 数据保存位置" << endl;
        ros::shutdown();//关闭节点
        return 1;
    }
	
	ros::NodeHandle nh;

	
	ROS_INFO_STREAM("正在初始化Mapper节点...");

	CallBackClass cb(argv[1],argv[2]);//初始化

	//接受RGB图和深度图
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/ps/rgb", 10);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/ps/depth", 10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
	sync.registerCallback(boost::bind(&CallBackClass::CallBack,&cb,_1,_2));//_1表示消息1，_2表示消息2
	
	//接收位姿
	ros::Subscriber poseSub = nh.subscribe("/pose", 1000, &CallBackClass::CallBackPose,&cb);

	//发布全局点云
	ros::Publisher pubPointCloud=nh.advertise<PointCloud>("/point_cloud/raw",1);


	ROS_INFO_STREAM("Mapper节点初始化完成");
	
	int counter=0;

	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		counter++;

		//每隔30s而且全局点云更新的话，则发布点云
		if(counter%20*10==0 && cb.semanticMap->GetIsUpdateMap())
		{
			cb.semanticMap->SetIsUpdateMap(false);
			PointCloud::Ptr cloud=cb.semanticMap->GetPointCloud();
			pubPointCloud.publish(*cloud);
			ROS_INFO_STREAM("发布稠密点云");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	cb.semanticMap->SetRun(false);
	
	ros::Duration(3).sleep();
	
	ROS_INFO_STREAM("Mapper节点结束");
		
	return 0;
}



CallBackClass::CallBackClass(const string &strSettingsFile,const string &dataSaveFile)
{
	frameCounter=0;
	listener=new tf::TransformListener();//tf监听器

	semanticMap=new SemanticMap(dataSaveFile);

	semanticMapThread = new thread(&SemanticMap::Mapping, semanticMap);//线程启动
	ROS_INFO_STREAM("已启动语义地图线程");


	//相机内参
	cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
	depthFactor=fSettings["Camera.factor"];


	voxelFilter01 = new pcl::ApproximateVoxelGrid<PointT>;
	voxelFilter01->setDownsampleAllData(false);
	voxelFilter01->setLeafSize( 0.01, 0.01, 0.01);
}







void CallBackClass::CallBackPose(const std_msgs::String::ConstPtr& msg)
{
	string poseString=msg->data.c_str();

	//切分
	vector<string> lines;
    split(poseString,lines,"\n");
	
	semanticMap->SetPoseUpdate(false);

	//转换为位姿，并保存到map中
	for(int i=0;i<lines.size();i++)
	{
		string line=lines[i];
		vector<string> tokens;
		split(line,tokens);
		
		Eigen::Vector3d t(stringToNum(tokens[1]),stringToNum(tokens[2]),stringToNum(tokens[3]));
		Eigen::Quaterniond q(stringToNum(tokens[7]),stringToNum(tokens[4]),stringToNum(tokens[5]),stringToNum(tokens[6]));
		Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
		pose.pretranslate(t);
		pose.rotate(q);
		
		//原始时间戳有9位小数，这里只保留6位
		string timestamp=tokens[0];
		timestamp=timestamp.substr(0,timestamp.length()-3);

		semanticMap->InsertPose(pose,timestamp);
	}
	semanticMap->SetPoseUpdate(true);
	ROS_INFO_STREAM("接收到位姿："+to_string(lines.size()));
	//cout<<poseString<<endl<<endl;
}


void CallBackClass::CallBack(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	frameCounter++;

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

	//点云构建
	PointCloud::Ptr cpc=BuildPointCloud(color,depth);
	
	ros::Time timestamp= msgRGB->header.stamp;
	std::string timestampString=to_string(timestamp.sec)+"."+to_string(timestamp.nsec).substr(0, 6);//保留6位小数
	
	//插入关键帧
	semanticMap->InsertFrame(cpc,timestampString);

	ROS_INFO_STREAM("插入关键帧："+to_string(frameCounter));
}





PointCloud::Ptr CallBackClass::BuildPointCloud(cv::Mat &color,cv::Mat &depth)
{
	PointCloud::Ptr cpc(new PointCloud);
	for ( int v=0; v<color.rows; v++ )
	{
		for ( int u=0; u<color.cols; u++ )
		{
			float d = depth.ptr<float> (v)[u]; // 深度值
			if ( d==0 ) continue; // 为0表示没有测量到
			if ( d >= 4.0 ) continue; // 深度太大时不稳定，去掉
			Eigen::Vector3d point; 
			point[2] = double(d)/depthFactor; 
			point[0] = (u-cx)*point[2]/fx;
			point[1] = (v-cy)*point[2]/fy; 
			//设点P,在相机坐标为Pc，世界坐标为Pw，该相机位姿为:Tcw，那么有：Pc=Tcw*Pw，Pw=Twc*Pc
			PointT p ;
			p.x = point[0];
			p.y = point[1];
			p.z = point[2];
			//p.z=0;
			p.b = color.data[ v*color.step+u*color.channels() ];
			p.g = color.data[ v*color.step+u*color.channels()+1 ];
			p.r = color.data[ v*color.step+u*color.channels()+2 ];
			cpc->points.push_back(p);
		}
	}

	//帧体素滤波
	PointCloud::Ptr pfpc ( new PointCloud );
	voxelFilter01->setInputCloud( cpc );
	voxelFilter01->filter( *pfpc );
	
	return pfpc;
}

