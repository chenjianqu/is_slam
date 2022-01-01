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
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 

#include"System.h"


using namespace std;


typedef pcl::PointXYZ PointT; 
typedef pcl::PointCloud<PointT> PointCloud;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
	void SetPublisher(image_transport::Publisher* pub_rgb_,image_transport::Publisher* pub_depth_,ros::Publisher* pub_pc_);
	
	 ORB_SLAM2::System* mpSLAM;
	unsigned long counter;
	unsigned long counterKeyFrame;
protected:
   
	tf::TransformBroadcaster* br;
	image_transport::Publisher* pub_rgb;
	image_transport::Publisher* pub_depth;
	ros::Publisher *pub_pc;
	
	double costSumTime;
	
	void MatToTransform(cv::Mat &Tcw,tf::Transform &m);
	void WritePose(cv::Mat &Tcw,ros::Time timestamp,string fname);
	PointCloud::Ptr MatToPointCloud(vector<cv::Mat> v);
	
};


int main(int argc, char** argv)
{
	setlocale(LC_ALL, "");//防止中文乱码
	ros::init(argc, argv, "orb_slam_node");//初始化节点
	ros::start();//启动节点
	if(argc != 6)
    {
        cout<<"需要传入参数：视觉词典路径 配置文件路径 轨迹保存路径" << endl;
        ros::shutdown();//关闭节点
        return 1;
    }
	ROS_INFO_STREAM("正在初始化ORB-SLAM...");
	//初始化ORB-SLAM2
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,false);
	ImageGrabber igb(&SLAM);
	
	ros::NodeHandle nh;

	//接受RGB图和深度图
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[4], 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[5], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_rgb = it.advertise("/orbslam2/rgb", 1);
	image_transport::Publisher pub_depth = it.advertise("/orbslam2/depth", 1);

	ros::Publisher pub_pc=nh.advertise<PointCloud>("/point_cloud/sparse",1);
	ros::Publisher pub_pose=nh.advertise<std_msgs::String>("/pose",1);
	
	igb.SetPublisher(&pub_rgb,&pub_depth,&pub_pc);
	
	ROS_INFO_STREAM("ORB-SLAM节点初始化完成");
	
	bool isPoseUpdate=false;
	ros::Rate loop_rate(30);
	while(ros::ok())
	{
		//进行位姿图优化或全局优化之后
		if(igb.mpSLAM->MapChanged())
		{
			string poseString=igb.mpSLAM->GetAllKeyFramesString();
			std_msgs::String msg;
			msg.data=poseString;
			pub_pose.publish(msg); 
			ROS_INFO_STREAM("ORB-SLAM——***地图改变***");
		}
		
		//因为每次轮循counter的值不一定变，因此以这样的方式判断
		if(igb.counter%30==27)
			isPoseUpdate=true;

		if(isPoseUpdate==true && igb.counter%30==28)
		{
			string poseString=igb.mpSLAM->GetAllKeyFramesString();
			ROS_INFO_STREAM("ORB-SLAM——位姿更新");
			//ROS_INFO_STREAM(poseString);
			
			std_msgs::String msg;
			msg.data=poseString;
			pub_pose.publish(msg);  

			isPoseUpdate=false;



			PointCloud::Ptr pointCloud(new PointCloud );
			vector<cv::Mat> vec=igb.mpSLAM->GetGlobalMapPoints();
			int len=vec.size();
			for(std::vector<cv::Mat>::iterator it=vec.begin();it!=vec.end();++it)
			{
				PointT p;
				p.x = (*it).at<float>(0,0);
				p.y = (*it).at<float>(1,0);
				p.z = (*it).at<float>(2,0);
				pointCloud->points.push_back(p);
			}
			pub_pc.publish(*pointCloud);
			ROS_INFO_STREAM("ORB-SLAM——发布稀疏地图 大小："+to_string(len));
		}


		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	SLAM.Shutdown();
	SLAM.SaveKeyFrameTrajectoryTUM(argv[3]);
	ros::shutdown();
	ROS_INFO_STREAM("关闭ORB-SLAM节点");
	return 0;
}


ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM):
	mpSLAM(pSLAM),
	counter(0),
	counterKeyFrame(0),
	costSumTime(0.0)
{
	br=new tf::TransformBroadcaster();


	//清空记录
	ofstream ofs0("/home/chen/orbslam.txt", ios::trunc);
	ofs0.close();
	ofstream ofs1("/home/chen/orbslam_keyframe.txt", ios::trunc);
	ofs1.close();

}

void ImageGrabber::SetPublisher(image_transport::Publisher* pub_rgb_,image_transport::Publisher* pub_depth_,ros::Publisher* pub_pc_)
{
	pub_rgb=pub_rgb_;
	pub_depth=pub_depth_;
	pub_pc=pub_pc_;
}

void ImageGrabber::MatToTransform(cv::Mat &Tcw,tf::Transform &m)
{
	//设置平移
	m.setOrigin(
		tf::Vector3(
			Tcw.at<float>(0,3),
			Tcw.at<float>(1,3),
			Tcw.at<float>(2,3)
		)
	);
	
	//设置旋转
	tf::Matrix3x3 Rcw;
	Rcw.setValue( //Mat转换为Matrix
		Tcw.at<float>(0,0),Tcw.at<float>(0,1),Tcw.at<float>(0,2),  
		Tcw.at<float>(1,0),Tcw.at<float>(1,1),Tcw.at<float>(1,2),
		Tcw.at<float>(2,0),Tcw.at<float>(2,1),Tcw.at<float>(2,2)
	);
	
	tf::Quaternion q;
	Rcw.getRotation(q);
	m.setRotation(q);	
}


void ImageGrabber::WritePose(cv::Mat &Tcw,ros::Time timestamp,string fname)
{
	tf::Matrix3x3 Rcw;
	Rcw.setValue( //Mat转换为Matrix
		Tcw.at<float>(0,0),Tcw.at<float>(0,1),Tcw.at<float>(0,2),  
		Tcw.at<float>(1,0),Tcw.at<float>(1,1),Tcw.at<float>(1,2),
		Tcw.at<float>(2,0),Tcw.at<float>(2,1),Tcw.at<float>(2,2)
	);
	
	tf::Quaternion q;
	Rcw.getRotation(q);

	Eigen::Quaterniond qe;
	tf::quaternionTFToEigen (q, qe);

	//写入矩阵
	ofstream ofs(fname, ios::app);
	ofs<<timestamp.sec<<"."<<timestamp.nsec<<" "<<Tcw.at<float>(0,3)<<" "<<Tcw.at<float>(1,3)<<" "<<Tcw.at<float>(2,3)
		<<" "<<qe.x()<<" "<<qe.y()<<" "<<qe.z()<<" "<<qe.w()<<endl;
	ofs.close();
}



PointCloud::Ptr ImageGrabber::MatToPointCloud(vector<cv::Mat> v)
{
	PointCloud::Ptr pointCloud(new PointCloud );
	for(std::vector<cv::Mat>::iterator it=v.begin();it!=v.end();++it)
    {
        PointT p;
		p.x = (*it).at<float>(0,0);
		p.y = (*it).at<float>(1,0);
		p.z = (*it).at<float>(2,0);
		pointCloud->points.push_back(p);
    }
    return pointCloud;
}



void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
	ros::Time timestamp= msgRGB->header.stamp;
	
    // Copy the ros image message to cv::Mat.
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
	
	time_t beginTime=clock();

	//调用ORB-SLAM2
	cv::Mat Tcw=mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
	counterKeyFrame++;
	

	double costTime=double(clock() - beginTime) / CLOCKS_PER_SEC;
	costSumTime+=costTime;
	cout<<"帧跟踪平均时间: "<<costSumTime/counterKeyFrame<<"s"<<endl;
	
	tf::Transform m;
	MatToTransform(Tcw,m);
	//发布坐标
	br->sendTransform(tf::StampedTransform(m, timestamp, "world", "orb_slam2"));

	//WritePose(Tcw,timestamp,"/home/chen/orbslam.txt");
	
	//如果不是关键帧,就退出吧
	if(!mpSLAM->GetIsKeyFrame())
		return;
	
	counter++;
	
	//发布rgb和深度图
	pub_rgb->publish(msgRGB);
	pub_depth->publish(msgD);
	
	
	ROS_INFO_STREAM("ORB-SLAM——关键帧："+to_string(counterKeyFrame));
	
	//WritePose(Tcw,timestamp,"/home/chen/orbslam_keyframe.txt");//关键帧位姿保存
	
}




