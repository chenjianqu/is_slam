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
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


using namespace std;


void split(const std::string& source, std::vector<std::string>& tokens, const string& delimiters = " ") {
    std::regex re(delimiters);
    std::copy(std::sregex_token_iterator(source.begin(), source.end(),re,-1), 
        std::sregex_token_iterator(),
        std::back_inserter(tokens));
}



int main(int argc, char** argv)
{
	setlocale(LC_ALL, "");//防止中文乱码
	ros::init(argc, argv, "DataSource");
	ros::start();


	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("正在初始化DataSource节点...");

    string data_path=argv[1];
    

    string isFloatDepth=argv[2];
	
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_rgb=it.advertise("/camera/rgb",1);
    image_transport::Publisher pub_depth=it.advertise("/camera/depth",1);


    vector<string> rgbTimeVector;
    vector<string> rgbPathVector;
    vector<string> depthPathVector;

    int counter=0;

    ifstream fin(data_path+"associate.txt");
    string s;
    while(getline(fin,s))
    {
        vector<string> token;
        split(s,token);
        rgbTimeVector.push_back(token[0]);//时间
        rgbPathVector.push_back(data_path+token[1]);
        depthPathVector.push_back(data_path+token[3]);
    }
    fin.close();
	
	cout<<"帧数量："<<rgbPathVector.size()<<endl;

    cout<<"发布的数据类型："<<endl;
    cout<<"COLOR:bgr8 ";
    if(isFloatDepth=="yes" || isFloatDepth=="y")
        cout<<" "<<"DEPTH:32fc1"<<endl;
    else
        cout<<" "<<"DEPTH:mono16"<<endl;


    ROS_INFO_STREAM("DataSource节点初始化完成");


    int i=0;

	ros::Rate loop_rate(5);
	while(ros::ok() && i<rgbPathVector.size())
	{
		cv::Mat color=cv::imread(rgbPathVector[i],-1);
		cv::Mat depth=cv::imread(depthPathVector[i],-1);

        cout<<i<<depthPathVector[i]<<endl;
		
        std_msgs::Header h;
        double timest=atof(rgbTimeVector[i].c_str());
        h.stamp=ros::Time(timest);
        h.seq=i;

        sensor_msgs::ImagePtr msgColor = cv_bridge::CvImage(h, "bgr8", color).toImageMsg();

        if(isFloatDepth=="yes" || isFloatDepth=="y"){
            cv::Mat dstDepth;
            depth.convertTo(dstDepth,5,1/5000.0);
            sensor_msgs::ImagePtr msgDepth = cv_bridge::CvImage(h,"32FC1",dstDepth).toImageMsg();
            pub_rgb.publish(msgColor);
            pub_depth.publish(msgDepth);
        }
        else
        {
            sensor_msgs::ImagePtr msgDepth = cv_bridge::CvImage(h, "mono16", depth).toImageMsg();
            pub_rgb.publish(msgColor);
            pub_depth.publish(msgDepth);
        }
        
        

		ros::spinOnce();
		loop_rate.sleep();
        i++;
	}

	
	ROS_INFO_STREAM("DataSource节点结束");
		
	return 0;
}