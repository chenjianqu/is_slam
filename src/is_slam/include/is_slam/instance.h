#ifndef INSTANCE_H
#define INSTANCE_H


#include <iostream>
#include <fstream>  
#include <string>
#include<chrono>
#include<thread>
#include <ctime>
#include<vector>
#include<map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/common/common.h>
#include <pcl/common/centroid.h>




// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;


class Instance
{
public:
	typedef shared_ptr<Instance> Ptr;
	Instance();
	Instance(int classId_,int instanceId_,float minX_,float maxX_,float minY_,float maxY_,float minZ_,float maxZ_,PointCloud::Ptr cloud_);
	
	double Volume();
	double Length();
	double Width();
	double Height();

	void AddObservation(int classId);
	void AddManyObservation(std::map<int,int> A);
	int GetObservationCount();

	int Size();
		
	
	int classId;
	int instanceId;
	
	float minX,maxX,minY,maxY,minZ,maxZ;
	
	PointCloud::Ptr cloud;
	
	//为了修正误分类
	std::map<int,int> observationMap;
	
	int flag;
};



class PoseStamped
{
public:
	PoseStamped(Eigen::Isometry3d Tcw,std::string timestamp);

	Eigen::Isometry3d Tcw_;
	std::string timestamp_;
};





#endif

