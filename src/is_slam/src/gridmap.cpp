/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of is_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#include "is_slam/gridmap.h"

#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

using namespace std;




GridMap::GridMap(string dataSavePath):
	dataPath(dataSavePath),
	cellResolution(0.05),
	xMax(0.0),yMax(0.0),xMin(0.0),yMin(0.0),
	xCells(0),yCells(0)
{
	//构造占据网格消息
	grid.reset(new nav_msgs::OccupancyGrid);
	grid->header.seq = 1;
	grid->header.frame_id = "map";//父坐标系
	grid->info.origin.position.z = 0;
	grid->info.origin.orientation.w = 1;
	grid->info.origin.orientation.x = 0;
	grid->info.origin.orientation.y = 0;
	grid->info.origin.orientation.z = 0;

	cloud.reset(new PointCloud);
}



void GridMap::CalcSize() 
{
	pcl::PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);	

	xMax=maxPt.x;
	yMax=maxPt.y;
	xMin=minPt.x;
	yMin=minPt.y;
}


void GridMap::UpdateGrid(std::vector<signed char> &ocGrid) 
{
	static int seq=0;

	grid->header.frame_id = "map";
	grid->header.seq=seq++;
	grid->header.stamp.sec = ros::Time::now().sec;
	grid->header.stamp.nsec = ros::Time::now().nsec;
	grid->info.map_load_time = ros::Time::now();
	grid->info.resolution = cellResolution;
	grid->info.width = xCells;
	grid->info.height = yCells;
	grid->info.origin.position.x = xMin;  //minx
	grid->info.origin.position.y = yMin;  //miny
	grid->info.origin.position.z = 0;
	grid->info.origin.orientation.w = 1;
	grid->info.origin.orientation.x = 0;
	grid->info.origin.orientation.y = 0;
	grid->info.origin.orientation.z = 0;
	grid->data = ocGrid;
}


void GridMap::ComputeGrid(std::vector<signed char> &ocGrid)
{
	PointCloud::Ptr cpc(new PointCloud);
	pcl::PassThrough<PointT> *passFilter=new pcl::PassThrough<PointT>;
	passFilter->setFilterFieldName("z");
	passFilter->setFilterLimitsNegative(false);//保留此区间内的数据
	passFilter->setFilterLimits(0,0.5);
	passFilter->setInputCloud(cloud);
	passFilter->filter(*cpc);
	
	int size=xCells*yCells;
	std::vector<int> countGrid(size);
	
	//将每个点云分配到各个网格
	for (size_t i = 0; i < cpc->size(); i++)
	{
		PointT p=cpc->points[i];

		int xc = (int) ((p.x - xMin) / cellResolution); //取整后默认按照cellResolution将点分配到ｃｅｌｌ
		int yc = (int) ((p.y - yMin) / cellResolution);

		countGrid[yc * xCells + xc]++; //统计一个ｃｅｌｌ中垂直方向满足条件的点数
	}
	
	for (int i = 0; i < size; i++)  //size:xCells * yCells
	{
		if (countGrid[i] < 10 && countGrid[i]>0) 
		  ocGrid[i] = 0;
		else if (countGrid[i] > 10) 
		  ocGrid[i] = 100;
		else if (countGrid[i] == 0) 
		  ocGrid[i] = 0; // TODO Should be -1      
	}
}
	






nav_msgs::OccupancyGridPtr GridMap::GenerateGridMap(PointCloud::Ptr pc)
{
	cloud=pc;
	
	/*计算点云的最大和最小值*/
	CalcSize(); 

	cout<<"极值："<<xMax<<" "<<yMax<<" "<<xMin<<" "<<yMin<<" "<<endl;
	
	/* 确定栅格地图的长和宽 */
	xCells = ((int) ((xMax - xMin) / cellResolution)) + 1;
	yCells = ((int) ((yMax - yMin) / cellResolution)) + 1;
	
	cout<<"地图大小："<<xCells<<" "<<yCells<<endl;

	/*计算栅格地图*/
	std::vector<signed char> ocGrid(yCells * xCells);  //存储每个ｃｅｌｌ的值　　０或者１００
	ComputeGrid(ocGrid);
	
	UpdateGrid(ocGrid);
	return grid;
}




