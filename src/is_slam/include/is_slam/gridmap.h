/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of is_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <iostream>
#include <fstream>  
#include <string>
#include<chrono>
#include<thread>
#include <ctime>
#include<vector>
#include<map>
#include<mutex>

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
#include <pcl/common/transforms.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>


// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

using namespace std;


class GridMap
{
public:
    typedef shared_ptr<GridMap> Ptr;

    GridMap(string dataSavePath);

    nav_msgs::OccupancyGridPtr GenerateGridMap(PointCloud::Ptr pc);


protected:
    void CalcSize();
    void ComputeGrid(std::vector<signed char> &ocGrid);
    void UpdateGrid(std::vector<signed char> &ocGrid);



    string dataPath;

    PointCloud::Ptr cloud;

    //栅格地图
    nav_msgs::OccupancyGridPtr grid;

    //栅格地图参数
    int xCells;
    int yCells;

    double xMax, yMax, xMin,yMin;

    double cellResolution;

};

#endif

