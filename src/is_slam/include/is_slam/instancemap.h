#ifndef INSTANCEMAP_H
#define INSTANCEMAP_H

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

#include "instance.h"


// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;


class InstanceMap
{
public:
    typedef shared_ptr<InstanceMap> Ptr;

    InstanceMap(const string &dataSavePath);

    Instance::Ptr CreateInstance(PointCloud::Ptr cloud,int clsIndex,int id);

    void Output();
    void Clear();
    int Size();

    Instance::Ptr InstanceMerge(Instance::Ptr A,Instance::Ptr B);

    double InstanceIoU(Instance::Ptr A,Instance::Ptr B);

    double InstanceMaxPercentage(Instance::Ptr A,Instance::Ptr B);

    int InstanceCreate(Instance::Ptr inst);

    int InstanceInsert(Instance::Ptr inst);
    void InstanceAffine(Eigen::Affine3d Taf);

    void SaveInstance();

    void InstanceNMS();

    vector<Instance::Ptr> instVector;


    string COCO_CLASSES[80];
protected:
    string dataPath;

};

#endif

