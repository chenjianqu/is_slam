#ifndef SEMANTICMAP_H
#define SEMANTICMAP_H


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
#include "instancemap.h"

// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;


class SemanticMap
{
public:
    SemanticMap(const string &dataSavePath);
    void Mapping();

    void InsertFrame(PointCloud::Ptr cloud,string &timestamp);

    void InsertPose(Eigen::Isometry3d Tcw,std::string &timestamp);
    int GetPoseListSize();
    PoseStamped GetHeadPose();
    PoseStamped PopPose();
    
    void SaveDenseMap();

    void SetRun(bool isRun_);
	bool GetRun();
    void SetPoseUpdate(bool flag);
    bool GetPoseUpdate();
    
    bool GetIsUpdateMap();
    void SetIsUpdateMap(bool isUpdate_);

    void BuildInstanceMap(PointCloud::Ptr cloud);
    void PointCloudAlign(PointCloud::Ptr cloud);

    PointCloud::Ptr GetPointCloud();
    int GetPointCloudSize();

    string COCO_CLASSES[80];

protected:
    //滤波器
	pcl::ExtractIndices<PointT> *indicesExtractor;//按点云索引提取点云子集
    pcl::ApproximateVoxelGrid<PointT> *voxelFilter01; 
    pcl::StatisticalOutlierRemoval<PointT> *statisticalFilter;
	pcl::RadiusOutlierRemoval<PointT> *outlierFilter;
	pcl::PassThrough<PointT> *passFilter;
	pcl::EuclideanClusterExtraction<PointT> *eucCluster;


    //实例地图
    InstanceMap::Ptr instsMap;

    //点云地图
    PointCloud::Ptr denseMap;
    PointCloud::Ptr instPointCloud;
    std::mutex denseMapMutex;


    //保存的帧点云
    map<string,PointCloud::Ptr> frameMap;
    std::mutex frameMapMutex;
    
    //帧点云的位姿
    map<string,Eigen::Affine3d> poseMap;


    //待处理的位姿
    std::list<PoseStamped> poseList;
    std::mutex poseListMutex;

    //用于对齐的仿射变换
    Eigen::Affine3d Talign;

    bool isRun;
    bool isPoseUpdate;
    bool isUpdateMap;
    std::mutex stateMutex;

    //保存路径
    string dataPath;
};

#endif

