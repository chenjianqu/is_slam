/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of is_slam.
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#include "is_slam/semanticmap.h"


SemanticMap::SemanticMap(const string &dataSavePath):
	isRun(true),
    isPoseUpdate(false),
	dataPath(dataSavePath),
	isUpdateMap(false)
{
    string coco_label_[]={"person", "bicycle", "car", "motorcycle", "airplane", "bus",
                "train", "truck", "boat", "traffic light", "fire hydrant",
                "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                "skis", "snowboard", "sports ball", "kite", "baseball bat",
                "baseball glove", "skateboard", "surfboard", "tennis racket",
                "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
                "hot dog", "pizza", "donut", "cake", "chair", "couch",
                "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
                "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
                "toaster", "sink", "refrigerator", "book", "clock", "vase",
                "scissors", "teddy bear", "hair drier", "toothbrush"};
    //复制到COCO_label数组
    std::copy(std::begin(coco_label_),std::end(coco_label_),std::begin(COCO_CLASSES));

	voxelFilter01 = new pcl::ApproximateVoxelGrid<PointT>;
	voxelFilter01->setDownsampleAllData(false);
	voxelFilter01->setLeafSize( 0.01, 0.01, 0.01);
	 
	passFilter=new pcl::PassThrough<PointT>;
	passFilter->setFilterFieldName("z");
	passFilter->setFilterLimitsNegative(true);//删除此区间内的数据
	passFilter->setFilterLimits(-1001,-999);

	statisticalFilter=new pcl::StatisticalOutlierRemoval<PointT>;
	statisticalFilter->setMeanK(50);
	statisticalFilter->setStddevMulThresh(1.0);
	
	outlierFilter=new pcl::RadiusOutlierRemoval<PointT>;
	outlierFilter->setRadiusSearch(0.1);
	outlierFilter->setMinNeighborsInRadius(100);

	//聚类分割
	eucCluster=new pcl::EuclideanClusterExtraction<PointT>;
	eucCluster->setClusterTolerance (0.01); // 2cm，在欧氏空间里设置空间聚类容差 tolerance ，其 实是在近邻搜索中所使用的半径。

	indicesExtractor=new pcl::ExtractIndices<PointT>;

    denseMap.reset(new PointCloud);

	instPointCloud.reset(new PointCloud);

	instsMap.reset(new InstanceMap(dataPath));


    Talign=Eigen::Affine3d::Identity();
}



void SemanticMap::InsertFrame(PointCloud::Ptr cloud,string &timestamp)
{
    unique_lock<mutex> lock(frameMapMutex);
	frameMap[timestamp]=cloud;
	poseMap[timestamp]=Eigen::Affine3d::Identity();
}

void SemanticMap::InsertPose(Eigen::Isometry3d Tcw,std::string &timestamp)
{
	unique_lock<mutex> lock(poseListMutex);
	PoseStamped pose(Tcw,timestamp);
	poseList.push_back(pose);
}

int SemanticMap::GetPoseListSize()
{
    unique_lock<mutex> lock(poseListMutex);
    return poseList.size();
}


PoseStamped SemanticMap::GetHeadPose()
{
	unique_lock<mutex> lock(poseListMutex);	
	return poseList.front();
}

PoseStamped SemanticMap::PopPose()
{
	unique_lock<mutex> lock(poseListMutex);
	PoseStamped p=poseList.front();
	poseList.pop_front();//删除第一个元素
	return p;
}


void SemanticMap::SetRun(bool isRun_)
{
	unique_lock<mutex> lock(stateMutex);
	isRun=isRun_;
}

bool SemanticMap::GetRun()
{
	unique_lock<mutex> lock(stateMutex);
	return isRun;
}


void SemanticMap::SetPoseUpdate(bool flag)
{
	unique_lock<mutex> lock(stateMutex);
	isPoseUpdate=flag;
}

bool SemanticMap::GetPoseUpdate()
{
	unique_lock<mutex> lock(stateMutex);
	return isPoseUpdate;
}

bool SemanticMap::GetIsUpdateMap()
{
    unique_lock<mutex> lock(stateMutex);
    bool state=isUpdateMap;
    isUpdateMap=false;
    return state;
}

void SemanticMap::SetIsUpdateMap(bool isUpdate_)
{
    unique_lock<mutex> lock(stateMutex);
    isUpdateMap=isUpdate_;
}


PointCloud::Ptr SemanticMap::GetPointCloud()
{
	PointCloud::Ptr fullMap(new PointCloud);
	unique_lock<mutex> lock(denseMapMutex);
	*(fullMap)+=*(instPointCloud);
	*(fullMap)+=*(denseMap);
	return fullMap;
}

int SemanticMap::GetPointCloudSize()
{
	unique_lock<mutex> lock(denseMapMutex);
	return denseMap->size();
}


void SemanticMap::BuildInstanceMap(PointCloud::Ptr cloud)
{
	std::map<int,std::map<int,PointCloud::Ptr>> classMap;
	int instanceID=0;
	
	//提取实例点云
	for(size_t i=0;i<cloud->size();++i)
	{
		if(cloud->points[i].g==0)
			continue;
		if(cloud->points[i].g>174)//实例点
		{
			int cls=254-cloud->points[i].g;
			int id=254-cloud->points[i].r;
			if (classMap.count(cls) == 0){
				std::map<int,PointCloud::Ptr> tmpMap;
				classMap[cls]=tmpMap;
			}
			
			if(classMap[cls].count(id) == 0){
				PointCloud::Ptr instPointCloud ( new PointCloud );
				classMap[cls][id] = instPointCloud;
			}
			
			classMap[cls][id]->push_back(cloud->points[i]);
		}
		cloud->points[i].z=-1000;//等用来滤波掉
	}
	passFilter->setInputCloud(cloud);
	passFilter->filter(*cloud);
	
	
	
	for (std::map<int,std::map<int,PointCloud::Ptr>>::iterator classMapIt=classMap.begin(); classMapIt!=classMap.end(); ++classMapIt)
	{
		std::map<int,PointCloud::Ptr> instancePointCloudMap=classMapIt->second;
		
		//cout<<"类别:"<<COCO_CLASSES[classMapIt->first]<<" 实例数量："<<instancePointCloudMap.size()<<endl;
		
		for(std::map<int,PointCloud::Ptr>::iterator instMapIt=instancePointCloudMap.begin(); instMapIt!=instancePointCloudMap.end(); ++instMapIt)
		{
			PointCloud::Ptr cpc=instMapIt->second;
			if(cpc->size()<100){
				//cout<<cpc->size()<<endl;
				continue;
			}
			
			//聚类
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
			tree->setInputCloud (cpc);
			eucCluster->setMinClusterSize (int(cpc->size()/3));//最小簇
			eucCluster->setMaxClusterSize (cpc->size());//最大簇
			eucCluster->setSearchMethod (tree);//设置搜索时所用的搜索机制，参数 tree 指向搜索时所用的搜索对象，例如kd-tree octree 等对
			eucCluster->setInputCloud (cpc);
			eucCluster->extract (cluster_indices);
			
			
			//cout<<"类别："<<COCO_CLASSES[classMapIt->first]<<" 实例:"<<instMapIt->first<<" 点云大小:"<<cpc->size()<<" 聚类个数："<<cluster_indices.size()<<endl;
			
			
			if(cluster_indices.size()==0)
				continue;

			
			instanceID++;
			//取出聚类后的点云
			pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
			
			std::vector<pcl::PointIndices>::const_iterator itp = cluster_indices.begin ();
			pcl::PointIndices::Ptr pi_ptr(new pcl::PointIndices(*itp));
			indicesExtractor->setInputCloud (cpc);
			indicesExtractor->setIndices (pi_ptr);
			indicesExtractor->setNegative (false);
			indicesExtractor->filter (*cloud_cluster);

			Instance::Ptr inst=instsMap->CreateInstance(cloud_cluster,254-cloud_cluster->points[0].g,254-cloud_cluster->points[0].r);
			instsMap->InstanceInsert(inst);
			
			//instsMap->InstanceOutput();
			
			//cout<<cloud_cluster->size()<<endl;
			
		}

	}
}




void SemanticMap::PointCloudAlign(PointCloud::Ptr cloud)
{
	pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients (true);// Optional
	seg.setModelType (pcl::SACMODEL_PLANE);// Mandatory
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coef);
	
	
	cout << "平面模型内点数量: " << inliers->indices.size () << std::endl;
	cout<<"平面模型："<<coef->values[0]<<coef->values[1]<<coef->values[2]<<coef->values[3]<<endl;
	
	if(inliers->indices.size ()==0)
		return;
	
	double direct=1;
	if(coef->values[2]<0)
		direct=-1;
	
	Eigen::Vector3d P1(coef->values[0],coef->values[1],coef->values[2]);
	Eigen::Vector3d P2(0,0,direct);
	Eigen::Vector3d P3(0,0,0);
	
	//叉乘，得到旋转轴向量
	Eigen::Vector3d n;
	n=P1.cross(P2); 
  
	//得到旋转角
	double theta=acos(P1.dot(P2)/(P1.norm()*P2.norm()));
	//构造旋转向量
	Eigen::AngleAxisd aa(theta,n);
	//构造仿射矩阵
	Eigen::Affine3d Taf=Eigen::Affine3d::Identity();
	Taf.rotate(aa);
	//执行变换
	pcl::transformPointCloud(*cloud, *cloud, Taf);
	
	Talign=Eigen::Affine3d::Identity();
	Talign=Taf*Talign;
  
	//修正偏移
	double bias=0;
	int size=10000 < inliers->indices.size()?10000:inliers->indices.size();
	for(int i=0;i<size;i++)
		bias+=cloud->points[inliers->indices[i]].z;
	bias/=size;
	
	Eigen::Affine3d Taf1=Eigen::Affine3d::Identity();
	Taf1.translate(Eigen::Vector3d(0,0,-bias));
	pcl::transformPointCloud(*cloud, *cloud, Taf1);
	Talign=Taf1*Talign;
	
	//看是否需要翻转
	int directFlag=0;
	for(int i=0;i<cloud->size();i+=100)
		directFlag+=cloud->points[i].z;
	
	if(directFlag<0) //若点云向下，则将点云翻转
	{
		//构造旋转向量
		Eigen::Vector3d n2(1,0,0);
		Eigen::AngleAxisd aa2(M_PI,n2);
		Eigen::Affine3d Taf2=Eigen::Affine3d::Identity();
		Taf2.rotate(aa2);
		pcl::transformPointCloud(*cloud, *cloud, Taf2);
		Talign=Taf2*Talign;
	}

	cout<<"directFlag:"<<directFlag<<endl;
	
}



void SemanticMap::SaveDenseMap()
{
	PointCloud::Ptr fullMap(new PointCloud);
	unique_lock<mutex> lock(denseMapMutex);
	*(fullMap)+=*(instPointCloud);
	*(fullMap)+=*(denseMap);
	pcl::io::savePCDFileBinary(dataPath+"/map.pcd", *fullMap);
}



void SemanticMap::Mapping()
{
    while(GetRun())
	{
        if(GetPoseUpdate())
        {
			SetPoseUpdate(false);

			cout<<"开始构建语义地图"<<endl;

			{
				unique_lock<mutex> lock(denseMapMutex);
				denseMap->clear();
			}
            
			instsMap->Clear();

			time_t beginTime=clock();

			int size=GetPoseListSize();

            for(int i=0;i<size;i++)
            {

                PoseStamped pose=PopPose();
                Eigen::Affine3d Taf=Eigen::Affine3d::Identity();
        		Taf=pose.Tcw_.affine();

				//cout<<pose.timestamp_<<endl;

				if(frameMap.count(pose.timestamp_)==0) //该帧不存在
					continue;

				//先变把帧点云变回去，再进行变换
				Eigen::Affine3d Ttr=Eigen::Affine3d::Identity();
				Ttr=poseMap[pose.timestamp_].inverse()*Taf;
				poseMap[pose.timestamp_]=Ttr;

		        PointCloud::Ptr cpc=frameMap[pose.timestamp_];
		        pcl::transformPointCloud(*cpc, *cpc, Ttr);

				//cout<<"帧序号："<<i<<" 坐标变换完成"<<endl;

				//构建语义地图
                BuildInstanceMap(cpc);

				{
					unique_lock<mutex> lock(denseMapMutex);
					(*denseMap) += *cpc;
				}
            }
			cout<<"语义建图线程——点云构建完成"<<" 帧数量："<<size<<" 点云数量："<<GetPointCloudSize()<<" 用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;

			beginTime=clock();
			{
				unique_lock<mutex> lock(denseMapMutex);
				voxelFilter01->setInputCloud( denseMap );
		        voxelFilter01->filter( *denseMap );
			}
			cout<<"语义建图线程——体素滤波  用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;
			
			beginTime=clock();
            instsMap->InstanceNMS();
			cout<<"语义建图线程——实例NMS  用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;
            
			beginTime=clock();
			{
				unique_lock<mutex> lock(denseMapMutex);
				PointCloudAlign(denseMap);
			}
			cout<<"语义建图线程——点云对齐 点云数量："<<GetPointCloudSize()<<" 用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;

            beginTime=clock();
            instsMap->InstanceAffine(Talign);
			cout<<"语义建图线程——实例对齐  用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;

			beginTime=clock();
            instsMap->Output();
            instsMap->SaveInstance();
			cout<<"语义建图线程——保存实例 实例数量："<<instsMap->Size()<<" 用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;

			beginTime=clock();
			{
				unique_lock<mutex> lock(denseMapMutex);
				instPointCloud->clear();
				for(int i=0;i<instsMap->instVector.size();i++)
				{
					*(instPointCloud)+=*(instsMap->instVector[i]->cloud);
				}
			}
			cout<<"语义建图线程——实例点云融合  用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;


			beginTime=clock();
			{
				unique_lock<mutex> lock(denseMapMutex);
				statisticalFilter->setInputCloud(denseMap);
				statisticalFilter->filter(*denseMap);
			}
			cout<<"语义建图线程——统计滤波 点云数量："<<GetPointCloudSize()<<" 用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;
			
			beginTime=clock();
			SaveDenseMap();
			cout<<"语义建图线程——保存地图 用时："<<double(clock() - beginTime) / CLOCKS_PER_SEC<<" s"<<endl;



			SetIsUpdateMap(true);
        }

        //休眠100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


	instsMap->SaveInstance();
    cout<<"InstanceMapping线程——关闭"<<endl;
}






