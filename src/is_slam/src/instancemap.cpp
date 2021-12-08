#include "is_slam/instancemap.h"

InstanceMap::InstanceMap(const string &dataSavePath):
	dataPath(dataSavePath)
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
}

Instance::Ptr InstanceMap::CreateInstance(PointCloud::Ptr cloud,int clsIndex,int id)
{
	PointT minPt, maxPt;//获得点云的各个轴的极值
	pcl::getMinMax3D(*cloud, minPt, maxPt);	
	Instance::Ptr inst(new Instance(clsIndex,id,minPt.x,maxPt.x,minPt.y,maxPt.y,minPt.z,maxPt.z,cloud));
    return inst;
}


void InstanceMap::Output()
{
	cout<<"输出地图实例："<<endl;
	for(int i=0;i<instVector.size();i++)
	{
		Instance::Ptr inst=instVector[i];
		cout<<COCO_CLASSES[inst->classId]<<": "<<inst->classId<<" "<<inst->instanceId<<" 观测："<<inst->GetObservationCount()<<" 体积："<<inst->Volume()<<endl;
	}
		
}



void InstanceMap::Clear()
{
	instVector.clear();
}


int InstanceMap::Size()
{
	return instVector.size();
}




//融合两个实例
Instance::Ptr InstanceMap::InstanceMerge(Instance::Ptr A,Instance::Ptr B)
{	
	Instance::Ptr inst(new Instance());
	inst->minX=min(A->minX,B->minX);
	inst->maxX=max(A->maxX,B->maxX);
	inst->minY=min(A->minY,B->minY);
	inst->maxY=max(A->maxY,B->maxY);
	inst->minZ=min(A->minZ,B->minZ);
	inst->maxZ=max(A->maxZ,B->maxZ);
	
	PointCloud::Ptr pc(new PointCloud);
	int classId=0;
	int instanceId=0;

	//判断谁向谁融合
	bool AorB=true;
	if(A->GetObservationCount() > B->GetObservationCount())
		AorB=true;
	else if(A->GetObservationCount() < B->GetObservationCount())
		AorB=false;
	else{
		if(A->Size() >= B->Size())
			AorB=true;
		else
			AorB=false;
	}

	//将两个实例进行融合
	if(AorB)
	{
		*(pc)+=*(A->cloud);
		classId=A->classId;
		int classIdColor=254-A->classId;
		instanceId=A->instanceId;
		int instanceIdColor=254-A->instanceId;
		
		if(classId !=B->classId && instanceId!=B->instanceId){
			for(int i=0;i<B->cloud->size();i++){
				B->cloud->points[i].g=classIdColor;
				B->cloud->points[i].r=instanceIdColor;
			}
		}
		else if(classId!=B->classId){
			for(int i=0;i<B->cloud->size();i++)
				B->cloud->points[i].g=classIdColor;
		}
		else if(instanceId!=B->instanceId){
			for(int i=0;i<B->cloud->size();i++)
				B->cloud->points[i].r=instanceIdColor;
		}
		*(pc)+=*(B->cloud);
	}
	else{
		*(pc)+=*(B->cloud);
		classId=B->classId;
		int classIdColor=254-B->classId;
		instanceId=B->instanceId;
		int instanceIdColor=254-B->instanceId;

		if(A->classId!=classId && A->instanceId!=instanceId){
			for(int i=0;i<A->cloud->size();i++){
				A->cloud->points[i].g=classIdColor;
				A->cloud->points[i].r=instanceIdColor;
			}
		}
		else if(A->classId!=classId){
			for(int i=0;i<A->cloud->size();i++)
				A->cloud->points[i].g=classIdColor;
		}
		else if(A->instanceId!=instanceId){
			for(int i=0;i<A->cloud->size();i++)
				A->cloud->points[i].r=instanceIdColor;
		}
		*(pc)+=*(A->cloud);
	}
	
	inst->classId=classId;
	inst->instanceId=instanceId;
	inst->cloud=pc;
	inst->AddManyObservation(A->observationMap);
	inst->AddManyObservation(B->observationMap);

	return inst;
}



double InstanceMap::InstanceIoU(Instance::Ptr A,Instance::Ptr B)
{
	//交界区域的6个边界
	double minX=max(A->minX,B->minX);
	double maxX=min(A->maxX,B->maxX);
	double minY=max(A->minY,B->minY);
	double maxY=min(A->maxY,B->maxY);
	double minZ=max(A->minZ,B->minZ);
	double maxZ=min(A->maxZ,B->maxZ);
	//长宽高
	double x=maxX-minX;
	double y=maxY-minY;
	double z=maxZ-minZ;
	
	//不相交
	if(x<0 || y<0 || z<0)
		return 0;
	
	double Vab=x*y*z;
	
	return Vab/(A->Volume()+B->Volume()-Vab);
}



double InstanceMap::InstanceMaxPercentage(Instance::Ptr A,Instance::Ptr B)
{
	//交界区域的6个边界
	double minX=max(A->minX,B->minX);
	double maxX=min(A->maxX,B->maxX);
	double minY=max(A->minY,B->minY);
	double maxY=min(A->maxY,B->maxY);
	double minZ=max(A->minZ,B->minZ);
	double maxZ=min(A->maxZ,B->maxZ);
	//长宽高
	double x=maxX-minX;
	double y=maxY-minY;
	double z=maxZ-minZ;
	
	//不相交
	if(x<0 || y<0 || z<0)
		return 0;
	
	double Vab=x*y*z;

	double Vap=Vab/A->Volume();
	double Vbp=Vab/B->Volume();
	
	return max(Vap,Vbp);
}





//在实例地图中插入实例
int InstanceMap::InstanceInsert(Instance::Ptr inst)
{
	int flag=-1;
	double maxIou=0;
	
	for(int i=0;i<instVector.size();i++)
	{
		double iou=InstanceIoU(inst,instVector[i]);
		if(iou>maxIou){
			flag=i;
			maxIou=iou;
		}
	}
	
	//与地图中存在的实例存在相交的地图
	if(flag>=0 && maxIou>0.2)
	{
		bool isSameClass=inst->classId == instVector[flag]->classId;
		if(isSameClass || (!isSameClass && maxIou>0.5))	//类别相同 或 类别不相同但是maxIoU>0.5
		{
			Instance::Ptr result=InstanceMerge(inst,instVector[flag]); //实例融合
			instVector[flag]=result;
		}
		else{
			instVector.push_back(inst);
		}
	}
	else{
		instVector.push_back(inst);
	}

	return 0;
}



void InstanceMap::InstanceAffine(Eigen::Affine3d Taf)
{
	for(int i=0;i<instVector.size();i++)
	{
		Instance::Ptr inst=instVector[i];
		pcl::transformPointCloud(*(inst->cloud), *(inst->cloud), Taf);

		PointT minPt, maxPt;//获得点云的各个轴的极值
		pcl::getMinMax3D(*(inst->cloud), minPt, maxPt);	
		
		inst->minX=minPt.x;
		inst->maxX=maxPt.x;
		inst->minY=minPt.y;
		inst->maxY=maxPt.y;
		inst->minZ=minPt.z;
		inst->maxZ=maxPt.z;
	}
}


void InstanceMap::InstanceNMS()
{
/******根据体积进行排序*****/
	vector<Instance::Ptr> sortedVector;
	int size=instVector.size();
	for(int i=0;i<size;i++)
	{			
		double vMax=0;
		int index=0;
		for(int j=0;j<size;j++)
		{
			if(instVector[j]->flag==1)
				continue;
			double vCurrent=instVector[j]->Volume();
			if(vMax<vCurrent){
				vMax=vCurrent;
				index=j;
			}
		}
		sortedVector.push_back(instVector[index]);
		instVector[index]->flag=1;
	}
	
	instVector.clear();
	
	for(int i=0;i<size;i++)
	{
		Instance::Ptr inst=sortedVector[i];
		if(inst->flag==0) //表示该实例被处理过了
			continue;
		
		for(int j=i+1;j<size;j++)
		{
			if(sortedVector[j]->flag==0) //表示该实例被处理过了
				continue;
			
			if(inst->classId != sortedVector[j]->classId)
				continue;
			//计算IoU
			double perc=InstanceMaxPercentage(inst,sortedVector[j]);
			
			if(perc>0.8){ //同类别
				inst=InstanceMerge(inst,sortedVector[j]);
				inst->flag=0;//标记访问过
				sortedVector[j]->flag=0;
			}
		}
		instVector.push_back(inst);
	}
}


void InstanceMap::SaveInstance()
{
    cv::Mat instMatPack=cv::Mat::zeros(instVector.size(),8,5);
    for(int i=0;i<instVector.size();i++)
    {
        Instance::Ptr inst=instVector[i];
        instMatPack.at<float>(i,0)=float(inst->classId);
        instMatPack.at<float>(i,1)=float(inst->instanceId);
        instMatPack.at<float>(i,2)=float(inst->minX);
        instMatPack.at<float>(i,3)=float(inst->maxX);
        instMatPack.at<float>(i,4)=float(inst->minY);
        instMatPack.at<float>(i,5)=float(inst->maxY);
        instMatPack.at<float>(i,6)=float(inst->minZ);
        instMatPack.at<float>(i,7)=float(inst->maxZ);
    }
    cv::imwrite(dataPath+"/InstanceMat.exr",instMatPack);
}

