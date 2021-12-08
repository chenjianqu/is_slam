#include "is_slam/instance.h"


PoseStamped::PoseStamped(Eigen::Isometry3d Tcw,std::string timestamp):
    Tcw_(Tcw),timestamp_(timestamp)
{

}


Instance::Instance():
    classId(0),instanceId(0),minX(0.0),maxX(0.0),minY(0.0),maxY(0.0),minZ(0.0),maxZ(0.0),flag(-1)
{
    
}

Instance::Instance(int classId_,int instanceId_,float minX_,float maxX_,float minY_,float maxY_,float minZ_,float maxZ_,PointCloud::Ptr cloud_):
    classId(classId_),instanceId(instanceId_),minX(minX_),maxX(maxX_),minY(minY_),maxY(maxY_),minZ(minZ_),maxZ(maxZ_),cloud(cloud_),flag(-1)
{
    AddObservation(classId_);//增加观测
}


double Instance::Volume()
{
    return (maxX-minX)*(maxY-minY)*(maxZ-minZ);
}

double Instance::Length()
{
    return maxX-minX;
}

double Instance::Width()
{
    return maxY-minY;
}

double Instance::Height()
{
    return maxZ-minZ;
}

void Instance::AddObservation(int classId)
{
    if(observationMap.count(classId)==0)
        observationMap[classId]=0;
    observationMap[classId]++;
}

void Instance::AddManyObservation(std::map<int,int> A)
{
    for (std::map<int,int>::iterator it=A.begin(); it!=A.end(); ++it)
    {
        int cls=it->first;
        int count=it->second;
        
        if(observationMap.count(cls)==0)
            observationMap[cls]=0;
        observationMap[cls]+=count;
    }
}

int Instance::GetObservationCount()
{
    int sum=0;
    for (std::map<int,int>::iterator it=observationMap.begin(); it!=observationMap.end(); ++it)
        sum+=it->second;
    return sum;
}


int Instance::Size()
{
    return cloud->size();
}
    

