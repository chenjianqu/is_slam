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

/* 全局变量 */
PointCloud::ConstPtr currentPC;
bool newPointCloud = false;

double cellResolution=0.05;
double deviation = 0.785398;
int buffer_size=50;//每个栅格法线垂直的阈值
ros::Publisher pub;


void calcSize(double *xMax, double *yMax, double *xMin, double *yMin) 
{
  pcl::PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D(*currentPC, minPt, maxPt);	

  *xMax=maxPt.x;
  *yMax=maxPt.y;
  *xMin=minPt.x;
  *yMin=minPt.y;
}

//得到栅格地图
void computeGrid( std::vector<signed char> &ocGrid, double xMin, double yMin,int xCells, int yCells) 
{
	cout<<"开始计算法线"<<endl;
	//计算点云的法线
	NormalCloud::Ptr cloud_normals(new NormalCloud);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(currentPC);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.06);
	ne.compute(*cloud_normals);
	
	cout<<"判断法线是否垂直"<<endl;
	
	int size=xCells*yCells;
	std::vector<int> countGrid(size);
	
	//判断每个点云的法向量是否垂直
	for (size_t i = 0; i < currentPC->size(); i++)
	{
		double x = currentPC->points[i].x;
		double y = currentPC->points[i].y;
		double z = cloud_normals->points[i].normal_z;

		int xc = (int) ((x - xMin) / cellResolution); //取整后默认按照cellResolution将点分配到ｃｅｌｌ
		int yc = (int) ((y - yMin) / cellResolution);

		/*
			法线是单位向量，z是发现的z值，z越接近于1,表明法线越垂直。
			而acos(z)使得z值越接近于1,结果越小.
			即acos(z)的结果越大，z越不垂直
		*/
		double normal_value = acos(fabs(z));//值域　０－－ｐｈｉ   地面fabs(z)应该是１　acos是０，最小值
		if (normal_value > deviation)       //根据acos函数的递减性质，非地面点的值应该都比地面点大。可以设置deviation值，决定障碍物点的阈值
		  countGrid[yc * xCells + xc]++; //统计一个ｃｅｌｌ中垂直方向满足条件的点数
	}
	
	cout<<"计算占据概率"<<endl;
	
	//根据阈值计算占据概率
	for (int i = 0; i < size; i++)  //size:xCells * yCells
	{
		if (countGrid[i] < buffer_size && countGrid[i]>0) 
		  ocGrid[i] = 0;
		else if (countGrid[i] > buffer_size) 
		  ocGrid[i] = 100;
		else if (countGrid[i] == 0) 
		  ocGrid[i] = 0; // TODO Should be -1      
	}
}


void updateGrid(nav_msgs::OccupancyGridPtr grid, int xCells, int yCells,
                               double originX, double originY, std::vector<signed char> *ocGrid) 
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
	grid->info.origin.position.x = originX;  //minx
	grid->info.origin.position.y = originY;  //miny
	grid->info.origin.position.z = 0;
	grid->info.origin.orientation.w = 1;
	grid->info.origin.orientation.x = 0;
	grid->info.origin.orientation.y = 0;
	grid->info.origin.orientation.z = 0;
	grid->data = *ocGrid;
}


void computeGrid2(std::vector<signed char> &ocGrid, double xMin, double yMin,int xCells, int yCells)
{
	PointCloud::Ptr cpc(new PointCloud);
	pcl::PassThrough<PointT> *passFilter=new pcl::PassThrough<PointT>;
	passFilter->setFilterFieldName("z");
	passFilter->setFilterLimitsNegative(false);//保留此区间内的数据
	passFilter->setFilterLimits(0,0.5);
	passFilter->setInputCloud(currentPC);
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
	


void callback(const PointCloud::ConstPtr& msg) 
{
  currentPC=msg;
  ROS_INFO_STREAM("Convertor节点——接收到点云");
	
		/*计算点云的最大和最小值*/
	double xMax = 0, yMax = 0, xMin = 0, yMin = 0;
	calcSize(&xMax, &yMax, &xMin, &yMin); 

	cout<<"极值："<<xMax<<" "<<yMax<<" "<<xMin<<" "<<yMin<<" "<<endl;
	
	/* 确定栅格地图的长和宽 */
	int xCells = ((int) ((xMax - xMin) / cellResolution)) + 1;
	int yCells = ((int) ((yMax - yMin) / cellResolution)) + 1;
	
	cout<<"地图大小："<<xCells<<" "<<yCells<<endl;

	/*计算栅格地图*/
	std::vector<signed char> ocGrid(yCells * xCells);  //存储每个ｃｅｌｌ的值　　０或者１００
	//computeGrid(ocGrid, xMin, yMin, xCells, yCells);
	computeGrid2(ocGrid, xMin, yMin, xCells, yCells);
	
	cout<<"成功计算得到栅格地图"<<endl;

	//发布地图消息
	nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
	updateGrid(grid, xCells, yCells, xMin, yMin, &ocGrid);
	pub.publish(grid);
	ROS_INFO_STREAM("Convertor节点——发布栅格地图");
}


int main(int argc, char** argv) 
	{
	setlocale(LC_ALL, "");
	
	ros::init(argc, argv, "convertor_node");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<PointCloud>("/point_cloud/raw", 1, callback); 
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
	
	//构造占据网格消息
	nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
	grid->header.seq = 1;
	grid->header.frame_id = "map";//父坐标系
	grid->info.origin.position.z = 0;
	grid->info.origin.orientation.w = 1;
	grid->info.origin.orientation.x = 0;
	grid->info.origin.orientation.y = 0;
	grid->info.origin.orientation.z = 0;

	
	ROS_INFO_STREAM("Convertor节点初始化完成");
	ros::Rate loop_rate(0.2);
	ros::Duration t(10);
	while (ros::ok()) 
	{
		ros::spinOnce();

		t.sleep();
	}
}
