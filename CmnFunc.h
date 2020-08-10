#pragma once
#define BOOST_TYPEOF_EMULATION

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <io.h>
#include <windows.h>

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

struct PtsCloudType
{
	float x, y, z;
	float intensity;
	int rowID;
	int colID;
	int label;					///< 标记点云的特征类型

	PtsCloudType()
	{
		x = -1; y = -1; z = -1;
		intensity = -999;
		rowID = -1; colID = -1; label = -999;
	}
};

struct smoothness_t {
	float value;
	int ind;
};

struct SegCloudInfo
{
	vector<int> startRingIndex;
	vector<int> endRingIndex;

	float startOrientation;
	float endOrientation;
	float orientationDiff;

	vector<bool> segmentedCloudGroundFlag;
	vector<uint32_t> segmentedCloudColInd;
	vector<float> segmentedCloudRange;
};

struct by_value {
	bool operator()(smoothness_t const &left, smoothness_t const &right) {
		return left.value < right.value;
	}
};

struct POS {
	double time;
	double XYZ[3];
	double RPY[3];
	double RLw[9];
	double NDTScore;
	double NDTTime;
};

void CheckTxtFilelist(string fpath, vector<string>& filenames);
void txt2pcd(string fpath, string filename);

bool Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout);
bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<vector<float>> pointCloudout);

double rad2deg(double radians);
double deg2rad(double degrees);

inline void CheckTxtFilelist(string fpath, vector<string>& filenames)
{
	//文件句柄  
	intptr_t   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(fpath.c_str()).append("\\*.txt").c_str(), &fileinfo)) != -1)
	{
		do
		{
			filenames.push_back(fileinfo.name);
		} while (_findnext(hFile, &fileinfo) == 0);
	}
	_findclose(hFile);
}

inline void txt2pcd(string fpath, string filename)
{
	vector<float> vPoint;
	vector<vector<float>> pointCloud;

	// load point cloud
	FILE *instream, *outstream;

	string fin = fpath + filename;
	char buff[1024];
	strcpy_s(buff, filename.c_str());

	char* buff1;
	char* buff2;
	buff1 = strtok_s(buff, ".", &buff2);

	string pcdFilename;
	pcdFilename = buff1;
	pcdFilename = pcdFilename + ".pcd";

	string fout = fpath + "..//gridmap_pcd//";
	CreateDirectory(fout.c_str(), NULL);		// 创建velodyne_pcd文件夹,依赖于windows.h
	fout = fout + pcdFilename;

	fopen_s(&instream, fin.c_str(), "r");
	fopen_s(&outstream, fout.c_str(), "w");
	int num = 0;
	while (!feof(instream))
	{
		double x = 0;
		double y = 0;
		double z = 0;
		double intensity = 0;
		fscanf_s(instream, "%lf %lf %lf %lf\n", &x, &y, &z, &intensity);
		vPoint.push_back(x); vPoint.push_back(y); vPoint.push_back(z); vPoint.push_back(intensity);
		pointCloud.push_back(vPoint);
		vector<float>().swap(vPoint);
		num++;
	}

	// 写入pcd文件
	fprintf(outstream, "# .PCD v0.7 - Point Cloud Data file format\n");
	fprintf(outstream, "VERSION 0.7\n");
	fprintf(outstream, "FIELDS x y z intensity\n");
	fprintf(outstream, "SIZE 4 4 4 4\n");
	fprintf(outstream, "TYPE F F F F\n");
	fprintf(outstream, "COUNT 1 1 1 1\n");
	fprintf(outstream, "WIDTH %d\n", num);
	fprintf(outstream, "HEIGHT 1\n");
	fprintf(outstream, "VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf(outstream, "POINTS %d\n", num);
	fprintf(outstream, "DATA ascii\n");


	for (int j = 0; j < num; j++)
	{
		fprintf(outstream, "%f %f %f %f\n", pointCloud[j][0], pointCloud[j][1], pointCloud[j][2], pointCloud[j][3]);
	}

	printf("decode2pcd %s\n", pcdFilename.c_str());
	fclose(instream);
	fclose(outstream);

	vector<vector<float>>().swap(pointCloud);
}

inline bool Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout)
{
	int num = pointCloudIn.size();
	laserCloudout->width = num;
	laserCloudout->height = 1;			///< 无序点云 高度置为1
	laserCloudout->points.resize(laserCloudout->width);
	for (int i = 0; i < num; i++)
	{
		laserCloudout->points[i].x = pointCloudIn[i].at(0);
		laserCloudout->points[i].y = pointCloudIn[i].at(1);
		laserCloudout->points[i].z = pointCloudIn[i].at(2);
		laserCloudout->points[i].intensity = pointCloudIn[i].at(3);
	}

	return true;
}

inline bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<vector<float>> pointCloudout)
{
	int num = 0;
	if (laserCloudIn->height == 1)
	{
		num = laserCloudIn->width;		///< PCL点云无序
	}
	else
	{
		num = laserCloudIn->width*laserCloudIn->height;  ///< PCL点云有序
	}

	for (int i = 0; i < num; i++)
	{
		pointCloudout[i].push_back(laserCloudIn->points[i].x);
		pointCloudout[i].push_back(laserCloudIn->points[i].y);
		pointCloudout[i].push_back(laserCloudIn->points[i].z);
		pointCloudout[i].push_back(laserCloudIn->points[i].intensity);
	}

	return true;
}

inline double rad2deg(double radians)
{
	return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
	return degrees * M_PI / 180.0;
}