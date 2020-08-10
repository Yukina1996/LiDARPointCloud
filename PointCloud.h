#pragma once

#include "CmnFunc.h"
#include "FeatureExtraction.h"
#include <direct.h>
#include <CPose3D.h>
#include <BaseMatrix.h>

using namespace basetk;


class CPointCloud
{
public:
	CPointCloud();
	~CPointCloud();

	bool RunLiDARPointCloud(string folder);
	bool ReadGTFile();
	void CheckFilelist(vector<string> &filenames);
	bool RunCurrentPointCloud(int count);
	int GetCurrentPosture(int count);
	bool TransformToWorld(PointType point);
	bool GridingPointCloud();
	bool DownsamplePointCloud(PointCloudType::Ptr pointcloud, PointCloudType::Ptr pointcloudDS, float leafsize);
	bool WriteCSV();

	static bool MyComparison(const vector<float> &v1, const vector<float> &v2)
	{
		return v1.back() < v2.back();//ϵͳĬ��a<bʱ�����棬���Ǵ�С������
	}

	//���񻮷�׼��
	int m_grid_size;
	int m_dist;

	//�����ļ���ȡ
	vector<PointCloudType::Ptr> mglobalCloud;
	PointCloudType::Ptr mlocalCloud;
	PointCloudType::Ptr mlocalFeatureCloud;
	PointCloudType::Ptr mlocalCloudDS;
	PointCloudType::Ptr mlocalFeatureCloudDS;

	string mfilefolder;
	vector<string> mlistPC;
	vector<string> mlistGridMap;
	vector<POS> mvGT;
	POS mCurrentPos;
	POS mLastPos;
};

inline CPointCloud::CPointCloud()
{
	//��ʼ������
	m_grid_size = 50;	//������С
	m_dist = 3;			//���XXm����һ�ε���

	//�����ռ����
	mlocalCloud.reset(new PointCloudType());
	mlocalCloudDS.reset(new PointCloudType());
	mlocalFeatureCloud.reset(new PointCloudType());
	mlocalFeatureCloudDS.reset(new PointCloudType());
}

inline CPointCloud::~CPointCloud()
{
}

inline bool CPointCloud::RunLiDARPointCloud(string folder)
{
	this->mfilefolder = folder;
	ReadGTFile();
	CheckFilelist(this->mlistPC);
	
	for (int i = 0; i < this->mvGT.size(); i++)
	//for (int i = 0; i < 500; i++)
	{
		if (GetCurrentPosture(i))
		{
			// ���ļ��ж�ȡ��ǰ֡��ԭʼ���ƣ������Զ�ĵ�ɾ��
			RunCurrentPointCloud(i);

			// �ӵ�ǰ��������ȡ��Χ��Ч����
			CFeatureExtraction m_CloudExtraction;
			m_CloudExtraction.RunCloudSegmentation(mlocalCloud);
			mlocalCloud->clear();
			mlocalCloud = m_CloudExtraction.m_cornerPointsLessSharp;
			//mlocalCloud = m_CloudExtraction.m_segmentedPoints;
			cout << "localcloud size: " << mlocalCloud->size() << endl;

			// դ�񻯵�ǰ���ƣ��洢����ͼ����
			GridingPointCloud();
			mlocalCloud->clear();
			mlocalCloudDS->clear();
			cout << "********************************************" << endl;
		}
	}

	WriteCSV();

	return true;
}

inline bool CPointCloud::ReadGTFile()
{
	string gtfilename = this->mfilefolder + "//GroundTruth.txt";
	FILE* fp;
	if (fopen_s(&fp, gtfilename.c_str(), "rt") != 0)
	{
		cout << "can't find GroundTruth.txt in this file folder" << endl;
		return false;
	}
	while (!feof(fp))
	{
		POS tempPos;
		fscanf(fp, "%*d %lf %*d %lf %lf %lf %*lf %*lf %*lf %*lf %*lf %*lf %lf %lf %lf %*lf %*lf %*lf %*lf %*lf %*lf\n",
			&tempPos.time, &tempPos.XYZ[0], &tempPos.XYZ[1], &tempPos.XYZ[2], &tempPos.RPY[0], &tempPos.RPY[1], &tempPos.RPY[2]);

		RotaAngle2RotaMatrix(tempPos.RPY, tempPos.RLw);
		RotaMatrix2IMatrix(tempPos.RLw);
		MatrixInv(3, 3, tempPos.RLw);
		this->mvGT.push_back(tempPos);
	}
	return true;
}

inline void CPointCloud::CheckFilelist(vector<string>& filenames)
{
	string filePath = this->mfilefolder + "//velodyne//";
	//�ļ����  
	intptr_t   hFile = 0;
	//�ļ���Ϣ  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(filePath.c_str()).append("\\*.bin").c_str(), &fileinfo)) != -1)
	{
		do
		{
			filenames.push_back(fileinfo.name);
		} while (_findnext(hFile, &fileinfo) == 0);
	}
	_findclose(hFile);
}

inline bool CPointCloud::RunCurrentPointCloud(int count)
{
	string currentfilename = this->mfilefolder + "//velodyne//" + this->mlistPC[count];
	mlocalCloud.reset(new PointCloudType());
	mlocalCloudDS.reset(new PointCloudType());

	cout << "reading file " << count << ".bin" << endl;

	float *data = (float*)malloc(4 * sizeof(float));

	// pointers
	float *px;
	float *py;
	float *pz;
	float *pr;
	float tmpx, tmpy, tmpz, tmpr;

	float Point[4];
	vector<float> vPoint;
	vPoint.resize(4);

	FILE *instream;
	vector<vector<float>> v_PointCloud;
	fopen_s(&instream, currentfilename.c_str(), "rb");

	while (!feof(instream))
	{
		if (fread(data, sizeof(float), 4, instream) == 4)
		{
			px = data + 0; py = data + 1; pz = data + 2; pr = data + 3;
			vPoint[0] = *px; vPoint[1] = *py; vPoint[2] = *pz; vPoint[3] = *pr;
			if (*pz > -1.5)
			{
				v_PointCloud.push_back(vPoint);
			}
		}
	}

	// �޳���������1�����ĵ��
	sort(v_PointCloud.begin(), v_PointCloud.end(), MyComparison);
	for (int i = 0; i < 0.1 * v_PointCloud.size(); i++)
	{
		v_PointCloud.pop_back();
	}

	///< ԭʼ����ת��PCL��ʽ����
	Rawdata2PCL(v_PointCloud, mlocalCloud);

	//pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
	//approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
	//approximate_voxel_filter.setInputCloud(mlocalCloud);
	//approximate_voxel_filter.filter(*mlocalCloudDS);

	//for (int i = 0; i < mlocalCloudDS->size(); i++)
	//{
	//	double pointXYZ[3];
	//	pointXYZ[0] = -mlocalCloudDS->points[i].y;
	//	pointXYZ[1] = mlocalCloudDS->points[i].z;
	//	pointXYZ[2] = mlocalCloudDS->points[i].x;
	//	//cout << pointXYZ[0] << "," << pointXYZ[1] << "," << pointXYZ[2] << endl;
	//	M33XM31(mCurrentPos.RLw, pointXYZ);
	//	M31M31(mCurrentPos.XYZ, pointXYZ);

	//	mlocalCloudDS->points[i].x = pointXYZ[0];
	//	mlocalCloudDS->points[i].y = pointXYZ[1];
	//	mlocalCloudDS->points[i].z = pointXYZ[2];
	//	//cout << mlocalCloudDS->points[i].x << "," << mlocalCloudDS->points[i].y << "," << mlocalCloudDS->points[i].z << endl;
	//}


	vector<vector<float>>().swap(v_PointCloud);///< ��յ���vector

	fclose(instream);
	free(data);

	return true;
}

inline int CPointCloud::GetCurrentPosture(int count)
{
	memset(&this->mCurrentPos, 0, sizeof(this->mCurrentPos));
	mCurrentPos = mvGT[count];

	double dx = mCurrentPos.XYZ[0] - mLastPos.XYZ[0];
	double dz = mCurrentPos.XYZ[2] - mLastPos.XYZ[2];

	if (count == 0)
	{
		mLastPos = mCurrentPos;
		return 1;
	}
	else
	{
		if (sqrt(dx * dx + dz * dz) > m_dist)	///< ˮƽ��ÿ��XXm����һ֡����
		{
			mLastPos = mCurrentPos;
			return 1;
		}
		else {
			return 0;
		}
	}
}

inline bool CPointCloud::TransformToWorld(PointType point)
{
	double pointXYZ[3];
	pointXYZ[0] = -point.y;
	pointXYZ[1] = point.z;
	pointXYZ[2] = point.x;
	M33XM31(mCurrentPos.RLw, pointXYZ);
	M31M31(mCurrentPos.XYZ, pointXYZ);

	point.x = pointXYZ[0];
	point.y = pointXYZ[1];
	point.z = pointXYZ[2];
	return true;
}

inline bool CPointCloud::GridingPointCloud()
{
	vector<string> v_gridfilenames;
	vector<vector<vector<float>>> v_pointCloud;
	v_pointCloud.resize(100);
	vector<float> vPoint;

	pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
	approximate_voxel_filter.setInputCloud(mlocalCloud);
	approximate_voxel_filter.filter(*mlocalCloudDS);

	//for (int i = 0; i < mlocalCloudDS->size(); i++)
	//{
	//	double pointXYZ[3];
	//	pointXYZ[0] = -mlocalCloudDS->points[i].y;
	//	pointXYZ[1] = mlocalCloudDS->points[i].z;
	//	pointXYZ[2] = mlocalCloudDS->points[i].x;
	//	//cout << pointXYZ[0] << "," << pointXYZ[1] << "," << pointXYZ[2] << endl;
	//	M33XM31(mCurrentPos.RLw, pointXYZ);
	//	M31M31(mCurrentPos.XYZ, pointXYZ);

	//	mlocalCloudDS->points[i].x = pointXYZ[0];
	//	mlocalCloudDS->points[i].y = pointXYZ[1];
	//	mlocalCloudDS->points[i].z = pointXYZ[2];
	//	//cout << mlocalCloudDS->points[i].x << "," << mlocalCloudDS->points[i].y << "," << mlocalCloudDS->points[i].z << endl;
	//}
	for (int i = 0; i < mlocalCloud->size(); i++)
	{
		double pointXYZ[3];
		pointXYZ[0] = -mlocalCloud->points[i].y;
		pointXYZ[1] = mlocalCloud->points[i].z;
		pointXYZ[2] = mlocalCloud->points[i].x;
		//cout << pointXYZ[0] << "," << pointXYZ[1] << "," << pointXYZ[2] << endl;
		M33XM31(mCurrentPos.RLw, pointXYZ);
		M31M31(mCurrentPos.XYZ, pointXYZ);

		mlocalCloud->points[i].x = pointXYZ[0];
		mlocalCloud->points[i].y = pointXYZ[1];
		mlocalCloud->points[i].z = pointXYZ[2];
		//cout << mlocalCloudDS->points[i].x << "," << mlocalCloudDS->points[i].y << "," << mlocalCloudDS->points[i].z << endl;
	}

	//for (int i = 0; i < mlocalCloudDS->size(); i++)
	for (int i = 0; i < mlocalCloud->size(); i++)
	{
		//����Ӧ�����ĸ�������
		//double x = mlocalCloudDS->points[i].x;
		//double y = mlocalCloudDS->points[i].z;
		double x = mlocalCloud->points[i].x;
		double y = mlocalCloud->points[i].z;
		int min_x_b = m_grid_size * static_cast<int>(floor(x / m_grid_size));
		int min_y_b = m_grid_size * static_cast<int>(floor(y / m_grid_size));

		//�ҵ���Ӧ�������ͼ�ļ�
		string PCDfilename;
		PCDfilename = this->mfilefolder + "//gridmap//" + std::to_string(m_grid_size) + "_" +
			std::to_string(min_x_b) + "_" +
			std::to_string(min_y_b) + ".bin";
		//PCDfilename = this->mfilefolder + "//gridmap//" + std::to_string(m_grid_size) + "_" +
		//	std::to_string(min_x_b) + "_" +
		//	std::to_string(min_y_b) + ".txt";

		int m = 0;
		for (m = 0; m < v_gridfilenames.size(); m++)
		{
			if (PCDfilename == v_gridfilenames[m])
			{
				//vPoint.push_back(mlocalCloudDS->points[i].x);
				//vPoint.push_back(mlocalCloudDS->points[i].y);
				//vPoint.push_back(mlocalCloudDS->points[i].z);
				//vPoint.push_back(mlocalCloudDS->points[i].intensity);
				vPoint.push_back(mlocalCloud->points[i].x);
				vPoint.push_back(mlocalCloud->points[i].y);
				vPoint.push_back(mlocalCloud->points[i].z);
				vPoint.push_back(mlocalCloud->points[i].intensity);
				v_pointCloud[m].push_back(vPoint);
				vPoint.clear();
				break;
			}
		}
		if (m >= v_gridfilenames.size())
		{
			v_gridfilenames.push_back(PCDfilename);

			//vPoint.push_back(mlocalCloudDS->points[i].x);
			//vPoint.push_back(mlocalCloudDS->points[i].y);
			//vPoint.push_back(mlocalCloudDS->points[i].z);
			//vPoint.push_back(mlocalCloudDS->points[i].intensity);
			vPoint.push_back(mlocalCloud->points[i].x);
			vPoint.push_back(mlocalCloud->points[i].y);
			vPoint.push_back(mlocalCloud->points[i].z);
			vPoint.push_back(mlocalCloud->points[i].intensity);
			v_pointCloud[m].push_back(vPoint);
			vPoint.clear();

		}

	}

	FILE* fpcd;
	for (int i = 0; i < v_gridfilenames.size(); i++)
	{
		if (!fopen_s(&fpcd, v_gridfilenames[i].c_str(), "ab+"))
		{
			vector<vector<float>> tempPoint;
			tempPoint = v_pointCloud[i];
			for (int j = 0; j < tempPoint.size(); j++)
			{
				// д��.txt
				//fprintf(fpcd, "%lf %lf %lf %lf\n", tempPoint[j].at(0), tempPoint[j].at(1),
				//	tempPoint[j].at(2), tempPoint[j].at(3));

				// д��.bin
				float p[4];
				p[0] = tempPoint[j].at(0);				
				p[1] = tempPoint[j].at(1);
				p[2] = tempPoint[j].at(2);
				p[3] = tempPoint[j].at(3);
				fwrite(&p, sizeof(float), 4, fpcd);
			}

		}
		fclose(fpcd);
	}

	return true;
}

inline bool CPointCloud::DownsamplePointCloud(PointCloudType::Ptr pointcloud, PointCloudType::Ptr pointcloudDS, float leafsize)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
	approximate_voxel_filter.setInputCloud(pointcloud);
	approximate_voxel_filter.filter(*pointcloudDS);
	return true;
}

inline bool CPointCloud::WriteCSV()
{
	FILE* fcsv;
	vector<string> filenames;
	string filePath = this->mfilefolder + "//gridmap//";
	string fileCSV = this->mfilefolder + "//gridmap//" + "gridmap.csv";
	//�ļ����  
	intptr_t   hFile = 0;
	//�ļ���Ϣ  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(filePath.c_str()).append("\\*.txt").c_str(), &fileinfo)) != -1)
	{
		do
		{
			filenames.push_back(fileinfo.name);
		} while (_findnext(hFile, &fileinfo) == 0);
	}
	_findclose(hFile);


	fopen_s(&fcsv, fileCSV.c_str(), "wt");
	for (int i = 0; i < filenames.size(); i++)
	{
		fprintf(fcsv, "%s\n", filenames[i]);
	}

	fclose(fcsv);
	return true;
}
