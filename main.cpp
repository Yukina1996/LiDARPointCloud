#include "PointCloud.h"
#include "CmnFunc.h"

int main()
{
	CPointCloud PC;
	
	string folder = "F://KITTIData//data_odometry_velodyne//dataset//sequences//00";
	PC.RunLiDARPointCloud(folder);

	vector<string> filenamelist;	//�洢�ļ��������е��ļ���

	//CheckTxtFilelist(filepath, filenamelist);
	//for (int i = 0; i < filenamelist.size(); i++)
	//{
	//	txt2pcd(filepath, filenamelist[i]);
	//}

	return 1;
}