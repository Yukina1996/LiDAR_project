/*
LIO�����õĳ��ú���
*/

#ifndef LIODLL_LIOCMNFUNC_H
#define LIODLL_LIOCMNFUNC_H

#include "LIOBase.h"

using namespace std;

bool Rawdata2PCL(vector<float> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout);
bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<float> pointCloudout);

bool Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout);
bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<vector<float>> pointCloudout);

double rad2deg(double radians);
double deg2rad(double degrees);

void M13xM33(const double M1[3], const double M2[9], double M3[3]);


inline bool Rawdata2PCL(vector<float> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout)
{
	return false;
}

inline bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<float> pointCloudout)
{
	return false;
}

// vector�洢��ԭʼ�������-> PCL��ʽ�ĵ���
inline bool Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout)
{
	int num = pointCloudIn.size();
	laserCloudout->width = num;
	laserCloudout->height = 1;			///< ������� �߶���Ϊ1
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
		num = laserCloudIn->width;		///< PCL��������
	}
	else
	{
		num = laserCloudIn->width*laserCloudIn->height;  ///< PCL��������
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

//bool MatrixSkewSymmetric(const double M1[3], double M2[9])
//{
//	M2[0] = 0; M2[1] = -M1[2]; M2[2] = M1[1];
//	M2[3] = M1[2]; M2[4] = 0; M2[5] = -M1[0];
//	M2[6] = -M1[1]; M2[7] = M1[0]; M2[8] = 0;
//	return true;
//}

inline void M13xM33(const double M1[3], const double M2[9], double M3[3])
{
	M3[0] = M1[0] * M2[0] + M1[1] * M2[3] + M1[2] * M2[6];
	M3[1] = M1[0] * M2[1] + M1[1] * M2[4] + M1[2] * M2[7];
	M3[2] = M1[0] * M2[2] + M1[1] * M2[5] + M1[2] * M2[8];
}

inline bool EulerAngle2RotationMat_RzRxRy(const double M1[3], double M2[9])
{
	// �������ת�����˳����RzRxRy ˳ʱ��Ϊ��
	double rx = M1[0];
	double ry = M1[1];
	double rz = M1[2];

	M2[0] = cos(ry)*cos(rz) - sin(rx)*sin(ry)*sin(rz);
	M2[1] = -cos(rx)*sin(rz);
	M2[2] = cos(rz)*sin(ry) + cos(ry)*sin(rx)*sin(rz);
	M2[3] = cos(ry)*sin(rz) + cos(rz)*sin(rx)*sin(ry);
	M2[4] = cos(rx)*cos(rz);
	M2[5] = sin(ry)*sin(rz) - cos(ry)*cos(rz)*sin(rx);
	M2[6] = -cos(rx)*sin(ry);
	M2[7] = sin(rx);
	M2[8] = cos(rx)*cos(ry);

	return true;
}

inline bool EulerAngle2RotationMat_RyRxRz(const double M1[3], double M2[9])
{
	// �������ת�����˳����RyRxRz ˳ʱ��Ϊ��
	double rx = M1[0];
	double ry = M1[1];
	double rz = M1[2];

	M2[0] = cos(ry)*cos(rz) + sin(rx)*sin(ry)*sin(rz);
	M2[1] = cos(rz)*sin(rx)*sin(ry) - cos(ry)*sin(rz);
	M2[2] = cos(rx)*sin(ry);
	M2[3] = cos(rx)*sin(rz);
	M2[4] = cos(rx)*cos(rz);
	M2[5] = -sin(rx);
	M2[6] = cos(ry)*sin(rx)*sin(rz) - cos(rz)*sin(ry);
	M2[7] = sin(ry)*sin(rz) + cos(ry)*cos(rz)*sin(rx);
	M2[8] = cos(rx)*cos(ry);
	return true;
}

#endif // !LIODLL_LIOCMNFUNC_H

