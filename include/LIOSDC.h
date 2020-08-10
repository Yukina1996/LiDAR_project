/*
�����õ�һЩ�ṹ�塢�����Ķ���
*/
#ifndef LIODLL_LIOSDC_H
#define LIODLL_LIOSDC_H

//#define M_PI 3.14159265

#define MAXSIZE             (1024)

#include "LIOBase.h"

typedef pcl::PointXYZI  PointType;

using namespace std;

extern long gs_SystemCount;									///< ÿ����һ֡���Ƽ�����һ
static const float SegmentTheta = 1.0472;					///< ���Ʒָ�ʱ�ĽǶȿ�����ޣ���/3��
static const int SegmentValidPointNum = 5;					///< ���������������5������Ϊ�ָ����������
static const int SegmentValidLineNum = 3;

static const int EdgeFeatureNum = 2;						///< ÿ������������ṩ2����������
static const int SurfFeatureNum = 4;						///< ÿ������������ṩ4����������
static const int SectionsTotal = 6;							///< 360�� ����6��������
static const float EdgeThreshold = 0.1;						///< ��or���������������ֵ
static const float SurfThreshold = 0.1;
static const float NearestFeatureSearchSqDist = 15;

static const double SurroundingKeyframeSearchRadius = 80;
//static const float KeyFramesDistance = 3;					///< �ؼ�֡�˴˾���4m����
//static const float KeyFramesAttitude = 15;					///< �ؼ�֡�˴�ת��30������


struct PointCloudType
{
	float x, y, z;
	float intensity;
	int rowID;
	int colID;
	int label;					///< ��ǵ��Ƶ���������

	PointCloudType()
	{
		x = -1; y = -1; z = -1;
		intensity = -999;
		rowID = -1; colID = -1; label = -999;
	}
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

struct smoothness_t {
	float value;
	int ind;
};

struct by_value {
	bool operator()(smoothness_t const &left, smoothness_t const &right) {
		return left.value < right.value;
	}
};

struct Pose3D {
	double yaw, pitch, roll;
	double posX, posY, posZ;
	Pose3D() {
		yaw = pitch = roll = 0;
		posX = posY = posZ = 0;
	}
};

///< PCL�Զ��������
struct PointXYZIRPYT
{
	PCL_ADD_POINT4D		///// �õ�������4��Ԫ��
		PCL_ADD_INTENSITY;
	float roll;
	float pitch;
	float yaw;
	double time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;// ǿ��SSE����

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,// ע������ͺ�
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif // !LIODLL_LIOSDC_H

