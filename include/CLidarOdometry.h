#ifndef LIODLL_CLIDAR_ODOMETRY_H
#define LIODLL_CLIDAR_ODOMETRY_H

#include "LIOBase.h"
#include "LIOCmnFunc.h"
#include "LIOSDC.h"
#include "CLIOOption.h"
#include "CLIOFFStream.h"
#include "CFeatureExtraction.h"

class CLidarOdometry
{
public:
	CLidarOdometry();
	virtual ~CLidarOdometry();

	CLIOOPTION* m_LIOOpt;
	CPointCloudFFSTREAM* m_PCFFStream;
	CFeatureExtraction* m_FeatureExtraction;

public:

	bool RunLidarOdometry(const int frameIndex);

	bool Init();														///< ����ǰ��ĵ����ļ�����ʼ��vector������

	bool ResetParameters();

	bool InitPreviousPointsCloud(const int frameIndex);

	void TransformToStart(pcl::PointXYZI * const pi, pcl::PointXYZI * const po);

	void TransformToEnd(pcl::PointXYZI * const pi, pcl::PointXYZI * const po);

	int UpdateTransformation();

	void FindCorrespondingSurfFeatures(int iterCount);

	void FindCorrespondingCornerFeatures(int iterCount);

	bool CalculateTransformation(int iterCount);

	bool calculateTransformationSurf(int iterCount);

	bool calculateTransformationCorner(int iterCount);

	void integrateTransformation();

	void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
		float &ox, float &oy, float &oz);

					
	int CornerFeatureNum, SurfFeatureNum;

	float transformCur[6] = { 0 };			///< ��ǰ֡�����һ֡��״̬ת������in the local frame
	float transformSum[6] = { 0 };			///< ��ǰ֡����ڵ�һ֡��״̬ת������in the global frame
	Pose3D m_TransformCur;					///< ��Ե�һ֡�� P(X,Y,Z) angle(yaw,pitch,roll)
	vector<Pose3D> m_TransformSum;

	/*double m_ErrorState[6];							///< deltaP phi
	double m_SWErrorState[6 * (SlidingWindow - 1)];
	double m_PriorStateP[(SlidingWindow - 1)*(SlidingWindow - 1) * 36];	///< ��������Լ����Ȩ��
	double m_PriorStateD[(SlidingWindow - 1)*(SlidingWindow - 1) * 36];
			
	int m_nFFMeasurenments[SlidingWindow - 1];						///< ��¼֡��֮֡���ж���ƥ�䵽�ĵ��
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SW_laserCloudOri;		///< SW:sliding window
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SW_coeffSel;*/

	//��ǰʱ��
	//struct SegCloudInfo m_SegInfo;									///< �ָ���Ƶ�����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SegmentedPointsCur;			///< �ָ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_CornerPointsSharpCur;				///< ����������,���ʴ�
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_CornerPointsLessSharpCur;			///< ���������ƣ����ʽϴ�
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SurfPointsFlatCur;					///< ���������ƣ�����С
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SurfPointsLessFlatCur;				///< ���������ƣ����ʽ�С

	//��һʱ�� ���ָ�����ļ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_CornerPointsLast;					
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SurfPointsLast;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScan;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScanDS;

	pcl::VoxelGrid<pcl::PointXYZI> m_DownSizeFilter;
	int m_LaserCloudCornerLastNum;
	int m_LaserCloudSurfLastNum;

	pcl::PointCloud<pcl::PointXYZI>::Ptr m_laserCloudOri;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_coeffSel;

	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kdtreeCornerLast;	
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kdtreeSurfLast;

	vector<int> m_vPointSearchIndex;
	vector<float> m_vPointSearchSqDis;

	vector<float> m_vpointSearchCornerInd1;
	vector<float> m_vpointSearchCornerInd2;

	vector<float> m_vpointSearchSurfInd1;
	vector<float> m_vpointSearchSurfInd2;
	vector<float> m_vpointSearchSurfInd3;

	pcl::PointXYZI m_pointOri, m_pointSel, m_pointClose, m_coeff, pointSel, tripod1, tripod2, tripod3;

	int m_SystemCount;

	bool isDegenerate;
	cv::Mat matP;
};

inline CLidarOdometry::CLidarOdometry()
{
	m_SegmentedPointsCur.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_CornerPointsSharpCur.reset(new pcl::PointCloud <pcl::PointXYZI>);
	m_CornerPointsLessSharpCur.reset(new pcl::PointCloud <pcl::PointXYZI>);
	m_SurfPointsFlatCur.reset(new pcl::PointCloud <pcl::PointXYZI>);
	m_SurfPointsLessFlatCur.reset(new pcl::PointCloud <pcl::PointXYZI>);
	m_CornerPointsLast.reset(new pcl::PointCloud <pcl::PointXYZI>);
	m_SurfPointsLast.reset(new pcl::PointCloud <pcl::PointXYZI>);
	m_surfPointsLessFlatScan.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_surfPointsLessFlatScanDS.reset(new pcl::PointCloud<pcl::PointXYZI>());

	m_laserCloudOri.reset(new pcl::PointCloud <pcl::PointXYZI>);
	m_coeffSel.reset(new pcl::PointCloud <pcl::PointXYZI>);

	m_DownSizeFilter.setLeafSize(0.2, 0.2, 0.2);

	m_kdtreeCornerLast.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
	m_kdtreeSurfLast.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

	m_TransformSum.push_back(m_TransformCur);

}

inline CLidarOdometry::~CLidarOdometry()
{

}

inline bool CLidarOdometry::RunLidarOdometry(const int frameIndex)
{
	Init();

	m_SystemCount = frameIndex;

	// ����ǰһ֡����������
	InitPreviousPointsCloud(m_SystemCount - 2);

	UpdateTransformation();

	integrateTransformation();

	ResetParameters();

	return true;
}


inline bool CLidarOdometry::Init()
{
	float tmpx, tmpy, tmpz;
	//ֱ�Ӵ�m_FeatureExtraction���л�õ�ǰ���Ƶ�������
	m_SegmentedPointsCur = m_FeatureExtraction->m_segmentedPoints;
	m_CornerPointsSharpCur = m_FeatureExtraction->m_cornerPointsSharp;
	m_CornerPointsLessSharpCur = m_FeatureExtraction->m_cornerPointsLessSharp;
	m_SurfPointsFlatCur = m_FeatureExtraction->m_surfPointsFlat;
	m_SurfPointsLessFlatCur = m_FeatureExtraction->m_surfPointsLessFlat;

	vector<int>().swap(m_vPointSearchIndex);
	vector<float>().swap(m_vPointSearchSqDis);

	m_vpointSearchCornerInd1.resize(m_LIOOpt->m_N_SCAN*m_LIOOpt->m_Horizon_SCAN);
	m_vpointSearchCornerInd2.resize(m_LIOOpt->m_N_SCAN*m_LIOOpt->m_Horizon_SCAN);

	m_vpointSearchSurfInd1.resize(m_LIOOpt->m_N_SCAN*m_LIOOpt->m_Horizon_SCAN);
	m_vpointSearchSurfInd2.resize(m_LIOOpt->m_N_SCAN*m_LIOOpt->m_Horizon_SCAN);
	m_vpointSearchSurfInd3.resize(m_LIOOpt->m_N_SCAN*m_LIOOpt->m_Horizon_SCAN);

	isDegenerate = false;
	matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

	for (int i = 0; i < 6; ++i) {
		//transformCur[i] = 0;
	}


	return true;
}

inline bool CLidarOdometry::ResetParameters()
{
	// ���m_TransformCur
	memset(&m_TransformCur, 0, sizeof(m_TransformCur));

	return true;
}

inline bool CLidarOdometry::InitPreviousPointsCloud(const int frameIndex)
{
	string PreSegmentedPCFile;
	PointCloudType PtsCloudType;
	pcl::PointXYZI ThisPoint;
	FILE * fin;

	PreSegmentedPCFile = m_LIOOpt->m_SegmentedPCPath + "//" + m_PCFFStream->m_vPointCloudFileNames[frameIndex];
	PreSegmentedPCFile = PreSegmentedPCFile.substr(0, PreSegmentedPCFile.rfind("\n"));

	// fopen_s(&fin, PreSegmentedPCFile.c_str(), "rb");
	fin = fopen(PreSegmentedPCFile.c_str(), "rb");

	m_surfPointsLessFlatScan->clear();
	m_SurfPointsLast->clear();
	m_CornerPointsLast->clear();

	while (!feof(fin))
	{
		fread(&PtsCloudType, sizeof(PtsCloudType), 1, fin);
		ThisPoint.x = PtsCloudType.x;
		ThisPoint.y = PtsCloudType.y;
		ThisPoint.z = PtsCloudType.z;
		ThisPoint.intensity = PtsCloudType.intensity;
		if (PtsCloudType.label == 0)			///< surf features
		{
			m_surfPointsLessFlatScan->push_back(ThisPoint);
		}
		else if (PtsCloudType.label == -1)		///< surf features
		{
			m_SurfPointsLast->push_back(ThisPoint);
		}
		if (PtsCloudType.label == 1 || PtsCloudType.label == 2)		///< edge features
		{
			m_CornerPointsLast->push_back(ThisPoint);
		}
	}
	fclose(fin);

	///< ��less flat�������㽵����
	m_surfPointsLessFlatScanDS->clear();
	m_DownSizeFilter.setInputCloud(m_surfPointsLessFlatScan);
	m_DownSizeFilter.filter(*m_surfPointsLessFlatScanDS);
	*m_SurfPointsLast += *m_surfPointsLessFlatScanDS;

	m_LaserCloudCornerLastNum = m_CornerPointsLast->points.size();
	m_LaserCloudSurfLastNum = m_SurfPointsLast->points.size();

	for (int i = 0; i < m_LaserCloudCornerLastNum; i++) {
		TransformToEnd(&m_CornerPointsLast->points[i], &m_CornerPointsLast->points[i]);
	}

	for (int i = 0; i < m_LaserCloudSurfLastNum; i++) {
		TransformToEnd(&m_SurfPointsLast->points[i], &m_SurfPointsLast->points[i]);
	}

	m_kdtreeCornerLast.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
	m_kdtreeSurfLast.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

	m_kdtreeCornerLast->setInputCloud(m_CornerPointsLast);
	m_kdtreeSurfLast->setInputCloud(m_SurfPointsLast);

	return true;
}

inline void CLidarOdometry::TransformToStart(pcl::PointXYZI * const pi, pcl::PointXYZI * const po)
{
	//s��ʾĳ����һ��ɨ�������е����ʱ�䣬����ģ�ͼ��裬��λ���ڲ�
	float s = 10 * (pi->intensity - int(pi->intensity));
	//float s = 1;

	float rx = s * transformCur[0];
	float ry = s * transformCur[1];
	float rz = s * transformCur[2];
	float tx = s * transformCur[3];
	float ty = s * transformCur[4];
	float tz = s * transformCur[5];

	float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
	float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
	float z1 = (pi->z - tz);

	float x2 = x1;
	float y2 = cos(rx) * y1 + sin(rx) * z1;
	float z2 = -sin(rx) * y1 + cos(rx) * z1;

	po->x = cos(ry) * x2 - sin(ry) * z2;
	po->y = y2;
	po->z = sin(ry) * x2 + cos(ry) * z2;
	po->intensity = pi->intensity;

}

inline void CLidarOdometry::TransformToEnd(pcl::PointXYZI * const pi, pcl::PointXYZI * const po)
{
	//s��ʾĳ����һ��ɨ�������е����ʱ�䣬��λ���ڲ�
	float s = 10 * (pi->intensity - int(pi->intensity));

	float rx = s * transformCur[0];
	float ry = s * transformCur[1];
	float rz = s * transformCur[2];
	float tx = s * transformCur[3];
	float ty = s * transformCur[4];
	float tz = s * transformCur[5];

	float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
	float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
	float z1 = (pi->z - tz);

	float x2 = x1;
	float y2 = cos(rx) * y1 + sin(rx) * z1;
	float z2 = -sin(rx) * y1 + cos(rx) * z1;

	float x3 = cos(ry) * x2 - sin(ry) * z2;
	float y3 = y2;
	float z3 = sin(ry) * x2 + cos(ry) * z2;

	rx = transformCur[0];
	ry = transformCur[1];
	rz = transformCur[2];
	tx = transformCur[3];
	ty = transformCur[4];
	tz = transformCur[5];

	float x4 = cos(ry) * x3 + sin(ry) * z3;
	float y4 = y3;
	float z4 = -sin(ry) * x3 + cos(ry) * z3;

	float x5 = x4;
	float y5 = cos(rx) * y4 - sin(rx) * z4;
	float z5 = sin(rx) * y4 + cos(rx) * z4;

	po->x = cos(rz) * x5 - sin(rz) * y5 + tx;
	po->y = sin(rz) * x5 + cos(rz) * y5 + ty;
	po->z = z5 + tz;
	po->intensity = int(pi->intensity);
}

inline int CLidarOdometry::UpdateTransformation()
{
	// ������̫�ٲ��ܽ���ƥ��
	if (m_LaserCloudCornerLastNum < 10 || m_LaserCloudSurfLastNum < 100)
		return 0;

	// �趨����L - M�˶����Ƶĵ�������Ϊ25��
	//for (int iterCount = 0; iterCount < 25; iterCount++)
	//{
	//	m_laserCloudOri->clear();
	//	m_coeffSel->clear();

	//	FindCorrespondingCornerFeatures(iterCount);

	//	FindCorrespondingSurfFeatures(iterCount);

	//	if (m_laserCloudOri->points.size() < 20)
	//		continue;		
	//	if (CalculateTransformation(iterCount) == 2) {
	//		break;
	//	}
	//}

	for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
		m_laserCloudOri->clear();
		m_coeffSel->clear();

		FindCorrespondingSurfFeatures(iterCount1);

		if (m_laserCloudOri->points.size() < 10)
			continue;
		if (calculateTransformationSurf(iterCount1) == false)
			break;
	}

	for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

		m_laserCloudOri->clear();
		m_coeffSel->clear();

		FindCorrespondingCornerFeatures(iterCount2);

		if (m_laserCloudOri->points.size() < 10)
			continue;
		if (calculateTransformationCorner(iterCount2) == false)
			break;
	}

	return 0;
}

inline void CLidarOdometry::FindCorrespondingSurfFeatures(int iterCount)
{
	int surfPointsFlatNum = m_SurfPointsFlatCur->points.size();

	for (int i = 0; i < surfPointsFlatNum; i++)
	{
		TransformToStart(&m_SurfPointsFlatCur->points[i], &pointSel);

		// ÿ����5������һ�������
		if (iterCount % 1 == 0) {
			//kd-tree�������ң��ھ�������դ���˲�֮���ƽ����в��ң�һ��ƽ���̫�࣬�˲�����������������С
			m_kdtreeSurfLast->nearestKSearch(pointSel, 1, m_vPointSearchIndex, m_vPointSearchSqDis);

			int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

			if (m_vPointSearchSqDis[0] < NearestFeatureSearchSqDist)
			{
				closestPointInd = m_vPointSearchIndex[0];

				int closestPointScan = int(m_SurfPointsLast->points[closestPointInd].intensity);


				float pointSqDis, minPointSqDis2 = NearestFeatureSearchSqDist, minPointSqDis3 = NearestFeatureSearchSqDist;

				for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
					if (int(m_SurfPointsLast->points[j].intensity) > closestPointScan + 2.5) {
						break;
					}

					pointSqDis = (m_SurfPointsLast->points[j].x - pointSel.x) *
						(m_SurfPointsLast->points[j].x - pointSel.x) +
						(m_SurfPointsLast->points[j].y - pointSel.y) *
						(m_SurfPointsLast->points[j].y - pointSel.y) +
						(m_SurfPointsLast->points[j].z - pointSel.z) *
						(m_SurfPointsLast->points[j].z - pointSel.z);

					if (int(m_SurfPointsLast->points[j].intensity) <= closestPointScan) {//�������ߺ�С�ڵ����������ߺ�(Ӧ�����ȡ�ȣ�Ҳ��ͬһ���ϵĵ�)
						if (pointSqDis < minPointSqDis2) {
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					}
					else {//����㴦�ڴ��ڸ�����
						if (pointSqDis < minPointSqDis3) {
							minPointSqDis3 = pointSqDis;
							minPointInd3 = j;
						}
					}
				}

				//ͬ��
				for (int j = closestPointInd - 1; j >= 0; j--) {
					if (int(m_SurfPointsLast->points[j].intensity) < closestPointScan - 2.5) {
						break;
					}

					pointSqDis = (m_SurfPointsLast->points[j].x - pointSel.x) *
						(m_SurfPointsLast->points[j].x - pointSel.x) +
						(m_SurfPointsLast->points[j].y - pointSel.y) *
						(m_SurfPointsLast->points[j].y - pointSel.y) +
						(m_SurfPointsLast->points[j].z - pointSel.z) *
						(m_SurfPointsLast->points[j].z - pointSel.z);

					if (int(m_SurfPointsLast->points[j].intensity) >= closestPointScan) {
						if (pointSqDis < minPointSqDis2) {
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					}
					else {
						if (pointSqDis < minPointSqDis3) {
							minPointSqDis3 = pointSqDis;
							minPointInd3 = j;
						}
					}
				}



			}

			m_vpointSearchSurfInd1[i] = closestPointInd;	//kd-tree��������,-1��ʾδ�ҵ�����Ҫ��ĵ�
			m_vpointSearchSurfInd2[i] = minPointInd2;		//ͬһ�ߺ��ϵľ�������ĵ㣬-1��ʾδ�ҵ�����Ҫ��ĵ�
			m_vpointSearchSurfInd3[i] = minPointInd3;		//��ͬ�ߺ��ϵľ�������ĵ㣬-1��ʾδ�ҵ�����Ҫ��ĵ�

		}

		if (m_vpointSearchSurfInd2[i] >= 0 && m_vpointSearchSurfInd3[i] >= 0) {//�ҵ���������
			tripod1 = m_SurfPointsLast->points[m_vpointSearchSurfInd1[i]];//A��
			tripod2 = m_SurfPointsLast->points[m_vpointSearchSurfInd2[i]];//B��
			tripod3 = m_SurfPointsLast->points[m_vpointSearchSurfInd3[i]];//C��

			float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
				- (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);

			float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
				- (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);

			float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
				- (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
			float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

			float ps = sqrt(pa * pa + pb * pb + pc * pc);

			pa /= ps;
			pb /= ps;
			pc /= ps;
			pd /= ps;

			float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

			//ͬ������Ȩ��
			float s = 1;
			if (iterCount >= 5) {
				s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
					+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));
			}

			//����Ȩ��
			if (s > 0.1 && pd2 != 0)
			{
				m_coeff.x = s * pa;
				m_coeff.y = s * pb;
				m_coeff.z = s * pc;
				m_coeff.intensity = s * pd2;

				//����ԭʼ������Ӧ��ϵ��
				m_laserCloudOri->push_back(m_SurfPointsFlatCur->points[i]);
				m_coeffSel->push_back(m_coeff);
			}
		}

	}
}


inline void CLidarOdometry::FindCorrespondingCornerFeatures(int iterCount)
{
	int cornerPointsSharpNum = m_CornerPointsSharpCur->points.size();

	for (int i = 0; i < cornerPointsSharpNum; i++)
	{
		TransformToStart(&m_CornerPointsSharpCur->points[i], &pointSel);

		//ÿ������Σ����²��������
		if (iterCount % 1 == 0) {

			//kd-tree����һ���������㣬���ص�δ��������դ���˲���һ����ص㱾���ͱȽ��٣������˲�
			m_kdtreeCornerLast->nearestKSearch(pointSel, 1, m_vPointSearchIndex, m_vPointSearchSqDis);

			int closestPointInd = -1, minPointInd2 = -1;

			//Ѱ�������߾���Ŀ��������С�ĵ�
			//�ٴ����ѣ�velodyne��2��һ�ߣ�scanID���ڲ��������ߺ����ڣ������߶������2�ȣ�Ҳ���ߺ�scanID���2
			if (m_vPointSearchSqDis[0] < NearestFeatureSearchSqDist)
			{
				//�ҵ������������ȷ�ܽ��Ļ�
				closestPointInd = m_vPointSearchIndex[0];
				//��ȡ������ߺ�

				int closestPointScan = int(m_CornerPointsLast->points[closestPointInd].intensity);

				float pointSqDis, minPointSqDis2 = NearestFeatureSearchSqDist;//��ʼ�ż�ֵ25�ף��ɴ��¹��˵�scanID���ڣ���ʵ���߲����ڵ�ֵ

				for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {//��scanID����ķ������
					if (int(m_CornerPointsLast->points[j].intensity) > closestPointScan + 2.5) {//��������
						break;
					}

					pointSqDis =
						(m_CornerPointsLast->points[j].x - pointSel.x) *
						(m_CornerPointsLast->points[j].x - pointSel.x) +
						(m_CornerPointsLast->points[j].y - pointSel.y) *
						(m_CornerPointsLast->points[j].y - pointSel.y) +
						(m_CornerPointsLast->points[j].z - pointSel.z) *
						(m_CornerPointsLast->points[j].z - pointSel.z);

					if (int(m_CornerPointsLast->points[j].intensity) > closestPointScan) {//ȷ�������㲻��ͬһ��scan�ϣ������߲���Ӧ�ÿ�����scanID == closestPointScan +/- 1 ������
						if (pointSqDis < minPointSqDis2) {//���������ҪС�ڳ�ʼֵ5��
							//������С���������
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					}
				}

				//ͬ��
				for (int j = closestPointInd - 1; j >= 0; j--) {//��scanID��С�ķ������
					if (int(m_CornerPointsLast->points[j].intensity) < closestPointScan - 2.5) {
						break;
					}

					pointSqDis = (m_CornerPointsLast->points[j].x - pointSel.x) *
						(m_CornerPointsLast->points[j].x - pointSel.x) +
						(m_CornerPointsLast->points[j].y - pointSel.y) *
						(m_CornerPointsLast->points[j].y - pointSel.y) +
						(m_CornerPointsLast->points[j].z - pointSel.z) *
						(m_CornerPointsLast->points[j].z - pointSel.z);

					if (int(m_CornerPointsLast->points[j].intensity) < closestPointScan) {
						if (pointSqDis < minPointSqDis2) {
							minPointSqDis2 = pointSqDis;
							minPointInd2 = j;
						}
					}
				}

			}

			//��ס����ߵĵ���
			m_vpointSearchCornerInd1[i] = closestPointInd;//kd-tree�������㣬-1��ʾδ�ҵ�����ĵ�
			m_vpointSearchCornerInd2[i] = minPointInd2;//��һ������ģ�-1��ʾδ�ҵ�����ĵ�

		}

		if (m_vpointSearchCornerInd2[i] >= 0) {//���ڵ���0��������-1��˵�������㶼�ҵ���
			tripod1 = m_CornerPointsLast->points[m_vpointSearchCornerInd1[i]];
			tripod2 = m_CornerPointsLast->points[m_vpointSearchCornerInd2[i]];

			//ѡ����������ΪO��kd-tree���������ΪA����һ�����������ΪB
			float x0 = pointSel.x;
			float y0 = pointSel.y;
			float z0 = pointSel.z;
			float x1 = tripod1.x;
			float y1 = tripod1.y;
			float z1 = tripod1.z;
			float x2 = tripod2.x;
			float y2 = tripod2.y;
			float z2 = tripod2.z;

			float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
			float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
			float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

			float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

			float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

			// la ��ʾd��x0��ƫ��
			float la = ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;
			// lb ��ʾd��y0��ƫ��
			float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;
			// lc ��ʾd��z0��ƫ��
			float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

			float ld2 = a012 / l12;

			//Ȩ�ؼ��㣬����Խ��Ȩ��ԽС������ԽСȨ��Խ�󣬵õ���Ȩ�ط�Χ<=1
			float s = 1;
			if (iterCount >= 5) {//5�ε���֮��ʼ����Ȩ������
				s = 1 - 1.8 * fabs(ld2);
			}

			if (s > 0.1 && ld2 != 0) {//ֻ����Ȩ�ش�ģ�Ҳ������Ƚ�С�ĵ㣬ͬʱҲ��������Ϊ���
			//����Ȩ��
				m_coeff.x = s * la;
				m_coeff.y = s * lb;
				m_coeff.z = s * lc;
				m_coeff.intensity = s * ld2;

				m_laserCloudOri->push_back(m_CornerPointsSharpCur->points[i]);
				m_coeffSel->push_back(m_coeff);
			}
		}
	}
}

inline bool CLidarOdometry::CalculateTransformation(int iterCount)
{
	int pointSelNum = m_laserCloudOri->points.size();

	cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

	float srx = sin(transformCur[0]);
	float crx = cos(transformCur[0]);
	float sry = sin(transformCur[1]);
	float cry = cos(transformCur[1]);
	float srz = sin(transformCur[2]);
	float crz = cos(transformCur[2]);
	float tx = transformCur[3];
	float ty = transformCur[4];
	float tz = transformCur[5];

	float a1 = crx * sry*srz; float a2 = crx * crz*sry; float a3 = srx * sry; float a4 = tx * a1 - ty * a2 - tz * a3;
	float a5 = srx * srz; float a6 = crz * srx; float a7 = ty * a6 - tz * crx - tx * a5;
	float a8 = crx * cry*srz; float a9 = crx * cry*crz; float a10 = cry * srx; float a11 = tz * a10 + ty * a9 - tx * a8;

	float b1 = -crz * sry - cry * srx*srz; float b2 = cry * crz*srx - sry * srz; float b3 = crx * cry; float b4 = tx * -b1 + ty * -b2 + tz * b3;
	float b5 = cry * crz - srx * sry*srz; float b6 = cry * srz + crz * srx*sry; float b7 = crx * sry; float b8 = tz * b7 - ty * b6 - tx * b5;

	float c1 = -b6; float c2 = b5; float c3 = tx * b6 - ty * b5; float c4 = -crx * crz; float c5 = crx * srz; float c6 = ty * c5 + tx * -c4;
	float c7 = b2; float c8 = -b1; float c9 = tx * -b2 - ty * -b1;

	for (int i = 0; i < pointSelNum; i++) {

		m_pointOri = m_laserCloudOri->points[i];
		m_coeff = m_coeffSel->points[i];

		float arx = (-a1 * m_pointOri.x + a2 * m_pointOri.y + a3 * m_pointOri.z + a4) * m_coeff.x
			+ (a5*m_pointOri.x - a6 * m_pointOri.y + crx * m_pointOri.z + a7) * m_coeff.y
			+ (a8*m_pointOri.x - a9 * m_pointOri.y - a10 * m_pointOri.z + a11) * m_coeff.z;

		float ary = (b1*m_pointOri.x + b2 * m_pointOri.y - b3 * m_pointOri.z + b4) * m_coeff.x
			+ (b5*m_pointOri.x + b6 * m_pointOri.y - b7 * m_pointOri.z + b8) * m_coeff.z;

		float arz = (c1*m_pointOri.x + c2 * m_pointOri.y + c3) * m_coeff.x
			+ (c4*m_pointOri.x - c5 * m_pointOri.y + c6) * m_coeff.y
			+ (c7*m_pointOri.x + c8 * m_pointOri.y + c9) * m_coeff.z;

		float atx = -b5 * m_coeff.x + c5 * m_coeff.y + b1 * m_coeff.z;

		float aty = -b6 * m_coeff.x + c4 * m_coeff.y + b2 * m_coeff.z;

		float atz = b7 * m_coeff.x - srx * m_coeff.y - b3 * m_coeff.z;

		float d2 = m_coeff.intensity;

		matA.at<float>(i, 0) = arx;
		matA.at<float>(i, 1) = ary;
		matA.at<float>(i, 2) = arz;
		matA.at<float>(i, 3) = atx;
		matA.at<float>(i, 4) = aty;
		matA.at<float>(i, 5) = atz;
		matB.at<float>(i, 0) = -0.05 * d2;

	}

	cv::transpose(matA, matAt);
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);


	if (iterCount == 0) {
		cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

		cv::eigen(matAtA, matE, matV);
		matV.copyTo(matV2);

		isDegenerate = false;
		float eignThre[6] = { 10, 10, 10, 10, 10, 10 };
		for (int i = 5; i >= 0; i--) {
			if (matE.at<float>(0, i) < eignThre[i]) {
				for (int j = 0; j < 6; j++) {
					matV2.at<float>(i, j) = 0;
				}
				isDegenerate = true;
			}
			else {
				break;
			}
		}
		matP = matV.inv() * matV2;
	}

	if (isDegenerate) {
		cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
		matX.copyTo(matX2);
		matX = matP * matX2;
	}

	transformCur[0] += matX.at<float>(0, 0);
	transformCur[1] += matX.at<float>(1, 0);
	transformCur[2] += matX.at<float>(2, 0);
	transformCur[3] += matX.at<float>(3, 0);
	transformCur[4] += matX.at<float>(4, 0);
	transformCur[5] += matX.at<float>(5, 0);


	for (int i = 0; i < 6; i++) {
		if (isnan(transformCur[i]))
			transformCur[i] = 0;
	}

	float deltaR = sqrt(
		pow(rad2deg(matX.at<float>(0, 0)), 2) +
		pow(rad2deg(matX.at<float>(1, 0)), 2) +
		pow(rad2deg(matX.at<float>(2, 0)), 2));
	float deltaT = sqrt(
		pow(matX.at<float>(3, 0) * 100, 2) +
		pow(matX.at<float>(4, 0) * 100, 2) +
		pow(matX.at<float>(5, 0) * 100, 2));


	if (deltaR < 0.1 && deltaT < 0.1) {
		return false;
	}
	return true;
}


inline bool CLidarOdometry::calculateTransformationSurf(int iterCount)
{
	int pointSelNum = m_laserCloudOri->points.size();

	// pointSelNum ��ʾ�ж��ٸ���ӦԼ��
	cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

	float srx = sin(transformCur[0]);
	float crx = cos(transformCur[0]);
	float sry = sin(transformCur[1]);
	float cry = cos(transformCur[1]);
	float srz = sin(transformCur[2]);
	float crz = cos(transformCur[2]);
	float tx = transformCur[3];
	float ty = transformCur[4];
	float tz = transformCur[5];

	float a1 = crx * sry*srz; float a2 = crx * crz*sry; float a3 = srx * sry; float a4 = tx * a1 - ty * a2 - tz * a3;
	float a5 = srx * srz; float a6 = crz * srx; float a7 = ty * a6 - tz * crx - tx * a5;
	float a8 = crx * cry*srz; float a9 = crx * cry*crz; float a10 = cry * srx; float a11 = tz * a10 + ty * a9 - tx * a8;

	float b1 = -crz * sry - cry * srx*srz; float b2 = cry * crz*srx - sry * srz;
	float b5 = cry * crz - srx * sry*srz; float b6 = cry * srz + crz * srx*sry;

	float c1 = -b6; float c2 = b5; float c3 = tx * b6 - ty * b5; float c4 = -crx * crz; float c5 = crx * srz; float c6 = ty * c5 + tx * -c4;
	float c7 = b2; float c8 = -b1; float c9 = tx * -b2 - ty * -b1;

	for (int i = 0; i < pointSelNum; i++) {

		m_pointOri = m_laserCloudOri->points[i];
		m_coeff = m_coeffSel->points[i];

		float arx = (-a1 * m_pointOri.x + a2 * m_pointOri.y + a3 * m_pointOri.z + a4) * m_coeff.x
			+ (a5*m_pointOri.x - a6 * m_pointOri.y + crx * m_pointOri.z + a7) * m_coeff.y
			+ (a8*m_pointOri.x - a9 * m_pointOri.y - a10 * m_pointOri.z + a11) * m_coeff.z;

		float arz = (c1*m_pointOri.x + c2 * m_pointOri.y + c3) * m_coeff.x
			+ (c4*m_pointOri.x - c5 * m_pointOri.y + c6) * m_coeff.y
			+ (c7*m_pointOri.x + c8 * m_pointOri.y + c9) * m_coeff.z;

		float aty = -b6 * m_coeff.x + c4 * m_coeff.y + b2 * m_coeff.z;

		float d2 = m_coeff.intensity;

		matA.at<float>(i, 0) = arx;
		matA.at<float>(i, 1) = arz;
		matA.at<float>(i, 2) = aty;
		matB.at<float>(i, 0) = -0.05*d2;
	}

	cv::transpose(matA, matAt);
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

	if (iterCount == 0) {
		cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

		cv::eigen(matAtA, matE, matV);
		matV.copyTo(matV2);

		isDegenerate = false;
		float eignThre[3] = { 10, 10, 10 };
		for (int i = 2; i >= 0; i--) {
			if (matE.at<float>(0, i) < eignThre[i]) {
				for (int j = 0; j < 3; j++) {
					matV2.at<float>(i, j) = 0;
				}
				isDegenerate = true;
			}
			else {
				break;
			}
		}
		matP = matV.inv() * matV2;
	}

	if (isDegenerate) {
		cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
		matX.copyTo(matX2);
		matX = matP * matX2;
	}

	transformCur[0] += matX.at<float>(0, 0);
	transformCur[2] += matX.at<float>(1, 0);
	transformCur[4] += matX.at<float>(2, 0);

	for (int i = 0; i < 6; i++) {
		if (isnan(transformCur[i]))
			transformCur[i] = 0;
	}

	float deltaR = sqrt(
		pow(rad2deg(matX.at<float>(0, 0)), 2) +
		pow(rad2deg(matX.at<float>(1, 0)), 2));
	float deltaT = sqrt(
		pow(matX.at<float>(2, 0) * 100, 2));

	if (deltaR < 0.1 && deltaT < 0.1) {
		return false;
	}
	return true;
}

inline bool CLidarOdometry::calculateTransformationCorner(int iterCount)
{
	int pointSelNum = m_laserCloudOri->points.size();

	cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

	float srx = sin(transformCur[0]);
	float crx = cos(transformCur[0]);
	float sry = sin(transformCur[1]);
	float cry = cos(transformCur[1]);
	float srz = sin(transformCur[2]);
	float crz = cos(transformCur[2]);
	float tx = transformCur[3];
	float ty = transformCur[4];
	float tz = transformCur[5];

	float b1 = -crz * sry - cry * srx*srz; float b2 = cry * crz*srx - sry * srz; float b3 = crx * cry; float b4 = tx * -b1 + ty * -b2 + tz * b3;
	float b5 = cry * crz - srx * sry*srz; float b6 = cry * srz + crz * srx*sry; float b7 = crx * sry; float b8 = tz * b7 - ty * b6 - tx * b5;

	float c5 = crx * srz;

	for (int i = 0; i < pointSelNum; i++) {

		m_pointOri = m_laserCloudOri->points[i];
		m_coeff = m_coeffSel->points[i];

		float ary = (b1*m_pointOri.x + b2 * m_pointOri.y - b3 * m_pointOri.z + b4) * m_coeff.x
			+ (b5*m_pointOri.x + b6 * m_pointOri.y - b7 * m_pointOri.z + b8) * m_coeff.z;

		float atx = -b5 * m_coeff.x + c5 * m_coeff.y + b1 * m_coeff.z;

		float atz = b7 * m_coeff.x - srx * m_coeff.y - b3 * m_coeff.z;

		float d2 = m_coeff.intensity;

		matA.at<float>(i, 0) = ary;
		matA.at<float>(i, 1) = atx;
		matA.at<float>(i, 2) = atz;
		matB.at<float>(i, 0) = -0.05*d2;
	}
	// ��С���˼���(QR�ֽⷨ)
	cv::transpose(matA, matAt);
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

	//����������Loam����Zhang J��<<On Degeneracy of Optimization-based State Estimation Problems>>
	//��ŷ�����ͨ��Jacobian��eigenvalue�ж��ĸ�������Լ������, �������Ǹ������ϵĵ���

	if (iterCount == 0) {
		cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

		cv::eigen(matAtA, matE, matV);	// ����������������E�����������ķ��Գ���V

		matV.copyTo(matV2);

		isDegenerate = false;			// �˻�����false
		float eignThre[3] = { 10, 10, 10 };
		for (int i = 2; i >= 0; i--) {
			if (matE.at<float>(0, i) < eignThre[i]) {
				for (int j = 0; j < 3; j++) {
					matV2.at<float>(i, j) = 0;
				}
				isDegenerate = true;// ���ڱ�10С������ֵ������˻�
			}
			else {
				break;
			}
		}
		matP = matV.inv() * matV2;
	}

	if (isDegenerate) {
		cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
		matX.copyTo(matX2);
		matX = matP * matX2;
	}

	transformCur[1] += matX.at<float>(0, 0);
	transformCur[3] += matX.at<float>(1, 0);
	transformCur[5] += matX.at<float>(2, 0);

	for (int i = 0; i < 6; i++) {
		if (isnan(transformCur[i]))
			transformCur[i] = 0;
	}

	//deltaR\deltaT�ж��Ƿ�������
	float deltaR = sqrt(
		pow(rad2deg(matX.at<float>(0, 0)), 2));
	float deltaT = sqrt(
		pow(matX.at<float>(1, 0) * 100, 2) +
		pow(matX.at<float>(2, 0) * 100, 2));

	if (deltaR < 0.1 && deltaT < 0.1) {
		return false;
	}
	return true;
}

inline void CLidarOdometry::integrateTransformation()
{
	// ����֡���Ƶ�����˶���ϵת������������ϵ��
	// ��Rk_k+1 ��Tk_k+1������ǰһ֡��ȫ��λ��Rk_w��Pk_w ת��ΪRk+1_w ��Pk+1_w
	// Ry(transformSum[1])*Rx(transformSum[0])*Rz(transformSum[2]) = Rc_w
	// tx = transformSum[0] ty = transformSum[1] tz = transformSum[2] ��ʾ������wϵ�¹۲�L{k+1}ϵ������
	float rx, ry, rz, tx, ty, tz;

	// ������ת�ǵ��ۼƱ仯��
	AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
		-transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);
	//-transformCur[0], -transformCur[1], -transformCur[2]ע�⸺���Һ���ľ���˳����RyRxRz���൱���Ƕ�Rk_k+1����
	//Rk_w * Rk+1_k = Rk+1_w

	float x1 = cos(rz) * (transformCur[3]) - sin(rz) * (transformCur[4]);
	float y1 = sin(rz) * (transformCur[3]) + cos(rz) * (transformCur[4]);
	float z1 = transformCur[5];

	float x2 = x1;
	float y2 = cos(rx) * y1 - sin(rx) * z1;
	float z2 = sin(rx) * y1 + cos(rx) * z1;

	tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
	ty = transformSum[4] - y2;
	tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

	transformSum[0] = rx;
	transformSum[1] = ry;
	transformSum[2] = rz;
	transformSum[3] = tx;
	transformSum[4] = ty;
	transformSum[5] = tz;

	m_TransformCur.posX = tx; m_TransformCur.posY = ty; m_TransformCur.posZ = tz;
	m_TransformCur.pitch = rx; m_TransformCur.yaw = ry; m_TransformCur.roll = rz;
	m_TransformSum.push_back(m_TransformCur);
}

//����ڵ�һ�����Ƽ�ԭ�㣬������ת��
inline void CLidarOdometry::AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, float & ox, float & oy, float & oz)
{
	float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
	ox = -asin(srx);

	float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz)
		+ sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
	float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy)
		- cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
	oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

	float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz)
		+ sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
	float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz)
		- cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
	oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}


#endif // !CLIDAR_ODOMETRY_H
