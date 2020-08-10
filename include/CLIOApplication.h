#ifndef LIODLL_CLIOAPPLICATION_H
#define LIODLL_CLIOAPPLICATION_H

#include "LIOBase.h"
#include "LIOCmnFunc.h"
#include "LIOSDC.h"
#include "CLIOOption.h"
#include "CLIOFFStream.h"
#include "CFeatureExtraction.h"
#include "CLidarOdometry.h"
#include <time.h>

using namespace std;

class CLIOAPP
{
public:
	CLIOAPP();
	~CLIOAPP();

	int RunLIOApp(string optFile);
	 
	int ClearLIOApp();

	CLIOOPTION m_LIOOpt;
	CPointCloudFFSTREAM m_PCFFStream;
	CFeatureExtraction m_FeatureExtract;
	CLidarOdometry m_LidarOdometry;
};


inline CLIOAPP::CLIOAPP()
{
	m_PCFFStream.m_LIOOpt = &(this->m_LIOOpt);

	m_FeatureExtract.m_LIOOpt = &(this->m_LIOOpt);
	m_FeatureExtract.m_PCFFStream = &(this->m_PCFFStream);

	m_LidarOdometry.m_LIOOpt = &(this->m_LIOOpt);
	m_LidarOdometry.m_PCFFStream = &(this->m_PCFFStream);
	m_LidarOdometry.m_FeatureExtraction = &(this->m_FeatureExtract);
}

inline CLIOAPP::~CLIOAPP()
{
}

inline int CLIOAPP::RunLIOApp(string optFile)
{

	// 1����ȡ�����ļ�
	m_LIOOpt.ReadOptFile(optFile);
	
	m_PCFFStream.Init();

	// 2�����������ļ�
	FILE *flog;
	string LogFile,OutPutPath;
	OutPutPath = m_LIOOpt.m_PrjDataPath + "//output";
	//CreateDirectory(OutPutPath.c_str(), NULL);

	LogFile = OutPutPath + "//log.txt";
	// fopen_s(&flog, LogFile.c_str(), "wt");
	flog = fopen(LogFile.c_str(), "wt");

	clock_t start1, start2, start3, end1, end2, end3;

	//2.5����ȡGNSS����λ��
	m_PCFFStream.ReadGNSSPrediction();


	// while (true)
	// {
	// 	gs_SystemCount++;

	// 	if (gs_SystemCount > m_PCFFStream.m_vPointCloudFileNames.size())
	// 	//if (gs_SystemCount > 100)
	// 	{
	// 		break;
	// 	}

	// 	cout << "Processing " << gs_SystemCount << "th point cloud" << endl;
	// 	m_PCFFStream.ReadOnePCFile(gs_SystemCount);
	// 	// ���Ʒָ�
	// 	start1 = clock();
	// 	m_FeatureExtract.RunCloudSegmentation(gs_SystemCount);
	// 	end1 = (clock() - start1);
	// 	cout << "RunCloudSegmentation time comsumption is " << end1 << endl;
	// 	if (gs_SystemCount > 1) 
	// 		fprintf(flog, "%8ld", end1);		

	// 	if (gs_SystemCount > 1)
	// 	{
	// 		// ǰ����̼�
	// 		start2 = clock();
	// 		m_LidarOdometry.RunLidarOdometry(gs_SystemCount);
	// 		end2 = (clock() - start2);
	// 		cout << "RunLidarOdometry time comsumption is " << end2 << endl;
	// 		fprintf(flog, "%8ld", end2);
	// 	}

	// 	//// ���������һ֡���ƺ� ������ͼ
	// 	//if (gs_SystemCount == m_PCFFStream.m_vPointCloudFileNames.size())
	// 	////if (gs_SystemCount == 100)
	// 	//	m_MapOptimization.PublishPointCloudMap();

	// 	//cout << "******************************************************" << endl;
	// }
	// fclose(flog);
	
	// // 3�����λ�˽��
	// FILE * fout;
	// Pose3D Pose;
	// string resultfilename;
	// resultfilename = OutPutPath + "//result.txt";
	// // fopen_s(&fout, resultfilename.c_str(), "wt");
	// fout = fopen(resultfilename.c_str(), "wt");

	// for (int i = 0; i < m_LidarOdometry.m_TransformSum.size(); i++)
	// {
	// 	Pose = m_LidarOdometry.m_TransformSum[i];
	// 	fprintf(fout, "%f\t%f\t%f\t%f\t%f\t%f\n", -Pose.posX, Pose.posY, Pose.posZ, Pose.roll, Pose.pitch, Pose.yaw);
	// }
	// fclose(fout);

	// 4�������ͼ���
	/*FILE *fmap;
	string mapfilename;
	mapfilename = OutPutPath + "//map.txt";
	fopen_s(&fmap, mapfilename.c_str(), "wt");

	PointType mapPoint;
	float intense = 1;
	for (int i = 0; i < m_MapOptimization.m_PointCloudMap.size(); i++)
	{
		for (int j = 0; j < m_MapOptimization.m_PointCloudMap[i]->size(); j++)
		{			
			mapPoint = m_MapOptimization.m_PointCloudMap[i]->points[j];
			
			fprintf(fmap, "%f\t\t%f\t\t%f\t\t%f\n", -mapPoint.x, mapPoint.y, mapPoint.z, intense);
		}		
	}
	fclose(fmap);*/

	return 0;
}

inline int CLIOAPP::ClearLIOApp()
{
	return 0;
}



#endif // !CLIOAPPLICATION
