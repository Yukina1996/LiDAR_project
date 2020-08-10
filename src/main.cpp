#include "LIOBase.h"
#include "LIOSDC.h"
#include "LIOCmnFunc.h"
#include "CLIOApplication.h"
#include "ndt.h"

int main()
{
	CLIOAPP mylioapp;
	mylioapp.RunLIOApp("//home//slam//SGGSLAM//WYY//LiDAR_project//src//LIO_OPTION.txt");
	int count;

	string folder = mylioapp.m_LIOOpt.m_PrjDataPath;
	CNDTMatching ndt(folder);
	ndt.GetInitialGuess();
	ndt.Checkfilelist(folder, ndt.m_listPC);

	for (int i = startID; i <= endID; i++)
	{
		//int i = 10;
		auto start = clock();
		// 帧间ICP预测下一帧位姿
		cout << "---" << "Running Iterative Closest Point" << "---" << endl;	
		count = i + 1;
		mylioapp.m_PCFFStream.ReadOnePCFile(count);
		mylioapp.m_FeatureExtract.RunCloudSegmentation(count);

		if (count == startID + 1)
		{					
			mylioapp.m_LidarOdometry.transformSum[0] = mylioapp.m_PCFFStream.m_initialPose[startID].at(4);
			mylioapp.m_LidarOdometry.transformSum[1] = -mylioapp.m_PCFFStream.m_initialPose[startID].at(5);
			mylioapp.m_LidarOdometry.transformSum[2] = mylioapp.m_PCFFStream.m_initialPose[startID].at(3);
			mylioapp.m_LidarOdometry.transformSum[3] = -mylioapp.m_PCFFStream.m_initialPose[startID].at(0);
			mylioapp.m_LidarOdometry.transformSum[4] = mylioapp.m_PCFFStream.m_initialPose[startID].at(1);
			mylioapp.m_LidarOdometry.transformSum[5] = mylioapp.m_PCFFStream.m_initialPose[startID].at(2);
		}
		if (count > startID + 1)
		{
			mylioapp.m_LidarOdometry.RunLidarOdometry(count);
		}

		// 以ICP预测的初始位姿作为NDT匹配定位的初值
		ndt.m_initialGuess.XYZ[0] = -mylioapp.m_LidarOdometry.transformSum[3];
		ndt.m_initialGuess.XYZ[1] = mylioapp.m_LidarOdometry.transformSum[4];
		ndt.m_initialGuess.XYZ[2] = mylioapp.m_LidarOdometry.transformSum[5];
		ndt.m_initialGuess.RPY[0] = mylioapp.m_LidarOdometry.transformSum[2];
		ndt.m_initialGuess.RPY[1] = mylioapp.m_LidarOdometry.transformSum[0];
		ndt.m_initialGuess.RPY[2] = -mylioapp.m_LidarOdometry.transformSum[1];

		if (ndt.RunNDTMatching(i))
		{
			cout << "---" << "Running Normal Distribution Transform" << "---" << endl;
			auto end = (clock() - start)/1000000.0;
			cout << "Process current pointcloud takes" << end << " s" << endl;
			fprintf(flog, "Process current pointcloud takes: ");
			fprintf(flog, "%13f\n", end);

			mylioapp.m_LidarOdometry.transformSum[3] = -ndt.m_finalPOS.back().XYZ[0];
			mylioapp.m_LidarOdometry.transformSum[4] = ndt.m_finalPOS.back().XYZ[1];
			mylioapp.m_LidarOdometry.transformSum[5] = ndt.m_finalPOS.back().XYZ[2];
			mylioapp.m_LidarOdometry.transformSum[2] = ndt.m_finalPOS.back().RPY[0];
			mylioapp.m_LidarOdometry.transformSum[0] = ndt.m_finalPOS.back().RPY[1];
			mylioapp.m_LidarOdometry.transformSum[1] = -ndt.m_finalPOS.back().RPY[2];
		}		
		cout << "**********************************************************" << endl;
	}

	FILE* fp;
	//fopen_s(&fp, ndt.m_fileResult.c_str(), "wt");
	fp=fopen(ndt.m_fileResult.c_str(), "wt");

	for (int i = 0; i < ndt.m_finalPOS.size(); i++)
	{
		fprintf(fp, "%10lf %13lf %13lf %13lf %13lf %13lf %13lf %13lf %13lf %13lf %13lf %13lf\n",
			ndt.m_finalPOS[i].time,ndt.m_finalPOS[i].XYZ[0], ndt.m_finalPOS[i].XYZ[1], ndt.m_finalPOS[i].XYZ[2],
			ndt.m_finalPOS[i].deltaXYZ[0], ndt.m_finalPOS[i].deltaXYZ[1], ndt.m_finalPOS[i].deltaXYZ[2],
			ndt.m_finalPOS[i].RPY[0], ndt.m_finalPOS[i].RPY[1], ndt.m_finalPOS[i].RPY[2],
			ndt.m_finalPOS[i].NDTTime, ndt.m_finalPOS[i].NDTScore);
	}

	getchar();
	return 0;
}
