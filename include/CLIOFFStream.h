#ifndef LIODLL_CLIOFFSTREAM_H
#define LIODLL_CLIOFFSTREAM_H

#include "LIOCmnFunc.h"
#include "LIOBase.h"
#include "CLIOOption.h"
//#include <io.h>

class CPointCloudFFSTREAM 
{
public:
	CPointCloudFFSTREAM();
	virtual ~CPointCloudFFSTREAM();

	CLIOOPTION* m_LIOOpt;

	bool Init();									
	int ReadOnePCFile(string pointcloud_filename);
	int ReadOnePCFile(const int SystemCount);
	int ReadOneSegmentedPCFile();

	int ReadGNSSPrediction();

public:
	vector<vector<float>> m_PointCloud;								
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_PCL_PointCloud;			
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SegmentedPointCloud;		
	vector<string> m_vPointCloudFileNames;

	vector<vector<float>> m_initialPose;			

private:
	//int m_SystemCount;
	string m_CurrentPCFile;
};


inline CPointCloudFFSTREAM::CPointCloudFFSTREAM()
{
	m_PCL_PointCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_SegmentedPointCloud.reset(new pcl::PointCloud < pcl::PointXYZI>());
}

inline CPointCloudFFSTREAM::~CPointCloudFFSTREAM()
{
}

inline bool CPointCloudFFSTREAM::Init()
{
	DIR* dir;	
	string filePath = m_LIOOpt->m_PCPath + "//";
	dir = opendir(filePath.c_str());
	struct dirent* ptr;
	std::vector<std::string> file;
	while((ptr = readdir(dir)) != NULL)
	{
		if(ptr->d_name[0] == '.') 
		{continue;}
		file.push_back(ptr->d_name);
	}
	closedir(dir);
	sort(file.begin(), file.end());
	m_vPointCloudFileNames = file;

	return true;
}

inline int CPointCloudFFSTREAM::ReadOnePCFile(string pointcloud_filename)
{
	int num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;
	float tmpx, tmpy, tmpz, tmpr;

	float Point[4];
	vector<float> vPoint;
	vector<vector<float>> pointCloud;

	FILE *instream;
	pointcloud_filename = pointcloud_filename.substr(0, pointcloud_filename.rfind("\n"));
	//fopen_s(&instream, pointcloud_filename.c_str(), "rb");
	instream = fopen(pointcloud_filename.c_str(), "rb");
	num = fread(data, sizeof(float), num, instream) / 4;
	
	for (int i = 0; i < num; i++)
	{
		vPoint.push_back(*px); vPoint.push_back(*py); vPoint.push_back(*pz); vPoint.push_back(*pr);

		m_PointCloud.push_back(vPoint);
		
		px += 4; py += 4; pz += 4; pr += 4;		
		vector<float>().swap(vPoint);		
	}

	Rawdata2PCL(m_PointCloud, m_PCL_PointCloud);

	fclose(instream);

	return 0;
}

inline int CPointCloudFFSTREAM::ReadOnePCFile(const int SystemCount)
{
	m_CurrentPCFile = m_LIOOpt->m_PCPath + "//" + m_vPointCloudFileNames[SystemCount - 1];
	
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
	m_CurrentPCFile = m_CurrentPCFile.substr(0, m_CurrentPCFile.rfind("\n"));
	//fopen_s(&instream, m_CurrentPCFile.c_str(), "rb");
	instream = fopen(m_CurrentPCFile.c_str(), "rb");

	while (!feof(instream))
	{
		if (fread(data, sizeof(float), 4, instream) == 4)
		{
			px = data + 0; py = data + 1; pz = data + 2; pr = data + 3;
			vPoint[0] = *px; vPoint[1] = *py; vPoint[2] = *pz; vPoint[3] = *pr;
			//if (*px >= -10)								
			{
				m_PointCloud.push_back(vPoint);
			}
		}
	}

	m_PCL_PointCloud->clear();
	Rawdata2PCL(m_PointCloud, m_PCL_PointCloud);
	
	vector<vector<float>>().swap(m_PointCloud);

	fclose(instream);
	free(data);

	return 0;
}


inline int CPointCloudFFSTREAM::ReadOneSegmentedPCFile()
{
	return 0;
}

inline int CPointCloudFFSTREAM::ReadGNSSPrediction()
{
	string fileGuess = m_LIOOpt->m_PrjDataPath + "//GNSSPrediction.txt";
	FILE* fp;
	// fopen_s(&fp, fileGuess.c_str(), "rt");
	fp = fopen(fileGuess.c_str(), "rt");
	
	while (!feof(fp))
	{
		vector<float> tmp; tmp.resize(7);

		fscanf(fp, "%f %f %f %f %f %f\n", &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]);

		this->m_initialPose.push_back(tmp);
	
		vector<float>().swap(tmp);
	}


	return 0;
}



#endif // !LIODLL_CLIOFFSTREAM_H

