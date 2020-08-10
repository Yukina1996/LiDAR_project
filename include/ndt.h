#pragma once
#include <stdio.h>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define BOOST_TYPEOF_EMULATION
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>
#include <time.h>

#define startID 3001
#define endID   4540

#ifndef _CLOCK_T_DEFINED
typedef long clock_t;
#define _CLOCK_T_DEFINED
#endif

using namespace std;

typedef pcl::PointXYZI PointsType;
typedef pcl::PointCloud<PointsType> PointsCloudType;

FILE* flog;


struct POS {
	double time;
	double XYZ[3];
	double deltaXYZ[3];
	double RPY[3];
	double RLw[9];
	double NDTScore;
	double NDTTime;
};

class CNDTMatching
{
public:
	CNDTMatching();
	~CNDTMatching();

	CNDTMatching(string folder);
	bool RunNDTMatching(int count);
	bool GetInitialGuess();
	bool LoadCurrentPointCloud(const int count);
	bool LoadSurroundingPointCloudMap(const int count);
	int Txt2PointCloud(string filename, vector<vector<float>> &pointCloud);
	int Bin2PointCloud(string filename, vector<vector<float>> &pointCloud);
	void Checkfilelist(string filePath, vector<string> &filenames);

	bool Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout);
	static bool MyComparison(const vector<float> &v1, const vector<float> &v2)
	{
		return v1.back()<v2.back();//系统默认a<b时返回真，于是从小到大排
	}

	int m_grid_size;	//地图格网尺度
	string m_filefolder;
	string m_fileResult;
	string m_fileLogout;
	vector<string> m_listPC;
	vector<POS> m_initialPOS;
	POS m_lastKeyPOS;	//存储上一次更新地图时的关键帧位姿
	vector<POS> m_finalPOS;
	POS m_diffPOS;		//存储上一次算得的相邻帧的位移
	POS m_initialGuess;	//存储ICP计算过来的位置估计

	PointsCloudType::Ptr m_surroundingMapCloud;
	PointsCloudType::Ptr m_currentCloud;
	PointsCloudType::Ptr m_surroundingMapCloudDS;
	PointsCloudType::Ptr m_currentCloudDS;
	PointsCloudType::Ptr m_outputCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_target_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_source_cloud;

	pcl::visualization::PCLVisualizer::Ptr viewer_final;
};

inline CNDTMatching::CNDTMatching()
{
}

inline CNDTMatching::~CNDTMatching()
{
}

inline CNDTMatching::CNDTMatching(string folder)
{
	this->m_grid_size = 30;

	this->m_filefolder = folder;
	this->m_fileResult = folder + "//ndt_output//ndt_result.txt";
	this->m_fileLogout = folder + "//ndt_output//logout.txt";

	//fopen_s(&flog, this->m_fileLogout.c_str(), "wt");
	flog = fopen(this->m_fileLogout.c_str(), "wt");

	m_surroundingMapCloud.reset(new PointsCloudType());
	m_surroundingMapCloudDS.reset(new PointsCloudType());
	m_currentCloud.reset(new PointsCloudType());
	m_currentCloudDS.reset(new PointsCloudType());
	m_outputCloud.reset(new PointsCloudType());
	m_target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	m_source_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

	memset(&m_lastKeyPOS, 0, sizeof(POS));
	memset(&m_diffPOS, 0, sizeof(POS));
}

inline bool CNDTMatching::RunNDTMatching(int count)
{
	m_target_cloud->clear();
	m_source_cloud->clear();

	//加载当前帧点云
	m_currentCloud->clear();
	m_currentCloudDS->clear();
	m_outputCloud->clear();

	if (LoadCurrentPointCloud(count) == false) 
	{
		return false;
	}

	// 初始化当前帧位姿
	float init_posX ,init_posY ,init_posZ ,init_roll,init_pitch,init_yaw;
	//if(count>startID && (count-startID)%100 != 0)
	// if(count>startID)
	{
		// init_posX = this->m_finalPOS.back().XYZ[0] + m_diffPOS.XYZ[0];
		// init_posY = this->m_finalPOS.back().XYZ[1] + m_diffPOS.XYZ[1];
		// init_posZ = this->m_finalPOS.back().XYZ[2] + m_diffPOS.XYZ[2];

		init_posX = this->m_initialGuess.XYZ[0];
		init_posY = this->m_initialGuess.XYZ[1];
		init_posZ = this->m_initialGuess.XYZ[2];
		init_roll = this->m_initialGuess.RPY[0];
		init_pitch = this->m_initialGuess.RPY[1];
		init_yaw = this->m_initialGuess.RPY[2];

		// init_posX = this->m_finalPOS.back().XYZ[0];
		// init_posY = this->m_finalPOS.back().XYZ[1];
		// init_posZ = this->m_finalPOS.back().XYZ[2];
		// init_roll = this->m_finalPOS.back().RPY[0];
		// init_pitch = this->m_finalPOS.back().RPY[1];
		// init_yaw = this->m_finalPOS.back().RPY[2];
		
		cout << "initial position: " << init_posX <<", " << init_posY <<", "<< init_posZ <<endl;
		cout << "initial altitude: " << init_roll << ", "<< init_pitch <<", "<< init_yaw <<endl;
	}
	//else if(count==startID || (count-startID)%100 == 0)
	// else if(count==startID)
	// {
	// 	init_posX = this->m_initialPOS[count].XYZ[0];
	// 	init_posY = this->m_initialPOS[count].XYZ[1];
	// 	init_posZ = this->m_initialPOS[count].XYZ[2];
	// 	init_roll = this->m_initialPOS[count].RPY[0];
	// 	init_pitch = this->m_initialPOS[count].RPY[1];
	// 	init_yaw = this->m_initialPOS[count].RPY[2];

	// 	cout << "initial position: " << init_posX << ", "<< init_posY <<", "<< init_posZ <<endl;
	// 	cout << "initial altitude: " << init_roll << ", "<< init_pitch <<", "<< init_yaw <<endl;
	// }

	Eigen::Vector3f eulerAngle(init_yaw,init_pitch,init_roll);
	Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitZ()));
	Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitX()));
	Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitY()));

	Eigen::Translation3f init_translation(init_posX, init_posY, init_posZ);
	Eigen::Matrix4f init_guess = (init_translation*yawAngle*pitchAngle*rollAngle).matrix();

	// 根据初始化的位置提取附近地图网格中的点云数据
	float dist = sqrt((init_posX - m_lastKeyPOS.XYZ[0])*(init_posX - m_lastKeyPOS.XYZ[0]) +
		(init_posZ - m_lastKeyPOS.XYZ[2])*(init_posZ - m_lastKeyPOS.XYZ[2]));
	if ( dist > 15 || m_surroundingMapCloudDS->size() == 0)
	{
		m_surroundingMapCloud->clear();
		m_surroundingMapCloudDS->clear();
		LoadSurroundingPointCloudMap(count);
		m_lastKeyPOS.XYZ[0] = init_posX;		
		m_lastKeyPOS.XYZ[2] = init_posZ;

		pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter1;
		approximate_voxel_filter1.setLeafSize(0.2, 0.2, 0.2);
		approximate_voxel_filter1.setInputCloud(m_surroundingMapCloud);
		approximate_voxel_filter1.filter(*m_surroundingMapCloudDS);
		// cout << "Filtered cloud contains " << m_surroundingMapCloudDS->size()
		// 	<< " data points from " << "map" << std::endl;
		m_surroundingMapCloud->clear();
	}

	// 对子地图二次滤波
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr KdtreeSurrounding_Cloud;
	KdtreeSurrounding_Cloud.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
	std::vector<int> vPointSearchInd;
	std::vector<float> vPointSearchSqDis;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(*m_surroundingMapCloudDS, *temp_cloud);

	pcl::PointXYZ CurrentRobotPosPoint;
	CurrentRobotPosPoint.x = init_posX; CurrentRobotPosPoint.y = init_posY; CurrentRobotPosPoint.z = init_posZ;
	double SurroundingSearchRadius = 30;
	KdtreeSurrounding_Cloud->setInputCloud(temp_cloud);
	KdtreeSurrounding_Cloud->radiusSearch(CurrentRobotPosPoint, (double)SurroundingSearchRadius, vPointSearchInd, vPointSearchSqDis, 0);
	for (int i = 0; i < vPointSearchInd.size(); ++i)
		m_target_cloud->points.push_back(temp_cloud->points[vPointSearchInd[i]]);
	cout << "Filtered cloud contains " << m_target_cloud->size()
	<< " data points from " << "map" << std::endl;

	// 对当前帧点云降采样
	pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter2;
	approximate_voxel_filter2.setLeafSize(0.1, 0.1, 0.1);
	approximate_voxel_filter2.setInputCloud(m_currentCloud);
	approximate_voxel_filter2.filter(*m_currentCloudDS);
	cout << "Filtered cloud contains " << m_currentCloudDS->size()
		<< " data points from " << m_listPC[count] << std::endl;

	//copyPointCloud(*m_surroundingMapCloudDS, *m_target_cloud);
	copyPointCloud(*m_currentCloudDS, *m_source_cloud);

	//---------------------------------NDT-OMP-------------------------------------

	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

	// std::vector<int> num_threads = {1, omp_get_max_threads()};
	// std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
	// 	{"KDTREE", pclomp::KDTREE},
	// 	{"DIRECT7", pclomp::DIRECT7},
	// 	{"DIRECT1", pclomp::DIRECT1}
	// };

	pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
	ndt_omp->setResolution(2.0);
	ndt_omp->setNumThreads(omp_get_max_threads());
	ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
	ndt_omp->setStepSize(0.2);
	std::cout << "--- pclomp::NDT (" << "pclomp::DIRECT7" << ", " << omp_get_max_threads() << " threads) ---" << std::endl;
	
	ndt_omp->setInputTarget(m_target_cloud);
	ndt_omp->setInputSource(m_source_cloud);

	auto start1 = clock();
	//for (int i = 0; i < 10; i++)
	{
		ndt_omp->align(*aligned,init_guess);
	}

	// 整理NDT输出结果
	cout << "Normal Distributions Transform has converged: " << ndt_omp->hasConverged() << endl;
	cout <<  "iterations: " << ndt_omp->getFinalNumIteration() <<endl;

	auto end1 =  (clock()-start1)/1000000.0;
	std::cout << "times: " << end1 << "[sec]" << std::endl;
	std::cout << "fitness: " << ndt_omp->getFitnessScore() << std::endl << std::endl;

	Eigen::Matrix4f transformation_matrix = ndt_omp->getFinalTransformation();
    Eigen::Vector3f p = transformation_matrix.block<3, 1>(0, 3);
    Eigen::Matrix3f rx = transformation_matrix.block<3, 3>(0, 0);
	Eigen::Vector3f ea = rx.eulerAngles(1,0,2);		//ZXY  RyRxRz  (x,y,z)

	POS temp;
	temp.time = count;
	temp.XYZ[0] = transformation_matrix(0, 3); temp.XYZ[1] = transformation_matrix(1, 3); temp.XYZ[2] = transformation_matrix(2, 3);
	cout<<"Transformation: " << temp.XYZ[0] << " , " << temp.XYZ[1] << " , " << temp.XYZ[2] << endl;
	temp.deltaXYZ[0] = temp.XYZ[0] - this->m_initialPOS[count].XYZ[0];	
	temp.deltaXYZ[1] = temp.XYZ[1] - this->m_initialPOS[count].XYZ[1];
	temp.deltaXYZ[2] = temp.XYZ[2] - this->m_initialPOS[count].XYZ[2];
	temp.RPY[0] = ea[2]; temp.RPY[1] = ea[1]; temp.RPY[2] = ea[0];//输出顺序是ry rx rz,对应物理含义是yaw,pitch,roll
	cout<<"Rotation: " << temp.RPY[0] << " , " << temp.RPY[1] << " , " << temp.RPY[2] << endl;
	temp.NDTScore = ndt_omp->getFitnessScore();//???
	//temp.NDTScore = ndt.getFitnessScore();
	temp.NDTTime = end1;

	// 更新相邻帧之间的位姿
	if (count > startID)
	{
		m_diffPOS.time = count;
		m_diffPOS.XYZ[0] = temp.XYZ[0] - m_finalPOS.back().XYZ[0];
		m_diffPOS.XYZ[1] = temp.XYZ[1] - m_finalPOS.back().XYZ[1];
		m_diffPOS.XYZ[2] = temp.XYZ[2] - m_finalPOS.back().XYZ[2];
	}

	// 更新当前帧的位姿
	this->m_finalPOS.push_back(temp);
	cout << "deltaX= " << temp.deltaXYZ[0] << endl;
	cout << "deltaY= " << temp.deltaXYZ[1] << endl;
	cout << "deltaZ= " << temp.deltaXYZ[2] << endl;

	

	//visulization
	if (count >= 10)
	// {
	// 	pcl::visualization::PCLVisualizer vis("vis");
	// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(m_target_cloud, 255.0, 0.0, 0.0);
	// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(m_source_cloud, 0.0, 255.0, 0.0);
	// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned, 0.0, 255.0, 0.0);
	// 	vis.addPointCloud(m_target_cloud, target_handler, "target");
	// 	////vis.addPointCloud(m_source_cloud, source_handler, "source");
	// 	vis.addPointCloud(aligned, aligned_handler, "aligned");
	// 	vis.spin();
	// }

	return true;
}

inline bool CNDTMatching::GetInitialGuess()
{
	string fileGuess = this->m_filefolder + "//GNSSPrediction.txt";
	FILE* fp;
	/*fopen_s(&fp, fileGuess.c_str(), "rt");*/
	fp = fopen(fileGuess.c_str(), "rt");

	POS temp;
	while (!feof(fp))
	{
		memset(&temp, 0, sizeof(temp));
		fscanf(fp, "%lf %lf %lf %lf %lf %lf\n", &temp.XYZ[0], &temp.XYZ[1], &temp.XYZ[2],
			&temp.RPY[0], &temp.RPY[1], &temp.RPY[2]);
		this->m_initialPOS.push_back(temp);
	}

	return true;
}

inline bool CNDTMatching::LoadCurrentPointCloud(const int count)
{
	string currentfilename = this->m_filefolder + "//velodyne//" + this->m_listPC[count];

	cout << "reading file " << this->m_listPC[count] << endl;

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
	vector<vector<float>> v_PointCloud;
	//v_PointCloud.resize();

	FILE *instream = NULL;

	if (!(instream = fopen(currentfilename.c_str(), "rb")))
	{
		cout << currentfilename << endl;
		printf("Error to open .bin\n");
		getchar();
		return false;
	}

	while (!feof(instream))
	{
		if (fread(data, sizeof(float), 4, instream) == 4)
		{
			px = data + 0; py = data + 1; pz = data + 2; pr = data + 3;
			vPoint[0] = *px; vPoint[1] = *py; vPoint[2] = *pz; vPoint[3] = *pr;
			float dist=sqrt(vPoint[0]*vPoint[0]+vPoint[1]*vPoint[1]+vPoint[2]*vPoint[2]);
			vPoint[3] = dist;			
			v_PointCloud.push_back(vPoint);			
		}
	}

	// 剔除距离最大的1０％的点对
	sort(v_PointCloud.begin(),v_PointCloud.end(),MyComparison);
	for (int i = 0; i < 0.1 * v_PointCloud.size(); i++)
	{
		v_PointCloud.pop_back();
	}
	cout << "Distance maximum: " << v_PointCloud.back().at(3) << endl;

	// 转为PCL点云格式
	Rawdata2PCL(v_PointCloud, this->m_currentCloud);

	// 将点云坐标和地图坐标系对齐
	PointType points;
	for (int i = 0; i < this->m_currentCloud->points.size(); i++)
	{			
		points.x = -this->m_currentCloud->points[i].y;
		points.y = this->m_currentCloud->points[i].z;
		points.z = this->m_currentCloud->points[i].x;
		this->m_currentCloud->points[i] = points;
	}

	vector<vector<float>>().swap(v_PointCloud);///< 清空大vector
	vector<float>().swap(vPoint);
	fclose(instream);
	free(data);

	return true;
}

inline bool CNDTMatching::LoadSurroundingPointCloudMap(const int count)
{
	vector<string> v_gridmap_filenames; 
	string tempfilename;

	// 计算当前位置在哪个地图网格
	float init_posX = this->m_initialPOS[count].XYZ[0];
	float init_posZ = this->m_initialPOS[count].XYZ[2];
	int grid_x = m_grid_size * static_cast<int>(floor(init_posX / m_grid_size));
	int grid_y = m_grid_size * static_cast<int>(floor(init_posZ / m_grid_size));
	
	//tempfilename = this->m_filefolder + "//gridmap//" + std::to_string(m_grid_size) + "_" +
	//	std::to_string(grid_x) + "_" +
	//	std::to_string(grid_y) + ".txt";
	tempfilename = this->m_filefolder + "//gridmap//" + std::to_string(m_grid_size) + "_" +
		std::to_string(grid_x) + "_" +
		std::to_string(grid_y) + ".bin";
	v_gridmap_filenames.push_back(tempfilename);

	// 计算周边的8个相邻网格
	vector<pair<int, int> >VP;
	VP.push_back(make_pair<int, int>(grid_x - m_grid_size, grid_y + m_grid_size));
	VP.push_back(make_pair<int, int>(grid_x - m_grid_size, grid_y + 0));
	VP.push_back(make_pair<int, int>(grid_x - m_grid_size, grid_y - m_grid_size));
	VP.push_back(make_pair<int, int>(grid_x + 0, grid_y + m_grid_size));
	VP.push_back(make_pair<int, int>(grid_x + 0, grid_y - m_grid_size));
	VP.push_back(make_pair<int, int>(grid_x + m_grid_size, grid_y + m_grid_size));
	VP.push_back(make_pair<int, int>(grid_x + m_grid_size, grid_y + 0));
	VP.push_back(make_pair<int, int>(grid_x + m_grid_size, grid_y - m_grid_size));

	for (vector<pair<int, int> > ::iterator iter = VP.begin(); iter != VP.end(); iter++)
	{
		//tempfilename = this->m_filefolder + "//gridmap//" + std::to_string(m_grid_size) + "_" +
		//	std::to_string(iter->first) + "_" +
		//	std::to_string(iter->second) + ".txt";
		tempfilename = this->m_filefolder + "//gridmap//" + std::to_string(m_grid_size) + "_" +
			std::to_string(iter->first) + "_" +
			std::to_string(iter->second) + ".bin";
		v_gridmap_filenames.push_back(tempfilename);
	}

	// 确定地图网格文件是否存在，如果有点云，则放入附近点云中
	vector<vector<float>> pointCloud;
	vector<vector<float>> pointCloudAll;
	for (int i = 0; i < v_gridmap_filenames.size(); i++)
	{	
		cout << "Load " << v_gridmap_filenames[i] << endl;
		if (Bin2PointCloud(v_gridmap_filenames[i],pointCloud) == 1)
		{
			//加入到周围的地图点云中
			pointCloudAll.insert(pointCloudAll.end(), pointCloud.begin(), pointCloud.end());
		}
		pointCloud.clear();
	}

	Rawdata2PCL(pointCloudAll, this->m_surroundingMapCloud);

	v_gridmap_filenames.clear();
	
	return true;
}

inline int CNDTMatching::Txt2PointCloud(string filename, vector<vector<float>> &pointCloud)
{
	FILE* fmap;
	fmap = fopen(filename.c_str(), "r");
	if (fmap != NULL)
	//if (!fopen_s(&fmap, filename.c_str(), "r"))	//fopen_s如果成功打开文件返回0
	{
		vector<float> vPoint;

		int num = 0;
		while (!feof(fmap))
		{
			double x = 0;
			double y = 0;
			double z = 0;
			double intensity = 0;
			//fscanf_s(fmap, "%lf %lf %lf %lf\n", &x, &y, &z, &intensity);
			fscanf(fmap, "%lf %lf %lf %lf\n", &x, &y, &z, &intensity);
			vPoint.push_back(x); vPoint.push_back(y); vPoint.push_back(z); vPoint.push_back(intensity);
			pointCloud.push_back(vPoint);
			vector<float>().swap(vPoint);
			num++;
		}

		return 1;
	}
	return 0;		

}

inline int CNDTMatching::Bin2PointCloud(string filename, vector<vector<float>> &pointCloud)
{

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

	FILE *fmap;
	fmap = fopen(filename.c_str(), "rb");
	if (fmap != NULL)
	//if (!fopen_s(&fmap, filename.c_str(), "rb"))
	{
		while (!feof(fmap))
		{
			if (fread(data, sizeof(float), 4, fmap) == 4)
			{
				px = data + 0; py = data + 1; pz = data + 2; pr = data + 3;
				vPoint[0] = *px; vPoint[1] = *py; vPoint[2] = *pz; vPoint[3] = *pr;
				pointCloud.push_back(vPoint);
			}
		}
		return 1;
	}
	return 0;
}

inline void CNDTMatching::Checkfilelist(string fPath, vector<string>& filenames)
{
	DIR* dir;	
	string filePath = fPath + "//velodyne//";
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
	filenames = file;

	// string filePath = fPath + "//velodyne//";

	// intptr_t   hFile = 0;

	// struct _finddata_t fileinfo;
	// string p;
	// if ((hFile = _findfirst(p.assign(filePath.c_str()).append("\\*.bin").c_str(), &fileinfo)) != -1)
	// {
	// 	do
	// 	{
	// 		filenames.push_back(fileinfo.name);
	// 	} while (_findnext(hFile, &fileinfo) == 0);
	// }
	// _findclose(hFile);
}

inline bool CNDTMatching::Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout)
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

