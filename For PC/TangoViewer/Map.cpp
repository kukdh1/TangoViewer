#include "Map.h"

namespace kukdh1
{
	Map::Map()
	{
		pclPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}

	Map::~Map()
	{
		pclPointCloud->clear();
	}

	void * Map::operator new(size_t i)
	{
		return _aligned_malloc(i, 16);
	}

	void Map::operator delete(void *ptr)
	{
		_aligned_free(ptr);
	}

	void Map::AddPointCloud(PointCloud *input)
	{
		input->SaveTransformMatrix();

		*pclPointCloud += *input->GetPointCloudData();
	}

	void Map::DownSampling()
	{
		pcl::VoxelGrid<pcl::PointXYZRGB> pclVG;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclOutput(new pcl::PointCloud<pcl::PointXYZRGB>);

		pclVG.setInputCloud(pclPointCloud);
		pclVG.setLeafSize(0.01f, 0.01f, 0.01f);
		pclVG.filter(*pclOutput);

		pclPointCloud = pclOutput;
	}

	BOOL Map::FromFile(WCHAR *pszFilePath)
	{
		FileIO fiFile;

		return fiFile.ReadFromFile(pszFilePath, pclPointCloud);
	}

	void Map::ToFile(WCHAR *pszFilePath)
	{
		FileIO fiFile;

		fiFile.WriteToFile(pszFilePath, FLAG_MAP_FILE | FLAG_NO_MUL | FLAG_COLOR_DATA, pclPointCloud);		//Flag : Map | No Mul | No ij
	}
}