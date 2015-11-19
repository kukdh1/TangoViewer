#pragma once

#ifndef _MAP_H_
#define _MAP_H_

#define NOMINMAX

#include <Windows.h>

#include <pcl/filters/voxel_grid.h>

#include "Constant.h"
#include "PointCloud.h"
#include "File.h"

namespace kukdh1
{
	class Map
	{
		private:
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud;

		public:
			Map();
			~Map();

			static void * operator new(size_t);
			static void operator delete(void *);

			void AddPointCloud(PointCloud *input);

			void DownSampling();

			BOOL FromFile(WCHAR *pszFilePath);
			void ToFile(WCHAR *pszFilePath);
	};
}

#endif