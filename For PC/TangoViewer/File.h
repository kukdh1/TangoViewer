#pragma once

#ifndef _FILE_H_
#define _FILE_H_

#define NOMINMAX

#include <Windows.h>

#include "PointCloud.h"
#include "Constant.h"

#define FLAG_MAP_FILE	0x00000001
#define FLAG_NO_MUL		0x00000002
#define FLAG_IJ_DATA	0x00000004
#define FLAG_COLOR_DATA	0x00000008

namespace kukdh1
{
	class FileIO
	{
		public:
			BOOL ReadFromFile(WCHAR *pszFilePath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud, float *pfTimestamp = NULL, pcl::PointXYZRGB *ppclOrigin = NULL);
			BOOL WriteToFile(WCHAR *pszFilePath, unsigned int uiFlag, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud);
	};
}

#endif