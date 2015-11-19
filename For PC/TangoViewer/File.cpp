#include "File.h"

namespace kukdh1
{
	BOOL FileIO::ReadFromFile(WCHAR *pszFilePath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud, float *pfTimestamp, pcl::PointXYZRGB *ppclOrigin)
	{
		HANDLE hFile;
		DWORD dwRead;
		LPVOID ppPoint;
		glm::mat4 m4Temp;
		unsigned int uiPointCount;
		unsigned int uiBlockSize;

		hFile = CreateFile(pszFilePath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

		if (hFile != INVALID_HANDLE_VALUE)
		{
			float fMatrix[16];
			char cTag[4];

			//Check File Header
			ReadFile(hFile, cTag, 4, &dwRead, NULL);

			if (strncmp("PCD2", cTag, 4) == 0)
			{
				BOOL bMulOpenGLToDepth;
				BOOL bNoij;
				BOOL bNoColor;
				unsigned int uiFlag;

				//Read Flag
				ReadFile(hFile, &uiFlag, 4, &dwRead, NULL);

				bMulOpenGLToDepth = !(uiFlag & FLAG_NO_MUL);
				bNoij = !(uiFlag & FLAG_IJ_DATA);
				bNoColor = !(uiFlag & FLAG_COLOR_DATA);

				//Read Timestamp
				ReadFile(hFile, pfTimestamp, 4, &dwRead, NULL);

				//Read Point Cloud
				ReadFile(hFile, &uiPointCount, 4, &dwRead, NULL);

				if (uiPointCount)
				{
					//Read Matrix
					ReadFile(hFile, fMatrix, 16 * 4, &dwRead, NULL);
					memcpy(glm::value_ptr(m4Temp), fMatrix, 16 * 4);

					if (bMulOpenGLToDepth)
						m4Temp *= kOpengGL_T_Depth;

					//Allocate Memory
					uiBlockSize = 20;

					if (bNoColor)
						uiBlockSize -= 4;
					if (bNoij)
						uiBlockSize -= 4;

					ppPoint = calloc(uiPointCount, uiBlockSize);

					if (ppPoint)
					{
						ReadFile(hFile, ppPoint, uiBlockSize * uiPointCount, &dwRead, NULL);

						pclPointCloud->clear();
						pclPointCloud->width = uiPointCount;
						pclPointCloud->height = 1;
						pclPointCloud->resize(uiPointCount);

						//Set Read Point Cloud
						concurrency::parallel_for(size_t(0), uiPointCount, [&](size_t i)
						{
							glm::vec4 v4Temp;
							Pointij *ppNow;

							ppNow = (Pointij *)((BYTE *)ppPoint + i * uiBlockSize);
							v4Temp.w = 1;

							memcpy(glm::value_ptr(v4Temp), ppNow, 12);
							v4Temp = m4Temp * v4Temp;

							pclPointCloud->at(i).x = v4Temp.x;
							pclPointCloud->at(i).y = v4Temp.y;
							pclPointCloud->at(i).z = v4Temp.z;

							if (bNoColor)
								pclPointCloud->at(i).rgba = 0xFF000000;
							else
								pclPointCloud->at(i).rgba = ppNow->c;
						});

						free(ppPoint);

						//Set Origin
						if (ppclOrigin)
						TransformPoint(*ppclOrigin, m4Temp);
					}
				}

				CloseHandle(hFile);

				return TRUE;
			}
		}

		return FALSE;
	}

	BOOL FileIO::WriteToFile(WCHAR *pszFilePath, unsigned int uiFlag, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud)
	{
		HANDLE hFile;
		Point *ppPointij;
		DWORD dwWrite;
		glm::mat4 m4Transform;
		float fTimestamp;
		unsigned int uiPointCount;

		hFile = CreateFile(pszFilePath, GENERIC_WRITE, NULL, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

		if (hFile != INVALID_HANDLE_VALUE)
		{
			//Write Header
			WriteFile(hFile, "PCD2", 4, &dwWrite, NULL);

			//Write Flag
			WriteFile(hFile, &uiFlag, 4, &dwWrite, NULL);

			//Write Timestamp
			fTimestamp = 0;
			WriteFile(hFile, &fTimestamp, 4, &dwWrite, NULL);

			//Write Point Count
			uiPointCount = pclPointCloud->width;

			WriteFile(hFile, &uiPointCount, sizeof(unsigned int), &dwWrite, NULL);

			WriteFile(hFile, glm::value_ptr(m4Transform), 16 * 4, &dwWrite, NULL);

			ppPointij = (Point *)calloc(uiPointCount, sizeof(Point));

			if (ppPointij)
			{
				concurrency::parallel_for(size_t(0), uiPointCount, [&](size_t i)
				{
					Point *ppNow;

					ppNow = ppPointij + i;

					ppNow->x = pclPointCloud->at(i).x;
					ppNow->y = pclPointCloud->at(i).y;
					ppNow->z = pclPointCloud->at(i).z;
					ppNow->c = pclPointCloud->at(i).rgba;
				});

				WriteFile(hFile, ppPointij, sizeof(Point) * uiPointCount, &dwWrite, NULL);

				free(ppPointij);
			}

			CloseHandle(hFile);

			return TRUE;
		}

		return FALSE;
	}
}