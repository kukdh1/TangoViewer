#include "PointCloud.h"

namespace kukdh1
{
	PointCloud::PointCloud()
	{
		ppPoints = NULL;
		uiPointCount = 0;
		timestamp = 0.0;
	}

	PointCloud::~PointCloud()
	{
		if (ppPoints)
			free(ppPoints);
	}

	void PointCloud::DrawOnGLWindow()
	{
		glPushMatrix();

		glMultMatrixf(glm::value_ptr(m4Transform));
		glMultMatrixf(glm::value_ptr(kOpengGL_T_Depth));

		glPointSize(3.0f);
		glBegin(GL_POINTS);

		for (unsigned int i = 0; i < uiPointCount; i++)
		{
			glColor4ub(ppPoints[i].c & 0xFF, (ppPoints[i].c & 0xFF00) >> 8, (ppPoints[i].c & 0xFF0000) >> 16, (ppPoints[i].c & 0xFF000000) >> 24);

			glVertex3f(ppPoints[i].x, ppPoints[i].y, ppPoints[i].z);
		}

		glEnd();

		glPopMatrix();
	}

	void PointCloud::AdjustTransformMatrix(glm::mat4 &m4AdjustMatrix)
	{
		m4Transform *= m4AdjustMatrix;
	}

	BOOL PointCloud::FromFile(WCHAR * pszFilePath)
	{
		HANDLE hFile;
		DWORD dwRead;

		hFile = CreateFile(pszFilePath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

		if (hFile != INVALID_HANDLE_VALUE)
		{
			float fMatrix[16];

			ReadFile(hFile, &uiPointCount, 4, &dwRead, NULL);

			if (uiPointCount)
			{
				ReadFile(hFile, fMatrix, 16 * 4, &dwRead, NULL);
				memcpy(glm::value_ptr(m4Transform), fMatrix, 16 * 4);
			//	m4Transform = glm::inverse(m4Transform);

				ppPoints = (Point *)calloc(uiPointCount, sizeof(Point));

				if (ppPoints)
					ReadFile(hFile, ppPoints, 16 * uiPointCount, &dwRead, NULL);
			}

			CloseHandle(hFile);

			return TRUE;
		}

		return FALSE;
	}
}