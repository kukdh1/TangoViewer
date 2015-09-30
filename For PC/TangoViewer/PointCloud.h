#pragma once

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <Windows.h>
#include <gl/GL.h>
#include "glm/mat4x4.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_inverse.hpp"

const glm::mat4 kOpengGL_T_Depth =
glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

namespace kukdh1
{
    /* Point struct
     *  It contains 3D coordinate (XYZ) and its color (ARGB)
     */
    struct Point
    {
        float x;
        float y;
        float z;
        COLORREF c;
    };
    
    /* PointCloud class
     *  It contains PointCloud at such time
     */
    class PointCloud
    {
        private:
            unsigned int uiPointCount;
            glm::mat4 m4Transform;
            Point *ppPoints;
        
            double timestamp;
        
        public:
            PointCloud();
            ~PointCloud();
        
            void DrawOnGLWindow();
            void AdjustTransformMatrix(glm::mat4 &m4AdjustMatrix);
        
            BOOL FromFile(WCHAR *pszFilePath);
    };
    
}

#endif
