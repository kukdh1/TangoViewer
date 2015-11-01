#pragma once

#ifndef _CONSTANT_H_
#define _CONSTANT_H_

//Constants for Log
#define MAX_LOG_LINE		1000
#define LOG_CLASS_NAME		L"Log Window"
#define TEXT_BUFFER_SIZE	4096

//Constants for PointCloudMerge
#define ANGLE_THRESHOLD_SAME_PLANE				0.087266f	//5degree
#define ANGLE_THRESHOLD_DIFFRENT_PLANE			0.523599f	//30degree

#define DISTANCE_THRESHOLD_SAME_PLANE			0.05f		//5cm
#define DISTANCE_THRESHOLD_DEFFERENT_PLANE		0.5f		//50cm

//Constants for PointCloud
#define TANGO_IMAGE_WIDTH		1280
#define TANGO_IMAGE_HEIGHT		720

#define SAC_SEG_MAX_ITERATION			1000
#define SAC_SEG_DISTANCE_THRESHOLD		0.05
#define SAC_SEG_ANGLE_THRESHOLD			0.052360
#define SAC_SEG_RATIO					0.1

#define SOR_MEAN_K				50
#define SOR_STDDEV_MUL_THRES	1.0

#define PI		3.141592f
#define PI_2	1.570796f

//Constants for GLWindow
#define GL_GRID_SIZE	50
#define GL_ROT_RATIO	0.01f
#define GL_MOVE_RATIO	0.01f
#define GL_ZOOM_RATIO	0.01f

//Constants for Main
#define WINDOW_WIDTH	1400
#define WINDOW_HEIGHT	1000
#define LIST_WIDTH		400
#define LIST_HEIGHT		WINDOW_HEIGHT
#define GL_WIDTH		1000
#define GL_HEIGHT		WINDOW_HEIGHT
#define LOG_WIDTH		300

//Constants for LUM
#define CORR_MAX_DISTANCE		2.5f
#define CENTROID_MAX_DISTANCE	5.0f
#define LUM_MAX_ITERATION		100

//Child ID Definition
#define ID_LISTBOX		10
#define ID_LOG_EDIT		20

//Message Definition
#define WM_REGISTER_THIS		WM_APP
#define	WM_MOUSEWHEEL_PASS		WM_APP + 0x01
#define WM_LOG_SHOW				WM_APP + 0x02

#endif