//
// Created by kukdh1 on 2015-07-12.
//

#ifndef POINT_CLOUD_JNI_EXAMPLE_IMAGE_FRAME_H
#define POINT_CLOUD_JNI_EXAMPLE_IMAGE_FRAME_H

#include <jni.h>
#include <mutex>

#include <tango_client_api.h>  // NOLINT
#include <tango-gl/util.h>
#include <tango-gl/camera.h>

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#define MAX_IMAGE_FRAME     10
#define IMAGE_WIDTH         1280
#define IMAGE_HEIGHT        720

namespace tango_point_cloud {
    class ImageFrame {
    public:
        ImageFrame();
        ~ImageFrame();

        pthread_mutex_t *imageMutex;

        void UpdateImageFrame(const TangoImageBuffer *buffer);

        void GetClosestImageFrame(double second, TangoImageBuffer **buffer);

    private:
        int index;
        TangoImageBuffer imageBuffer[MAX_IMAGE_FRAME];
        cv::Mat src, dst;
    };
}

#endif //POINT_CLOUD_JNI_EXAMPLE_IMAGE_FRAME_H
