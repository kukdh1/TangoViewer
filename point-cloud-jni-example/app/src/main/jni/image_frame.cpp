//
// Created by kukdh1 on 2015-07-12.
//

#include <tango-point-cloud/image_frame.h>

namespace tango_point_cloud {
    ImageFrame::ImageFrame()
    {
        index = 0;
        memset(imageBuffer, 0, sizeof(TangoImageBuffer) * MAX_IMAGE_FRAME);
        src.create(IMAGE_HEIGHT * 3 / 2, IMAGE_WIDTH, CV_8UC1);
        dst.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);
    }

    ImageFrame::~ImageFrame()
    {
        for (int i = 0; i < MAX_IMAGE_FRAME; i++)
            if (imageBuffer[i].data)
                delete[] imageBuffer[i].data;
        src.release();
        dst.release();
    }

    void ImageFrame::UpdateImageFrame(const TangoImageBuffer *buffer)
    {
        int lastIndex;

        int width = buffer->width;
        int height = buffer->height;

        if (index == 0)
            lastIndex = MAX_IMAGE_FRAME - 1;
        else
            lastIndex = index - 1;

        if (imageBuffer[lastIndex].timestamp != buffer->timestamp)
        {
            pthread_mutex_lock(imageMutex);

            imageBuffer[index].width = width;
            imageBuffer[index].height = height;
            imageBuffer[index].stride = buffer->stride;
            imageBuffer[index].timestamp = buffer->timestamp;

            if (imageBuffer[index].data == nullptr)
                imageBuffer[index].data = new uint8_t[width * height * 4];

            memcpy(src.data, buffer->data, width * height * 3 / 2);
            cv::cvtColor(src, dst, CV_YUV2RGBA_NV21);
            memcpy(imageBuffer[index].data, dst.data, width * height * 4);

            pthread_mutex_unlock(imageMutex);

            index++;

            if (index == MAX_IMAGE_FRAME)
                index = 0;
        }
    }

    void ImageFrame::GetClosestImageFrame(double second, TangoImageBuffer **buffer)
    {
        int indices[MAX_IMAGE_FRAME];
        int lastindex, count = 0;
        TangoImageBuffer *ret = nullptr;

        if (index == 0)
            lastindex = MAX_IMAGE_FRAME - 1;
        else
            lastindex = index - 1;

        for (int i = lastindex; i >= 0; i--)
            indices[count++] = i;
        for (int i = MAX_IMAGE_FRAME - 1; i > lastindex; i--)
            indices[count++] = i;

        for (int i = 0; i < MAX_IMAGE_FRAME; i++)
        {
            count = indices[i];

            if (second >= imageBuffer[count].timestamp)
                ret = &imageBuffer[count];
        }

        if (ret == nullptr)
            ret = &imageBuffer[MAX_IMAGE_FRAME - 1];

        *buffer = ret;
    }
}
