//
// Created by kukdh1 on 2015-07-17.
//

#ifndef POINT_CLOUD_JNI_EXAMPLE_POINT_CLOUD_BUFFER_H
#define POINT_CLOUD_JNI_EXAMPLE_POINT_CLOUD_BUFFER_H

#include <jni.h>
#include <mutex>
#include <vector>

#include <tango_client_api.h>  // NOLINT
#include <tango-gl/util.h>

//#define WRITE_IJ_DATA
#define WRITE_COLOR_DATA

typedef struct POINT_CLOUD_BUFFER {
    POINT_CLOUD_BUFFER(std::vector<float> &pts, std::vector<uint8_t> &cls, std::vector<uint16_t> &ij, glm::mat4 &tf, double time)
    {
        points = std::vector<float>(pts);
        colors = std::vector<uint8_t>(cls);
        ijs = std::vector<uint16_t>(ij);
        transform = glm::mat4(tf);
        timestamp = time;
    }
    std::vector<float> points;
    std::vector<uint8_t> colors;
    std::vector<uint16_t> ijs;
    glm::mat4 transform;
    double timestamp;
}POINT_CLOUD_BUFFER;

namespace tango_point_cloud {
    class PointCloudBuffer {
    public:
        void addPointCloud(std::vector<float> &oldpoints, std::vector<uint8_t> &oldcolors, std::vector<uint16_t> &oldijs, glm::mat4 &oldtransform, double timestamp);
        bool getPointCloud(uint32_t i, std::vector<float> &oldpoints, std::vector<uint8_t> &oldcolors, glm::mat4 &oldtransform);
        void resetPointCloud();
        void saveToFile();
    private:
        std::mutex mutex;
        std::vector<POINT_CLOUD_BUFFER> vBuffer;
        void *thread(void *data);
    };
}

#endif //POINT_CLOUD_JNI_EXAMPLE_POINT_CLOUD_BUFFER_H
