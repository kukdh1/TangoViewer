//
// Created by kukdh1 on 2015-07-17.
//

#include <tango-point-cloud/point_cloud_buffer.h>

void * filethread(void *data)
{
    POINT_CLOUD_BUFFER *buf = (POINT_CLOUD_BUFFER *)data;
    char filename[256];
    uint8_t buffer[1024];
    float fTimestamp;

    sprintf(filename, "/storage/emulated/0/TangoData/PointCloud_%.3lf.pcdx", buf->timestamp);

    FILE *out = fopen(filename, "w+");

    uint32_t num;
    float *fpt = buf->points.data();
    uint8_t *cpt = buf->colors.data();
    uint16_t *ijpt = buf->ijs.data();
    uint32_t flag;

    fTimestamp = buf->timestamp;

    if (out) {
        //Write Header
        fwrite("PCD2", sizeof(char), 4, out);

        //Make Flag
        flag = 0x00000000;
#ifdef WRITE_IJ_DATA
        flag |= 0x00000004;
#endif
#ifdef WRITE_COLOR_DATA
        flag |= 0x00000008;
#endif
        //Write Flag
        fwrite(&flag, sizeof(uint32_t), 1, out);

        //Write Timestamp
        fwrite(&fTimestamp, sizeof(float), 1, out);

        //Write the number of points
        num = buf->colors.size() / 4;
        fwrite(&num, sizeof(uint32_t), 1, out);

        //Write Transform Matrix
        fwrite(glm::value_ptr(buf->transform), sizeof(float), 16, out);

        //Write Points and colors [X/Y/Z/RGBA/J/I]
        for (uint32_t i = 0; i < num; i++)
        {
            fwrite(fpt + 3 * i, sizeof(float), 3, out);
#ifdef WRITE_COLOR_DATA
            fwrite(cpt + 4 * i, sizeof(uint8_t), 4, out);
#endif
#ifdef WRITE_IJ_DATA
            fwrite(ijpt + 2 * i, sizeof(uint16_t), 2, out);
#endif
        }

        fclose(out);
    }

    delete buf;
}

namespace tango_point_cloud {
    void PointCloudBuffer::addPointCloud(std::vector < float> &oldpoints, std::vector < uint8_t > &oldcolors, std::vector<uint16_t> &oldijs, glm::mat4 & oldtransform, double timestamp)
    {
        std::lock_guard<std::mutex> guard(mutex);

        vBuffer.push_back(POINT_CLOUD_BUFFER(oldpoints, oldcolors, oldijs, oldtransform, timestamp));
    }

    bool PointCloudBuffer::getPointCloud(uint32_t i, std::vector <float> &oldpoints, std::vector <uint8_t> &oldcolors, glm::mat4 &oldtransform)
    {
        bool bReturn = false;

        std::lock_guard<std::mutex> guard(mutex);

        oldpoints.clear();
        oldcolors.clear();

        if (i < vBuffer.size())
        {
            oldpoints = std::vector<float>(vBuffer.at(i).points);
            oldcolors = std::vector<uint8_t>(vBuffer.at(i).colors);
            oldtransform = glm::mat4(vBuffer.at(i).transform);

            bReturn = true;
        }

        return bReturn;
    }

    void PointCloudBuffer::resetPointCloud()
    {
        std::lock_guard<std::mutex> guard(mutex);

        vBuffer.clear();
    }

    void PointCloudBuffer::saveToFile()
    {
        POINT_CLOUD_BUFFER *param;
        pthread_t tid;

        std::lock_guard<std::mutex> guard(mutex);

        for (uint32_t i = 0; i < vBuffer.size(); i++)
        {
            param = new POINT_CLOUD_BUFFER(vBuffer.at(i).points, vBuffer.at(i).colors, vBuffer.at(i).ijs, vBuffer.at(i).transform, vBuffer.at(i).timestamp);

            pthread_create(&tid, nullptr, filethread, (void *)param);
        }
    }
}