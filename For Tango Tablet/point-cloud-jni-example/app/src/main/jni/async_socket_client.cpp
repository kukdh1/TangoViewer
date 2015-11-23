//
// Created by kukdh1 on 2015-11-22.
//

#include <tango-point-cloud/async_socket_client.h>

namespace tango_point_cloud {
    int NetworkClient::connectToServer()
    {
        sock = socket(PF_INET, SOCK_STREAM, 0);

        if (sock < 0)
        {
            LOGE("Socket Creation Error");

            return 0;
        }

        memset(&serveraddr, 0, sizeof(struct sockaddr_in));

        serveraddr.sin_family = AF_INET;
        serveraddr.sin_port = htons(PORT);
        serveraddr.sin_addr.s_addr = inet_addr(SERVER_ADDR);

        return connect(sock, (struct sockaddr *)&serveraddr, sizeof(serveraddr));
    }

    int NetworkClient::sendToServer(unsigned int length, const unsigned char *buf)
    {
        return send(sock, buf, length, 0);
    }

    void NetworkClient::closeSocket()
    {
    //    close(sock);
    }

    struct wrapper
    {
        NetworkClient *ncClient;
        std::vector<float> pPoints;
        std::vector<uint8_t> pColors;
        glm::mat4 transform;
    };

    void *NetworkThread(void *arg)
    {
        wrapper *pData = (wrapper *)arg;
        unsigned int length;
        unsigned int bytelength;
        unsigned char *pBuffer;

        bytelength = 8 + 16 * 4 + pData->pColors.size() * 4;

        pBuffer = (unsigned char *)calloc(bytelength, sizeof(unsigned char));

        length = pData->pColors.size() / 4;

        memcpy(pBuffer, "PCD2", 4);
        memcpy(pBuffer + 4, &length, 4);
        memcpy(pBuffer + 8, glm::value_ptr(pData->transform), 16 * 4);

        unsigned char *pPointBuf;

        pPointBuf = pBuffer + 72;

        for (unsigned int i = 0; i < length; i++)
        {
            memcpy(pPointBuf + i * 20, &pData->pPoints.at(i * 3), 4);
            memcpy(pPointBuf + i * 20 + 4, &pData->pPoints.at(i * 3 + 1), 4);
            memcpy(pPointBuf + i * 20 + 8, &pData->pPoints.at(i * 3 + 2), 4);
            pPointBuf[i * 20 + 12] = pData->pColors.at(i * 4);
            pPointBuf[i * 20 + 13] = pData->pColors.at(i * 4 + 1);
            pPointBuf[i * 20 + 14] = pData->pColors.at(i * 4 + 2);
            pPointBuf[i * 20 + 15] = pData->pColors.at(i * 4 + 3);
        }

        if (pData->ncClient->sendToServer(bytelength, pBuffer) < 0)
        {
            pData->ncClient->closeSocket();
            pData->ncClient->connectToServer();
            pData->ncClient->sendToServer(bytelength, pBuffer);
        }

        free(pBuffer);
        free(pData);
    }

    void SendPointCloudToServer(NetworkClient *ncClient, std::vector<float> &points, std::vector<uint8_t> &colors, glm::mat4 &transform)
    {
        wrapper *pData;
        pthread_t tid;

        pData = (wrapper *)calloc(1, sizeof(wrapper));

        pData->ncClient = ncClient;
        pData->pPoints = std::vector<float>(points);
        pData->pColors = std::vector<uint8_t>(colors);
        pData->transform = glm::mat4(transform);

        LOGI("CALLED");

    //    pthread_create(&tid, nullptr, NetworkThread, (void *)pData);
    }
}