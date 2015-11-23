//
// Created by kukdh1 on 2015-11-22.
//

#ifndef POINT_CLOUD_JNI_EXAMPLE_ASYNC_SOCKET_CLIENT_H
#define POINT_CLOUD_JNI_EXAMPLE_ASYNC_SOCKET_CLIENT_H

#include <jni.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include <tango_client_api.h>  // NOLINT
#include <tango-gl/util.h>

#define PORT 9000
#define SERVER_ADDR "1.233.237.248"

namespace tango_point_cloud {
    class NetworkClient {
        private:
            int sock;
            struct sockaddr_in serveraddr;

        public:
            int connectToServer();
            int sendToServer(unsigned int length, const unsigned char *buf);
            void closeSocket();
    };

    void SendPointCloudToServer(NetworkClient *ncClient, std::vector<float> &points, std::vector<uint8_t> &colors, glm::mat4 &transform);
}

#endif //POINT_CLOUD_JNI_EXAMPLE_ASYNC_SOCKET_CLIENT_H
