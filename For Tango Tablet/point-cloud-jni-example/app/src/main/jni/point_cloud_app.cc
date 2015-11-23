/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tango-gl/conversions.h>

#include "tango-point-cloud/point_cloud_app.h"

namespace {
const int kVersionStringLength = 27;

// This function routes onXYZijAvailable callbacks to the application object for
// handling.
//
// @param context, context will be a pointer to a PointCloudApp
//        instance on which to call callbacks.
// @param pose, pose data to route to onPoseAvailable function.
void onPointCloudAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
  using namespace tango_point_cloud;
  PointCloudApp* app = static_cast<PointCloudApp*>(context);
  app->onPointCloudAvailable(xyz_ij);
}

// This function routes onPoseAvailable callbacks to the application object for
// handling.
//
// @param context, context will be a pointer to a PointCloudApp
//        instance on which to call callbacks.
// @param pose, pose data to route to onPoseAvailable function.
void onPoseAvailableRouter(void* context, const TangoPoseData* pose) {
  using namespace tango_point_cloud;
  PointCloudApp* app = static_cast<PointCloudApp*>(context);
  app->onPoseAvailable(pose);
}

void onFrameAvailableRouter(void* context, TangoCameraId id, const TangoImageBuffer *buffer)
{
    using namespace tango_point_cloud;
    PointCloudApp* app = static_cast<PointCloudApp*>(context);
    app->onFrameAvailable(id, buffer);
}

// This function routes onTangoEvent callbacks to the application object for
// handling.
//
// @param context, context will be a pointer to a PointCloudApp
//        instance on which to call callbacks.
// @param event, TangoEvent to route to onTangoEventAvailable function.
void onTangoEventAvailableRouter(void* context, const TangoEvent* event) {
  using namespace tango_point_cloud;
  PointCloudApp* app = static_cast<PointCloudApp*>(context);
  app->onTangoEventAvailable(event);
}
}  // namespace

namespace tango_point_cloud {
void PointCloudApp::onPointCloudAvailable(const TangoXYZij* xyz_ij) {
  std::lock_guard<std::mutex> lock(point_cloud_mutex_);
  point_cloud_data_.UpdatePointCloud(xyz_ij);
}

void PointCloudApp::onPoseAvailable(const TangoPoseData* pose) {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  pose_data_.UpdatePose(pose);
}

void PointCloudApp::onTangoEventAvailable(const TangoEvent* event) {
  std::lock_guard<std::mutex> lock(tango_event_mutex_);
  tango_event_data_.UpdateTangoEvent(event);
}

void PointCloudApp::onFrameAvailable(TangoCameraId id, const TangoImageBuffer *buffer)
{
    image_frame_.UpdateImageFrame(buffer);
}

PointCloudApp::PointCloudApp() {}

PointCloudApp::~PointCloudApp() {
  if (tango_config_ != nullptr) {
    TangoConfig_free(tango_config_);
  }
}

int PointCloudApp::TangoInitialize(JNIEnv* env, jobject caller_activity) {
  // The first thing we need to do for any Tango enabled application is to
  // initialize the service. We'll do that here, passing on the JNI environment
  // and jobject corresponding to the Android activity that is calling us.
  return TangoService_initialize(env, caller_activity);
}

int PointCloudApp::TangoSetupConfig(bool is_atuo_recovery) {
  // Here, we'll configure the service to run in the way we'd want. For this
  // application, we'll start from the default configuration
  // (TANGO_CONFIG_DEFAULT). This enables basic motion tracking capabilities.
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    LOGE("PointCloudApp: Failed to get default config form");
    return TANGO_ERROR;
  }

  // Set auto-recovery for motion tracking as requested by the user.
  int ret = TangoConfig_setBool(tango_config_, "config_enable_auto_recovery",
                                is_atuo_recovery);
  if (ret != TANGO_SUCCESS) {
    LOGE("PointCloudApp: config_enable_auto_recovery() failed with error"
         "code: %d", ret);
    return ret;
  }

  // Enable depth.
  ret = TangoConfig_setBool(tango_config_, "config_enable_depth", true);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "PointCloudApp: config_enable_depth() failed with error"
        "code: %d",
        ret);
    return ret;
  }

  if (TangoConfig_setBool(tango_config_, "config_enable_color_camera", true) !=
      TANGO_SUCCESS) {
    LOGE("config_enable_color_camera Failed");
    return false;
  }

  // Get TangoCore version string from service.
  ret = TangoConfig_getString(
      tango_config_, "tango_service_library_version",
      const_cast<char*>(tango_core_version_string_.c_str()),
      kVersionStringLength);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "PointCloudApp: get tango core version failed with error"
        "code: %d",
        ret);
    return ret;
  }

  if (TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &camera_params_) != TANGO_SUCCESS)
  {
      LOGE("TangoService_getCameraIntrinsics() Failed");
      return false;
  }

  return ret;
}

int PointCloudApp::TangoConnectCallbacks() {
  // Attach the OnXYZijAvailable callback.
  // The callback will be called after the service is connected.
  int ret = TangoService_connectOnXYZijAvailable(onPointCloudAvailableRouter);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "PointCloudApp: Failed to connect to point cloud callback with error"
        "code: %d",
        ret);
    return ret;
  }

  // Setting up the frame pair for the onPoseAvailable callback.
  TangoCoordinateFramePair pairs;
  pairs.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pairs.target = TANGO_COORDINATE_FRAME_DEVICE;

  // Attach the onPoseAvailable callback.
  // The callback will be called after the service is connected.
  ret = TangoService_connectOnPoseAvailable(1, &pairs, onPoseAvailableRouter);
  if (ret != TANGO_SUCCESS) {
    LOGE("PointCloudApp: Failed to connect to pose callback with error"
         "code: %d", ret);
    return ret;
  }

  // Attach the onEventAvailable callback.
  // The callback will be called after the service is connected.
  ret = TangoService_connectOnTangoEvent(onTangoEventAvailableRouter);
  if (ret != TANGO_SUCCESS) {
    LOGE("PointCloudApp: Failed to connect to event callback with error"
         "code: %d", ret);
    return ret;
  }

  if (TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, (void *)this, onFrameAvailableRouter) != TANGO_SUCCESS) {
    LOGE("TangoService_connectOnFrameAvailable(): Failed");
    return false;
  }

  return ret;
}

// Connect to the Tango Service, the service will start running:
// poses can be queried and callbacks will be called.
int PointCloudApp::TangoConnect() {
  TangoErrorType ret = TangoService_connect(this, tango_config_);
  if (ret != TANGO_SUCCESS) {
    LOGE("PointCloudApp: Failed to connect to the Tango service with"
         "error code: %d", ret);
    return ret;
  }
  ret = UpdateExtrinsics();
  if (ret != TANGO_SUCCESS) {
    LOGE("PointCloudApp: Failed to query sensor extrinsic with error code: %d",
         ret);
    return ret;
  }
  return ret;
}

void PointCloudApp::TangoDisconnect() {
  // When disconnecting from the Tango Service, it is important to make sure to
  // free your configuration object. Note that disconnecting from the service,
  // resets all configuration, and disconnects all callbacks. If an application
  // resumes after disconnecting, it must re-register configuration and
  // callbacks with the service.
  TangoConfig_free(tango_config_);
  tango_config_ = nullptr;
  TangoService_disconnect();
}

void PointCloudApp::TangoResetMotionTracking() {
  TangoService_resetMotionTracking();
}

void PointCloudApp::InitializeGLContent() {
  main_scene_.InitGLContent();

    if (ncClient.connectToServer() < 0)
        LOGE("Socket Failed");
}

void PointCloudApp::SetViewPort(int width, int height) {
  main_scene_.SetupViewPort(width, height);
}

void PointCloudApp::Render() {
  // Query the latest pose transformation and point cloud frame transformation.
  // Point cloud data comes in with a specific timestamp, in order to get the
  // closest pose for the point cloud, we will need to use the
  // TangoService_getPoseAtTime() to query pose at timestamp.
  glm::mat4 cur_pose_transformation;
  glm::mat4 point_cloud_transformation;

    std::vector<float> vertices_cpy;
    std::vector<uint16_t> ijs_cpy;
    std::vector<uint8_t> color_buffer;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    cur_pose_transformation = pose_data_.GetLatestPoseMatrix();
  }

  double point_cloud_timestamp;
    vertices_cpy.clear();
    color_buffer.clear();
    ijs_cpy.clear();

  // We make another copy for rendering and depth computation.
  {
    std::lock_guard<std::mutex> lock(point_cloud_mutex_);
    point_cloud_timestamp = point_cloud_data_.GetCurrentTimstamp();
    std::vector<float> vertices = point_cloud_data_.GetVerticeVector();
    vertices_cpy = std::vector<float>(vertices);
  }

  // Get the latest pose transformation in opengl frame and apply extrinsics to
  // it.
  cur_pose_transformation =
      pose_data_.GetExtrinsicsAppliedOpenGLWorldFrame(cur_pose_transformation);

  // Query pose based on point cloud frame's timestamp.
  point_cloud_transformation = GetPoseMatrixAtTimestamp(point_cloud_timestamp);
  // Get the point cloud transformation in opengl frame and apply extrinsics to
  // it.
  point_cloud_transformation = pose_data_.GetExtrinsicsAppliedOpenGLWorldFrame(
          point_cloud_transformation);


  // Compute the average depth value.
  float average_depth_ = 0.0f;
  size_t iteration_size = vertices_cpy.size();
  size_t vertices_count = 0;
  for (size_t i = 2; i < iteration_size; i += 3) {
    average_depth_ += vertices_cpy[i];
    vertices_count++;
  }
  if (vertices_count) {
    average_depth_ /= vertices_count;
  }

    float x, y;
    int nx, ny;
    TangoImageBuffer *buffer;
    glm::mat4 transform;
    glm::vec4 src, dst;

    color_buffer.reserve(vertices_count * 4);
    ijs_cpy.reserve(vertices_count * 2);

    pthread_mutex_lock(image_frame_.imageMutex);
    image_frame_.GetClosestImageFrame(point_cloud_timestamp, &buffer);

    for (size_t i = 0; i < iteration_size; i += 3)
    {
        depthPointToImageAxis(vertices_cpy[i + 0], vertices_cpy[i + 1], vertices_cpy[i + 2], &x, &y);

        nx = (int)(x + 0.5);
        ny = (int)(y + 0.5);

        if (nx < 0 || nx >= buffer->width || ny < 0 || ny >= buffer->height) {
            color_buffer.push_back(0);
            color_buffer.push_back(0);
            color_buffer.push_back(0);
            color_buffer.push_back(0);
            ijs_cpy.push_back(0xFFFF);
            ijs_cpy.push_back(0xFFFF);
        }
        else
        {
            color_buffer.push_back(*(buffer->data + (buffer->width * ny + nx) * 4 + 0)); //R
            color_buffer.push_back(*(buffer->data + (buffer->width * ny + nx) * 4 + 1)); //G
            color_buffer.push_back(*(buffer->data + (buffer->width * ny + nx) * 4 + 2)); //B
            color_buffer.push_back(*(buffer->data + (buffer->width * ny + nx) * 4 + 3)); //A
            ijs_cpy.push_back((uint16_t)nx);
            ijs_cpy.push_back((uint16_t)ny);
        }

        vertices_count++;
    }

    pthread_mutex_unlock(image_frame_.imageMutex);

  {
    std::lock_guard<std::mutex> lock(point_cloud_mutex_);
    point_cloud_data_.SetAverageDepth(average_depth_);
  }

  main_scene_.Render(cur_pose_transformation, point_cloud_transformation, vertices_cpy, color_buffer, ijs_cpy, point_cloud_timestamp);

    if(isIMUStable() && SignificantChangeInTransform(point_cloud_transformation)) {
        SendPointCloudToServer(&ncClient, vertices_cpy, color_buffer, point_cloud_transformation);
        oldTransform = glm::mat4(point_cloud_transformation);
    }
}

    bool PointCloudApp::SignificantChangeInTransform(glm::mat4 &now)
    {
        glm::mat4 temp = now - oldTransform;
        float mean;
        float *data;

        mean = 0;
        data = glm::value_ptr(temp);
        for (unsigned int i = 0; i < 16; i++)
            mean += data[i] * data[i];

        if (mean > 0.01)
            return true;
        else
            return false;
    }

void PointCloudApp::depthPointToImageAxis(float dx, float dy, float dz, float *ix, float *iy)
{
    float x, y;
    float r2, r4, r6;
    float temp;

    x = dx / dz;
    y = dy / dz;
    r2 = x * x + y * y;
    r4 = r2 * r2;
    r6 = r2 * r4;
    temp = 1.0 + camera_params_.distortion[0] * r2 + camera_params_.distortion[1] * r4 + camera_params_.distortion[2] * r6;
    x *= temp;
    y *= temp;

    *ix = camera_params_.fx * x + camera_params_.cx;
    *iy = camera_params_.fy * y + camera_params_.cy;
}

void PointCloudApp::FreeGLContent() { main_scene_.FreeGLContent(); }

std::string PointCloudApp::GetPoseString() {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return pose_data_.GetPoseDebugString();
}

    bool PointCloudApp::isIMUStable() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return pose_data_.isIMUStable();
    }

std::string PointCloudApp::GetEventString() {
  std::lock_guard<std::mutex> lock(tango_event_mutex_);
  return tango_event_data_.GetTangoEventString().c_str();
}

std::string PointCloudApp::GetVersionString() {
  return tango_core_version_string_.c_str();
}

int PointCloudApp::GetPointCloudVerticesCount() {
  std::lock_guard<std::mutex> lock(point_cloud_mutex_);
  return point_cloud_data_.GetPointCloudVerticesCount();
}

float PointCloudApp::GetAverageZ() {
  std::lock_guard<std::mutex> lock(point_cloud_mutex_);
  return point_cloud_data_.GetAverageDepth();
}

float PointCloudApp::GetDepthFrameDeltaTime() {
  std::lock_guard<std::mutex> lock(point_cloud_mutex_);
  return point_cloud_data_.GetDepthFrameDeltaTime();
}

void PointCloudApp::SetCameraType(
    tango_gl::GestureCamera::CameraType camera_type) {
        main_scene_.SetCameraType(camera_type);
}

void PointCloudApp::SetCaptureType(int index){
    main_scene_.SetCaptureType(index);
}

void PointCloudApp::OnTouchEvent(int touch_count,
                                      tango_gl::GestureCamera::TouchEvent event,
                                      float x0, float y0, float x1, float y1) {
  main_scene_.OnTouchEvent(touch_count, event, x0, y0, x1, y1);
}

glm::mat4 PointCloudApp::GetPoseMatrixAtTimestamp(double timstamp) {
  TangoPoseData pose_start_service_T_device;
  TangoCoordinateFramePair frame_pair;
  frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
  TangoErrorType status = TangoService_getPoseAtTime(
      timstamp, frame_pair, &pose_start_service_T_device);
  if (status != TANGO_SUCCESS) {
    LOGE(
        "PoseData: Failed to get transform between the Start of service and "
        "device frames at timstamp %lf",
        timstamp);
  }
  if (pose_start_service_T_device.status_code != TANGO_POSE_VALID) {
    return glm::mat4(1.0f);
  }
  return pose_data_.GetMatrixFromPose(pose_start_service_T_device);
}

TangoErrorType PointCloudApp::UpdateExtrinsics() {
  TangoErrorType ret;
  TangoPoseData pose_data;
  TangoCoordinateFramePair frame_pair;

  // TangoService_getPoseAtTime function is used for query device extrinsics
  // as well. We use timestamp 0.0 and the target frame pair to get the
  // extrinsics from the sensors.
  //
  // Get device with respect to imu transformation matrix.
  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
  ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "PointCloudApp: Failed to get transform between the IMU frame and "
        "device frames");
    return ret;
  }
  pose_data_.SetImuTDevice(pose_data_.GetMatrixFromPose(pose_data));

  // Get color camera with respect to imu transformation matrix.
  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "PointCloudApp: Failed to get transform between the color camera frame "
        "and device frames");
    return ret;
  }
  pose_data_.SetImuTColorCamera(pose_data_.GetMatrixFromPose(pose_data));
  return ret;
}

}  // namespace tango_point_cloud
