
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

#include <sstream>

#include "tango-point-cloud/point_cloud_drawable.h"

namespace {
const std::string kPointCloudVertexShader =
    "attribute vec4 vertex;\n"
            "attribute vec4 a_color;\n"
    "uniform mat4 mvp;\n"
    "varying vec4 v_color;\n"
    "void main() {\n"
    "  gl_PointSize = 5.0;\n"
    "  gl_Position = mvp*vertex;\n"
    "  v_color = a_color;\n"
    "}\n";
const std::string kPointCloudFragmentShader =
    "varying vec4 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = vec4(v_color);\n"
    "}\n";

const glm::mat4 kOpengGL_T_Depth =
    glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
              -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}  // namespace

namespace tango_point_cloud {

PointCloudDrawable::PointCloudDrawable() {
  LOGI("PointCloudDrawable constructor");
  shader_program_ = tango_gl::util::CreateProgram(
      kPointCloudVertexShader.c_str(), kPointCloudFragmentShader.c_str());

  mvp_handle_ = glGetUniformLocation(shader_program_, "mvp");
  vertices_handle_ = glGetAttribLocation(shader_program_, "vertex");
    color_handle_ = glGetAttribLocation(shader_program_, "a_color");
  glGenBuffers(1, &vertex_buffers_);
    glGenBuffers(1, &color_buffers_);
}

PointCloudDrawable::~PointCloudDrawable() { glDeleteProgram(shader_program_); }

void PointCloudDrawable::Render(glm::mat4 projection_mat, glm::mat4 view_mat,
                                glm::mat4 model_mat,
                                const std::vector<float>& vertices, const std::vector<uint8_t>& colors) {
  glUseProgram(shader_program_);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  mvp_handle_ = glGetUniformLocation(shader_program_, "mvp");

  // Calculate model view projection matrix.
  glm::mat4 mvp_mat = projection_mat * view_mat * model_mat * kOpengGL_T_Depth;
  glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(vertices_handle_);
  glVertexAttribPointer(vertices_handle_, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, color_buffers_);
    glBufferData(GL_ARRAY_BUFFER, colors.size(), colors.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(color_handle_);
    glVertexAttribPointer(color_handle_, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDrawArrays(GL_POINTS, 0, vertices.size());

  glUseProgram(0);
  tango_gl::util::CheckGlError("Pointcloud::Render");
}

}  // namespace tango_point_cloud
