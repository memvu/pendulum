#pragma once

#include "ball.hpp"
#include "constant.hpp"
#include "shader.hpp"

struct Trail {
  Trail(Ball &toTrack);
  void updateTrail();
  void render();
private:
  Ball &tracked;
  // std::deque<glm::vec3> trail;
  float trail[MAX_LENGTH_TRAIL*2];

  void setupVAO();
  unsigned int VAO, VBO;

  static Shader shader;
};
