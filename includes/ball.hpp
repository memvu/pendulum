#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "constant.hpp"
#include "shader.hpp"
#include <cmath>

struct Ball {
   Ball();
   void render(float customScale = SCALE);
   void updatePosition(const glm::vec3);
   const glm::vec3 getPosition() const;

private:
   void setupVAO();
   unsigned int VAO, VBO;
   float vertices[resolution * 2 + 4];

   glm::vec3 position = glm::vec3(0.0f);

   static Shader shader;
};
