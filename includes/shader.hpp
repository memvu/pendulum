#pragma once

#include <GLFW/glfw3.h>
#include "glm/gtc/type_ptr.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

class Shader {
private:
   unsigned int shaderID = 0;
   bool maken = false;

   void CreateShaderProgram(unsigned int &vertex, unsigned int &fragment);
   void CreateShader(const char *source, unsigned int &ID, std::string shaderType);

public:
   Shader() = default;
   Shader(const char *vertexPath, const char *fragmentPath);
   void makeShader(const char *vertexPath, const char *fragmentPath);
   void use();
   void setBool(const char *var_name, const bool value);
   void setFloat(const char *var_name, const float value);
   void setInt(const char *var_name, const int value);
   void setMat4(const char *var_name, const glm::mat4 &matrix);
};
