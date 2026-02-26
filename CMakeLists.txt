cmake_minimum_required(VERSION 3.10)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(pendulum)
# cmake_policy(SET CMP0072 NEW)

set(CMAKE_CXX_STANDARD 20)

list(APPEND CMAKE_PREFIX_PATH cmake/glfw3)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

add_library(glad STATIC src/glad.c)
target_include_directories(glad PUBLIC includes)

find_package(glm CONFIG REQUIRED)

add_executable(pendulum src/main.cpp src/glad.c src/shader.cpp src/ball.cpp src/rod.cpp src/trail.cpp)

target_link_libraries(pendulum glm::glm glfw OpenGL::GL glad)
