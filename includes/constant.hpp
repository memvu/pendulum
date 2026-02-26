#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

constexpr unsigned int MAX_LENGTH_TRAIL = 3000;
constexpr unsigned int resolution = 30;
constexpr double PI = 3.14159265358979323846;
constexpr unsigned int HEIGHT = 900;
constexpr unsigned int WIDTH = 1200;
constexpr float ASPECT = static_cast<float>(WIDTH) / HEIGHT;
constexpr float BALL_0_Y = 0.5;
constexpr float GRAVITY = 9.80665;

constexpr float SCALE = 0.03;
constexpr float TRAIL_WIDTH = 0.006;
// of ball
constexpr float BALL_MASS = 1;

// of rod
constexpr float ROD_LENGTH = 0.42;
constexpr float ROD_WIDTH = SCALE / 5;

constexpr glm::mat4 view = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0.0f));
const glm::mat4 projection = glm::ortho(-1.0f * ASPECT, 1.0f * ASPECT, -1.0f, 1.0f, -1.0f, 1.0f);
