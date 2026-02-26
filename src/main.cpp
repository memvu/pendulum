#include "ball.hpp"
#include "constant.hpp"
#include "glm/ext/matrix_clip_space.hpp"
#include "glm/ext/matrix_transform.hpp"
#include "glm/ext/vector_float4.hpp"
#include "rod.hpp"
#include "shader.hpp"
#include "trail.hpp"
#include <GLFW/glfw3.h>
#include <cmath>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

// for solving triple pendulum equations
#include <Eigen/Dense>
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;

using std::cos;
using std::sin;

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void updatePosition(Ball *balls, float *theta);
Vec3 solve_with_fallbacks(const Mat3 &A, const Vec3 &b);
Vec6 dydt(const Vec6 &y);
Vec6 rk4(const Vec6 &y, float h);

int main() {
  GLFWwindow *window;

  if (!glfwInit())
    return -1;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window = glfwCreateWindow(WIDTH, HEIGHT, "May thang mien tay", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD\n";
    return -1;
  }

  glViewport(0, 0, WIDTH * 2, HEIGHT * 2);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  Ball balls[4];
  float theta[] = {PI / 2, PI, PI / 3};

  balls[0].updatePosition(glm::vec3(0, BALL_0_Y, 0));

  updatePosition(balls, theta);

  std::vector<Rod> rods;
  rods.emplace_back(balls[0], balls[1]);
  rods.emplace_back(balls[1], balls[2]);
  rods.emplace_back(balls[2], balls[3]);

  Vec6 y;
  y << theta[0], theta[1], theta[2], 0, 0, 0;

  Trail trail(balls[3]);

  float prev_t = glfwGetTime();
  // float dt = 0.001;

  while (!glfwWindowShouldClose(window)) {
    float curr_t = glfwGetTime();
    float dt = curr_t - prev_t;
    prev_t = curr_t;

    glClear(GL_COLOR_BUFFER_BIT);

    y = rk4(y, dt);

    theta[0] = y(0);
    theta[1] = y(1);
    theta[2] = y(2);
    updatePosition(balls, theta);

    for (int i = 0; i < 4; ++i) {
      balls[i].render();
    }

    for (int i = 0; i < 3; ++i) {
      rods[i].render();
    }

    // trail.updateTrail();
    // trail.render();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwTerminate();
  return 0;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}

void updatePosition(Ball *balls, float *theta) {
  balls[1].updatePosition(balls[0].getPosition() +
                          ROD_LENGTH *
                              glm::vec3(sin(theta[0]), -cos(theta[0]), 0));
  balls[2].updatePosition(balls[1].getPosition() +
                          ROD_LENGTH *
                              glm::vec3(sin(theta[1]), -cos(theta[1]), 0));
  balls[3].updatePosition(balls[2].getPosition() +
                          ROD_LENGTH *
                              glm::vec3(sin(theta[2]), -cos(theta[2]), 0));
}

Vec6 rk4(const Vec6 &y, float h) {
  Vec6 k1, k2, k3, k4;
  k1 = dydt(y);
  k2 = dydt(y + k1 * h * 0.5f);
  k3 = dydt(y + k2 * h * 0.5f);
  k4 = dydt(y + k3 * h);
  return y + (k1 + 2.0f * k2 + 2.0f * k3 + k4) * (h / 6);
}

static inline void mass_gravity_coriolis(const Vec3 &th, const Vec3 &w,
                                         double m, double l, double g, Mat3 &M,
                                         Vec3 &h, Vec3 &G) {
  const double t1 = th[0], t2 = th[1], t3 = th[2];
  const double w1 = w[0], w2 = w[1], w3 = w[2];

  const double c1 = std::cos(t1), s1 = std::sin(t1);
  const double c2 = std::cos(t2), s2 = std::sin(t2);
  const double c3 = std::cos(t3), s3 = std::sin(t3);

  // Jacobians for bob positions r_i(θ):
  // r1 = l[sin t1, -cos t1]
  // r2 = r1 + l[sin t2, -cos t2]
  // r3 = r2 + l[sin t3, -cos t3]

  // Each J_i is 2x3. Columns are ∂r_i/∂θ_j (j=1..3)
  Eigen::Matrix<double, 2, 3> J1, J2, J3;
  J1 << l * c1, 0, 0, l * s1, 0, 0;

  J2 << l * c1, l * c2, 0, l * s1, l * s2, 0;

  J3 << l * c1, l * c2, l * c3, l * s1, l * s2, l * s3;

  // Time derivatives of J_i multiplied by θdot:
  // d/dt [c] = -s * ω,  d/dt [s] =  c * ω.
  Eigen::Matrix<double, 2, 3> dJ1w, dJ2w, dJ3w;
  dJ1w << -l * s1 * w1, 0, 0, l * c1 * w1, 0, 0;

  dJ2w << -l * s1 * w1, -l * s2 * w2, 0, l * c1 * w1, l * c2 * w2, 0;

  dJ3w << -l * s1 * w1, -l * s2 * w2, -l * s3 * w3, l * c1 * w1, l * c2 * w2,
      l * c3 * w3;

  // Assemble mass matrix: M = m * Σ J_i^T J_i (SPD by construction)
  M = m * (J1.transpose() * J1 + J2.transpose() * J2 + J3.transpose() * J3);

  // Velocity coupling term h(θ,θ̇) = Σ m * J_i^T * (dJ_i/dt * θ̇)
  // (This corresponds to Coriolis/centrifugal contributions; note no θ̈ here.)
  Eigen::Vector2d z2(0.0, 0.0);
  h = m * (J1.transpose() * (dJ1w * Vec3(w1, w2, w3)) +
           J2.transpose() * (dJ2w * Vec3(w1, w2, w3)) +
           J3.transpose() * (dJ3w * Vec3(w1, w2, w3)));

  // Gravity generalized forces G(θ) = ∂V/∂θ with V = Σ m g y_i
  // y up is +, our r_i second component is y; for r = [x, y],
  // ∂V/∂θ = Σ m g * (∂y_i/∂θ) = Σ m g * (row_y of J_i) .
  // Our y components are: r1_y = l*s1, r2_y = l*s1 + l*s2, r3_y = l*s1 + l*s2 +
  // l*s3. So row_y(Ji) is exactly the second row of Ji.
  Eigen::RowVector3d Jy_sum =
      (J1.row(1) + J2.row(1) + J3.row(1)); // sum of y-rows
  G = m * g * Jy_sum.transpose();          // ∂V/∂θ
}
Vec6 dydt(const Vec6 &y) {
  const double m = 1.0;        // your masses (equal)
  const double l = ROD_LENGTH; // your common length
  const double g = GRAVITY;

  Vec3 th = y.head<3>();
  Vec3 w = y.tail<3>();

  Mat3 M;
  Vec3 h;
  Vec3 G;
  mass_gravity_coriolis(th, w, m, l, g, M, h, G);

  // Solve M * θ̈ + h + G = 0   ⇒   θ̈ = - M^{-1} (h + G)
  Vec3 rhs = -(h + G);

  // Robust solve (try SPD first, then fall back)
  Eigen::LLT<Mat3> llt(M);
  Vec3 dd;
  if (llt.info() == Eigen::Success)
    dd = llt.solve(rhs);
  else {
    Eigen::LDLT<Mat3> ldlt(M);
    if (ldlt.info() == Eigen::Success)
      dd = ldlt.solve(rhs);
    else
      dd = M.fullPivLu().solve(rhs);
  }

  Vec6 out;
  out << w, dd;
  return out;
}

Vec3 solve_with_fallbacks(const Mat3 &A, const Vec3 &b) {
  Eigen::LLT<Mat3> llt(A);
  if (llt.info() == Eigen::Success)
    return llt.solve(b);

  Eigen::LDLT<Mat3> ldlt(A);
  if (ldlt.info() == Eigen::Success)
    return ldlt.solve(b);

  return A.fullPivLu().solve(b);
}
