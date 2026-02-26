#include "trail.hpp"
#include "ball.hpp"
#include "constant.hpp"
#include <iterator>

Shader Trail::shader;

Trail::Trail(Ball &toTrack)
    : tracked(toTrack) {
   updateTrail();

   shader.makeShader("../src/shader/trail.vert", "../src/shader/trail.frag");
   shader.use();
   shader.setMat4("view", view);
   shader.setMat4("projection", projection);
}

void Trail::render() {
   shader.use();
}

void Trail::updateTrail() {
   setupVAO();
}

void Trail::setupVAO() {
}
