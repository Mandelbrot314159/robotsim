#include "camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

namespace robotsim {

void OrbitCamera::orbit(float dx, float dy) {
    yaw   -= dx * 0.005f;
    pitch -= dy * 0.005f;
    const float lim = 1.5f;  // ~86 degrees
    pitch = std::clamp(pitch, -lim, lim);
}

void OrbitCamera::pan(float dx, float dy) {
    glm::vec3 forward = glm::normalize(target - position());
    glm::vec3 right   = glm::normalize(glm::cross(forward, glm::vec3(0, 0, 1)));
    glm::vec3 up      = glm::normalize(glm::cross(right, forward));
    float scale = distance * 0.002f;
    target += (-right * dx + up * dy) * scale;
}

void OrbitCamera::zoom(float scroll_y) {
    distance *= std::pow(0.9f, scroll_y);
    distance = std::clamp(distance, 0.3f, 50.0f);
}

glm::vec3 OrbitCamera::position() const {
    float cp = std::cos(pitch);
    glm::vec3 dir(std::cos(yaw) * cp, std::sin(yaw) * cp, std::sin(pitch));
    return target + dir * distance;
}

glm::mat4 OrbitCamera::view() const {
    return glm::lookAt(position(), target, glm::vec3(0, 0, 1));
}

glm::mat4 OrbitCamera::projection(float aspect) const {
    return glm::perspective(glm::radians(50.0f), aspect, 0.05f, 100.0f);
}

}  // namespace robotsim
