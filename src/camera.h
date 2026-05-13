#pragma once
#include <glm/glm.hpp>

namespace robotsim {

class OrbitCamera {
public:
    glm::vec3 target{0.0f, 0.0f, 0.5f};
    float distance = 2.5f;
    float yaw   = 0.7f;
    float pitch = 0.5f;

    void orbit(float dx, float dy);
    void pan(float dx, float dy);
    void zoom(float scroll_y);

    glm::vec3 position() const;
    glm::mat4 view() const;
    glm::mat4 projection(float aspect) const;
};

}  // namespace robotsim
