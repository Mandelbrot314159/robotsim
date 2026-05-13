#include "arm.h"
#include <glm/gtc/matrix_transform.hpp>

namespace robotsim {

Arm::Arm() {
    // 7-DOF anthropomorphic-ish layout. Axes are in the joint's local frame.
    // Order: base yaw, shoulder pitch, upper arm roll, elbow pitch,
    //        forearm roll, wrist pitch, hand roll.
    joints_[0] = { glm::vec3(0, 0, 1), 0.30f, -3.1416f,  3.1416f };
    joints_[1] = { glm::vec3(1, 0, 0), 0.35f, -1.5708f,  1.5708f };
    joints_[2] = { glm::vec3(0, 0, 1), 0.05f, -3.0000f,  3.0000f };
    joints_[3] = { glm::vec3(1, 0, 0), 0.30f, -2.3000f,  0.1000f };
    joints_[4] = { glm::vec3(0, 0, 1), 0.05f, -3.0000f,  3.0000f };
    joints_[5] = { glm::vec3(1, 0, 0), 0.15f, -1.8000f,  1.8000f };
    joints_[6] = { glm::vec3(0, 0, 1), 0.10f, -3.0000f,  3.0000f };
}

void Arm::setAngle(int j, float r) {
    angles_[j] = glm::clamp(r, joints_[j].lower, joints_[j].upper);
}

void Arm::setAngles(const std::array<float, NUM_JOINTS>& a) {
    for (int i = 0; i < NUM_JOINTS; ++i) setAngle(i, a[i]);
}

std::array<glm::mat4, NUM_JOINTS> Arm::linkTransforms() const {
    std::array<glm::mat4, NUM_JOINTS> out;
    glm::mat4 T(1.0f);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        T = T * glm::rotate(glm::mat4(1.0f), angles_[i], joints_[i].axis);
        out[i] = T;
        T = T * glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, joints_[i].length));
    }
    return out;
}

glm::mat4 Arm::endEffector() const {
    glm::mat4 T(1.0f);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        T = T * glm::rotate(glm::mat4(1.0f), angles_[i], joints_[i].axis);
        T = T * glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, joints_[i].length));
    }
    return T;
}

}  // namespace robotsim
