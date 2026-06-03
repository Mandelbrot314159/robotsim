#include "ik.h"
#include <glm/gtc/matrix_transform.hpp>

namespace robotsim {

static glm::vec3 eePosition(const Arm& arm) {
    return glm::vec3(arm.endEffector()[3]);
}

IKResult solveIKPosition(Arm& arm, const glm::vec3& target, const IKConfig& cfg) {
    const float lambda2 = cfg.damping * cfg.damping;

    for (int iter = 0; iter < cfg.iterations; ++iter) {
        auto Ts = arm.linkTransforms();
        glm::vec3 pe  = eePosition(arm);
        glm::vec3 err = target - pe;
        float dist = glm::length(err);
        if (dist < cfg.tolerance) break;

        // Chase only a small displacement each iteration for stability.
        glm::vec3 dx = (dist > cfg.maxStep) ? err * (cfg.maxStep / dist) : err;

        // Position Jacobian: column i = (world axis of joint i) x (pe - joint origin i).
        glm::vec3 col[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; ++i) {
            glm::vec3 axisW = glm::normalize(glm::mat3(Ts[i]) * arm.config(i).axis);
            glm::vec3 pi    = glm::vec3(Ts[i][3]);
            col[i] = glm::cross(axisW, pe - pi);
        }

        // Damped least squares: dtheta = Jt (J Jt + lambda^2 I)^-1 dx.
        // J Jt is 3x3, so the inverse is cheap.
        glm::mat3 JJt(0.0f);
        for (int i = 0; i < NUM_JOINTS; ++i) JJt += glm::outerProduct(col[i], col[i]);
        JJt[0][0] += lambda2;
        JJt[1][1] += lambda2;
        JJt[2][2] += lambda2;
        glm::vec3 v = glm::inverse(JJt) * dx;

        // dtheta_i = col_i . v  (i-th row of Jt times v). Clamped to limits by setAngle.
        for (int i = 0; i < NUM_JOINTS; ++i) {
            float dtheta = glm::dot(col[i], v);
            arm.setAngle(i, arm.getAngle(i) + dtheta);
        }
    }

    IKResult r;
    r.error   = glm::length(target - eePosition(arm));
    r.reached = r.error < 0.01f;
    return r;
}

}  // namespace robotsim
