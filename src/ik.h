#pragma once
#include <glm/glm.hpp>
#include "arm.h"

namespace robotsim {

struct IKConfig {
    int   iterations = 10;     // DLS iterations applied per call
    float damping    = 0.06f;  // lambda for damped least squares (singularity robustness)
    float maxStep    = 0.06f;  // max end-effector displacement chased per iteration (world units)
    float tolerance  = 1e-4f;  // stop early once within this distance of the target
};

struct IKResult {
    float error   = 0.0f;   // final distance from the end-effector to the target
    bool  reached = false;  // true when error is within a small threshold
};

// Damped-least-squares position IK. Nudges the arm's joint angles so the
// end-effector moves toward `target`, honoring joint limits (via Arm::setAngle).
// Mutates `arm`; intended to be called once per frame while a target is active.
IKResult solveIKPosition(Arm& arm, const glm::vec3& target, const IKConfig& cfg = {});

}  // namespace robotsim
