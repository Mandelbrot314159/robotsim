#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "arm.h"

namespace robotsim {

struct IKConfig {
    int   iterations = 10;     // DLS iterations applied per call
    float damping    = 0.06f;  // lambda for damped least squares (singularity robustness)
    float maxStep    = 0.06f;  // max end-effector displacement chased per iteration (world units)
    float maxAngStep = 0.10f;  // max end-effector reorientation chased per iteration (radians)
    float rotWeight  = 0.8f;   // relative importance of orientation vs position
    float postureGain = 0.05f; // null-space pull toward joint mid-range (0 disables); keeps
                               // joints off their limits so the arm avoids locking up straight
    float tolerance  = 1e-4f;  // stop early once within this distance of the target
};

struct IKResult {
    float error    = 0.0f;  // final distance from the end-effector to the target position
    float angError = 0.0f;  // final orientation error (radians)
    bool  reached  = false; // true when position error is within a small threshold
};

// Damped-least-squares position IK. Nudges the arm's joint angles so the
// end-effector moves toward `target`, honoring joint limits (via Arm::setAngle).
// Mutates `arm`; intended to be called once per frame while a target is active.
IKResult solveIKPosition(Arm& arm, const glm::vec3& target, const IKConfig& cfg = {});

// Damped-least-squares full-pose IK: drives the end-effector toward both a
// target position and orientation (6-DOF). Uses the arm's redundancy (7 DOF)
// to satisfy both where reachable. Honors joint limits; mutates `arm`.
IKResult solveIKPose(Arm& arm, const glm::vec3& targetPos, const glm::quat& targetRot,
                     const IKConfig& cfg = {});

}  // namespace robotsim
