#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "picking.h"

namespace robotsim {

class Renderer;

// A world-aligned transform manipulator for a single pose (position + orientation):
//   - Center  : free translation on a camera-facing plane
//   - Axis     : translation constrained to a world X / Y / Z line
//   - Plane    : translation constrained to a world XY / YZ / XZ plane
//   - Rotation : reorientation about a world X / Y / Z axis (rings around the point)
enum class GizmoHandle {
    None,
    Center,
    AxisX, AxisY, AxisZ,
    PlaneXY, PlaneYZ, PlaneXZ,
    RotX, RotY, RotZ,
};

// Per-drag bookkeeping, captured when a handle is grabbed.
struct GizmoDrag {
    GizmoHandle active = GizmoHandle::None;
    glm::vec3   targetAtGrab{0.0f};
    glm::vec3   grabHit{0.0f};      // grab point on the constraint line/plane
    glm::vec3   axisWorld{0.0f};    // axis for line-translate / ring-rotate
    glm::vec3   planeNormal{0.0f};  // plane normal for plane/center-translate
    glm::vec3   refDir{0.0f};       // in-plane reference direction for rotation
    glm::quat   rotAtGrab{1.0f, 0.0f, 0.0f, 0.0f};
};

// Overall gizmo size in world units; scales with camera distance so the handles
// stay a comfortable on-screen size at any zoom.
float gizmoScale(float cameraDistance);

// Ray-pick the handle under the cursor (None if the ray misses every handle).
GizmoHandle gizmoPick(const Ray& ray, const glm::vec3& target, float scale);

// Begin dragging `handle`; records the references used by gizmoDrag.
void gizmoBeginDrag(GizmoDrag& drag, GizmoHandle handle, const Ray& ray,
                    const glm::vec3& target, const glm::quat& rot,
                    const glm::vec3& cameraForward);

// Continue an in-progress drag, updating `target` and/or `rot` in place.
void gizmoDrag(const GizmoDrag& drag, const Ray& ray,
               glm::vec3& target, glm::quat& rot);

// Render the gizmo handles; `activeOrHovered` is drawn highlighted.
void gizmoRender(Renderer& renderer, const glm::vec3& target, float scale,
                 GizmoHandle activeOrHovered);

}  // namespace robotsim
