#include "gizmo.h"
#include "renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>

namespace robotsim {

namespace {

// Handle dimensions as fractions of the overall gizmo scale S.
constexpr float kAxisLen     = 1.00f;
constexpr float kAxisThick   = 0.05f;
constexpr float kAxisPickR   = 0.13f;
constexpr float kTipBox      = 0.10f;
constexpr float kPlaneOff    = 0.40f;  // center of the plane quad, along each in-plane axis
constexpr float kPlaneHalf   = 0.16f;  // half-extent of the plane quad
constexpr float kPlaneThick  = 0.02f;
constexpr float kRingR       = 1.35f;
constexpr float kRingSeg     = 0.04f;
constexpr float kRingPickW   = 0.16f;
constexpr float kCenterBox   = 0.16f;
constexpr float kCenterPickR = 0.22f;
constexpr int   kRingSegments = 48;

const glm::vec3 kColX(0.90f, 0.25f, 0.25f);
const glm::vec3 kColY(0.30f, 0.80f, 0.35f);
const glm::vec3 kColZ(0.30f, 0.50f, 0.95f);
const glm::vec3 kColHighlight(1.00f, 0.85f, 0.20f);
const glm::vec3 kColCenter(0.92f, 0.92f, 0.92f);

glm::vec3 axisUnit(GizmoHandle h) {
    switch (h) {
        case GizmoHandle::AxisX: case GizmoHandle::RotX: return glm::vec3(1, 0, 0);
        case GizmoHandle::AxisY: case GizmoHandle::RotY: return glm::vec3(0, 1, 0);
        case GizmoHandle::AxisZ: case GizmoHandle::RotZ: return glm::vec3(0, 0, 1);
        default: return glm::vec3(0);
    }
}

// World-plane info for a plane-translate handle: normal + the two in-plane axes.
void planeInfo(GizmoHandle h, glm::vec3& n, glm::vec3& ua, glm::vec3& ub) {
    switch (h) {
        case GizmoHandle::PlaneXY: n = {0,0,1}; ua = {1,0,0}; ub = {0,1,0}; break;
        case GizmoHandle::PlaneYZ: n = {1,0,0}; ua = {0,1,0}; ub = {0,0,1}; break;
        case GizmoHandle::PlaneXZ: n = {0,1,0}; ua = {1,0,0}; ub = {0,0,1}; break;
        default: n = ua = ub = glm::vec3(0); break;
    }
}

// Ray vs plane (point p0, normal n). Returns true and sets hit point + ray param t.
bool planeHit(const Ray& r, const glm::vec3& p0, const glm::vec3& n,
              glm::vec3& hit, float& t) {
    float denom = glm::dot(n, r.dir);
    if (std::fabs(denom) < 1e-6f) return false;
    t = glm::dot(p0 - r.origin, n) / denom;
    if (t < 0.0f) return false;
    hit = r.origin + t * r.dir;
    return true;
}

// Closest points between the ray and the infinite line through A with unit dir u.
bool closestApproach(const Ray& r, const glm::vec3& A, const glm::vec3& u,
                     float& t, float& s, glm::vec3& Pc, glm::vec3& Qc) {
    glm::vec3 w0 = r.origin - A;
    float b = glm::dot(r.dir, u);
    float denom = 1.0f - b * b;  // a = c = 1 (both dirs unit)
    if (std::fabs(denom) < 1e-6f) return false;  // parallel
    float dd = glm::dot(r.dir, w0);
    float ee = glm::dot(u, w0);
    t = (b * ee - dd) / denom;
    s = (ee - b * dd) / denom;
    Pc = r.origin + r.dir * t;
    Qc = A + u * s;
    return true;
}

glm::vec3 normalizeSafe(const glm::vec3& v) {
    float l = glm::length(v);
    return (l > 1e-6f) ? v / l : glm::vec3(1, 0, 0);
}

glm::vec3 handleColor(GizmoHandle h, GizmoHandle active) {
    if (h == active) return kColHighlight;
    switch (h) {
        case GizmoHandle::AxisX: case GizmoHandle::RotX: case GizmoHandle::PlaneYZ: return kColX;
        case GizmoHandle::AxisY: case GizmoHandle::RotY: case GizmoHandle::PlaneXZ: return kColY;
        case GizmoHandle::AxisZ: case GizmoHandle::RotZ: case GizmoHandle::PlaneXY: return kColZ;
        default: return kColCenter;
    }
}

void drawCube(Renderer& r, const glm::vec3& center, const glm::vec3& size,
              const glm::vec3& color) {
    glm::mat4 m = glm::translate(glm::mat4(1.0f), center) *
                  glm::scale(glm::mat4(1.0f), size);
    r.drawBox(m, color);
}

}  // namespace

float gizmoScale(float cameraDistance) {
    return cameraDistance * 0.15f;
}

GizmoHandle gizmoPick(const Ray& ray, const glm::vec3& target, float S) {
    GizmoHandle best = GizmoHandle::None;
    float bestT = 1e30f;

    // Center (free move).
    {
        float t;
        if (raySphere(ray, target, kCenterPickR * S, t) && t < bestT) {
            bestT = t; best = GizmoHandle::Center;
        }
    }
    // Axis lines.
    const GizmoHandle axes[3] = { GizmoHandle::AxisX, GizmoHandle::AxisY, GizmoHandle::AxisZ };
    for (GizmoHandle h : axes) {
        glm::vec3 u = axisUnit(h);
        float t, s; glm::vec3 Pc, Qc;
        if (closestApproach(ray, target, u, t, s, Pc, Qc) &&
            t > 0.0f && s >= 0.0f && s <= kAxisLen * S &&
            glm::length(Pc - Qc) <= kAxisPickR * S && t < bestT) {
            bestT = t; best = h;
        }
    }
    // Plane quads.
    const GizmoHandle planes[3] = { GizmoHandle::PlaneXY, GizmoHandle::PlaneYZ, GizmoHandle::PlaneXZ };
    for (GizmoHandle h : planes) {
        glm::vec3 n, ua, ub; planeInfo(h, n, ua, ub);
        glm::vec3 hit; float t;
        if (planeHit(ray, target, n, hit, t) && t > 0.0f) {
            float a = glm::dot(hit - target, ua);
            float b = glm::dot(hit - target, ub);
            float lo = (kPlaneOff - kPlaneHalf) * S, hi = (kPlaneOff + kPlaneHalf) * S;
            if (a >= lo && a <= hi && b >= lo && b <= hi && t < bestT) {
                bestT = t; best = h;
            }
        }
    }
    // Rotation rings.
    const GizmoHandle rings[3] = { GizmoHandle::RotX, GizmoHandle::RotY, GizmoHandle::RotZ };
    for (GizmoHandle h : rings) {
        glm::vec3 n = axisUnit(h);
        glm::vec3 hit; float t;
        if (planeHit(ray, target, n, hit, t) && t > 0.0f) {
            float d = glm::length(hit - target);
            if (std::fabs(d - kRingR * S) <= kRingPickW * S && t < bestT) {
                bestT = t; best = h;
            }
        }
    }
    return best;
}

void gizmoBeginDrag(GizmoDrag& drag, GizmoHandle handle, const Ray& ray,
                    const glm::vec3& target, const glm::quat& rot,
                    const glm::vec3& cameraForward) {
    drag.active       = handle;
    drag.targetAtGrab = target;
    drag.rotAtGrab    = rot;

    glm::vec3 hit; float t;
    switch (handle) {
        case GizmoHandle::Center:
            drag.planeNormal = normalizeSafe(cameraForward);
            drag.grabHit = planeHit(ray, target, drag.planeNormal, hit, t) ? hit : target;
            break;
        case GizmoHandle::AxisX: case GizmoHandle::AxisY: case GizmoHandle::AxisZ: {
            drag.axisWorld = axisUnit(handle);
            float s; glm::vec3 Pc, Qc;
            drag.grabHit = closestApproach(ray, target, drag.axisWorld, t, s, Pc, Qc) ? Qc : target;
            break;
        }
        case GizmoHandle::PlaneXY: case GizmoHandle::PlaneYZ: case GizmoHandle::PlaneXZ: {
            glm::vec3 ua, ub; planeInfo(handle, drag.planeNormal, ua, ub);
            drag.grabHit = planeHit(ray, target, drag.planeNormal, hit, t) ? hit : target;
            break;
        }
        case GizmoHandle::RotX: case GizmoHandle::RotY: case GizmoHandle::RotZ:
            drag.axisWorld = axisUnit(handle);
            if (planeHit(ray, target, drag.axisWorld, hit, t))
                drag.refDir = normalizeSafe(hit - target);
            else
                drag.refDir = glm::vec3(1, 0, 0);
            break;
        default:
            break;
    }
}

void gizmoDrag(const GizmoDrag& drag, const Ray& ray,
               glm::vec3& target, glm::quat& rot) {
    glm::vec3 hit; float t;
    switch (drag.active) {
        case GizmoHandle::Center:
        case GizmoHandle::PlaneXY: case GizmoHandle::PlaneYZ: case GizmoHandle::PlaneXZ:
            if (planeHit(ray, drag.targetAtGrab, drag.planeNormal, hit, t))
                target = drag.targetAtGrab + (hit - drag.grabHit);
            break;
        case GizmoHandle::AxisX: case GizmoHandle::AxisY: case GizmoHandle::AxisZ: {
            float s; glm::vec3 Pc, Qc;
            if (closestApproach(ray, drag.targetAtGrab, drag.axisWorld, t, s, Pc, Qc)) {
                float delta = glm::dot(Qc - drag.grabHit, drag.axisWorld);
                target = drag.targetAtGrab + drag.axisWorld * delta;
            }
            break;
        }
        case GizmoHandle::RotX: case GizmoHandle::RotY: case GizmoHandle::RotZ:
            if (planeHit(ray, drag.targetAtGrab, drag.axisWorld, hit, t)) {
                glm::vec3 v = hit - drag.targetAtGrab;
                if (glm::length(v) > 1e-5f) {
                    v = normalizeSafe(v);
                    float delta = std::atan2(glm::dot(glm::cross(drag.refDir, v), drag.axisWorld),
                                             glm::dot(drag.refDir, v));
                    rot = glm::normalize(glm::angleAxis(delta, drag.axisWorld) * drag.rotAtGrab);
                }
            }
            break;
        default:
            break;
    }
}

void gizmoRender(Renderer& r, const glm::vec3& target, float S, GizmoHandle hot) {
    // Center handle.
    drawCube(r, target, glm::vec3(kCenterBox * S),
             hot == GizmoHandle::Center ? kColHighlight : kColCenter);

    // Axis bars + tips.
    struct AxisDef { GizmoHandle h; glm::vec3 dir; };
    const AxisDef axisDefs[3] = {
        { GizmoHandle::AxisX, {1,0,0} },
        { GizmoHandle::AxisY, {0,1,0} },
        { GizmoHandle::AxisZ, {0,0,1} },
    };
    for (const auto& a : axisDefs) {
        glm::vec3 col = handleColor(a.h, hot);
        glm::vec3 size = glm::vec3(kAxisThick * S) + a.dir * (kAxisLen * S - kAxisThick * S);
        drawCube(r, target + a.dir * (kAxisLen * S * 0.5f), size, col);
        drawCube(r, target + a.dir * (kAxisLen * S), glm::vec3(kTipBox * S), col);
    }

    // Plane quads.
    const GizmoHandle planes[3] = { GizmoHandle::PlaneXY, GizmoHandle::PlaneYZ, GizmoHandle::PlaneXZ };
    for (GizmoHandle h : planes) {
        glm::vec3 n, ua, ub; planeInfo(h, n, ua, ub);
        glm::vec3 center = target + (ua + ub) * (kPlaneOff * S);
        glm::vec3 size = (ua + ub) * (kPlaneHalf * 2.0f * S) + n * (kPlaneThick * S);
        drawCube(r, center, size, handleColor(h, hot));
    }

    // Rotation rings (a loop of small cubes in each world plane).
    const GizmoHandle rings[3] = { GizmoHandle::RotX, GizmoHandle::RotY, GizmoHandle::RotZ };
    for (GizmoHandle h : rings) {
        glm::vec3 col = handleColor(h, hot);
        // Two in-plane basis axes for this ring.
        glm::vec3 e1 = (h == GizmoHandle::RotX) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
        glm::vec3 e2 = (h == GizmoHandle::RotZ) ? glm::vec3(0,1,0) : glm::vec3(0,0,1);
        for (int k = 0; k < kRingSegments; ++k) {
            float th = 2.0f * 3.14159265f * (float)k / (float)kRingSegments;
            glm::vec3 p = target + (e1 * std::cos(th) + e2 * std::sin(th)) * (kRingR * S);
            drawCube(r, p, glm::vec3(kRingSeg * S), col);
        }
    }
}

}  // namespace robotsim
