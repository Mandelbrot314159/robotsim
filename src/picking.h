#pragma once
#include <cmath>
#include <glm/glm.hpp>

namespace robotsim {

struct Ray {
    glm::vec3 origin{0.0f};
    glm::vec3 dir{0.0f, 0.0f, -1.0f};  // normalized
};

// Build a world-space ray from a mouse position given in window pixels.
inline Ray screenRay(float px, float py, float win_w, float win_h,
                     const glm::mat4& view, const glm::mat4& proj) {
    float nx = 2.0f * px / win_w - 1.0f;
    float ny = 1.0f - 2.0f * py / win_h;
    glm::mat4 inv = glm::inverse(proj * view);
    glm::vec4 n = inv * glm::vec4(nx, ny, -1.0f, 1.0f);
    glm::vec4 f = inv * glm::vec4(nx, ny,  1.0f, 1.0f);
    n /= n.w;
    f /= f.w;
    Ray r;
    r.origin = glm::vec3(n);
    r.dir    = glm::normalize(glm::vec3(f - n));
    return r;
}

// Ray vs sphere. Returns true and sets `t` (>= 0) for the nearest hit.
inline bool raySphere(const Ray& r, const glm::vec3& center, float radius, float& t) {
    glm::vec3 oc = r.origin - center;
    float b    = glm::dot(oc, r.dir);
    float c    = glm::dot(oc, oc) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.0f) return false;
    float s  = std::sqrt(disc);
    float t0 = -b - s;
    float t1 = -b + s;
    t = (t0 >= 0.0f) ? t0 : t1;
    return t >= 0.0f;
}

// Ray vs plane (point `p0`, normal `n`). Returns true and sets `hit`.
inline bool rayPlane(const Ray& r, const glm::vec3& p0, const glm::vec3& n,
                     glm::vec3& hit) {
    float denom = glm::dot(n, r.dir);
    if (std::fabs(denom) < 1e-6f) return false;
    float t = glm::dot(p0 - r.origin, n) / denom;
    if (t < 0.0f) return false;
    hit = r.origin + t * r.dir;
    return true;
}

}  // namespace robotsim
