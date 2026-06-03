#include "ik.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <utility>

namespace robotsim {

static glm::vec3 eePosition(const Arm& arm) {
    return glm::vec3(arm.endEffector()[3]);
}

// Solve the 6x6 system A x = b in place via Gauss-Jordan elimination with
// partial pivoting. A is symmetric positive-(semi)definite here; damping keeps
// it well-conditioned. Singular rows fall back to x = 0 for that component.
static void solveLinear6(double A[6][6], const double b[6], double x[6]) {
    double M[6][7];
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) M[r][c] = A[r][c];
        M[r][6] = b[r];
    }
    for (int col = 0; col < 6; ++col) {
        int piv = col;
        double best = std::fabs(M[col][col]);
        for (int r = col + 1; r < 6; ++r) {
            double v = std::fabs(M[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (piv != col)
            for (int c = 0; c < 7; ++c) std::swap(M[col][c], M[piv][c]);
        double d = M[col][col];
        if (std::fabs(d) < 1e-12) continue;
        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            double f = M[r][col] / d;
            for (int c = col; c < 7; ++c) M[r][c] -= f * M[col][c];
        }
    }
    for (int r = 0; r < 6; ++r)
        x[r] = (std::fabs(M[r][r]) > 1e-12) ? M[r][6] / M[r][r] : 0.0;
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

// Orientation error as a world-frame rotation vector taking qcur -> qtar.
static glm::vec3 rotationError(const glm::quat& qcur, const glm::quat& qtar) {
    glm::quat qe = qtar * glm::inverse(qcur);
    if (qe.w < 0.0f) qe = glm::quat(-qe.w, -qe.x, -qe.y, -qe.z);  // shortest path
    float angle = 2.0f * std::acos(glm::clamp(qe.w, -1.0f, 1.0f));
    glm::vec3 axis(qe.x, qe.y, qe.z);
    float s = glm::length(axis);
    return (s > 1e-6f) ? axis * (angle / s) : glm::vec3(0.0f);
}

IKResult solveIKPose(Arm& arm, const glm::vec3& targetPos, const glm::quat& targetRot,
                     const IKConfig& cfg) {
    const float lambda2 = cfg.damping * cfg.damping;
    const float wr      = cfg.rotWeight;

    for (int iter = 0; iter < cfg.iterations; ++iter) {
        auto Ts = arm.linkTransforms();
        glm::mat4 E = arm.endEffector();
        glm::vec3 pe = glm::vec3(E[3]);
        glm::quat qcur = glm::quat_cast(glm::mat3(E));

        // Position error (clamped to a small step).
        glm::vec3 perr = targetPos - pe;
        float pd = glm::length(perr);
        if (pd > cfg.maxStep) perr *= cfg.maxStep / pd;

        // Orientation error (clamped to a small angular step).
        glm::vec3 rerr = rotationError(qcur, targetRot);
        float rd = glm::length(rerr);
        if (rd > cfg.maxAngStep) rerr *= cfg.maxAngStep / rd;

        if (pd < cfg.tolerance && rd < cfg.tolerance) break;

        // Stacked 6x7 Jacobian: rows 0-2 linear, rows 3-5 angular (weighted).
        double J[6][NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; ++i) {
            glm::vec3 axisW = glm::normalize(glm::mat3(Ts[i]) * arm.config(i).axis);
            glm::vec3 pi    = glm::vec3(Ts[i][3]);
            glm::vec3 lin   = glm::cross(axisW, pe - pi);
            J[0][i] = lin.x;        J[1][i] = lin.y;        J[2][i] = lin.z;
            J[3][i] = axisW.x * wr; J[4][i] = axisW.y * wr; J[5][i] = axisW.z * wr;
        }

        // Weighted 6-vector error.
        double e[6] = { perr.x, perr.y, perr.z,
                        rerr.x * wr, rerr.y * wr, rerr.z * wr };

        // A = J Jt + lambda^2 I  (6x6), then solve A y = e.
        double A[6][6];
        for (int a = 0; a < 6; ++a)
            for (int b = 0; b < 6; ++b) {
                double s = 0.0;
                for (int i = 0; i < NUM_JOINTS; ++i) s += J[a][i] * J[b][i];
                A[a][b] = s + (a == b ? lambda2 : 0.0);
            }
        double y[6];
        solveLinear6(A, e, y);

        // Primary task: dtheta_p = Jt y  (drives the end-effector toward the target).
        // Secondary task projected into the null space so it does not disturb the
        // end-effector: pull each joint gently toward the middle of its range. This
        // keeps the elbow (and others) off their limits, so the arm doesn't get stuck
        // in a fully-extended singular pose it can't recover from.
        double z[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; ++i) {
            float mid = 0.5f * (arm.config(i).lower + arm.config(i).upper);
            z[i] = -cfg.postureGain * (arm.getAngle(i) - mid);
        }
        // nullspace(z) = z - Jt (J Jt + lambda^2 I)^-1 (J z)
        double Jz[6];
        for (int a = 0; a < 6; ++a) {
            double s = 0.0;
            for (int i = 0; i < NUM_JOINTS; ++i) s += J[a][i] * z[i];
            Jz[a] = s;
        }
        double AinvJz[6];
        solveLinear6(A, Jz, AinvJz);

        for (int i = 0; i < NUM_JOINTS; ++i) {
            double dtheta = 0.0;
            for (int a = 0; a < 6; ++a) dtheta += J[a][i] * y[a];  // primary
            double proj = 0.0;
            for (int a = 0; a < 6; ++a) proj += J[a][i] * AinvJz[a];
            dtheta += z[i] - proj;  // null-space posture bias
            arm.setAngle(i, arm.getAngle(i) + (float)dtheta);
        }
    }

    glm::mat4 E = arm.endEffector();
    IKResult r;
    r.error    = glm::length(targetPos - glm::vec3(E[3]));
    r.angError = glm::length(rotationError(glm::quat_cast(glm::mat3(E)), targetRot));
    r.reached  = r.error < 0.01f;
    return r;
}

}  // namespace robotsim
