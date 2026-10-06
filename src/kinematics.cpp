#include "kinematics.h"
#include "arm.h"

#include <math.h>

namespace {
// Original calibration: lengths in mm, angles in degrees.
constexpr float L1 = 80.0f;
constexpr float L2 = 80.0f;
constexpr float RIGHT_ZERO = 184.0f;
constexpr float ELBOW_OFFSET = 116.9f;
}

FKResult solveFK(float shoulderAngle, float elbowAngle)
{
    FKResult result;
    result.alpha = RIGHT_ZERO - elbowAngle;
    result.gamma = shoulderAngle + elbowAngle - ELBOW_OFFSET;
    result.beta = result.alpha - (180.0f - result.gamma);

    float alphaRad = result.alpha * PI / 180.0f;
    float betaRad = result.beta * PI / 180.0f;

    result.position.x = L1 * cos(alphaRad) + L2 * cos(betaRad);
    result.position.z = L1 * sin(alphaRad) + L2 * sin(betaRad);
    return result;
}

IKResult solveIK(float x, float z)
{
    IKResult result{};
    if (!isfinite(x) || !isfinite(z)) {
        return result;
    }

    float d = (x * x + z * z - L1 * L1 - L2 * L2)
              / (2.0f * L1 * L2);
    if (!isfinite(d) || d < -1.0f || d > 1.0f) {
        return result;
    }

    // Preserve the original physical branch and arithmetic order.
    float q = -acos(d);
    float alpha = atan2(z, x)
                  - atan2(L2 * sin(q), L1 + L2 * cos(q));
    float beta = alpha + q;
    float alphaDeg = alpha * 180.0f / PI;
    float betaDeg = beta * 180.0f / PI;
    float gammaDeg = 180.0f + q * 180.0f / PI;
    float targetJ2 = RIGHT_ZERO - alphaDeg;
    float targetJ1 = gammaDeg + ELBOW_OFFSET - targetJ2;

    if (!isSafePlanarPose(targetJ1, targetJ2)) {
        return result;
    }

    result.valid = true;
    result.alpha = alphaDeg;
    result.beta = betaDeg;
    result.gamma = gammaDeg;
    result.j1 = targetJ1;
    result.j2 = targetJ2;
    return result;
}
