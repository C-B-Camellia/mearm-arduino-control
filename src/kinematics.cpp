#include "kinematics.h"
#include "arm.h"
#include "hardware_config.h"

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

Point3D solveFK3D(float j0,float j1,float j2)
{
    Point3D invalid={NAN,NAN,NAN};
    if(!cartesianCalibrationValid() || !isfinite(j0) || !isfinite(j1) || !isfinite(j2)) return invalid;
    const CartesianCalibration& c=hardwareConfig.cartesian;
    FKResult planar=solveFK(j1,j2);
    float radial=planar.position.x+c.shoulderRadial;
    float height=planar.position.z+c.shoulderHeight;
    if(c.toolModel==ToolModel::FixedPlanar) { radial+=c.toolRadial; height+=c.toolVertical; }
    else {
        float beta=planar.beta*PI/180.0f;
        radial+=c.toolRadial*cos(beta)-c.toolVertical*sin(beta);
        height+=c.toolRadial*sin(beta)+c.toolVertical*cos(beta);
    }
    float yaw=(j0-c.yawZero)*c.yawSign*PI/180.0f;
    return {radial*cos(yaw)-c.toolTangential*sin(yaw),radial*sin(yaw)+c.toolTangential*cos(yaw),height};
}

IKResult3D solveIK3D(const Point3D& p)
{
    IKResult3D out{};
    if(!cartesianCalibrationValid() || !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z)) return out;
    const CartesianCalibration& c=hardwareConfig.cartesian;
    float squared=p.x*p.x+p.y*p.y-c.toolTangential*c.toolTangential;
    if(!isfinite(squared) || squared<=0.0001f) return out;
    float radial=sqrt(squared);
    float yaw=(atan2(p.y,p.x)-atan2(c.toolTangential,radial))*180.0f/PI;
    float bestDistance=10000;
    bool baseFound=false;
    for(int8_t turn=-1;turn<=1;++turn) {
        float angle=c.yawZero+c.yawSign*(yaw+turn*360.0f);
        if(angle<hardwareConfig.base.minimum || angle>hardwareConfig.base.maximum) continue;
        float distance=fabs(angle-joints[Base].commandedAngle);
        if(distance<bestDistance) { out.j0=angle; bestDistance=distance; baseFound=true; }
    }
    if(!baseFound) return out;
    float x=radial-c.shoulderRadial, z=p.z-c.shoulderHeight;
    if(c.toolModel==ToolModel::FixedPlanar) {
        IKResult ik=solveIK(x-c.toolRadial,z-c.toolVertical);
        if(!ik.valid) return out;
        out.j1=ik.j1; out.j2=ik.j2;
    } else {
        float lx=L2+c.toolRadial, lz=c.toolVertical;
        float effective=sqrt(lx*lx+lz*lz);
        if(effective<0.001f) return out;
        float d=(x*x+z*z-L1*L1-effective*effective)/(2.0f*L1*effective);
        if(!isfinite(d) || d<-1 || d>1) return out;
        float q=-acos(d);
        float alpha=atan2(z,x)-atan2(effective*sin(q),L1+effective*cos(q));
        float beta=alpha+q-atan2(lz,lx);
        out.j2=RIGHT_ZERO-alpha*180.0f/PI;
        out.j1=beta*180.0f/PI+180.0f+ELBOW_OFFSET-RIGHT_ZERO;
        if(!isSafePlanarPose(out.j1,out.j2)) return out;
    }
    Point3D check=solveFK3D(out.j0,out.j1,out.j2);
    out.valid=isfinite(check.x) && fabs(check.x-p.x)<0.05f && fabs(check.y-p.y)<0.05f && fabs(check.z-p.z)<0.05f;
    return out;
}
