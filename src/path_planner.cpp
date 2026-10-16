#include "path_planner.h"
#include <math.h>
float pointDistance(const TeachPoint& a,const TeachPoint& b) { float x=a.x-b.x,y=a.y-b.y; return sqrtf(x*x+y*y); }
uint8_t shapeVertexCount(ShapeId s) { return s==ShapeId::N?4:0; }
bool shapeVertex(ShapeId s,uint8_t i,const TeachPoint& lo,const TeachPoint& hi,TeachPoint& p) {
    if(s!=ShapeId::N || i>=4 || !isfinite(lo.x) || !isfinite(lo.y) || !isfinite(hi.x) || !isfinite(hi.y) || hi.x-lo.x<0.1f || hi.y-lo.y<0.1f) return false;
    // N's original coefficients are (0,0),(0,1),(1,0),(1,1).
    // Preserve the exact Cartesian interpolation below; no shape table remains.
    float x=i>>1,y=i&1;
    p={lo.x+x*(hi.x-lo.x),lo.y+y*(hi.y-lo.y)}; return true;
}
namespace {
constexpr float kCurveTangentGain=1.20f;
struct CurveWeights { float before,after,nextBefore,nextAfter,h0,h1,h2,h3; };
__attribute__((noinline)) float curveAxis(float p0,float p1,float p2,float p3,const CurveWeights& w) {
    float m1=w.before*(p1-p0)+w.after*(p2-p1);
    float m2=w.nextBefore*(p2-p1)+w.nextAfter*(p3-p2);
    return w.h0*p1+w.h1*m1+w.h2*p2+w.h3*m2;
}
}
// Keep curve arithmetic scratch off the caller's persistent IK stack frame.
// Alpha=0.5 parameterization is retained; only Hermite tangent terms are scaled.
__attribute__((noinline)) bool centripetalSample(const TeachPoint& p0,const TeachPoint& p1,const TeachPoint& p2,const TeachPoint& p3,float t,TeachPoint& p) {
    if(!isfinite(t) || t<0 || t>1) return false;
    float a=sqrtf(pointDistance(p0,p1)),b=sqrtf(pointDistance(p1,p2)),c=sqrtf(pointDistance(p2,p3));
    if(!isfinite(a) || !isfinite(b) || !isfinite(c) || a<0.001f || b<0.001f || c<0.001f) return false;
    CurveWeights w;
    w.before=b*b/(a*(a+b)); w.after=a/(a+b);
    w.nextBefore=c/(b+c); w.nextAfter=b*b/(c*(b+c));
    float t2=t*t,t3=t2*t;
    w.h0=2*t3-3*t2+1; w.h1=kCurveTangentGain*(t3-2*t2+t); w.h2=-2*t3+3*t2; w.h3=kCurveTangentGain*(t3-t2);
    p={curveAxis(p0.x,p1.x,p2.x,p3.x,w),curveAxis(p0.y,p1.y,p2.y,p3.y,w)};
    return isfinite(p.x) && isfinite(p.y);
}
