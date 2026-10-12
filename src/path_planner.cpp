#include "path_planner.h"
#include <math.h>
float pointDistance(const TeachPoint& a,const TeachPoint& b) { float x=a.x-b.x,y=a.y-b.y; return sqrtf(x*x+y*y); }
uint8_t shapeVertexCount(ShapeId s) { return s==ShapeId::V?3:4; }
bool shapeVertex(ShapeId s,uint8_t i,const TeachPoint& lo,const TeachPoint& hi,TeachPoint& p) {
    if(i>=shapeVertexCount(s) || !isfinite(lo.x) || !isfinite(lo.y) || !isfinite(hi.x) || !isfinite(hi.y) || hi.x-lo.x<0.1f || hi.y-lo.y<0.1f) return false;
    float x=0,y=0;
    switch(s) {
    case ShapeId::N: x=i>=2?1:0; y=i%2?1:0; break;
    case ShapeId::Triangle: x=i==1?1:i==2?0.5f:0; y=i==2?1:0; break;
    case ShapeId::Z: x=(i==1 || i==3)?1:0; y=i<2?1:0; break;
    case ShapeId::V: x=i*0.5f; y=i==1?0:1; break;
    }
    p={lo.x+x*(hi.x-lo.x),lo.y+y*(hi.y-lo.y)}; return true;
}
bool centripetalSample(const TeachPoint& p0,const TeachPoint& p1,const TeachPoint& p2,const TeachPoint& p3,float t,TeachPoint& p) {
    if(!isfinite(t) || t<0 || t>1) return false;
    float a=sqrtf(pointDistance(p0,p1)),b=sqrtf(pointDistance(p1,p2)),c=sqrtf(pointDistance(p2,p3));
    if(!isfinite(a) || !isfinite(b) || !isfinite(c) || a<0.001f || b<0.001f || c<0.001f) return false;
    float before=b*b/(a*(a+b)),after=a/(a+b);
    float nextBefore=c/(b+c),nextAfter=b*b/(c*(b+c));
    float m1x=before*(p1.x-p0.x)+after*(p2.x-p1.x);
    float m1y=before*(p1.y-p0.y)+after*(p2.y-p1.y);
    float m2x=nextBefore*(p2.x-p1.x)+nextAfter*(p3.x-p2.x);
    float m2y=nextBefore*(p2.y-p1.y)+nextAfter*(p3.y-p2.y);
    float t2=t*t,t3=t2*t;
    float h0=2*t3-3*t2+1,h1=t3-2*t2+t,h2=-2*t3+3*t2,h3=t3-t2;
    p={h0*p1.x+h1*m1x+h2*p2.x+h3*m2x,h0*p1.y+h1*m1y+h2*p2.y+h3*m2y};
    return isfinite(p.x) && isfinite(p.y);
}
