// Real production geometry; desktop-only virtual paper, never a calibration.
#include "kinematics.h"
static unsigned ikCalls=0,fkCalls=0;
static IKResult3D countedIK(const Point3D& p) { ++ikCalls; return solveIK3D(p); }
static Point3D countedFK(float a,float b,float c) { ++fkCalls; return solveFK3D(a,b,c); }
#define solveIK3D countedIK
#define solveFK3D countedFK
#define main existingSpeedMain
#include "speed_tests.cpp"
#undef main
#undef solveIK3D
#undef solveFK3D
#include <chrono>
static void trace(float feed) {
    reset(); setCommand(lowerPose(116)); CHECK(captureTeachHeight(true)); CHECK(setDrawingFeed(feed));
    Point3D a=fk(hardwareConfig.home);
    CHECK(startDrawLine({a.x,a.y},{a.x,a.y+20})); preflight();
    unsigned maximumIK=0,maximumFK=0,changed=0,frames=0,physical=0;
    uint32_t start=simulatedTime,strokeStart=0,strokeEnd=0;
    float maxZ=0,maxXY=0; Pose prior=commandedPose();
    uint16_t penUp=task.status.totalWaypoints-hardwareConfig.homeViaCount-1;
    for(unsigned guard=0;isMotionBusy() && guard<100000;++guard) {
        simulatedTime+=20; auto before=task.status.completedWaypoints;
        ikCalls=fkCalls=0; updateMotion(simulatedTime,true);
        maximumIK=max(maximumIK,ikCalls); maximumFK=max(maximumFK,fkCalls);
        if(hasPendingPose()) {
            Pose q=goalPose(); size_t previousWrites=servoWrites.size(); CHECK(applyPendingPose()); Pose integer=commandedPose();
            if(servoWrites.size()!=previousWrites) ++physical;
            ++frames; if(maxDifference(prior,integer)>0) ++changed; prior=integer;
            bool stroke=before>=4 && before<penUp-1;
            if(stroke) {
                Point3D point=fk(q);
                maxZ=max(maxZ,fabsf(point.z-teachSettings().drawZ));
                maxXY=max(maxXY,fabsf(point.x-a.x));
            }
            std::cout<<"FRAME,"<<feed<<','<<simulatedTime-start<<','<<before<<','<<stroke;
            for(float v:q.angles) std::cout<<','<<v;
            for(float v:integer.angles) std::cout<<','<<v;
            std::cout<<'\n';
        }
        updateMotion(simulatedTime,false);
        if(before<4 && task.status.completedWaypoints>=4) strokeStart=simulatedTime;
        if(before<penUp-1 && task.status.completedWaypoints>=penUp-1) strokeEnd=simulatedTime;
    }
    CHECK(!isMotionBusy() && !isArmHeld()); CHECK(strokeEnd>strokeStart);
    CHECK(maxZ<0.15f && maxXY<0.15f);
    std::cout<<"TRACE20 feed="<<feed<<" totalMs="<<simulatedTime-start<<" strokeMs="<<strokeEnd-strokeStart
             <<" frames="<<frames<<" physical="<<physical<<" changed="<<changed<<" maxDirectIK="<<maximumIK<<" maxDirectFK="<<maximumFK
             <<" floatZError="<<maxZ<<" floatCrossTrack="<<maxXY<<'\n';
}
static void allPaths() {
    for(unsigned mode=0;mode<2;++mode) for(unsigned kind=0;kind<4;++kind) {
        float feed=mode?100:25;
        if(kind>=2) prepareFive(feed,kind==3);
        else {
            reset(); setCommand(lowerPose(116)); CHECK(captureTeachHeight(true)); CHECK(setDrawingFeed(feed));
            Point3D a=fk(hardwareConfig.home);
            CHECK(kind==0?startDrawLine({a.x,a.y},{a.x,a.y+20}):startBasicShape(ShapeId::N,{a.x,a.y},{a.x+10,a.y+10}));
        }
        unsigned directIK=0,directFK=0;
        uint16_t down=hardwareConfig.homeViaCount+4,up=task.status.totalWaypoints-hardwareConfig.homeViaCount-1;
        Pose p=commandedPose();
        for(uint16_t i=down;i<up;++i) {
            CHECK(sampleGoal(i,p)); Point3D q=fk(p); CHECK(fabsf(q.z-teachSettings().drawZ)<0.05f);
            if(kind>=2) for(uint8_t j=0;j<5;++j) {
                TeachPoint knot; CHECK(getTeachPoint(j,knot));
                // Every exact knot is asserted below independently of sample count.
            }
        }
        if(kind>=2) for(uint8_t j=0;j<5;++j) {
            TeachPoint knot; CHECK(getTeachPoint(j,knot)); float nearest=10000;
            for(uint16_t i=down;i<up;++i) { CHECK(sampleGoal(i,p)); Point3D q=fk(p); nearest=min(nearest,pointDistance(knot,{q.x,q.y})); }
            CHECK(nearest<0.05f);
        }
        std::vector<TeachPoint> reference;
        if(kind==3) for(uint8_t segment=0;segment<4;++segment) {
            TeachPoint a,b,c,d,q; CHECK(curveKnots(segment,a,b,c,d));
            for(unsigned k=0;k<=256;++k) { CHECK(centripetalSample(a,b,c,d,k/256.f,q)); reference.push_back(q); }
        }
        preflight(); float floatCurveError=0,quantCurveError=0,maxZError=0;
        while(isMotionBusy()) {
            simulatedTime+=20; auto before=task.status.completedWaypoints; ikCalls=fkCalls=0; updateMotion(simulatedTime,true);
            directIK=max(directIK,ikCalls); directFK=max(directFK,fkCalls);
            Pose target=goalPose(); CHECK(applyPendingPose());
            if(before>=down && before<up-1) {
                Point3D point=fk(target),integer=fk(commandedPose());
                maxZError=max(maxZError,fabsf(point.z-teachSettings().drawZ));
                if(kind==3) {
                    float f=10000,q=10000;
                    for(const TeachPoint& sample:reference) {
                        f=min(f,pointDistance(sample,{point.x,point.y}));
                        q=min(q,pointDistance(sample,{integer.x,integer.y}));
                    }
                    floatCurveError=max(floatCurveError,f); quantCurveError=max(quantCurveError,q);
                }
            }
            updateMotion(simulatedTime,false);
        }
        CHECK(!isArmHeld());
        CHECK(maxZError<0.15f && floatCurveError<0.15f && quantCurveError<3.0f);
        std::cout<<"PATH kind="<<kind<<" feed="<<feed<<" floatZ="<<maxZError<<" floatCurve="<<floatCurveError<<" quantCurve="<<quantCurveError<<" maxDirectIK="<<directIK<<" maxDirectFK="<<directFK<<'\n';
    }
}
int main() {
    measuredConfig=hardwareConfig; for(int i=0;i<4;++i) measuredJoints[i]=joints[i];
    std::cout<<std::setprecision(8);
    #ifndef MINIMAL_BASELINE
    // An unchanged integer pulse still acknowledges the continuous goal.
    reset(); Pose exact=commandedPose(); size_t n=servoWrites.size();
    CHECK(requestPose(exact)); CHECK(applyPendingPose());
    CHECK(!hasPendingPose() && servoWrites.size()==n);
    Pose fraction=exact; fraction.angles[Base]+=.25f;
    CHECK(requestPose(fraction)); CHECK(applyPendingPose());
    CHECK(goalPose().angles[Base]==fraction.angles[Base]);
    CHECK(commandedPose().angles[Base]==exact.angles[Base] && servoWrites.size()==n);
    fraction.angles[Base]+=1;
    CHECK(requestPose(fraction)); CHECK(applyPendingPose()); CHECK(servoWrites.size()==n+4);
    #endif
    trace(15); trace(25); allPaths();
    std::cout<<"PASS minimal executor "<<assertions<<" assertions\n";
}
