#pragma once

struct Point2D {
    float x;
    float z;
};

struct Point3D {
    float x;
    float y;
    float z;
};

struct FKResult {
    float alpha;
    float beta;
    float gamma;
    Point2D position;
};

struct IKResult {
    bool valid;
    float alpha;
    float beta;
    float gamma;
    float j1;
    float j2;
};

struct IKResult3D {
    bool valid;
    float j0;
    float j1;
    float j2;
};

FKResult solveFK(float shoulderAngle, float elbowAngle);
IKResult solveIK(float x, float z);

// Future base rotation and pencil offsets belong in a separately calibrated
// coordinate conversion. This module currently models the original x/z plane.
Point3D solveFK3D(float j0, float j1, float j2);

IKResult3D solveIK3D(const Point3D& target);
