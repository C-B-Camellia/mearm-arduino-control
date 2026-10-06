#pragma once

struct Point2D {
    float x;
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

FKResult solveFK(float shoulderAngle, float elbowAngle);
IKResult solveIK(float x, float z);

// Future base rotation and pencil offsets belong in a separately calibrated
// coordinate conversion. This module currently models the original x/z plane.
