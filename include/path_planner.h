#pragma once
#include "teaching.h"
enum class ShapeId : uint8_t { N, Triangle, Z, V };
uint8_t shapeVertexCount(ShapeId shape);
bool shapeVertex(ShapeId shape,uint8_t index,const TeachPoint& low,const TeachPoint& high,TeachPoint& out);
bool centripetalSample(const TeachPoint& p0,const TeachPoint& p1,const TeachPoint& p2,const TeachPoint& p3,float t,TeachPoint& out);
float pointDistance(const TeachPoint& a,const TeachPoint& b);
