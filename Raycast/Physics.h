#pragma once
#include "Math.h"

namespace Physics 
{
    class AABB {
    public:
        Vector3 mMin;
        Vector3 mMax;
        AABB(Vector3 min, Vector3 max);
    };

    class LineSegment {
    public:
        Vector3 mFrom;
        Vector3 mTo;
        LineSegment(const Vector3* from, const Vector3* to);
    };

    class Plane {
    public:
        Vector3 mNormal;
        float mD;
        Plane(Vector3 point, Vector3 normal);
        Plane(Vector3 normal, float d);
    };

    class Triangle {
    public:
        Vector3 mPoints[3];
        Vector3 GetNormal() const;
        Plane GetPlane() const;
        bool IsPointInside(const Vector3& p) const;
        Triangle(const Vector3& a, const Vector3& b, const Vector3& c);
    };

    class CastInfo {
    public:
        Vector3 mPoint;
        Vector3 mNormal;
    };

    bool Intersect(const LineSegment& line, const Plane& p, CastInfo* info=nullptr);
    bool Intersect(const LineSegment& line, const Triangle& b, CastInfo* info = nullptr);
};
