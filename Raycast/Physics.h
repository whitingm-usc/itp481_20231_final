#pragma once
#include "Math.h"

namespace Physics 
{
    class CastInfo {
    public:
        Vector3 mPoint;
        Vector3 mNormal;
        float mFraction;
    };

    class LineSegment {
    public:
        Vector3 mFrom;
        Vector3 mTo;
        LineSegment(const Vector3& from, const Vector3& to);
    };

    class Plane {
    public:
        Vector3 mNormal;
        float mD;
        Plane(const Vector3& point, const Vector3& normal);
        Plane(const Vector3& normal, float d);

        bool RayCast(const LineSegment& line, CastInfo* info=nullptr) const;
    };

    class Triangle {
    public:
        Vector3 mPoints[3];
        Triangle() {}
        Triangle(const Vector3& a, const Vector3& b, const Vector3& c);

        Vector3 GetNormal() const;
        Plane GetPlane() const;
        bool IsPointInside(const Vector3& p) const;
    
        bool RayCast(const LineSegment& line, CastInfo* info=nullptr) const;
    };

    class TriangleSoup {
    public:
        TriangleSoup(int vertCount, Vector3* pVerts, int triCount, int* pIndices);
        ~TriangleSoup();

        bool RayCast(const LineSegment& line, CastInfo* info = nullptr) const;

    private:
        Triangle* mTris;
        int mTriCount;
    };

    class SoupObj {
    public:
        const TriangleSoup* mSoup;
        Matrix4 mObj2World;

        SoupObj(const TriangleSoup* pSoup, const Matrix4& obj2World);

        bool RayCast(const LineSegment& line, CastInfo* info = nullptr) const;
    };
};
