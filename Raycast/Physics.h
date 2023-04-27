#pragma once
#include "Math.h"
#include <vector>

//#define TRISOUP_AABB  // turns out this doesn't speed anything up

namespace Physics 
{
    /// <summary>
    /// When we do a RayCast, this will be used to return data about the point we hit with the ray
    /// </summary>
    class CastInfo {
    public:
        Vector3 mPoint;     // the point of intersection in world space
        Vector3 mNormal;    // the normal at the point of intersection in world space
        float mFraction;    // how far along the line segment is the intersection (range 0 to 1)
    };

    class AABB {
    public:
        Vector3 mMin;
        Vector3 mMax;

        AABB(const Vector3& _min, const Vector3& _max);

        void AddPoint(const Vector3& p);

        bool Intersect(const AABB& other) const;
        bool Intersect(const class LineSegment& line) const;
    };

    /// <summary>
    /// We'll be calling these things a Ray Cast, but it's really a Line Segment test
    /// You may add elements if you want to
    /// </summary>
    class LineSegment {
    public:
        Vector3 mFrom;
        Vector3 mTo;
        LineSegment() {}
        LineSegment(const Vector3& from, const Vector3& to);
    };

    /// <summary>
    /// Plane equation P dot mNormal + mD = 0
    /// You may add data if you want to
    /// </summary>
    class Plane {
    public:
        Vector3 mNormal;
        float mD;
        Plane() : mD(0.0f) {}
        Plane(const Vector3& point, const Vector3& normal);
        Plane(const Vector3& normal, float d);

        bool RayCast(const LineSegment& line, CastInfo* info=nullptr) const;
    };

    /// <summary>
    /// A triangle
    /// The constructor assumes you'll be giving 3 points
    /// You may add data if you want to
    /// </summary>
    class Triangle {
    public:
        Vector3 mPoints[3];
        Triangle() : mIsDirty(true) {}
        Triangle(const Vector3& a, const Vector3& b, const Vector3& c);

        Vector3 GetNormal() const;
        Plane GetPlane() const;
        bool IsPointInside(const Vector3& p) const;
    
        bool RayCast(const LineSegment& line, CastInfo* info=nullptr) const;

    private:
        mutable bool mIsDirty;
        mutable Plane mPlane;
    };

    /// <summary>
    /// A TriangleSoup is a collision mesh made out of a bunch of Triangle
    /// We cannot guarantee that the soup is entirely convex
    /// You may add data if you want to
    /// </summary>
    class TriangleSoup {
    public:
        TriangleSoup(int vertCount, Vector3* pVerts, int triCount, int* pIndices);
        ~TriangleSoup();

        bool RayCast(const LineSegment& line, CastInfo* info = nullptr) const;

    private:
        Triangle* mTris;
        int mTriCount;
#ifdef TRISOUP_AABB
        AABB mAABB;
#endif
    };

    /// <summary>
    /// A SoupObj is a TriangleSoup attached to a Matrix4.
    /// This is essentially an instance of a TriangleSoup... thus we have a POINTER to a soup
    /// It is presumed we'll re-use the same soup on multiple different SoupObj
    /// so do not delete that pointer in the destructor
    /// You may add data if you want to
    /// </summary>
    class SoupObj {
    public:
        const TriangleSoup* mSoup;
        Matrix4 mObj2World;

        SoupObj();
        SoupObj(const TriangleSoup* pSoup, const Matrix4& obj2World);

        bool RayCast(const LineSegment& line, CastInfo* info = nullptr) const;

    private:
        mutable bool mIsDirty;
        mutable Matrix4 mWorld2Obj;
    };

    /// <summary>
    /// The World is the container for all the SoupObj.
    /// You may add data if you want to
    /// </summary>
    class World {
    public:
        World();
        ~World();

        void AddObj(const SoupObj& obj);
        bool RayCast(const LineSegment& line, CastInfo* info = nullptr) const;

    private:
        std::vector<SoupObj> mObj;
    };
};
