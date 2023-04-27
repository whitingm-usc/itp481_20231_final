#include "Physics.h"

namespace Physics {

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // LineSegment
    ///////////////////////////////////////////////////////////////////////////////////////////////
    LineSegment::LineSegment(const Vector3& from, const Vector3& to)
        : mFrom(from)
        , mTo(to)
    {}

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Plane
    ///////////////////////////////////////////////////////////////////////////////////////////////
    Plane::Plane(const Vector3& point, const Vector3& normal)
        : mNormal(normal)
    {
        mD = -Vector3::Dot(point, mNormal);
    }

    Plane::Plane(const Vector3& normal, float d)
        : mNormal(normal)
        , mD(d)
    {}

    /// <summary>
    /// Cast the LineSegment across the Plane and return true if it intersects
    /// LineSegments coming from behind the Plane do not count as intersections
    /// </summary>
    /// <param name="line">the LineSegment to check against the Plane</param>
    /// <param name="info">OPTIONAL if there is an intersection, info will be filled it</param>
    /// <returns>true if the LineSegment hits the Plane</returns>
    bool Plane::RayCast(const LineSegment& line, CastInfo* info) const
    {
        Vector3 v = line.mTo - line.mFrom;
        float d = Vector3::Dot(v, mNormal);
        if (Math::NearZero(d))
            return false;   // parallel
        if (d > 0.0f)
            return false;   // backwards
        float t = -Vector3::Dot(line.mFrom, mNormal) - mD;
        t = t / d;
        if (t < 0.0f || t > 1.0f)
            return false;

        if (nullptr != info)
        {
            info->mNormal = mNormal;
            info->mPoint = line.mFrom + v * t;
            info->mFraction = t;
        }

        return true;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Triangle
    ///////////////////////////////////////////////////////////////////////////////////////////////
    Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c)
    {
        mPoints[0] = a;
        mPoints[1] = b;
        mPoints[2] = c;
        GetPlane();
    }

    /// <summary>
    /// Calculate and return the normal of the Triangle
    /// </summary>
    /// <returns>the normal of the Triangle</returns>
    Vector3 Triangle::GetNormal() const
    {
        Vector3 ab = mPoints[1] - mPoints[0];
        Vector3 ac = mPoints[2] - mPoints[0];
        Vector3 n = Vector3::Cross(ab, ac);
        return Vector3::Normalize(n);
    }

    /// <summary>
    /// Calculate and return the plane the Triangle is within
    /// </summary>
    /// <returns>the plane of the Triangle</returns>
    Plane Triangle::GetPlane() const
    {
        return Plane(mPoints[0], GetNormal());
    }

    /// <summary>
    /// Cast the LineSegment across the Triangle and return true if it intersects
    /// LineSegments coming from behind the Triangle do not count as intersections
    /// </summary>
    /// <param name="line">the LineSegment to check against the Triangle</param>
    /// <param name="info">OPTIONAL if there is an intersection, info will be filled it</param>
    /// <returns>true if the LineSegment hits the Triangle</returns>
    bool Triangle::RayCast(const LineSegment& line, CastInfo* info) const
    {
        Plane p = GetPlane();
        CastInfo tempInfo;
        if (p.RayCast(line, &tempInfo))
        {
            if (IsPointInside(tempInfo.mPoint))
            {
                if (info)
                    *info = tempInfo;
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Returns true if the point "p" is inside the triangle or false if not.
    /// This function ASSUMES point "p" is in the plane of the triangle, and results are undefined if it is not.
    /// </summary>
    /// <param name="p">The point to test</param>
    /// <returns>true if p is inside the triangle</returns>
    bool Triangle::IsPointInside(const Vector3& p) const
    {
        Vector3 n = GetNormal();

        {
            Vector3 ab = mPoints[1] - mPoints[0];
            Vector3 ap = p - mPoints[0];
            Vector3 c = Vector3::Cross(ab, ap);
            if (Vector3::Dot(c, n) < 0.0f)
                return false;
        }

        {
            Vector3 bc = mPoints[2] - mPoints[1];
            Vector3 bp = p - mPoints[1];
            Vector3 c = Vector3::Cross(bc, bp);
            if (Vector3::Dot(c, n) < 0.0f)
                return false;
        }

        {
            Vector3 ca = mPoints[0] - mPoints[2];
            Vector3 cp = p - mPoints[2];
            Vector3 c = Vector3::Cross(ca, cp);
            if (Vector3::Dot(c, n) < 0.0f)
                return false;
        }

        return true;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // TriangleSoup
    ///////////////////////////////////////////////////////////////////////////////////////////////
    TriangleSoup::TriangleSoup(int vertCount, Vector3* pVerts, int numTri, int* pIndices)
        : mTriCount(numTri)
    {
        mTris = new Triangle[mTriCount]();
        for (int i = 0; i < numTri; ++i)
        {
            for (int j = 0; j < 3; ++j)
                mTris[i].mPoints[j] = pVerts[pIndices[i*3 + j]];
        }
    }

    TriangleSoup::~TriangleSoup()
    {
        delete[] mTris;
    }

    /// <summary>
    /// Cast the LineSegment across the soup and return true if it intersects
    /// LineSegments coming from behind the soup do not count as intersections
    /// </summary>
    /// <param name="line">the LineSegment to check against the soup</param>
    /// <param name="info">OPTIONAL if there is an intersection, info will be filled it</param>
    /// <returns>true if the LineSegment hits the soup</returns>
    bool TriangleSoup::RayCast(const LineSegment& line, CastInfo* info) const
    {
#ifdef TRISOUP_AABB
#if 0
        AABB lineAABB(line.mFrom, line.mFrom);
        lineAABB.AddPoint(line.mTo);

        if (false == mAABB.Intersect(lineAABB))
            return false;
#elif 0
        if (false == mAABB.Intersect(line))
            return false;
#endif
#endif
        bool hit = false;
        CastInfo tempInfo;
        CastInfo bestInfo;
        bestInfo.mFraction = 100.0f;

        for (int i = 0; i < mTriCount; ++i)
        {
            const Triangle& tri = mTris[i];
            if (tri.RayCast(line, &tempInfo))
            {
                hit = true;
                if (tempInfo.mFraction < bestInfo.mFraction)
                    bestInfo = tempInfo;
            }
        }
        if (hit && info)
        {
            *info = bestInfo;
        }
        return hit;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // SoupObj
    ///////////////////////////////////////////////////////////////////////////////////////////////
    SoupObj::SoupObj()
        : mSoup(nullptr)
    {}

    SoupObj::SoupObj(const TriangleSoup* pSoup, const Matrix4& obj2World)
        : mSoup(pSoup)
        , mObj2World(obj2World)
    {}

    /// <summary>
    /// Cast the LineSegment across the soup and return true if it intersects
    /// Note: the LineSegment could hit multiple Triangles in the Soup. In that case, the one closest to the start point of the segment will be returned.
    /// </summary>
    /// <param name="line">the LineSegment to check against the soup</param>
    /// <param name="info">OPTIONAL if there is an intersection, info will be filled it</param>
    /// <returns>true if the LineSegment hits the soup</returns>
    bool SoupObj::RayCast(const LineSegment& line, CastInfo* info) const
    {
        Matrix4 world2Obj = mObj2World;
        world2Obj.Invert();
        LineSegment tempSegment(line);
        tempSegment.mFrom = Vector3::Transform(tempSegment.mFrom, world2Obj);
        tempSegment.mTo = Vector3::Transform(tempSegment.mTo, world2Obj);
        bool ret = mSoup->RayCast(tempSegment, info);
        if (ret && info)
        {
            info->mNormal = Vector3::Transform(info->mNormal, mObj2World, 0.0f);
            info->mNormal.Normalize();
            info->mPoint = Vector3::Transform(info->mPoint, mObj2World, 1.0f);
        }
        return ret;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // World
    ///////////////////////////////////////////////////////////////////////////////////////////////
    World::World()
    {}

    World::~World()
    {}

    void World::AddObj(const SoupObj& obj)
    {
        mObj.push_back(obj);
    }

    /// <summary>
    /// Cast the LineSegment across the World and return true if it intersects anything
    /// Note: the LineSegment could hit multiple objext in the World. In that case, the one closest to the start point of the segment will be returned.
    /// </summary>
    /// <param name="line">the LineSegment to check against the World</param>
    /// <param name="info">OPTIONAL if there is an intersection, info will be filled it</param>
    /// <returns>true if the LineSegment hits the anything in the World</returns>
    bool World::RayCast(const LineSegment& line, CastInfo* info) const
    {
        bool hit = false;
        CastInfo tempInfo;
        CastInfo bestInfo;
        bestInfo.mFraction = 100.0f;

        for (const SoupObj& obj : mObj)
        {
            if (obj.RayCast(line, &tempInfo))
            {
                hit = true;
                if (tempInfo.mFraction < bestInfo.mFraction)
                    bestInfo = tempInfo;
            }
        }
        if (hit && info)
        {
            *info = bestInfo;
        }
        return hit;
    }
}