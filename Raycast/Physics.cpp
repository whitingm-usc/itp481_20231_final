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
        // TODO
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Triangle
    ///////////////////////////////////////////////////////////////////////////////////////////////
    Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c)
    {
        mPoints[0] = a;
        mPoints[1] = b;
        mPoints[2] = c;
    }

    /// <summary>
    /// Calculate and return the normal of the Triangle
    /// </summary>
    /// <returns>the normal of the Triangle</returns>
    Vector3 Triangle::GetNormal() const
    {
        // TODO
        return Vector3::Zero;
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
        // TODO
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
        // TODO
        return false;
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
        // TODO
        return false;
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
        // TODO
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // World
    ///////////////////////////////////////////////////////////////////////////////////////////////
    World::World()
    {}

    World::~World()
    {}

    /// <summary>
    /// Add an object to the world.
    /// Feel free to edit this function if you want to
    /// </summary>
    /// <param name="obj">the object being added</param>
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
        // TODO
        return false;
    }
}