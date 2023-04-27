#include "Physics.h"

namespace Physics {

    LineSegment::LineSegment(const Vector3& from, const Vector3& to)
        : mFrom(from)
        , mTo(to)
    {}

    Plane::Plane(const Vector3& point, const Vector3& normal)
        : mNormal(normal)
    {
        mD = -Vector3::Dot(point, mNormal);
    }

    Plane::Plane(const Vector3& normal, float d)
        : mNormal(normal)
        , mD(d)
    {}

    Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c)
    {
        mPoints[0] = a;
        mPoints[1] = b;
        mPoints[2] = c;
    }

    Vector3 Triangle::GetNormal() const
    {
        Vector3 ab = mPoints[1] - mPoints[0];
        Vector3 ac = mPoints[2] - mPoints[0];
        Vector3 n = Vector3::Cross(ab, ac);
        return Vector3::Normalize(n);
    }

    Plane Triangle::GetPlane() const
    {
        Plane p(mPoints[0], GetNormal());
        return p;
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

    bool Intersect(const LineSegment& line, const Plane& plane, CastInfo* info)
    {
        Vector3 v = line.mTo - line.mFrom;
        float d = Vector3::Dot(v, plane.mNormal);
        if (Math::NearZero(d))
            return false;   // parallel
        if (d > 0.0f)
            return false;   // backwards
        float t = -Vector3::Dot(line.mFrom, plane.mNormal) - plane.mD;
        t = t / d;
        if (t < 0.0f || t > 1.0f)
            return false;

        if (nullptr != info)
        {
            info->mNormal = plane.mNormal;
            info->mPoint = line.mFrom + v * t;
        }

        return true;
    }

    bool Intersect(const LineSegment& line, const Triangle& b, CastInfo* info)
    {
        Plane p = b.GetPlane();
        CastInfo tempInfo;
        if (Intersect(line, p, &tempInfo))
        {
            if (b.IsPointInside(tempInfo.mPoint))
            {
                if (info)
                    *info = tempInfo;
                return true;
            }
        }
        return false;
    }
}