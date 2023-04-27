#include "UnitTest.h"
#include "Physics.h"
#include <assert.h>

namespace Physics
{
    struct LineVsPlane {
        LineSegment mLine;
        Plane mPlane;
        bool mShouldHit;
        Vector3 mCorrectNormal;
        Vector3 mCorrectPosition;
    };
    static LineVsPlane s_lineVsPlane[] = {
        {   // hit down z-axis
            { Vector3(0.0f, 0.0f, 100.0f), Vector3(0.0f, 0.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 1.0f) },
            true,
            Vector3(0.0, 0.0f, 1.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // stops short down z-axis
            { Vector3(0.0f, 0.0f, 100.0f), Vector3(0.0f, 0.0f, 10.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 1.0f) },
            false,
            Vector3(0.0, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // starts late down z-axis
            { Vector3(0.0f, 0.0f, -10.0f), Vector3(0.0f, 0.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 1.0f) },
            false,
            Vector3(0.0, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // backwards up z-axis
            { Vector3(0.0f, 0.0f, -100.0f), Vector3(0.0f, 0.0f, 100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 1.0f) },
            false,
            Vector3(0.0, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // hit diagonal
            { Vector3(-100.0f, 50.0f, 100.0f), Vector3(100.0f, -50.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(-0.192f, 0.192f, 0.962f) },
            true,
            Vector3(-0.192f, 0.192f, 0.962f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
    };

    bool TestLineVsPlane(const LineVsPlane& test)
    {
        CastInfo info;
        bool result = Intersect(test.mLine, test.mPlane, &info);
        if (test.mShouldHit != result)
        {
            return false;
        }
        if (result)
        {
            if (false == Math::CloseEnough(info.mNormal, test.mCorrectNormal))
            {
                return false;
            }
            if (false == Math::CloseEnough(info.mPoint, test.mCorrectPosition))
            {
                return false;
            }
        }
        return true;
    }

    struct Tri2Plane {
        Triangle mTriangle;
        Plane mPlane;
    };

    static Tri2Plane s_testTri2Plane[] = {
        {   // flat on xy axis
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(100.0f, 100.0f, 0.0f) },
            { Vector3(0.0f, 0.0f, 1.0f), 0.0f }
        },
        {   // flat on xz axis
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 100.0f) },
            { Vector3(0.0f, -1.0f, 0.0f), 0.0f }
        },
        {   // flat on yz axis
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 100.0f, 100.0f) },
            { Vector3(0.0f, 0.0f, 1.0f), 0.0f }
        },
    };

    struct PointInTri {
        Triangle mTriangle;
        Vector3 mPoint;
        bool mIsInside;
    };

    static PointInTri s_testPointInTri[] = {
        // xy plane
        {   // inside
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            Vector3(20.0f, 20.0f, 0.0f),
            true
        },
        {   // -x
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            Vector3(-20.0f, 20.0f, 0.0f),
            false
        },
        {   // +x
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            Vector3(100.0f, 20.0f, 0.0f),
            false
        },
        {   // -y
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            Vector3(20.0f, -20.0f, 0.0f),
            false
        },
        {   // +y
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            Vector3(20.0f, 100.0f, 0.0f),
            false
        },
        // xz plane
        {   // inside
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 100.0f) },
            Vector3(20.0f, 0.0f, 20.0f),
            true
        },
        {   // -x
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 100.0f) },
            Vector3(-20.0f, 0.0f, 20.0f),
            false
        },
        {   // +x
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 100.0f) },
            Vector3(100.0f, 0.0f, 20.0f),
            false
        },
        {   // -z
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 100.0f) },
            Vector3(20.0f, 0.0f, -20.0f),
            false
        },
        {   // +z
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 100.0f) },
            Vector3(20.0f, 0.0f, 100.0f),
            false
        },
    };

    struct LineVsTri {
        LineSegment mLine;
        Triangle mTri;
        bool mShouldHit;
        Vector3 mCorrectNormal;
        Vector3 mCorrectPosition;
    };

    static LineVsTri s_testLineVTri[] = {
        {   // tri in xy plane, line on z-axis hits it
            { Vector3(20.0f, 20.0f, 100.0f), Vector3(20.0f, 20.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            true,
            Vector3(0.0f, 0.0f, 1.0f),
            Vector3(20.0f, 20.0f, 0.0f)
        },
        {   // tri in xy plane, line on z-axis backwards it
            { Vector3(20.0f, 20.0f, -100.0f), Vector3(20.0f, 20.0f, 100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            false,
            Vector3(0.0f, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // tri in xy plane, line on x-axis parallel
            { Vector3(-20.0f, 20.0f, 0.0f), Vector3(20.0f, 20.0f, 0.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            false,
            Vector3(0.0f, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // tri in xy plane, line on y-axis parallel
            { Vector3(20.0f, -20.0f, 0.0f), Vector3(20.0f, 20.0f, 0.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            false,
            Vector3(0.0f, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // tri in xy plane, line on z-axis misses on -x
            { Vector3(-20.0f, 20.0f, 100.0f), Vector3(-20.0f, 20.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            false,
            Vector3(0.0f, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // tri in xy plane, line on z-axis misses on +x
            { Vector3(100.0f, 20.0f, 100.0f), Vector3(100.0f, 20.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            false,
            Vector3(0.0f, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // tri in xy plane, line on z-axis misses on -y
            { Vector3(20.0f, -20.0f, 100.0f), Vector3(20.0f, -20.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            false,
            Vector3(0.0f, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },
        {   // tri in xy plane, line on z-axis misses on +y
            { Vector3(20.0f, 100.0f, 100.0f), Vector3(20.0f, 100.0f, -100.0f) },
            { Vector3(0.0f, 0.0f, 0.0f), Vector3(100.0f, 0.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f) },
            false,
            Vector3(0.0f, 0.0f, 0.0f),
            Vector3(0.0f, 0.0f, 0.0f)
        },    
    };

    bool TestLineVsTri(const LineVsTri& test)
    {
        CastInfo info;
        bool result = Intersect(test.mLine, test.mTri, &info);
        if (test.mShouldHit != result)
        {
            return false;
        }
        if (result)
        {
            if (false == Math::CloseEnough(info.mNormal, test.mCorrectNormal))
            {
                return false;
            }
            if (false == Math::CloseEnough(info.mPoint, test.mCorrectPosition))
            {
                return false;
            }
        }
        return true;
    }

    bool UnitTest()
    {
        bool result = true;
        {   // line vs plane
            for (const LineVsPlane& test : s_lineVsPlane)
            {
                bool ret = TestLineVsPlane(test);
                assert(ret);
                result &= ret;
            }
        }

        {   // triangle 2 plane
            for (const Tri2Plane& test : s_testTri2Plane)
            {
                Plane p = test.mTriangle.GetPlane();
                bool ret = Math::CloseEnough(p.mNormal, test.mPlane.mNormal);
                ret |= Math::NearZero(p.mD - test.mPlane.mD);
                assert(ret);
                result &= ret;
            }
        }

        {   // point in triangle
            for (const PointInTri& test : s_testPointInTri)
            {
                bool ret = test.mTriangle.IsPointInside(test.mPoint);
                ret = ret == test.mIsInside;
                assert(ret);
                result &= ret;
            }
        }

        {   // line vs triangle
            for (const LineVsTri& test : s_testLineVTri)
            {
                bool ret = TestLineVsTri(test);
                assert(ret);
                result &= ret;
            }
        }

        return result;
    }
}