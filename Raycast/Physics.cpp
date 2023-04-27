#include "Physics.h"

namespace Physics {
    bool Intersect(const LineSegment& line, const Plane& plane, CastInfo* info)
    {

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