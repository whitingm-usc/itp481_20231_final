#pragma once
#include "Physics.h"

#define MAKE_SOUP(vert, index) ARRAY_SIZE(vert), vert, ARRAY_SIZE(index)/3, index

namespace Physics {
    extern Physics::TriangleSoup g_cubeSoup;
}