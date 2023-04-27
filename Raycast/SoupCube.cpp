#include "SoupCube.h"

namespace Physics {

    static Vector3 s_cubeVert[] = {
        Vector3(-10.0f, -10.0f, -10.0f),
        Vector3(10.0f, -10.0f, -10.0f),
        Vector3(10.0f, 10.0f, -10.0f),
        Vector3(-10.0f, 10.0f, -10.0f),
        Vector3(-10.0f, -10.0f, 10.0f),
        Vector3(10.0f, -10.0f, 10.0f),
        Vector3(10.0f, 10.0f, 10.0f),
        Vector3(-10.0f, 10.0f, 10.0f),
    };
    static int s_cubeIndex[] = {
        // bottom
        0, 2, 1,
        0, 3, 2,

        // top
        4, 5, 6,
        4, 6, 7,

        // back
        0, 1, 5,
        0, 5, 4,
        
        // front
        2, 3, 7,
        2, 7, 6,

        // left
        0, 4, 7,
        0, 7, 3,

        // right
        1, 2, 6,
        1, 6, 5
    };

    TriangleSoup g_cubeSoup(MAKE_SOUP(s_cubeVert, s_cubeIndex));
}