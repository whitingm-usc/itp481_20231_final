#include "SpeedTest.h"
#include "Physics.h"
#include "Random.h"
#include "SoupCube.h"
#include <chrono>

const int NUM_OBJ = 10000;
const int NUM_RAY = 5000;
const float WORLD_RADIUS = 10000.0f;
const float MIN_SCALE = 0.1f;
const float MAX_SCALE = 100.0f;

Matrix4 RandomMatrix()
{
    Vector3 pos = WORLD_RADIUS * Random::GetVector(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
    Vector3 euler = Random::GetVector(Vector3(-Math::Pi, -Math::Pi, -Math::Pi), Vector3(Math::Pi, Math::Pi, Math::Pi));
    Vector3 scale = WORLD_RADIUS * Random::GetVector(Vector3(MIN_SCALE, MIN_SCALE, MIN_SCALE), Vector3(MAX_SCALE, MAX_SCALE, MAX_SCALE));
    Matrix4 mat = Matrix4::CreateScale(scale)
        * Matrix4::CreateRotationX(euler.x)
        * Matrix4::CreateRotationY(euler.y)
        * Matrix4::CreateRotationZ(euler.z)
        * Matrix4::CreateTranslation(pos);
    return mat;
}
    
Physics::LineSegment RandomLine()
{
    Vector3 from = WORLD_RADIUS * Random::GetVector(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
    Vector3 to = WORLD_RADIUS * Random::GetVector(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
    Physics::LineSegment line(from, to);
    return line;
}

float SpeedTest()
{
    Random::Init();
    Random::Seed(0x1337);
    Physics::World world;
    for (int i = 0; i < NUM_OBJ; ++i)
    {
        Matrix4 randMat = RandomMatrix();
        world.AddObj(Physics::SoupObj(&Physics::g_cubeSoup, randMat));
    }
    Physics::LineSegment* pLine = new Physics::LineSegment[NUM_RAY];
    for (int i = 0; i < NUM_RAY; ++i)
    {
        pLine[i] = RandomLine();
    }

	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    Physics::CastInfo info;
    for (int i = 0; i < NUM_RAY; ++i)
    {
        world.RayCast(pLine[i], &info);
    }

	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	float time = (float)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    delete[] pLine;

    return time;
}