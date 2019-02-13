#include "Util.h"

#include <sstream>

bool parseTtg(const std::string& input, std::vector<Vector3>& outVertices, std::vector<Vector2>& outUv, std::vector<UnsignedInt>& outTriangleIndices, std::vector<UnsignedInt>& outUvIndices, std::vector<UnsignedInt>& outTetrahedronIndices)
{
    std::vector<Vector3> vertices;
    std::vector<Vector2> uv{ {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}};
    std::vector<UnsignedInt> triangleIndices;
    std::vector<UnsignedInt> uvIndices;
    std::vector<UnsignedInt> tetrahedronIndices;

    std::stringstream stream(input);

    UnsignedInt vertexCount;
    UnsignedInt tetrahedronCount;

    // First line should have vertex count
    std::string c;
    stream >> c;
    if (c != std::string{'v'})
        return false;

    stream >> vertexCount;

    // Then the tetrahedrons
    stream >> c;
    if (c != std::string{'t'})
        return false;

    stream >> tetrahedronCount;
    Debug{} << "Reading ttg with " << vertexCount << " vertices" << "and " << tetrahedronCount << " tetrahedrons";

    for (UnsignedInt i = 0; i < vertexCount; ++i)
    {
        Float v0, v1, v2;
        stream >> v0 >> v1 >> v2;

        const Vector3 v{v0, v1, v2};

        vertices.push_back(v);
    }

    for (UnsignedInt i = 0; i < tetrahedronCount; ++i)
    {
        UnsignedInt t0, t1, t2, t3;
        stream >> t0 >> t1 >> t2 >> t3;

        for (UnsignedInt j = 0; j < 4; ++j)
        {
            uvIndices.push_back(0);
            uvIndices.push_back(1);
            uvIndices.push_back(2);
        }

        tetrahedronIndices.push_back(t0);
        tetrahedronIndices.push_back(t1);
        tetrahedronIndices.push_back(t2);
        tetrahedronIndices.push_back(t3);
    }


    extractTriangleIndices(tetrahedronIndices, triangleIndices);

    outVertices = vertices;
    outUv = uv;
    outTriangleIndices = triangleIndices;
    outUvIndices = uvIndices;
    outTetrahedronIndices = tetrahedronIndices;

    return true;
}

bool extractTriangleIndices(const std::vector<UnsignedInt>& tetrahedronIndices, std::vector<UnsignedInt>& triangleIndices)
{
    if (tetrahedronIndices.size() % 4 != 0)
    {
        Debug{} << "Wrong number of tetrahedron indices";
        return false;
    }

    for (UnsignedInt i = 0; i < tetrahedronIndices.size(); i+=4)
    {
        const UnsignedInt i0 = tetrahedronIndices[i];
        const UnsignedInt i1 = tetrahedronIndices[i+1];
        const UnsignedInt i2 = tetrahedronIndices[i+2];
        const UnsignedInt i3 = tetrahedronIndices[i+3];

        triangleIndices.push_back(i0);
        triangleIndices.push_back(i1);
        triangleIndices.push_back(i2);

        triangleIndices.push_back(i1);
        triangleIndices.push_back(i2);
        triangleIndices.push_back(i3);

        triangleIndices.push_back(i2);
        triangleIndices.push_back(i3);
        triangleIndices.push_back(i0);

        triangleIndices.push_back(i3);
        triangleIndices.push_back(i0);
        triangleIndices.push_back(i1);
    }

    return true;
}

void computeAABB(const std::vector<Vector3>& vertices, Vector3& origin, Vector3& extent)
{
    Vector3 min{std::numeric_limits<Float>::max()};
    Vector3 max{std::numeric_limits<Float>::min()};

    for (const auto v : vertices)
    {
        min = Math::min(v,min);
        max = Math::max(v,max);
    }

    const Vector3 diff = max - min;
    origin = min + 0.5*diff;
    extent = 0.5*diff;
}