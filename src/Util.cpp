#include "Util.h"

#include "Typedefs.h"
#include "MeshData.h"

#include <array>
#include <sstream>

bool parseTtg(const std::string& input,
              MeshData& outMesh)
{
    std::vector<Vector3> vertices;
    std::vector<std::vector<UnsignedInt>> meshElementIndices;
    std::vector<UnsignedInt> boundaryIndices;

    std::stringstream stream(input);

    UnsignedInt dim;
    UnsignedInt vertexCount;
    UnsignedInt meshElementCount;
    UnsignedInt boundaryCount;

    // First line should have dimension
    std::string c;
    stream >> c;
    if (c != std::string{'d'})
        return false;

    stream >> dim;

    // Second line should have vertex count
    stream >> c;
    if (c != std::string{'v'})
        return false;

    stream >> vertexCount;

    // Then the elements
    stream >> c;
    if (c != std::string{'t'})
        return false;

    stream >> meshElementCount;

    stream >> c;
    if (c != std::string{'b'})
        return false;

    stream >> boundaryCount;

    Magnum::Debug{} << "Reading ttg with " << vertexCount << " vertices, " << meshElementCount << " elements" << "and "
            << boundaryCount << " boundary nodes.";

    // Read vertex coordinates
    for (UnsignedInt i = 0; i < vertexCount; ++i)
    {
        std::array<Float, 3> vi{{0.f, 0.f, 0.f}};

        for (UnsignedInt j = 0; j < dim; ++j)
        {
            stream >> vi[j];

            if (!stream.good())
                return false;
        }

        const Vector3 v{vi[0], vi[1], vi[2]};
        vertices.push_back(v);
    }

    // Read mesh element indices
    for (UnsignedInt i = 0; i < meshElementCount; ++i)
    {
        std::array<UnsignedInt, 4> ti;

        for (UnsignedInt j = 0; j < dim + 1; ++j)
        {
            stream >> ti[j];

            if (!stream.good())
                return false;
        }

        meshElementIndices.push_back({ti.begin(), ti.begin() + dim + 1});
    }

    for (UnsignedInt i = 0; i < boundaryCount; ++i)
    {
        UnsignedInt ei;
        stream >> ei;
        if (!stream.good())
            return false;
        boundaryIndices.push_back(ei);
    }

    outMesh = MeshData(dim, std::move(vertices), std::move(meshElementIndices), std::move(boundaryIndices));

    return true;
}

bool createUVIndices(const std::vector<UnsignedInt>& triangleIndices,
                     std::vector<Vector2>& outUv,
                     std::vector<UnsignedInt>& outUvIndices)
{
    std::vector<Vector2> uv{{0.0f, 0.0f},
                            {1.0f, 0.0f},
                            {0.0f, 1.0f}};

    std::vector<UnsignedInt> uvIndices;

    for (UnsignedInt i = 0; i < triangleIndices.size(); i += 3)
    {
        for (UnsignedInt j = 0; j < 3; ++j)
        {
            uvIndices.push_back(j);
        }
    }

    outUv = uv;
    outUvIndices = uvIndices;

    return true;
}

std::vector<UnsignedInt> extractTriangleIndices(const std::vector<std::vector<UnsignedInt>>& tetrahedronIndices)
{
    std::vector<UnsignedInt> triangleIndices;

    for (auto& currentIndices : tetrahedronIndices)
    {
        const UnsignedInt i0 = currentIndices[0];
        const UnsignedInt i1 = currentIndices[1];
        const UnsignedInt i2 = currentIndices[2];
        const UnsignedInt i3 = currentIndices[3];

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

    return triangleIndices;
}

Eigen::Vector3f toEigen(const Vector3& v)
{
    return Eigen::Vector3f(v[0], v[1], v[2]);
}

Vector3 valToColor(const Float val)
{
    if (val <= 0.5f)
        return Vector3(1.f, 1.f, 0.f) * val * 2.0f + Vector3(0.f, 0.f, 1.f) * (0.5f - val) * 2.0f;
    else
        return Vector3(1.f, 0.f, 0.f) * (val - 0.5f) * 2.0f + Vector3(1.f, 1.f, 0.f) * (1.0f - val) * 2.0f;
}

std::vector<Vector3> valuesToHeatGradient(const std::vector<Float>& vals)
{
    std::vector<Vector3> colors;
    colors.reserve(vals.size());

    if (vals.size() == 0)
        return colors;

    const Float maxValInv = 1.f / *std::max_element(vals.begin(), vals.end());

    for (Float val : vals)
    {
        colors.push_back(valToColor(val * maxValInv));
    }

    return colors;
}

std::vector<Float> computeNorm(const std::vector<Eigen::Vector3f>& input)
{
    std::vector<Float> norms(input.size());

    UnsignedInt i = 0;
    for (auto& e : input)
    {
        norms[i] = e.norm();
        ++i;
    }

    return norms;
}

std::vector<Vector2i> bresenhamL(const Vector2i a, const Vector2i b);
std::vector<Vector2i> bresenhamL(const Vector2i a, const Vector2i b)
{
    std::vector<Vector2i> output;

    Vector2i delta = b - a;
    Int ys = Magnum::Math::sign(delta.y());
    delta.y() = abs(delta.y());

    Int err = 2 * delta.y() - delta.x();
    Int y = a.y();

    for (Int x = a.x(); x <= b.x(); ++x)
    {
        output.push_back({x, y});

        if (err > 0)
        {
            y += ys;
            err -= 2 * delta.x();
        }
        err += 2 * delta.y();
    }

    return output;
}

std::vector<Vector2i> bresenhamH(const Vector2i a, const Vector2i b);
std::vector<Vector2i> bresenhamH(const Vector2i a, const Vector2i b)
{
    std::vector<Vector2i> output;

    Vector2i delta = b - a;
    Int xs = Magnum::Math::sign(delta.x());
    delta.x() = abs(delta.x());

    Int err = 2 * delta.x() - delta.y();
    Int x = a.x();

    for (Int y = a.y(); y <= b.y(); ++y)
    {
        output.push_back({x, y});

        if (err > 0)
        {
            x += xs;
            err -= 2 * delta.y();
        }
        err += 2 * delta.x();
    }

    return output;
}

std::vector<Vector2i> bresenham(const Vector2i a, const Vector2i b)
{
    if (abs(b.y() - a.y()) < abs(b.x() - a.x()))
    {
        if (a.x() > b.x())
            return bresenhamL(b, a);
        else
            return bresenhamL(a, b);
    }
    else
    {
        if (a.y() > b.y())
            return bresenhamH(b, a);
        else
            return bresenhamH(a, b);
    }
}
