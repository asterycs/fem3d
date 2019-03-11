#ifndef FEM3D_UTIL_H
#define FEM3D_UTIL_H

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Vector.h>

#include "Eigen/Dense"

#include <string>
#include <vector>

using namespace Magnum;

template <typename V>
struct AABB
{
  V min;
  V max;
};

bool parseTtg(const std::string& input, std::vector<Vector3>& outVertices,
              std::vector<std::vector<UnsignedInt>>& outMeshElementIndices,
              std::vector<UnsignedInt>& outboundaryIndices, UnsignedInt& outDim);
bool createUVIndices(const std::vector<UnsignedInt>& triangleIndices, std::vector<Vector2>& outUv,
                     std::vector<UnsignedInt>& outUvIndices);

std::vector<UnsignedInt> extractTriangleIndices(const std::vector<std::vector<UnsignedInt>>& tetrahedronIndices);

std::vector<Vector2i> bresenham(const Vector2i a, const Vector2i b);


Vector3 valToColor(const Float val);
std::vector<Vector3> valuesToHeatGradient(const std::vector<Float>& vals);
std::vector<Float> computeNorm(const std::vector<Eigen::Vector3f>& input);

template <class V>
AABB<V> computeAABB(const std::vector<V>& elements)
{
    V min {std::numeric_limits<typename V::Type>::max()};
    V max {std::numeric_limits<typename V::Type>::min()};

    for (auto element : elements)
    {
        min = Math::min(min, element);
        max = Math::max(max, element);
    }

    return AABB<V>{min, max};
}

template<typename T>
std::vector<T> repeat(const std::vector<T>& values, const UnsignedInt times)
{
    std::vector<T> res;
    res.reserve(values.size() * times);

    for (const T value : values)
    {
        for (UnsignedInt j = 0; j < times; ++j)
            res.push_back(value);
    }

    return res;
}

template<typename T>
std::vector<T> expand(const std::vector<T>& values, std::vector<UnsignedInt>& indices)
{
    std::vector<T> out;

    std::transform(indices.begin(), indices.end(), std::back_inserter(out),
                   [&](const UnsignedInt index)
                   {
                     return values[index];
                   });

    return out;
}

Eigen::Vector3f toEigen(const Vector3& v);

#endif //FEM3D_UTIL_H
