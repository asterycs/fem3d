#ifndef FEM3D_UTIL_H
#define FEM3D_UTIL_H

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>

#include "Eigen/Dense"

#include <string>
#include <vector>

using namespace Magnum;

bool parseTtg(const std::string &input, std::vector<Vector3> &outVertices, std::vector<std::vector<UnsignedInt>> &outMeshElementIndices,std::vector<UnsignedInt> &outboundaryIndices,  UnsignedInt& outDim);
void computeAABB(const std::vector<Vector3> &vertices, Vector3 &origin, Vector3 &extent);
bool createUVIndices(const std::vector<UnsignedInt>& triangleIndices, std::vector<Vector2>& outUv, std::vector<UnsignedInt>& outUvIndices);
bool extractTriangleIndices(const std::vector<std::vector<UnsignedInt>> &tetrahedronIndices, std::vector<UnsignedInt> &triangleIndices);

Vector3 valToColor(const Float val);
std::vector<Vector3> valuesToHeatGradient(const std::vector<Float>& vals);
std::vector<Float> computeNorm(const std::vector<Eigen::Vector3f>& input);

// TODO: Working templatized size
template <typename T>
std::pair<Math::Vector2<T>, Math::Vector2<T>> getBbox(const std::vector<Math::Vector2<T>>& pixels)
{
    Math::Vector2<T> min = {std::numeric_limits<Int>::max(), std::numeric_limits<Int>::max()};
    Math::Vector2<T> max = {std::numeric_limits<Int>::min(), std::numeric_limits<Int>::min()};

    for (auto pixel : pixels)
    {
        min = Math::min(min, pixel);
        max = Math::max(max, pixel);
    }

    return std::make_pair(min, max);
}

template<typename T>
std::vector<T> repeat(const std::vector<T> &values, const UnsignedInt times)
{
    std::vector<T> res;
    res.reserve(values.size() * times);

    for (UnsignedInt i = 0; i < values.size(); ++i)
    {
        for (UnsignedInt j = 0; j < times; ++j)
            res.push_back(values[i]);
    }

    return res;
}

template<typename T>
std::vector<T> reorder(const std::vector<T> &attribute, std::vector<UnsignedInt> &indices)
{
    std::vector<T> out(indices.size());

    for (UnsignedInt i = 0; i < indices.size(); ++i)
        out[i] = attribute[indices[i]];

    return out;
}

template<typename T>
std::vector<T> expand(const std::vector<T> &values, std::vector<UnsignedInt> &indices)
{
    std::vector<T> res;

    for (UnsignedInt i = 0; i < indices.size(); ++i)
    {
        const auto &val = values[indices[i]];
        res.push_back(val);
    }

    return res;
}

Eigen::Vector3f toEigen(const Vector3 &v);

#endif //FEM3D_UTIL_H
