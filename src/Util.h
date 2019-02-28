#ifndef FEM3D_UTIL_H
#define FEM3D_UTIL_H

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>

#include "Eigen/Dense"

#include <string>
#include <vector>

using namespace Magnum;

bool parseTtg(const std::string &input, std::vector<Vector3> &outVertices, std::vector<UnsignedInt> &outMeshElementIndices, UnsignedInt& outDim);
void computeAABB(const std::vector<Vector3> &vertices, Vector3 &origin, Vector3 &extent);
bool createUVIndices(const std::vector<UnsignedInt>& triangleIndices, std::vector<Vector2>& outUv, std::vector<UnsignedInt>& outUvIndices);
bool extractTriangleIndices(const std::vector<UnsignedInt> &tetrahedronIndices, std::vector<UnsignedInt> &triangleIndices);

Vector3 valToColor(const Float val);
std::vector<Vector3> valuesToHeatGradient(const std::vector<Float>& vals);

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
