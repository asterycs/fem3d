#ifndef FEM3D_UTIL_H
#define FEM3D_UTIL_H

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Vector.h>

#include "Eigen/Dense"

#include <string>
#include <vector>

class MeshData;

template<typename V>
struct AABB {
  V min;
  V max;
};

bool parseTtg(const std::string& input, MeshData& outMesh);
bool createUVIndices(const std::vector<Magnum::UnsignedInt>& triangleIndices, std::vector<Magnum::Vector2>& outUv,
                     std::vector<Magnum::UnsignedInt>& outUvIndices);

std::vector<Magnum::UnsignedInt> extractTriangleIndices(
        const std::vector<std::vector<Magnum::UnsignedInt>>& tetrahedronIndices);

std::vector<Magnum::Vector2i> bresenham(const Magnum::Vector2i a, const Magnum::Vector2i b);

Magnum::Vector3 valToColor(const Magnum::Float val);
std::vector<Magnum::Vector3> valuesToHeatGradient(const std::vector<Magnum::Float>& vals);
std::vector<Magnum::Float> computeNorm(const std::vector<Eigen::Vector3f>& input);

template<class V>
AABB<V> computeAABB(const std::vector<V>& elements)
{
    V min{std::numeric_limits<typename V::Type>::max()};
    V max{std::numeric_limits<typename V::Type>::min()};

    for (auto element : elements)
    {
        min = Magnum::Math::min(min, element);
        max = Magnum::Math::max(max, element);
    }

    return AABB<V>{min, max};
}

template<typename T>
std::vector<T> repeat(const std::vector<T>& values, const Magnum::UnsignedInt times)
{
    std::vector<T> res;
    res.reserve(values.size() * times);

    for (const T value : values)
    {
        for (Magnum::UnsignedInt j = 0; j < times; ++j)
            res.push_back(value);
    }

    return res;
}

template<typename T>
std::vector<T> expand(const std::vector<T>& values, std::vector<Magnum::UnsignedInt>& indices)
{
    std::vector<T> out;

    std::transform(indices.begin(), indices.end(), std::back_inserter(out),
                   [&](const Magnum::UnsignedInt index)
                   {
                     return values[index];
                   });

    return out;
}

Eigen::Vector3f toEigen(const Magnum::Vector3& v);

using ScalarVectorized = Eigen::MatrixXf;
struct VectorVectorized3D {
  ScalarVectorized _x;
  ScalarVectorized _y;
  ScalarVectorized _z;

  explicit VectorVectorized3D(const ScalarVectorized& x, const ScalarVectorized& y, const ScalarVectorized& z)
          : VectorVectorized3D(ScalarVectorized{x}, ScalarVectorized{y}, ScalarVectorized{z})
  { };

  explicit VectorVectorized3D(ScalarVectorized&& x, ScalarVectorized&& y, ScalarVectorized&& z)
          :_x{std::move(x)}, _y{std::move(y)}, _z{std::move(z)}
  { };

  VectorVectorized3D(const VectorVectorized3D& other)
          : VectorVectorized3D{ScalarVectorized{other._x}, ScalarVectorized{other._y}, ScalarVectorized{other._z}}
  { }

  VectorVectorized3D(VectorVectorized3D&& other)
          :_x{std::move(other._x)}, _y{std::move(other._y)}, _z{std::move(other._z)}
  { }
};

// Strong typedef emulation. Not sure if this is a good idea.
struct PointsVectorized3D : VectorVectorized3D {
  explicit PointsVectorized3D(const ScalarVectorized& x, const ScalarVectorized& y, const ScalarVectorized& z)
          :PointsVectorized3D(ScalarVectorized{x}, ScalarVectorized{y}, ScalarVectorized{z})
  { };

  explicit PointsVectorized3D(ScalarVectorized&& x, ScalarVectorized&& y, ScalarVectorized&& z)
          :VectorVectorized3D(std::move(x), std::move(y), std::move(z))
  { };
};

#endif //FEM3D_UTIL_H
