#ifndef FEM3D_FEMTASKLINEAR3D_H
#define FEM3D_FEMTASKLINEAR3D_H

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>

#include "Eigen/SparseCore"

#include "MeshData.h"
#include "Typedefs.h"

#include <set>
#include <vector>

struct PointsVectorized3D {
  Eigen::MatrixXf x;
  Eigen::MatrixXf y;
  Eigen::MatrixXf z;

  explicit PointsVectorized3D(const Eigen::MatrixXf& x, const Eigen::MatrixXf& y, const Eigen::MatrixXf& z)
          :x{x}, y{y}, z{z}
  { };
};

// Strong typedef emulation. Not sure if needed.
struct ValuesVectorized3D : PointsVectorized3D {
  explicit ValuesVectorized3D(const Eigen::MatrixXf& x, const Eigen::MatrixXf& y, const Eigen::MatrixXf& z)
          :PointsVectorized3D(x, y, z)
  { };
};

class BilinearForm {
public:
    virtual Eigen::MatrixXf operator()(const ValuesVectorized3D& U, const ValuesVectorized3D& V, const ValuesVectorized3D& dU,
                       const ValuesVectorized3D& dV, const PointsVectorized3D& points) = 0;
};

class LinearForm {
public:
    virtual Eigen::MatrixXf operator()(const ValuesVectorized3D& V, const ValuesVectorized3D& dV, const PointsVectorized3D& points) = 0;
};

class FEMTaskLinear3D {
public:
    explicit FEMTaskLinear3D(const MeshData& mesh, const std::set<UnsignedInt>& pinnedVertexIds,
                             const BilinearForm& bilin, const LinearForm& linf);

    void initialize();
    Eigen::VectorXf solve() const;

    // Return function value and gradients at mesh vertices
    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> evaluateSolution(const Eigen::VectorXf& solution) const;

private:
    Eigen::Vector4f evaluateBasis(const Eigen::Vector3f& x) const;
    Eigen::MatrixXf evaluateDBasis(const Eigen::Vector3f& x) const;
    std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeAffine(const std::vector<UnsignedInt>& elemVertexIndices) const;

    std::vector<Vector3> _vertices;
    std::vector<std::vector<UnsignedInt>> _tetrahedronIndices;
    std::set<UnsignedInt> _pinnedVertexIds;

    Eigen::SparseMatrix<Float> _A;
    Eigen::SparseVector<Float> _b;
};

#endif //FEM3D_FEMTASKLINEAR3D_H
