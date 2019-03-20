#ifndef FEM3D_FEMTASKLINEAR3D_H
#define FEM3D_FEMTASKLINEAR3D_H

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>

#include "Eigen/SparseCore"

#include "MeshData.h"
#include "Typedefs.h"
#include "Util.h"

#include <set>
#include <vector>
#include <memory>

class BasisValuesVectorized : Eigen::Matrix<float, 4, Eigen::Dynamic> {
public:
    BasisValuesVectorized(const std::size_t points)
            :Eigen::Matrix<float, 4, Eigen::Dynamic>(4, points)
    { }

    RowXpr basis(const std::size_t basisIndex)
    {
        return row(basisIndex);
    }

    ConstRowXpr basis(const std::size_t basisIndex) const
    {
        return row(basisIndex);
    }
};

class DBasisValuesVectorized {
public:
    explicit DBasisValuesVectorized(const Eigen::MatrixXf& _1, const Eigen::MatrixXf& _2,
                                    const Eigen::MatrixXf& _3,
                                    const Eigen::MatrixXf& _4)
            :_values{{Eigen::MatrixXf{_1}, Eigen::MatrixXf{_2}, Eigen::MatrixXf{_3}, Eigen::MatrixXf{_4}}}
    { };

    explicit DBasisValuesVectorized(Eigen::MatrixXf&& _1, Eigen::MatrixXf&& _2, Eigen::MatrixXf&& _3,
                                    Eigen::MatrixXf&& _4)
            :_values{{std::move(_1), std::move(_2), std::move(_3), std::move(_4)}}
    { };

    ScalarVectorized matmulBasis(const Eigen::MatrixXf& mat, const std::size_t basisIndex) const
    {
        return mat * _values[basisIndex];
    }

    const Eigen::MatrixXf& basis(const std::size_t basisIndex) const
    {
        return _values[basisIndex];
    }

private:
    std::array<Eigen::MatrixXf, 4> _values;
};

struct BilinearForm {
  virtual ScalarVectorized operator()(const ScalarVectorized& U, const ScalarVectorized& V,
                                        const VectorVectorized3D& dU,
                                        const VectorVectorized3D& dV, const PointsVectorized3D& points) = 0;
};

struct BilinLaplace : BilinearForm {
  ScalarVectorized operator()(const ScalarVectorized& /*U*/, const ScalarVectorized& /*V*/,
                                const VectorVectorized3D& dU,
                                const VectorVectorized3D& dV, const PointsVectorized3D& /*p*/) override
  {
      return dU._x.array() * dV._x.array() + dU._y.array() * dV._y.array() + dU._z.array() * dV._z.array();
  }
};

struct LinearForm {
  virtual ScalarVectorized operator()(const ScalarVectorized& V, const VectorVectorized3D& dV,
                                        const PointsVectorized3D& points) = 0;
};

struct LinLaplace : LinearForm {
  ScalarVectorized operator()(const ScalarVectorized& V,
                                const VectorVectorized3D& /*dV*/, const PointsVectorized3D& /*p*/) override
  {
      return V;
  }
};

class FEMTaskLinear3D {
public:
    explicit FEMTaskLinear3D(const MeshData& mesh, const std::set<UnsignedInt>& pinnedVertexIds,
                             std::unique_ptr<BilinearForm> bilin, std::unique_ptr<LinearForm> linf);

    void initialize();
    Eigen::VectorXf solve() const;

    // Return function value and gradients at mesh vertices
    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> evaluateSolution(const Eigen::VectorXf& solution) const;

private:
    BasisValuesVectorized evaluateReferenceBasis(const Eigen::MatrixXf& x) const;
    DBasisValuesVectorized evaluateReferenceDBasis(const Eigen::MatrixXf& x) const;

    const MeshData& _mesh;
    const std::set<UnsignedInt>& _pinnedVertexIds;

    std::unique_ptr<BilinearForm> _bilin;
    std::unique_ptr<LinearForm> _linf;

    Eigen::SparseMatrix<Float> _A;
    Eigen::SparseVector<Float> _b;
};

#endif //FEM3D_FEMTASKLINEAR3D_H
