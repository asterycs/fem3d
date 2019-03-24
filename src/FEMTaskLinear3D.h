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
#include <array>

class ReferenceQuadraturePoints : public Eigen::MatrixXf {
public:
    PointsVectorized3D transform(const AffineTransformVectorized3D& tform) const
    {
        const PointsVectorized3D globalPoints { tform._Bkx * *this, tform._Bky * *this, tform._Bkz * *this };

        return globalPoints;
    }
};

class ReferenceBasisValuesVectorized : public Eigen::Matrix<float, 4, Eigen::Dynamic> {
public:
    explicit ReferenceBasisValuesVectorized(const std::size_t points)
            :Eigen::Matrix<float, 4, Eigen::Dynamic>(4, points)
    { }

    RowXpr basis(const Eigen::Index basisIndex)
    {
        return row(basisIndex);
    }

    ConstRowXpr basis(const Eigen::Index basisIndex) const
    {
        return row(basisIndex);
    }
};

class ReferenceDBasisValuesVectorized {
public:
    explicit ReferenceDBasisValuesVectorized(const Eigen::MatrixXf& _1, const Eigen::MatrixXf& _2,
                                    const Eigen::MatrixXf& _3,
                                    const Eigen::MatrixXf& _4)
            :_values{{Eigen::MatrixXf{_1}, Eigen::MatrixXf{_2}, Eigen::MatrixXf{_3}, Eigen::MatrixXf{_4}}}
    { };

    explicit ReferenceDBasisValuesVectorized(Eigen::MatrixXf&& _1, Eigen::MatrixXf&& _2, Eigen::MatrixXf&& _3,
                                    Eigen::MatrixXf&& _4)
            :_values{{std::move(_1), std::move(_2), std::move(_3), std::move(_4)}}
    { };

    VectorVectorized3D transform(const AffineTransformVectorized3D& tform, const std::size_t basisIndex) const
    {
        const VectorVectorized3D globalVectors { tform._Bkitx * _values[basisIndex], tform._Bkity * _values[basisIndex], tform._Bkitz * _values[basisIndex] };

        return globalVectors;
    }

    const Eigen::MatrixXf& basis(const std::size_t basisIndex) const
    {
        return _values[basisIndex];
    }

private:
    std::array<Eigen::MatrixXf, 4> _values{};
};

struct BilinearForm {
  virtual ScalarVectorized operator()(const ScalarVectorized& U, const ScalarVectorized& V,
                                        const VectorVectorized3D& dU,
                                        const VectorVectorized3D& dV, const PointsVectorized3D& points) = 0;

  virtual ~BilinearForm() {};
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

  virtual ~LinearForm() {};
};

struct LinLaplace : LinearForm {
  ScalarVectorized operator()(const ScalarVectorized& V,
                                const VectorVectorized3D& /*dV*/, const PointsVectorized3D& /*p*/) override
  {
      return V;
  }
};

class FEMTaskLinear3DSolution;
class FEMTaskLinear3D {
public:
    explicit FEMTaskLinear3D(const MeshData& mesh, const std::set<UnsignedInt>& pinnedVertexIds,
                             std::unique_ptr<BilinearForm> bilin, std::unique_ptr<LinearForm> linf);

    void initialize();
    FEMTaskLinear3DSolution solve() const;
    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> evaluateSolution(const Eigen::VectorXf& solution) const;

    const Eigen::SparseMatrix<Float>& getA() const;
    const Eigen::SparseVector<Float>& getb() const;

    ReferenceBasisValuesVectorized evaluateReferenceBasis(const ReferenceQuadraturePoints& x) const;
    ReferenceDBasisValuesVectorized evaluateReferenceDBasis(const ReferenceQuadraturePoints& x) const;

private:
    const MeshData& _mesh;
    const std::set<UnsignedInt>& _pinnedVertexIds;

    std::unique_ptr<BilinearForm> _bilin;
    std::unique_ptr<LinearForm> _linf;

    Eigen::SparseMatrix<Float> _A;
    Eigen::SparseVector<Float> _b;
};

class FEMTaskLinear3DSolution : public Eigen::VectorXf {
public:
    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> evaluate(const MeshData& mesh, const FEMTaskLinear3D& task) const
    {
        // Task is linear, we know the solution
        const std::vector<Float> U { this->data(), this->data() + this->size() };
        std::vector<Eigen::Vector3f> dU(size(), Eigen::Vector3f::Zero());

        const ReferenceQuadraturePoints localCorners{(Eigen::MatrixXf(3, 4) <<
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0
                ).finished()};

        VectorVectorized3D cumulativeGradient{ static_cast<Eigen::Index>(mesh.getElements().size()), 4};

        for (UnsignedInt basisIndex = 0; basisIndex < 4; ++basisIndex)
        {
            // Should be possible to skip since we know the bases are linear
            const ReferenceDBasisValuesVectorized cornerDBasisValues { task.evaluateReferenceDBasis( localCorners ) };
            const VectorVectorized3D dBasisValues{cornerDBasisValues.transform(mesh.getAffineTransform(), basisIndex)};

            ScalarVectorized expandedU(mesh.getElements().size(), 1);

            for (UnsignedInt elemIdx = 0; elemIdx < mesh.getElements().size(); ++elemIdx)
            {
                const auto& element = mesh.getElements()[elemIdx];
                expandedU(elemIdx) = U[element[basisIndex]];

            }

            // Vector * scalar
            cumulativeGradient += dBasisValues.operator*(expandedU.replicate(1,4));
        }


        for (UnsignedInt elemIdx = 0; elemIdx < mesh.getElements().size(); ++elemIdx)
        {
            const auto& element = mesh.getElements()[elemIdx];

            for (UnsignedInt cornerIdx = 0; cornerIdx < 4; ++cornerIdx)
                dU[element[cornerIdx]] += Eigen::Vector3f{cumulativeGradient._x(elemIdx, cornerIdx), cumulativeGradient._y(elemIdx, cornerIdx), cumulativeGradient._z(elemIdx, cornerIdx)};
        }

        return std::make_pair(U, dU);
    }
};

#endif //FEM3D_FEMTASKLINEAR3D_H
