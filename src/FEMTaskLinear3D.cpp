#include "FEMTaskLinear3D.h"

#include <cassert>

FEMTaskLinear3D::FEMTaskLinear3D(const MeshData& mesh, const std::set<UnsignedInt>& pinnedVertexIds,
                                 std::unique_ptr<BilinearForm> bilin, std::unique_ptr<LinearForm> linf)
        :_mesh{mesh}, _pinnedVertexIds{pinnedVertexIds}, _bilin{std::move(bilin)}, _linf{std::move(linf)}
{ }

void FEMTaskLinear3D::initialize()
{
    // Would be nice to have these generated on the fly.
    const Eigen::MatrixXf xip{(Eigen::MatrixXf(3, 4) <<
            0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f
            ).finished()};

    const Eigen::Vector4f w(Eigen::Array4f(0.25f, 0.25f, 0.25f, 0.25f) / 6.f);

    const PointsVectorized3D globalPoints = _mesh.referencePointsToGlobal(xip);
    const BasisValuesVectorized referenceBasisValues = evaluateReferenceBasis(xip);
    const DBasisValuesVectorized referenceDBasisValues = evaluateReferenceDBasis(xip);

    const std::size_t n_triangles = _mesh.getElementIndices().size();

    std::vector<Eigen::Triplet<float>> A_triplets;
    std::vector<Eigen::Triplet<float>> b_triplets;

    for (std::size_t i = 0; i < 4; ++i)
    {
        const ScalarVectorized iBasisValues{ referenceBasisValues.basis(i).replicate(n_triangles, 1) };
        const VectorVectorized3D iDBasisValues{ referenceDBasisValues.matmulBasis(_mesh._Bkx, i), referenceDBasisValues.matmulBasis(_mesh._Bky, i), referenceDBasisValues.matmulBasis(_mesh._Bkz, i) };

        const Eigen::VectorXf ff = ((*_linf)(iBasisValues, iDBasisValues, globalPoints) * w).array() * _mesh._detBk;

        // Memo:
        // for (i in size(ff))
        //     triplets.insert(elems(i)(basisIndex), 1, ff(i))
        //
        // First element index corresponds to first basis, second index to second basis etc.
    }

}

BasisValuesVectorized FEMTaskLinear3D::evaluateReferenceBasis(const Eigen::MatrixXf& x) const
{
    assert(x.rows() == 3);
    assert(x.cols() > 0);

    ScalarVectorized _1(1, x.cols());
    ScalarVectorized _2(1, x.cols());
    ScalarVectorized _3(1, x.cols());
    ScalarVectorized _4(1, x.cols());

    BasisValuesVectorized out(x.cols());
    out.basis(0) = 1.0f - x.row(0).array() - x.row(1).array() - x.row(2).array();
    out.basis(1) = x.row(0);
    out.basis(2) = x.row(1);
    out.basis(3) = x.row(2);

    return out;
}

DBasisValuesVectorized FEMTaskLinear3D::evaluateReferenceDBasis(const Eigen::MatrixXf& x) const
{
    assert(x.rows() == 3);
    assert(x.cols() > 0);

    const Eigen::RowVectorXf ones = Eigen::RowVectorXf::Constant(x.cols(), 1);
    const Eigen::RowVectorXf minusOnes = -ones;
    const Eigen::RowVectorXf zeros = Eigen::RowVectorXf::Zero(x.cols());

    Eigen::MatrixXf _1{3,x.cols()};

    _1 << minusOnes,
          minusOnes,
          minusOnes;

    Eigen::MatrixXf _2{3,x.cols()};
    _2 << ones,
          zeros,
          zeros;

    Eigen::MatrixXf _3{3,x.cols()};
    _3 << zeros,
          ones,
          zeros;

    Eigen::MatrixXf _4{3,x.cols()};
    _4 << zeros,
          zeros,
          ones;

    DBasisValuesVectorized out{std::move(_1), std::move(_2), std::move(_3), std::move(_4)};

    return out;
}