#include "FEMTaskLinear3D.h"

#include "Eigen/SparseCholesky"

#include <cassert>

FEMTaskLinear3D::FEMTaskLinear3D(const MeshData& mesh, const std::set<UnsignedInt>& pinnedVertexIds,
                                 std::unique_ptr<BilinearForm> bilin, std::unique_ptr<LinearForm> linf)
        :_mesh{mesh}, _pinnedVertexIds{pinnedVertexIds}, _bilin{std::move(bilin)}, _linf{std::move(linf)}
{

}

void FEMTaskLinear3D::initialize()
{
    // Would be nice to have these generated on the fly.
    const ReferenceQuadraturePoints xip{(Eigen::MatrixXf(3, 4) <<
            0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f
            ).finished()};

    const Eigen::Vector4f w { Eigen::Array4f(0.25f, 0.25f, 0.25f, 0.25f) / 6.f };

    const PointsVectorized3D globalPoints { xip.transform(_mesh.getAffineTransform()) };
    const ReferenceBasisValuesVectorized referenceBasisValues { evaluateReferenceBasis(xip) };
    const ReferenceDBasisValuesVectorized referenceDBasisValues { evaluateReferenceDBasis(xip) };

    const std::size_t nElements { _mesh.getElements().size() };
    const std::size_t nVertices { _mesh.getVertices().size() };

    std::vector<Eigen::Triplet<Float>> A_triplets;
    A_triplets.reserve(nElements*4);
    //std::vector<Eigen::Triplet<Float>> b_triplets;
    _A = Eigen::SparseMatrix<Float>(nVertices, nVertices);
    _b = Eigen::SparseVector<Float>(nVertices);

    for (std::size_t i = 0; i < 4; ++i)
    {
        const ScalarVectorized iBasisValues{ referenceBasisValues.basis(i).replicate(nElements, 1) };
        const VectorVectorized3D iDBasisValues{ referenceDBasisValues.transform(_mesh.getAffineTransform(), i) };

        const Eigen::VectorXf rhs_i { ((*_linf)(iBasisValues, iDBasisValues, globalPoints) * w).array() * _mesh.getAffineTransform()._absDetBk };

        for (Eigen::Index ri = 0; ri < rhs_i.size(); ++ri)
        {
            const std::vector<UnsignedInt>& elementVertexIndices { _mesh.getElements()[ri] };
            const Eigen::Index rowIndex = elementVertexIndices[i];

            _b.coeffRef(rowIndex) += rhs_i(ri);
        }

        for (std::size_t j = 0; j < 4; ++j)
        {
            const ScalarVectorized jBasisValues{ referenceBasisValues.basis(j).replicate(nElements, 1) };
            const VectorVectorized3D jDBasisValues{ referenceDBasisValues.transform(_mesh.getAffineTransform(), j) };

            const Eigen::VectorXf lhs_ij { ((*_bilin)(jBasisValues, iBasisValues, jDBasisValues, iDBasisValues, globalPoints) * w).array() * _mesh.getAffineTransform()._absDetBk };

            for (Eigen::Index li = 0; li < lhs_ij.size(); ++li)
            {
                const std::vector<UnsignedInt>& elementVertexIndices = _mesh.getElements()[li];
                const Eigen::Index rowIndex = elementVertexIndices[i];
                const Eigen::Index colIndex = elementVertexIndices[j];

                A_triplets.push_back(Eigen::Triplet<Float>(rowIndex, colIndex, lhs_ij(li)));
            }
        }
    }

    _A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    for (UnsignedInt pinnedVertexId : _pinnedVertexIds)
    {
        _A.prune([=](UnsignedInt i, UnsignedInt j, Float)
                { return i != pinnedVertexId && j != pinnedVertexId; });
        _A.coeffRef(pinnedVertexId, pinnedVertexId) = 1.f;

        _b.coeffRef(pinnedVertexId) = 0.0f;
    }
}

ReferenceBasisValuesVectorized FEMTaskLinear3D::evaluateReferenceBasis(const ReferenceQuadraturePoints& x) const
{
    assert(x.rows() == 3);
    assert(x.cols() > 0);

    ScalarVectorized _1(1, x.cols());
    ScalarVectorized _2(1, x.cols());
    ScalarVectorized _3(1, x.cols());
    ScalarVectorized _4(1, x.cols());

    ReferenceBasisValuesVectorized out(x.cols());
    out.basis(0) = 1.0f - x.row(0).array() - x.row(1).array() - x.row(2).array();
    out.basis(1) = x.row(0);
    out.basis(2) = x.row(1);
    out.basis(3) = x.row(2);

    return out;
}

ReferenceDBasisValuesVectorized FEMTaskLinear3D::evaluateReferenceDBasis(const ReferenceQuadraturePoints& x) const
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

    ReferenceDBasisValuesVectorized out{std::move(_1), std::move(_2), std::move(_3), std::move(_4)};

    return out;
}

const Eigen::SparseMatrix<Float>& FEMTaskLinear3D::getA() const
{
    return _A;
}

const Eigen::SparseVector<Float>& FEMTaskLinear3D::getb() const
{
    return _b;
}

FEMTaskLinear3DSolution FEMTaskLinear3D::solve() const
{
    Eigen::SimplicialLLT<Eigen::SparseMatrix<Float>> solver;
    solver.compute(_A);

    if (solver.info() != Eigen::Success)
    {
        Magnum::Warning{} << "Could not solve linear system";
        return FEMTaskLinear3DSolution();
    }

    FEMTaskLinear3DSolution x { solver.solve(_b) };

    if (solver.info() != Eigen::Success)
    {
        Magnum::Warning{} << "Could not solve linear system";
        return FEMTaskLinear3DSolution();
    }

    return x;
}