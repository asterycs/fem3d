#include "FEMTaskLinear3D.h"

#include <cassert>

FEMTaskLinear3D::FEMTaskLinear3D(const MeshData& mesh, const std::set<UnsignedInt>& pinnedVertexIds,
                                 std::unique_ptr<BilinearForm> bilin, std::unique_ptr<LinearForm> linf)
        :_mesh{mesh}, _pinnedVertexIds{pinnedVertexIds}, _bilin{std::move(bilin)}, _linf{std::move(linf)}
{ }

void FEMTaskLinear3D::initialize()
{
    // Would be nice to have these generated on the fly.
    Eigen::MatrixXf xip(3, 4);
    xip << 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f;

    const Eigen::Vector4f w(Eigen::Array4f(0.25f, 0.25f, 0.25f, 0.25f) / 6.f);
    
    const PointsVectorized3D globalPoints  = _mesh.referencePointsToGlobal(xip);

}

BasisValuesVectorized FEMTaskLinear3D::evaluateReferenceBasis(const Eigen::MatrixXf& x) const
{
    assert(x.rows() == 3);
    assert(x.cols() > 0);

    ScalarVectorized3D _1(1,x.cols());
    ScalarVectorized3D _2(1,x.cols());
    ScalarVectorized3D _3(1,x.cols());
    ScalarVectorized3D _4(1,x.cols());

    _1 = 1.0f - x.row(0).array() - x.row(1).array() - x.row(2).array();
    _2 = x.row(0);
    _3 = x.row(1);
    _4 = x.row(2);

    BasisValuesVectorized out(std::move(_1), std::move(_2), std::move(_3), std::move(_4));

    return out;
}

DBasisValuesVectorized FEMTaskLinear3D::evaluateReferenceDBasis(const Eigen::MatrixXf& x) const
{
    assert(x.rows() == 3);
    assert(x.cols() > 0);

    const Eigen::MatrixXf ones = Eigen::MatrixXf::Constant(1, x.cols(), 1);
    const Eigen::MatrixXf minusOnes = -ones;
    const Eigen::MatrixXf zeros = Eigen::MatrixXf::Zero(1, x.cols());

    VectorVectorized3D _1(minusOnes, minusOnes, minusOnes);
    VectorVectorized3D _2(ones, zeros, zeros);
    VectorVectorized3D _3(zeros, ones, zeros);
    VectorVectorized3D _4(zeros, zeros, ones);

    DBasisValuesVectorized out{std::move(_1), std::move(_2), std::move(_3), std::move(_4)};

    return out;
}