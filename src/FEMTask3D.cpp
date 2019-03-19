#include "FEMTask3D.h"
#include "Eigen/Dense"
#include "Eigen/SparseCholesky"

#include "Util.h"

#include <array>

FEMTask3D::FEMTask3D(const std::vector<Vector3> &vertices,
                     const std::vector<std::vector<UnsignedInt>> &tetrahedronIds,
                     const std::set<UnsignedInt> &pinnedVertexIds) : _vertices{vertices},
                                                                     _tetrahedronIndices{tetrahedronIds},
                                                                     _pinnedVertexIds{pinnedVertexIds}
{

}

void FEMTask3D::initialize()
{
    Eigen::SparseMatrix<Float> A(_vertices.size(), _vertices.size());
    Eigen::SparseVector<Float> b(_vertices.size());

    Eigen::MatrixXf xip(3, 4);
    xip << 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f;

    const Eigen::Vector4f w(Eigen::Array4f(0.25f, 0.25f, 0.25f, 0.25f) / 6.f);

    for (auto& vi : _tetrahedronIndices)
    {
        const auto [Bk, bk] = computeAffine(vi);

        for (UnsignedInt i = 0; i < 4; ++i)
        {
            for (UnsignedInt j = 0; j < 4; ++j)
            {
                for (UnsignedInt k = 0; k < 4; ++k)
                {
                    const Eigen::Vector3f globalQuadraturePoint = Bk * xip.col(k) + bk;
                    const Eigen::Vector3f l = Bk.transpose().inverse() * evaluateDBasis(globalQuadraturePoint).col(i);
                    const Eigen::Vector3f r = Bk.transpose().inverse() * evaluateDBasis(globalQuadraturePoint).col(j);

                    A.coeffRef(vi[i], vi[j]) += w(k) * l.dot(r) * std::abs(Bk.determinant());
                }
            }
        }
    }

    for (auto& vi : _tetrahedronIndices)
    {
        const auto [Bk, bk] = computeAffine(vi);

        Eigen::Vector4f bl(Eigen::Vector4f::Zero());

        for (UnsignedInt k = 0; k < 4; ++k)
        {
            for (UnsignedInt i = 0; i < 4; ++i)
            {
                //const Eigen::Vector3f globalQuadraturePoint = Bk * xip.col(k) + bk;
                bl(i) += w(k) * evaluateBasis(xip.col(k))(i) * std::abs(Bk.determinant());
            }
        }

        for (UnsignedInt i = 0; i < 4; ++i)
        {
            b.coeffRef(vi[i]) += bl(i);
        }
    }

    for (auto pinnedVertexId : _pinnedVertexIds)
    {
        A.prune([=](UnsignedInt i, UnsignedInt j, Float)
                { return i != pinnedVertexId && j != pinnedVertexId; });
        A.coeffRef(pinnedVertexId, pinnedVertexId) = 1.f;

        b.coeffRef(pinnedVertexId) = 0.0f;
    }


    _A = A;
    _b = b;
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> FEMTask3D::computeAffine(const std::vector<UnsignedInt>& elemVertexIndices) const
{
    const Eigen::Vector3f p0 = toEigen(_vertices[elemVertexIndices[0]]);
    const Eigen::Vector3f p1 = toEigen(_vertices[elemVertexIndices[1]]);
    const Eigen::Vector3f p2 = toEigen(_vertices[elemVertexIndices[2]]);
    const Eigen::Vector3f p3 = toEigen(_vertices[elemVertexIndices[3]]);

    Eigen::Matrix3f Bk;
    Bk << p1 - p0, p2 - p0, p3 - p0;
    Eigen::Vector3f bk = p0;

    return std::make_pair(Bk, bk);
}

Eigen::Vector4f FEMTask3D::evaluateBasis(const Eigen::Vector3f &x) const
{
    return Eigen::Vector4f(1.f - x(0) - x(1) - x(2), x(0), x(1), x(2));
}

Eigen::MatrixXf FEMTask3D::evaluateDBasis(const Eigen::Vector3f &) const
{
    Eigen::MatrixXf grad(3,4);
    grad << -1.f, 1.f, 0.f, 0.f,
            -1.f, 0.f, 1.f, 0.f,
            -1.f, 0.f, 0.f, 1.f;

    return grad;
}

std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> FEMTask3D::evaluateSolution(const Eigen::VectorXf& solution) const
{
    if (_vertices.size() == 0 || _tetrahedronIndices.size() == 0 || solution.size() != static_cast<long>(_vertices.size()))
        return std::make_pair(std::vector<Float>(), std::vector<Eigen::Vector3f>());

    std::vector<Float> U(solution.size(), 0.f);
    std::vector<Eigen::Vector3f> dU(solution.size(), Eigen::Vector3f::Zero());

    Eigen::MatrixXf localCorners(3, 4);
    localCorners << 0.f, 1.f, 0.f, 0.f,
                    0.f, 0.f, 1.f, 0.f,
                    0.f, 0.f, 0.f, 1.f;

    for (auto& vi : _tetrahedronIndices)
    {
        const auto [Bk, bk] = computeAffine(vi);

        Eigen::MatrixXf dL(3,4);
        Eigen::Vector4f multipliers;

        // Copy function values
        for (UnsignedInt i = 0; i < 4; ++i)
        {
            U[vi[i]] = solution(vi[i]);
            multipliers(i) = solution(vi[i]);
        }

        for (UnsignedInt i = 0; i < 4; ++i)
        {
            const Eigen::Vector3f currentCorner = localCorners.col(i);
            dL.col(i) = Bk.inverse().transpose() * evaluateDBasis(currentCorner).col(i);
        }

        const Eigen::Vector3f cornerGradient = dL * multipliers;
        // Sum gradients at vertices
        for (UnsignedInt i = 0; i < 4; ++i)
        {
            dU[vi[i]] += cornerGradient;
        }
    }

    return std::make_pair(U, dU);
}

Eigen::VectorXf FEMTask3D::solve() const
{
    Eigen::SimplicialLLT<Eigen::SparseMatrix<Float>> solver;
    solver.compute(_A);

    if (solver.info() != Eigen::Success)
    {
        Magnum::Warning{} << "Could not solve linear system";
        return Eigen::VectorXf();
    }

    Eigen::VectorXf x = solver.solve(_b);

    if (solver.info() != Eigen::Success)
    {
        Magnum::Warning{} << "Could not solve linear system";
        return Eigen::VectorXf();
    }

    return x;
}


