#include "FEMTask3D.h"
#include "Eigen/Dense"
#include "Eigen/SparseCholesky"

#include "Util.h"

Eigen::Vector4f phi(const Eigen::Vector3f &x);
Eigen::Vector4f phi(const Eigen::Vector3f &x)
{
    return Eigen::Vector4f(1.f - x(0) - x(1) - x(2), x(0), x(1), x(2));
}

FEMTask3D::FEMTask3D(const std::vector<Vector3> &vertices,
                     const std::vector<UnsignedInt> &tetrahedronIds,
                     const std::set<UnsignedInt> &pinnedVertexIds) : _isFeasible{true}, _vertices{vertices},
                                                                     _tetrahedronIndices{tetrahedronIds},
                                                                     _pinnedVertexIds{pinnedVertexIds}
{

}

void FEMTask3D::initialize()
{
    Eigen::SparseMatrix<Float> A(_vertices.size(), _vertices.size());
    Eigen::SparseVector<Float> b(_vertices.size());

    Eigen::MatrixXf gphi(3, 4);
    gphi << -1.f, 1.f, 0.f, 0.f,
            -1.f, 0.f, 1.f, 0.f,
            -1.f, 0.f, 0.f, 1.f;

    Eigen::MatrixXf xip(3, 4);
    xip << 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f, 0.1381966011250105f,
            0.1381966011250105f, 0.1381966011250105f, 0.1381966011250105f, 0.5854101966249685f;

    const Eigen::Vector4f w(Eigen::Array4f(0.25f, 0.25f, 0.25f, 0.25f) / 6.f);

    for (UnsignedInt ti = 0; ti < _tetrahedronIndices.size(); ti += 4)
    {
        const UnsignedInt vi[] = {_tetrahedronIndices[ti],
                                  _tetrahedronIndices[ti + 1],
                                  _tetrahedronIndices[ti + 2],
                                  _tetrahedronIndices[ti + 3]};

        const Eigen::Vector3f p0 = toEigen(_vertices[vi[0]]);
        const Eigen::Vector3f p1 = toEigen(_vertices[vi[1]]);
        const Eigen::Vector3f p2 = toEigen(_vertices[vi[2]]);
        const Eigen::Vector3f p3 = toEigen(_vertices[vi[3]]);

        Eigen::Matrix3f Bk;
        Bk << p1 - p0, p2 - p0, p3 - p0;
        //Eigen::Vector3f bk = p0;

        for (UnsignedInt i = 0; i < 4; ++i)
        {
            for (UnsignedInt j = 0; j < 4; ++j)
            {
                const Eigen::Vector3f l = Bk.transpose().inverse() * gphi.col(i);
                const Eigen::Vector3f r = Bk.transpose().inverse() * gphi.col(j);
                A.coeffRef(vi[i], vi[j]) += w.sum() * l.dot(r) * std::abs(Bk.determinant());
            }
        }
    }

    for (UnsignedInt ti = 0; ti < _tetrahedronIndices.size(); ti += 4)
    {
        const UnsignedInt vi[] = {_tetrahedronIndices[ti],
                                  _tetrahedronIndices[ti + 1],
                                  _tetrahedronIndices[ti + 2],
                                  _tetrahedronIndices[ti + 3]};

        const Eigen::Vector3f p0 = toEigen(_vertices[vi[0]]);
        const Eigen::Vector3f p1 = toEigen(_vertices[vi[1]]);
        const Eigen::Vector3f p2 = toEigen(_vertices[vi[2]]);
        const Eigen::Vector3f p3 = toEigen(_vertices[vi[3]]);

        Eigen::Matrix3f Bk;
        Bk << p1 - p0, p2 - p0, p3 - p0;
        //Eigen::Vector3f bk = p0;

        Eigen::Vector4f bl(Eigen::Vector4f::Zero());

        for (UnsignedInt k = 0; k < 4; ++k)
        {
            for (UnsignedInt i = 0; i < 4; ++i)
            {
                //const Eigen::Vector3f globalCoord = Bk * xip.col(k) + bk;

                bl(i) += w(k) * phi(xip.col(k))(i) * std::abs(Bk.determinant());
            }
        }

        for (UnsignedInt i = 0; i < 4; ++i)
        {
            b.coeffRef(vi[i]) += bl(i);
        }
    }

    if (_pinnedVertexIds.size() == 0)
    {
        _isFeasible = false;
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

std::vector<Float> FEMTask3D::solve()
{
    initialize();

    if (!_isFeasible)
    {
        Debug{} << "Problem is infeasible, doing nothing";
        return std::vector<Float>();
    }

    Eigen::SimplicialLLT<Eigen::SparseMatrix<Float>> solver;
    solver.compute(_A);

    if (solver.info() != Eigen::Success)
    {
        Debug{} << "Could not solve linear system";
        return std::vector<Float>();
    }

    Eigen::VectorXf x = solver.solve(_b);

    if (solver.info() != Eigen::Success)
    {
        Debug{} << "Could not solve linear system";
        return std::vector<Float>();
    }

    return std::vector<Float>(x.data(), x.data() + x.size());
}


