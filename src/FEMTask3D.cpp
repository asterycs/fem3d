#include "FEMTask3D.h"

#include "Eigen/Dense"

#include "Util.h"

FEMTask3D::FEMTask3D(const std::vector<Vector3>& vertices, const std::vector<UnsignedInt>& tetrahedronIds, const std::set<UnsignedInt>& pinnedVertexIds): _vertices{vertices}, _tetrahedronIndices{tetrahedronIds}, _pinnedVertexIds{pinnedVertexIds}
{
    /*Eigen::SparseMatrix<Float> A;

    for (UnsignedInt ti = 0; ti < tetrahedronIds.size(); ti += 4)
    {
        const UnsignedInt vi[] = {tetrahedronIds[ti],
                                  tetrahedronIds[ti+1],
                                  tetrahedronIds[ti+2],
                                  tetrahedronIds[ti+3]};

        const Eigen::Vector3f p0 = toEigen(_vertices[vi[0]]);
        const Eigen::Vector3f p1 = toEigen(_vertices[vi[1]]);
        const Eigen::Vector3f p2 = toEigen(_vertices[vi[2]]);
        const Eigen::Vector3f p3 = toEigen(_vertices[vi[3]]);

        Eigen::Matrix3f Bk; Bk << p1 - p0, p2 - p0, p3 - p0;
        Eigen::Vector3f bk = p0;

        for (UnsignedInt i = 0; i < 4; ++i)
        {
            for (UnsignedInt j = 0; j < 4; ++j)
            {
                //A(vi[i],vi[j]) += ...
            }
        }
    }*/
}

