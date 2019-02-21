#include "FEMTask3D.h"

#include "Eigen/Dense"

#include "Util.h"

FEMTask3D::FEMTask3D(const std::vector<Vector3>& vertices, const std::vector<UnsignedInt>& tetrahedronIds, const std::set<UnsignedInt>& pinnedVertexIds): _vertices{vertices}, _tetrahedronIndices{tetrahedronIds}, _pinnedVertexIds{pinnedVertexIds}
{
    /*Eigen::SparseMatrix<Float> A;

    for (UnsignedInt ti = 0; ti < tetrahedronIds.size(); ti += 4)
    {
        const UnsignedInt vi0 = tetrahedronIds[ti  ];
        const UnsignedInt vi1 = tetrahedronIds[ti+1];
        const UnsignedInt vi2 = tetrahedronIds[ti+2];
        const UnsignedInt vi3 = tetrahedronIds[ti+3];

        const Eigen::Vector3f p0 = toEigen(_vertices[vi0]);
        const Eigen::Vector3f p1 = toEigen(_vertices[vi1]);
        const Eigen::Vector3f p2 = toEigen(_vertices[vi2]);
        const Eigen::Vector3f p3 = toEigen(_vertices[vi3]);


    }*/
}

