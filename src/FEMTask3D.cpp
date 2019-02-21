#include "FEMTask3D.h"

#include "Eigen/Dense"

FEMTask3D::FEMTask3D(const std::vector<Vector3>& vertices, const std::vector<UnsignedInt>& tetrahedronIds, const std::set<UnsignedInt>& pinnedVertexIds): _vertices{vertices}, _tetrahedronIndices{tetrahedronIds}, _pinnedVertexIds{pinnedVertexIds}
{

    Eigen::SparseMatrix<Float> A;

    //for
}

