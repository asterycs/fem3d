#ifndef FEM3D_FEMTASK_H
#define FEM3D_FEMTASK_H

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>

#include "Eigen/SparseCore"

#include <set>
#include <vector>

using namespace Magnum;

// This class handles matrix assembling and solving of the resulting system
class FEMTask3D
{
public:
    explicit FEMTask3D(const std::vector<Vector3>& vertices, const std::vector<UnsignedInt>& tetrahedronIds,const std::set<UnsignedInt>& pinnedVertexIds);

    std::vector<Float> solve();

    Eigen::Vector4f evaluateBasis(const Eigen::Vector3f &x);
    Eigen::MatrixXf evaluateDBasis(const Eigen::Vector3f &x);


private:
    void initialize();

    bool _isFeasible;

    std::vector<Vector3> _vertices;
    std::vector<UnsignedInt> _tetrahedronIndices;
    std::set<UnsignedInt> _pinnedVertexIds;

    Eigen::SparseMatrix<Float> _A;
    Eigen::SparseVector<Float> _b;
};


#endif //FEM3D_FEMTASK_H
