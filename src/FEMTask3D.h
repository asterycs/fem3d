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
    explicit FEMTask3D(const std::vector<Vector3>& vertices, const std::vector<std::vector<UnsignedInt>>& tetrahedronIds,const std::set<UnsignedInt>& pinnedVertexIds);

    void initialize();
    Eigen::VectorXf solve() const;

    // Return function value and gradients at mesh vertices
    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> evaluateSolution(const Eigen::VectorXf& solution) const;

private:
    Eigen::Vector4f evaluateBasis(const Eigen::Vector3f &x) const;
    Eigen::MatrixXf evaluateDBasis(const Eigen::Vector3f &x) const;
    std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeAffine(const std::vector<UnsignedInt>& elemVertexIndices) const;

    std::vector<Vector3> _vertices;
    std::vector<std::vector<UnsignedInt>> _tetrahedronIndices;
    std::set<UnsignedInt> _pinnedVertexIds;

    Eigen::SparseMatrix<Float> _A;
    Eigen::SparseVector<Float> _b;
};


#endif //FEM3D_FEMTASK_H
