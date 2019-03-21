#ifndef FEM3D_MESHDATA_H
#define FEM3D_MESHDATA_H

#include <Magnum/Magnum.h>

#include "Eigen/Dense"

#include "Typedefs.h"
#include "Util.h"

#include <vector>

class MeshData {
public:
    MeshData() = default;

    MeshData(const Magnum::UnsignedInt dimensions, const std::vector<Magnum::Vector3>& vertices,
             const std::vector<std::vector<Magnum::UnsignedInt>>& elementIndices,
             const std::vector<Magnum::UnsignedInt>& boundaryIndices);

    MeshData(const Magnum::UnsignedInt dimensions, const std::vector<Magnum::Vector3>&& vertices,
             const std::vector<std::vector<Magnum::UnsignedInt>>&& elementIndices,
             const std::vector<Magnum::UnsignedInt>&& boundaryIndices);

    void centerToOrigin();

    const std::vector<Magnum::UnsignedInt>& getBoundaryIndices() const;
    const std::vector<Magnum::Vector3>& getVertices() const;
    const std::vector<std::vector<Magnum::UnsignedInt>>& getElementIndices() const;
    Magnum::UnsignedInt getDimensions() const;

    // Affine transformations from reference tetrahedron
    // TODO: Wrap these into a struct or something
    Eigen::MatrixXf _Bkx, _Bky, _Bkz; // Bk
    Eigen::MatrixXf _Bkitx, _Bkity, _Bkitz; // Inverse transpose of Bk
    Eigen::VectorXf _bkx, _bky, _bkz; // bk
    Eigen::ArrayXf _absDetBk;
private:
    void initAffine();

    std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeAffine(const std::vector<UnsignedInt>& elemVertexIndices) const;

    Magnum::UnsignedInt _dimensions;
    std::vector<Magnum::Vector3> _vertices;
    std::vector<std::vector<Magnum::UnsignedInt>> _elementIndices;
    std::vector<Magnum::UnsignedInt> _boundaryIndices;
};

#endif //FEM3D_MESHDATA_H
