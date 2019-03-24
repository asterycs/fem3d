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
             const std::vector<std::vector<Magnum::UnsignedInt>>& elements,
             const std::vector<Magnum::UnsignedInt>& boundaryIndices);

    MeshData(const Magnum::UnsignedInt dimensions, std::vector<Magnum::Vector3>&& vertices,
             std::vector<std::vector<Magnum::UnsignedInt>>&& elements,
             std::vector<Magnum::UnsignedInt>&& boundaryIndices);

    void centerToOrigin();

    const std::vector<Magnum::UnsignedInt>& getBoundaryIndices() const;
    const std::vector<Magnum::Vector3>& getVertices() const;
    const std::vector<std::vector<Magnum::UnsignedInt>>& getElements() const;
    Magnum::UnsignedInt getDimensions() const;

    const AffineTransformVectorized3D& getAffineTransform() const;
private:
    void initAffine();

    std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeAffine(const std::vector<UnsignedInt>& elemVertexIndices) const;

    Magnum::UnsignedInt _dimensions;
    std::vector<Magnum::Vector3> _vertices;
    std::vector<std::vector<Magnum::UnsignedInt>> _elements;
    std::vector<Magnum::UnsignedInt> _boundaryIndices;

    AffineTransformVectorized3D affineTransform;
};

#endif //FEM3D_MESHDATA_H
