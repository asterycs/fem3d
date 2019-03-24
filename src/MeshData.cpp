#include "MeshData.h"

#include <Magnum/MeshTools/Transform.h>

#include "Util.h"

MeshData::MeshData(const Magnum::UnsignedInt dimensions, const std::vector<Magnum::Vector3>& vertices,
         const std::vector<std::vector<Magnum::UnsignedInt>>& elements,
         const std::vector<Magnum::UnsignedInt>& boundaryIndices)
        : _dimensions{dimensions}, _vertices{vertices}, _elements{elements}, _boundaryIndices{boundaryIndices}
{
    initAffine();
}

MeshData::MeshData(const Magnum::UnsignedInt dimensions, std::vector<Magnum::Vector3>&& vertices,
         std::vector<std::vector<Magnum::UnsignedInt>>&& elements,
         std::vector<Magnum::UnsignedInt>&& boundaryIndices)
        : _dimensions{dimensions}, _vertices{std::move(vertices)}, _elements{std::move(elements)}, _boundaryIndices{std::move(boundaryIndices)}
{
    initAffine();
}


void MeshData::initAffine()
{
    const std::size_t n_elements = _elements.size();

    affineTransform._Bkx = Eigen::MatrixXf(n_elements, 3);
    affineTransform._Bky = Eigen::MatrixXf(n_elements, 3);
    affineTransform._Bkz = Eigen::MatrixXf(n_elements, 3);

    affineTransform._Bkitx = Eigen::MatrixXf(n_elements, 3);
    affineTransform._Bkity = Eigen::MatrixXf(n_elements, 3);
    affineTransform._Bkitz = Eigen::MatrixXf(n_elements, 3);

    affineTransform._bkx = Eigen::VectorXf(n_elements);
    affineTransform._bky = Eigen::VectorXf(n_elements);
    affineTransform._bkz = Eigen::VectorXf(n_elements);

    affineTransform._absDetBk = Eigen::ArrayXf(n_elements);

    UnsignedInt currentRow = 0;
    for (auto elem : _elements)
    {
        const auto[Bk, bk] = computeAffine(elem);
        const Eigen::Matrix3f Bkit = Bk.transpose().inverse();

        affineTransform._Bkx.row(currentRow) = Bk.row(0);
        affineTransform._Bky.row(currentRow) = Bk.row(1);
        affineTransform._Bkz.row(currentRow) = Bk.row(2);

        affineTransform._Bkitx.row(currentRow) = Bkit.row(0);
        affineTransform._Bkity.row(currentRow) = Bkit.row(1);
        affineTransform._Bkitz.row(currentRow) = Bkit.row(2);

        affineTransform._bkx(currentRow) = bk(0);
        affineTransform._bky(currentRow) = bk(1);
        affineTransform._bkz(currentRow) = bk(2);

        affineTransform._absDetBk(currentRow) = std::abs(Bk.determinant());

        ++currentRow;
    }
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> MeshData::computeAffine(const std::vector<UnsignedInt>& elemVertexIndices) const
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

UnsignedInt MeshData::getDimensions() const
{
    return _dimensions;
}

const std::vector<UnsignedInt>& MeshData::getBoundaryIndices() const
{
    return _boundaryIndices;
}

const std::vector<Vector3>& MeshData::getVertices() const
{
    return _vertices;
}

const std::vector<std::vector<UnsignedInt>>& MeshData::getElements() const
{
    return _elements;
}

void MeshData::centerToOrigin()
{
    const AABB<Vector3> aabb = computeAABB(_vertices);
    const Vector3 origin = 0.5f*(aabb.max - aabb.min) + aabb.min;
    Magnum::MeshTools::transformPointsInPlace(Matrix4::translation(-origin), _vertices);
}

const AffineTransformVectorized3D& MeshData::getAffineTransform() const
{
    return affineTransform;
}
