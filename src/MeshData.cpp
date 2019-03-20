#include "MeshData.h"

#include <Magnum/MeshTools/Transform.h>

#include "Util.h"

MeshData::MeshData(const Magnum::UnsignedInt dimensions, const std::vector<Magnum::Vector3>& vertices,
         const std::vector<std::vector<Magnum::UnsignedInt>>& elementIndices,
         const std::vector<Magnum::UnsignedInt>& boundaryIndices)
        : _dimensions{dimensions}, _vertices{vertices}, _elementIndices{elementIndices}, _boundaryIndices{boundaryIndices}
{
    initAffine();
}

MeshData::MeshData(const Magnum::UnsignedInt dimensions, const std::vector<Magnum::Vector3>&& vertices,
         const std::vector<std::vector<Magnum::UnsignedInt>>&& elementIndices,
         const std::vector<Magnum::UnsignedInt>&& boundaryIndices)
        : _dimensions{dimensions}, _vertices{vertices}, _elementIndices{elementIndices}, _boundaryIndices{boundaryIndices}
{
    initAffine();
}


void MeshData::initAffine()
{
    _Bkx = Eigen::MatrixXf(_elementIndices.size(), 3);
    _Bky = Eigen::MatrixXf(_elementIndices.size(), 3);
    _Bkz = Eigen::MatrixXf(_elementIndices.size(), 3);

    _Bkitx = Eigen::MatrixXf(_elementIndices.size(), 3);
    _Bkity = Eigen::MatrixXf(_elementIndices.size(), 3);
    _Bkitz = Eigen::MatrixXf(_elementIndices.size(), 3);

    _bkx = Eigen::VectorXf(_elementIndices.size());
    _bky = Eigen::VectorXf(_elementIndices.size());
    _bkz = Eigen::VectorXf(_elementIndices.size());

    UnsignedInt currentRow = 0;
    for (auto elem : _elementIndices)
    {
        const auto[Bk, bk] = computeAffine(elem);
        const Eigen::Matrix3f Bkit = Bk.transpose().inverse();

        _Bkx.row(currentRow) = Bk.row(0);
        _Bky.row(currentRow) = Bk.row(1);
        _Bkz.row(currentRow) = Bk.row(2);

        _Bkitx.row(currentRow) = Bkit.row(0);
        _Bkity.row(currentRow) = Bkit.row(0);
        _Bkitz.row(currentRow) = Bkit.row(0);

        _bkx(currentRow) = bk(0);
        _bky(currentRow) = bk(1);
        _bkz(currentRow) = bk(2);

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

const std::vector<std::vector<UnsignedInt>>& MeshData::getElementIndices() const
{
    return _elementIndices;
}

void MeshData::center()
{
    const AABB<Vector3> aabb = computeAABB(_vertices);
    const Vector3 origin = 0.5f*(aabb.max - aabb.min) + aabb.min;
    Magnum::MeshTools::transformPointsInPlace(Matrix4::translation(-origin), _vertices);
}

PointsVectorized3D MeshData::referencePointsToGlobal(const Eigen::MatrixXf& referencePoints) const
{

    const PointsVectorized3D globalPoints { _Bkx * referencePoints, _Bky * referencePoints, _Bkz * referencePoints };

    return globalPoints;
}