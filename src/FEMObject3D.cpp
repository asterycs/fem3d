#include "FEMObject3D.h"

#include <Magnum/GL/Renderer.h>
#include <Magnum/MeshTools/GenerateFlatNormals.h>
#include <Magnum/MeshTools/CompressIndices.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/CombineIndexedArrays.h>
#include <Magnum/MeshTools/Transform.h>

#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/Trade/MeshData3D.h>

#include "Util.h"

using namespace Math::Literals;

FEMObject3D::FEMObject3D(PhongIdShader& phongShader,
                         VertexShader& vertexShader,
                         const std::vector<Vector3>& vertices,
                         const std::vector<UnsignedInt>& boundaryIndices,
                         const std::vector<std::vector<UnsignedInt>>& tetrahedronIndices,
                         Object3D& parent,
                         SceneGraph::DrawableGroup3D& drawables)
        :Object3D{&parent},
         SceneGraph::Drawable3D{*this, &drawables},
         _drawVertexMarkers{true},
         _pinnedVertexIds{boundaryIndices.begin(), boundaryIndices.end()},
         _phongShader(phongShader),
         _vertexShader(vertexShader),
         _triangleBuffer{GL::Buffer::TargetHint::Array},
         _indexBuffer{GL::Buffer::TargetHint::ElementArray},
         _colorBuffer{GL::Buffer::TargetHint::Array},
         _meshVertices{vertices},
         _tetrahedronIndices{tetrahedronIndices},
         _boundaryIndices{boundaryIndices}
{
    // Expand tetrahedrons to triangles for visualization
    const auto triangleIndices = extractTriangleIndices(tetrahedronIndices);

    initVertexMarkers(vertices);
    initTriangles(vertices, triangleIndices);
}

void FEMObject3D::initTriangles(std::vector<Vector3> vertices, std::vector<UnsignedInt> triangleIndices)
{
    _triangleIndices = triangleIndices;

    auto [normalIndices, normals] = MeshTools::generateFlatNormals(triangleIndices, vertices);

    vertices = expand(vertices, triangleIndices);
    normals = expand(normals, normalIndices);

    std::vector<Vector3> colors(triangleIndices.size(), Vector3{0.f, 0.f, 1.f});

    _triangleBuffer.setData(MeshTools::interleave(vertices, normals), GL::BufferUsage::StaticDraw);
    _colorBuffer.setData(colors, GL::BufferUsage::StaticDraw);

    // Using a vertex buffer would be beneficial but that makes updating colors later much more difficult
    _triangles.setPrimitive(GL::MeshPrimitive::Triangles)
            .addVertexBuffer(_triangleBuffer, 0, PhongIdShader::Position{}, PhongIdShader::Normal{})
            .addVertexBuffer(_colorBuffer, 0, PhongIdShader::VertexColor{})
            .setCount(static_cast<Int>(triangleIndices.size()));
}

void FEMObject3D::initVertexMarkers(const std::vector<Vector3>& vertices)
{
    _vertexMarkerVertexBuffer.resize(vertices.size());
    _vertexMarkerIndexBuffer.resize(vertices.size());
    _vertexMarkerMesh.resize(vertices.size());

    const Trade::MeshData3D data = Primitives::uvSphereSolid(16, 32);

    for (UnsignedInt i = 0; i < vertices.size(); ++i)
    {
        const Vector3 center = vertices[i];

        const auto pointsTformed = MeshTools::transformPoints(
                Matrix4::translation(center) * Matrix4::scaling({0.03f, 0.03f, 0.03f}), data.positions(0));
        const auto normalsTformed = MeshTools::transformVectors(Matrix4::translation(center), data.normals(0));

        _vertexMarkerVertexBuffer[i].setTargetHint(GL::Buffer::TargetHint::Array);
        _vertexMarkerVertexBuffer[i].setData(MeshTools::interleave(pointsTformed, normalsTformed),
                                             GL::BufferUsage::StaticDraw);

        _vertexMarkerIndexBuffer[i].setTargetHint(GL::Buffer::TargetHint::ElementArray);
        _vertexMarkerIndexBuffer[i].setData(MeshTools::compressIndicesAs<UnsignedShort>(data.indices()),
                                            GL::BufferUsage::StaticDraw);

        _vertexMarkerMesh[i].setCount(static_cast<Int>(data.indices().size()))
                .setPrimitive(data.primitive())
                .addVertexBuffer(_vertexMarkerVertexBuffer[i], 0, PhongIdShader::Position{}, PhongIdShader::Normal{})
                .setIndexBuffer(_vertexMarkerIndexBuffer[i], 0, MeshIndexType::UnsignedShort);
    }
}

void FEMObject3D::setTetrahedronColors(const std::vector<Vector3>& colors)
{
    std::vector<Vector3> expandedColor = repeat(colors, 12);
    _colorBuffer.setData(expandedColor, GL::BufferUsage::StaticDraw);
}

void FEMObject3D::setVertexColors(const std::vector<Vector3>& colors)
{
    std::vector<Vector3> expandedColor = expand(colors, _triangleIndices);
    _colorBuffer.setData(expandedColor, GL::BufferUsage::StaticDraw);
}

const std::vector<std::vector<UnsignedInt>>& FEMObject3D::getTetrahedronIndices() const
{
    return _tetrahedronIndices;
}

const std::vector<Vector3>& FEMObject3D::getVertices() const
{
    return _meshVertices;
}

void FEMObject3D::draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera)
{
    drawMesh(transformationMatrix, camera);

    if (_drawVertexMarkers)
    {
        drawVertexMarkers(transformationMatrix, camera);
    }

}

void FEMObject3D::drawVertexMarkers(const Matrix4& transformationMatrix, const SceneGraph::Camera3D& camera)
{
    _vertexShader.setTransformationMatrix(
                    transformationMatrix * Matrix4::translation(transformationMatrix.inverted().backward() * 0.01f))
            .setProjectionMatrix(camera.projectionMatrix());

    for (UnsignedInt i = 0; i < _vertexMarkerMesh.size(); ++i)
    {
        if (_pinnedVertexIds.find(i) != _pinnedVertexIds.end())
            _vertexShader.setColor({1.f, 0.f, 0.f});
        else
            _vertexShader.setColor({1.f, 1.f, 1.f});

        _vertexShader.setObjectId(static_cast<Int>(i));

        _vertexMarkerMesh[i].draw(_vertexShader);
    }
}

void FEMObject3D::drawMesh(const Matrix4& transformationMatrix, const SceneGraph::Camera3D& camera)
{
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add, GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                   GL::Renderer::BlendFunction::OneMinusSourceAlpha);
    GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);

    _phongShader.setTransformationMatrix(transformationMatrix)
            .setNormalMatrix(transformationMatrix.rotationScaling())
            .setProjectionMatrix(camera.projectionMatrix())
            .setAmbientColor(Color3{})
            .setDepthScale(0.5f)
            .setLightPosition({13.0f, 2.0f, 5.0f}); // Relative to camera

    _triangles.draw(_phongShader);

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
}

void FEMObject3D::togglePinnedVertex(const UnsignedInt vertexId)
{
    const auto pos = _pinnedVertexIds.find(vertexId);
    if (pos != _pinnedVertexIds.end())
        _pinnedVertexIds.erase(pos);
    else
        _pinnedVertexIds.insert(vertexId);
}

void FEMObject3D::clearPinnedVertices()
{
    _pinnedVertexIds.clear();
}

void FEMObject3D::drawVertexMarkers(const bool draw)
{
    _drawVertexMarkers = draw;
}

std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> FEMObject3D::solve()
{
    FEMTask3D task(_meshVertices, _tetrahedronIndices, _pinnedVertexIds);
    task.initialize();
    Eigen::VectorXf solution = task.solve();

    if (solution.size() > 0)
    {
        return task.evaluateSolution(solution);
    }
    else

        return std::make_pair<std::vector<Float>, std::vector<Eigen::Vector3f>>({}, {});
}

void FEMObject3D::setPinnedVertex(const UnsignedInt vertexId, const bool pinned)
{
    const auto pos = _pinnedVertexIds.find(vertexId);
    if (!pinned)
    {
        if (pos != _pinnedVertexIds.end())
            _pinnedVertexIds.erase(pos);
    }
    else
    {
        _pinnedVertexIds.insert(vertexId);
    }
}

