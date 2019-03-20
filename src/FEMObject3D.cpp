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
#include "FEMTaskLinear3D.h"

#include <cassert>

using namespace Magnum::Math::Literals;

FEMObject3D::FEMObject3D(PhongIdShader& phongShader,
                         VertexShader& vertexShader,
                         const MeshData& mesh,
                         Object3D& parent,
                         Magnum::SceneGraph::DrawableGroup3D& drawables)
        :Object3D{&parent},
         Magnum::SceneGraph::Drawable3D{*this, &drawables},
         _drawVertexMarkers{true},
         _pinnedVertexIds{mesh.getBoundaryIndices().begin(), mesh.getBoundaryIndices().end()},
         _phongShader(phongShader),
         _vertexShader(vertexShader),
         _triangleBuffer{Magnum::GL::Buffer::TargetHint::Array},
         _indexBuffer{Magnum::GL::Buffer::TargetHint::ElementArray},
         _colorBuffer{Magnum::GL::Buffer::TargetHint::Array},
         _mesh{mesh}
{
    assert(mesh.getDimensions() == 3);

    // Expand tetrahedrons to triangles for visualization
    const auto triangleIndices = extractTriangleIndices(mesh.getElementIndices());

    initVertexMarkers(mesh.getVertices());
    initMeshTriangles(mesh.getVertices(), triangleIndices);
}

void FEMObject3D::initMeshTriangles(std::vector<Vector3> vertices, std::vector<UnsignedInt> triangleIndices)
{
    _triangleIndices = triangleIndices;

    auto [normalIndices, normals] = Magnum::MeshTools::generateFlatNormals(triangleIndices, vertices);

    vertices = expand(vertices, triangleIndices);
    normals = expand(normals, normalIndices);

    std::vector<Vector3> colors(triangleIndices.size(), Vector3{0.f, 0.f, 1.f});

    _triangleBuffer.setData(Magnum::MeshTools::interleave(vertices, normals), Magnum::GL::BufferUsage::StaticDraw);
    _colorBuffer.setData(colors, Magnum::GL::BufferUsage::StaticDraw);

    // Using a vertex buffer would be beneficial but that makes updating colors later much more difficult
    _triangles.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
            .addVertexBuffer(_triangleBuffer, 0, PhongIdShader::Position{}, PhongIdShader::Normal{})
            .addVertexBuffer(_colorBuffer, 0, PhongIdShader::VertexColor{})
            .setCount(static_cast<Int>(triangleIndices.size()));
}

void FEMObject3D::initVertexMarkers(const std::vector<Vector3>& vertices)
{
    _vertexMarkerVertexBuffer.resize(vertices.size());
    _vertexMarkerIndexBuffer.resize(vertices.size());
    _vertexMarkerMesh.resize(vertices.size());

    const Magnum::Trade::MeshData3D data = Magnum::Primitives::uvSphereSolid(16, 32);

    for (UnsignedInt i = 0; i < vertices.size(); ++i)
    {
        const Vector3 center = vertices[i];

        const auto pointsTformed = Magnum::MeshTools::transformPoints(
                Matrix4::translation(center) * Matrix4::scaling({0.03f, 0.03f, 0.03f}), data.positions(0));
        const auto normalsTformed = Magnum::MeshTools::transformVectors(Matrix4::translation(center), data.normals(0));

        _vertexMarkerVertexBuffer[i].setTargetHint(Magnum::GL::Buffer::TargetHint::Array);
        _vertexMarkerVertexBuffer[i].setData(Magnum::MeshTools::interleave(pointsTformed, normalsTformed),
                                             Magnum::GL::BufferUsage::StaticDraw);

        _vertexMarkerIndexBuffer[i].setTargetHint(Magnum::GL::Buffer::TargetHint::ElementArray);
        _vertexMarkerIndexBuffer[i].setData(Magnum::MeshTools::compressIndicesAs<UnsignedShort>(data.indices()),
                                            Magnum::GL::BufferUsage::StaticDraw);

        _vertexMarkerMesh[i].setCount(static_cast<Int>(data.indices().size()))
                .setPrimitive(data.primitive())
                .addVertexBuffer(_vertexMarkerVertexBuffer[i], 0, PhongIdShader::Position{}, PhongIdShader::Normal{})
                .setIndexBuffer(_vertexMarkerIndexBuffer[i], 0, Magnum::MeshIndexType::UnsignedShort);
    }
}

void FEMObject3D::setTetrahedronColors(const std::vector<Vector3>& colors)
{
    std::vector<Vector3> expandedColor = repeat(colors, 12);
    _colorBuffer.setData(expandedColor, Magnum::GL::BufferUsage::StaticDraw);
}

void FEMObject3D::setVertexColors(const std::vector<Vector3>& colors)
{
    std::vector<Vector3> expandedColor = expand(colors, _triangleIndices);
    _colorBuffer.setData(expandedColor, Magnum::GL::BufferUsage::StaticDraw);
}

void FEMObject3D::draw(const Matrix4& transformationMatrix, Magnum::SceneGraph::Camera3D& camera)
{
    drawMesh(transformationMatrix, camera);

    if (_drawVertexMarkers)
    {
        drawVertexMarkers(transformationMatrix, camera);
    }

}

void FEMObject3D::drawVertexMarkers(const Matrix4& transformationMatrix, const Magnum::SceneGraph::Camera3D& camera)
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

void FEMObject3D::drawMesh(const Matrix4& transformationMatrix, const Magnum::SceneGraph::Camera3D& camera)
{
    Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::DepthTest);
    Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
    Magnum::GL::Renderer::setBlendEquation(Magnum::GL::Renderer::BlendEquation::Add, Magnum::GL::Renderer::BlendEquation::Add);
    Magnum::GL::Renderer::setBlendFunction(Magnum::GL::Renderer::BlendFunction::SourceAlpha,
                                           Magnum::GL::Renderer::BlendFunction::OneMinusSourceAlpha);
    Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::FaceCulling);

    _phongShader.setTransformationMatrix(transformationMatrix)
            .setNormalMatrix(transformationMatrix.rotationScaling())
            .setProjectionMatrix(camera.projectionMatrix())
            .setAmbientColor(Color3{})
            .setDepthScale(0.5f)
            .setLightPosition({13.0f, 2.0f, 5.0f}); // Relative to camera

    _triangles.draw(_phongShader);

    Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
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
    FEMTaskLinear3D task(_mesh, _pinnedVertexIds, std::make_unique<BilinLaplace>(), std::make_unique<LinLaplace>());
    task.initialize();
    /*Eigen::VectorXf solution = task.solve();

    if (solution.size() > 0)
    {
        return task.evaluateSolution(solution);
    }
    else*/
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

