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

FEMObject3D::FEMObject3D(PhongIdShader &phongShader,
                         VertexShader &vertexShader,
                         std::vector<Vector3> vertices,
                         std::vector<UnsignedInt> triangleIndices,
			 std::vector<UnsignedInt> boundryIndices,
                         std::vector<Vector2> uv,
                         std::vector<UnsignedInt> uvIndices,
                         std::vector<UnsignedInt> tetrahedronIndices,
                         Object3D &parent,
                         SceneGraph::DrawableGroup3D &drawables) : Object3D{&parent},
                                                                   SceneGraph::Drawable3D{*this, &drawables},
                                                                   _drawVertexMarkers{true}, _phongShader(phongShader),
                                                                   _vertexShader(vertexShader),
                                                                   _triangleBuffer{GL::Buffer::TargetHint::Array},
                                                                   _indexBuffer{GL::Buffer::TargetHint::ElementArray},
                                                                   _colorBuffer{GL::Buffer::TargetHint::Array},
                                                                   _tetrahedronIndices{tetrahedronIndices},
								   _boundryIndices{boundryIndices}
{
    _vertexMarkerVertexBuffer.resize(vertices.size());
    _vertexMarkerIndexBuffer.resize(vertices.size());
    _vertexMarkerMesh.resize(vertices.size());
    _meshVertices = vertices;
    _boundryIndices=boundryIndices;
    for (UnsignedInt i = 0; i < vertices.size(); ++i)
    {
        const Vector3 center = vertices[i];
        Trade::MeshData3D data = Primitives::uvSphereSolid(16, 32);
        MeshTools::transformPointsInPlace(Matrix4::translation(center) * Matrix4::scaling({0.03f, 0.03f, 0.03f}),
                                          data.positions(0));
        MeshTools::transformVectorsInPlace(Matrix4::translation(center), data.normals(0));

        _vertexMarkerVertexBuffer[i].setTargetHint(GL::Buffer::TargetHint::Array);
        _vertexMarkerVertexBuffer[i].setData(MeshTools::interleave(data.positions(0), data.normals(0)),
                                             GL::BufferUsage::StaticDraw);

        _vertexMarkerIndexBuffer[i].setTargetHint(GL::Buffer::TargetHint::ElementArray);
        _vertexMarkerIndexBuffer[i].setData(MeshTools::compressIndicesAs<UnsignedShort>(data.indices()),
                                            GL::BufferUsage::StaticDraw);

        _vertexMarkerMesh[i].setCount(data.indices().size())
                .setPrimitive(data.primitive())
                .addVertexBuffer(_vertexMarkerVertexBuffer[i], 0, PhongIdShader::Position{}, PhongIdShader::Normal{})
                .setIndexBuffer(_vertexMarkerIndexBuffer[i], 0, MeshIndexType::UnsignedShort);
    }

    std::vector<UnsignedInt> normalIndices;
    std::vector<Vector3> normals;
    std::tie(normalIndices, normals) = MeshTools::generateFlatNormals(triangleIndices, vertices);

    _triangleIndices = triangleIndices;
    vertices = expand(vertices, triangleIndices);
    normals = expand(normals, normalIndices);
    uv = expand(uv, uvIndices);

    std::vector<Vector3> colors(triangleIndices.size(), Vector3{0.f, 0.f, 1.f});

    _triangleBuffer.setData(MeshTools::interleave(vertices, normals, uv), GL::BufferUsage::StaticDraw);
    _colorBuffer.setData(colors, GL::BufferUsage::StaticDraw);

    // Using a vertex buffer would be beneficial but that makes updating colors later much more difficult
    _triangles.setCount(triangleIndices.size())
            .setPrimitive(GL::MeshPrimitive::Triangles)
            .addVertexBuffer(_triangleBuffer, 0, PhongIdShader::Position{}, PhongIdShader::Normal{},
                             PhongIdShader::UV{})
            .addVertexBuffer(_colorBuffer, 0, PhongIdShader::VertexColor{});
}

void FEMObject3D::setTetrahedronColors(const std::vector<Vector3> &colors)
{
    std::vector<Vector3> expandedColor = repeat(colors, 12);
    _colorBuffer.setData(expandedColor, GL::BufferUsage::StaticDraw);
}

void FEMObject3D::setVertexColors(const std::vector<Vector3> &colors)
{
    std::vector<Vector3> expandedColor = expand(colors, _triangleIndices);
    _colorBuffer.setData(expandedColor, GL::BufferUsage::StaticDraw);
}

const std::vector<UnsignedInt>& FEMObject3D::getTetrahedronIndices() const
{
    return _tetrahedronIndices;
}

const std::vector<Vector3>& FEMObject3D::getVertices() const
{
    return _meshVertices;
}

void FEMObject3D::draw(const Matrix4 &transformationMatrix, SceneGraph::Camera3D &camera)
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
            .setLightPosition({13.0f, 2.0f, 5.0f}); // Relative to camera

    _triangles.draw(_phongShader);

    _vertexShader.setTransformationMatrix(
                    transformationMatrix * Matrix4::translation(transformationMatrix.inverted().backward() * 0.01f))
            .setProjectionMatrix(camera.projectionMatrix());

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    if (_drawVertexMarkers)
    {
        for (UnsignedInt i = 0; i < _vertexMarkerMesh.size(); ++i)
        {
            if (_pinnedVertexIds.find(static_cast<Int>(i)) != _pinnedVertexIds.end())
                _vertexShader.setColor({1.f, 0.f, 0.f});
            else
                _vertexShader.setColor({1.f, 1.f, 1.f});

            _vertexShader.setObjectId(i);

            _vertexMarkerMesh[i].draw(_vertexShader);
        }
    }

}

void FEMObject3D::togglePinnedVertex(const UnsignedInt vertexId)
{
    const auto pos = _pinnedVertexIds.find(vertexId);
    if (pos != _pinnedVertexIds.end())
        _pinnedVertexIds.erase(pos);
    else
        _pinnedVertexIds.insert(vertexId);

}

void FEMObject3D::toggleVertexMarkers()
{
    _drawVertexMarkers = !_drawVertexMarkers;
}

bool FEMObject3D::drawsVertexMarkers() const
{
    return _drawVertexMarkers;
}

void FEMObject3D::solve()
{
    FEMTask3D task(_meshVertices, _tetrahedronIndices,_boundryIndices, _pinnedVertexIds);
    std::vector<Float> vertexValues = task.solve();

    if (vertexValues.size() > 0)
    {
        std::vector<Vector3> vertexColors = valuesToHeatGradient(vertexValues);
        this->setVertexColors(vertexColors);
    }
}
