#include "FEMObject.h"

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

FEMObject::FEMObject(PhongIdShader& phongShader, VertexShader& vertexShader, std::vector<Vector3> vertices, std::vector<UnsignedInt> triangleIndices, std::vector<Vector2> uv, std::vector<UnsignedInt> uvIndices, std::vector<UnsignedInt> tetrahedronIndices, Object3D& parent, SceneGraph::DrawableGroup3D& drawables): Object3D{&parent}, SceneGraph::Drawable3D{*this, &drawables},  _highlightedVertexId{-1}, _selected{false}, _phongShader(phongShader), _vertexShader(vertexShader), _color{0xffffff_rgbf}, _triangleBuffer{GL::Buffer::TargetHint::Array}, _indexBuffer{GL::Buffer::TargetHint::ElementArray},_colorBuffer{GL::Buffer::TargetHint::Array}, _tetrahedronIndices{tetrahedronIndices}
{
    /*_cubeVertices{GL::Buffer::TargetHint::Array}, _cubeIndices{GL::Buffer::TargetHint::ElementArray},
            _sphereVertices{GL::Buffer::TargetHint::Array}, _sphereIndices{GL::Buffer::TargetHint::ElementArray},
            _planeVertices{GL::Buffer::TargetHint::Array}, _framebuffer{GL::defaultFramebuffer.viewport()}
*/
    _vertexMarkerVertexBuffer.resize(vertices.size());
    _vertexMarkerIndexBuffer.resize(vertices.size());
    _vertexMarkerMesh.resize(vertices.size());
    for (UnsignedInt i = 0; i < vertices.size(); ++i)
    {
        const Vector3 center = vertices[i];
        Trade::MeshData3D data = Primitives::uvSphereSolid(16, 32);
        MeshTools::transformPointsInPlace(Matrix4::translation(center)*Matrix4::scaling({0.05f, 0.05f, 0.05f}), data.positions(0));
        MeshTools::transformVectorsInPlace(Matrix4::translation(center), data.normals(0));

        _vertexMarkerVertexBuffer[i].setTargetHint(GL::Buffer::TargetHint::Array);
        _vertexMarkerVertexBuffer[i].setData(MeshTools::interleave(data.positions(0), data.normals(0)), GL::BufferUsage::StaticDraw);

        _vertexMarkerIndexBuffer[i].setTargetHint(GL::Buffer::TargetHint::ElementArray);
        _vertexMarkerIndexBuffer[i].setData(MeshTools::compressIndicesAs<UnsignedShort>(data.indices()), GL::BufferUsage::StaticDraw);

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

    std::vector<Vector3> colors(triangleIndices.size(),Vector3{0.25f, 0.25f, 1.f});

    _triangleBuffer.setData(MeshTools::interleave(vertices, normals, uv), GL::BufferUsage::StaticDraw);
    _colorBuffer.setData(colors, GL::BufferUsage::StaticDraw);

    _triangles.setCount(triangleIndices.size())
            .setPrimitive(GL::MeshPrimitive::Triangles)
            .addVertexBuffer(_triangleBuffer, 0, PhongIdShader::Position{}, PhongIdShader::Normal{}, PhongIdShader::UV{})
            .addVertexBuffer(_colorBuffer, 0, PhongIdShader::VertexColor{});
}

void FEMObject::setTetrahedronColors(const std::vector<Vector3> &colors)
{
    std::vector<UnsignedInt> triangleColorIndices;
    extractTriangleIndices(_tetrahedronIndices, triangleColorIndices);

    std::vector<Vector3> expandedColor = repeat(colors, 12);
    _colorBuffer.setData(expandedColor, GL::BufferUsage::StaticDraw);
}

std::vector<UnsignedInt> FEMObject::getTetrahedronIndices() const
{
    return _tetrahedronIndices;
}

bool FEMObject::isSelected() const
{
    return _selected;
}

void FEMObject::draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) {
    _phongShader.setTransformationMatrix(transformationMatrix)
        .setNormalMatrix(transformationMatrix.rotationScaling())
        .setProjectionMatrix(camera.projectionMatrix())
        .setAmbientColor(Color3{})
        .setLightPosition({13.0f, 2.0f, 5.0f}); // Relative to camera

    _triangles.draw(_phongShader);

    _vertexShader.setTransformationMatrix(transformationMatrix*Matrix4::translation(transformationMatrix.inverted().backward()*0.01f))
        .setProjectionMatrix(camera.projectionMatrix());

    for (UnsignedInt i = 0; i < _vertexMarkerMesh.size(); ++i)
    {
        if (_highlightedVertexId == Int(i) && _selected)
            _vertexShader.setColor({1.f, 0.f, 0.f});
        else
            _vertexShader.setColor({1.f, 1.f, 1.f});

        _vertexShader.setObjectId(i);

        _vertexMarkerMesh[i].draw(_vertexShader);
    }

    //_wireframeShader
    //       .setViewportSize(Vector2{GL::defaultFramebuffer.viewport().size()})
    //       .setTransformationProjectionMatrix(camera.projectionMatrix()*transformationMatrix*Matrix4::translation(transformationMatrix.inverted().backward()*0.01f));

    //_mesh.draw(_wireframeShader);
}

void FEMObject::setSelected(const bool selected)
{
    _selected = selected;
}

void FEMObject::setSelectedVertex(const Int vertexId)
{
    _highlightedVertexId = vertexId;
}
