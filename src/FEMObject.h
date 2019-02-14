#ifndef GP2_FEMOBJECT_H
#define GP2_FEMOBJECT_H

#include <Magnum/Math/Color.h>

#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

#include <Magnum/GL/Mesh.h>

#include <set>

#include "Shaders.h"
#include "Typedefs.h"

using namespace Magnum;

class FEMObject: public Object3D, SceneGraph::Drawable3D {
public:
    explicit FEMObject(PhongIdShader& phongShader, VertexShader& vertexShader, std::vector<Vector3> vertices, std::vector<UnsignedInt> triangleIndices, std::vector<Vector2> uv, std::vector<UnsignedInt> uvIndices, std::vector<UnsignedInt> tetrahedronIndices, Object3D& parent, SceneGraph::DrawableGroup3D& drawables);
    void togglePinnedVertec(const Int vertexId);

    // Something like this
    // void applyForce(

    std::vector<UnsignedInt> getTetrahedronIndices() const;
    void setTetrahedronColors(const std::vector<Vector3>& colors);

private:
    void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override;

    std::set<Int> _pinnedVertexIds;
    PhongIdShader& _phongShader;
    VertexShader& _vertexShader;

    Color3 _color;
    GL::Buffer _triangleBuffer, _indexBuffer, _colorBuffer;
    GL::Mesh _triangles;

    std::vector<GL::Buffer> _vertexMarkerVertexBuffer;
    std::vector<GL::Buffer> _vertexMarkerIndexBuffer;
    std::vector<GL::Mesh> _vertexMarkerMesh;

    std::vector<UnsignedInt> _triangleIndices;
    std::vector<UnsignedInt> _tetrahedronIndices;
};


#endif //GP2_FEMOBJECT_H
