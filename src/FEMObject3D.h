#ifndef FEM3D_FEMOBJECT_H
#define FEM3D_FEMOBJECT_H

#include <Magnum/Math/Color.h>

#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

#include <Magnum/GL/Mesh.h>

#include <set>

#include "Shaders.h"
#include "Typedefs.h"
#include "FEMTask3D.h"
#include "Util.h"

using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;
using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;

class FEMObject3D : public Object3D, Magnum::SceneGraph::Drawable3D {
public:
    explicit FEMObject3D(PhongIdShader& phongShader,
                         VertexShader& vertexShader,
                         const MeshData& mesh,
                         Object3D& parent,
                         Magnum::SceneGraph::DrawableGroup3D& drawables);

    void togglePinnedVertex(const UnsignedInt vertexId);
    void setPinnedVertex(const UnsignedInt vertexId, const bool pinned);
    void clearPinnedVertices();

    void drawVertexMarkers(const bool);

    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> solve();

    void setTetrahedronColors(const std::vector<Vector3>& colors);
    void setVertexColors(const std::vector<Vector3>& colors);

private:
    void draw(const Matrix4& transformationMatrix, Magnum::SceneGraph::Camera3D& camera) override;
    void drawMesh(const Matrix4& transformationMatrix, const Magnum::SceneGraph::Camera3D& camera);
    void drawVertexMarkers(const Matrix4& transformationMatrix, const Magnum::SceneGraph::Camera3D& camera);

    void initVertexMarkers(const std::vector<Vector3>& vertices);
    void initMeshTriangles(std::vector<Vector3> vertices, std::vector<UnsignedInt> triangleIndices);

    bool _drawVertexMarkers;
    std::set<UnsignedInt> _pinnedVertexIds;

    PhongIdShader& _phongShader;
    VertexShader& _vertexShader;

    Magnum::GL::Buffer _triangleBuffer, _indexBuffer, _colorBuffer;
    Magnum::GL::Mesh _triangles;

    std::vector<Magnum::GL::Buffer> _vertexMarkerVertexBuffer;
    std::vector<Magnum::GL::Buffer> _vertexMarkerIndexBuffer;
    std::vector<Magnum::GL::Mesh> _vertexMarkerMesh;

    MeshData _mesh;
    std::vector<UnsignedInt> _triangleIndices;
};

#endif //FEM3D_FEMOBJECT_H
