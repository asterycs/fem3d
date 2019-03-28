#ifndef FEM3D_FEMOBJECT3D_H
#define FEM3D_FEMOBJECT3D_H

#include <Magnum/Math/Color.h>

#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

#include <Magnum/GL/Mesh.h>

#include <set>

#include "Shaders.h"
#include "Typedefs.h"
#include "FEMObject.h"
#include "FEMTask3D.h"
#include "Util.h"
#include "MeshData.h"

using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;

class FEMObject3D : public FEMObject{
public:
    explicit FEMObject3D(PhongIdShader& phongShader,
                         VertexShader& vertexShader,
                         const MeshData& mesh,
                         Object3D& parent,
                         Magnum::SceneGraph::DrawableGroup3D& drawables);

    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> solve() override;

    void setTetrahedronColors(const std::vector<Vector3>& colors);

private:
};

#endif //FEM3D_FEMOBJECT3D_H
