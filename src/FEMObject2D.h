#ifndef FEM3D_FEMOBJECT2D_H
#define FEM3D_FEMOBJECT2D_H

#include "FEMObject.h"

class FEMObject2D : public FEMObject {
public:
    explicit FEMObject2D(PhongIdShader& phongShader,
                         VertexShader& vertexShader,
                         const MeshData& mesh,
                         Object3D& parent,
                         Magnum::SceneGraph::DrawableGroup3D& drawables);

    std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> solve() override;
};

#endif //FEM3D_FEMOBJECT2D_H
