#include "FEMObject2D.h"

FEMObject2D::FEMObject2D(PhongIdShader& phongShader,
                         VertexShader& vertexShader,
                         const MeshData& mesh,
                         Object3D& parent,
                         Magnum::SceneGraph::DrawableGroup3D& drawables)
        : FEMObject{phongShader, vertexShader, mesh, parent, drawables}
{

}

std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> FEMObject2D::solve()
{
    return std::make_pair<std::vector<Float>, std::vector<Eigen::Vector3f>>({}, {});
}
