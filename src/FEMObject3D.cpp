#include "FEMObject3D.h"

#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>

#include "FEMTaskLinear3D.h"

using namespace Magnum::Math::Literals;

FEMObject3D::FEMObject3D(PhongIdShader& phongShader,
                         VertexShader& vertexShader,
                         const MeshData& mesh,
                         Object3D& parent,
                         Magnum::SceneGraph::DrawableGroup3D& drawables)
        : FEMObject{phongShader, vertexShader, mesh, parent, drawables}
{

}

std::pair<std::vector<Float>, std::vector<Eigen::Vector3f>> FEMObject3D::solve()
{
    FEMTaskLinear3D task(getMeshData(), getPinnedVertexIds(), std::make_unique<BilinLaplace>(), std::make_unique<LinLaplace>());
    TIME_FUN(&FEMTaskLinear3D::initialize, task);

    FEMTaskLinear3DSolution solution = task.solve();

    const auto asd = solution.evaluate(getMeshData(), task).second;

    for (auto i : asd)
        Magnum::Debug{} << i.transpose();

    if (solution.size() > 0)
    {
        return solution.evaluate(getMeshData(), task);
    }
    else
        return std::make_pair<std::vector<Float>, std::vector<Eigen::Vector3f>>({}, {});
}
