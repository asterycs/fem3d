#ifndef FEM3D_FEMTASK_H
#define FEM3D_FEMTASK_H

#include <Magnum/Magnum.h>

#include <set>
#include <vector>

using namespace Magnum;

// This class handles matrix assembling and solving of the resulting system
class FEMTask3D
{
public:



private:
    std::set<Int> _pinnedVertexIds;

    std::vector<Vector3> _vertices;
    std::vector<UnsignedInt> _tetrahedronIndices;
};


#endif //FEM3D_FEMTASK_H
