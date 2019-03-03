#ifndef FEM3D_UI_H
#define FEM3D_UI_H

#include "Magnum/Ui/Anchor.h"
#include "Magnum/Ui/Button.h"
#include "Magnum/Ui/Input.h"
#include "Magnum/Ui/Label.h"
#include "Magnum/Ui/Modal.h"
#include "Magnum/Ui/Plane.h"
#include "Magnum/Ui/UserInterface.h"

#include <memory>

using namespace Magnum;
using namespace Magnum::Math::Literals;

struct UiPlane: Ui::Plane {
    explicit UiPlane(Ui::UserInterface& ui);

    Ui::Button toggleVertexMarkersButton;
    Ui::Button solveButton;
    Ui::Button geomButton;
};

#endif //FEM3D_UI_H
