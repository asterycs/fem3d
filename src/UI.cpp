#include "UI.h"

constexpr const Float WidgetHeight{36.0f};
constexpr const Vector2 ButtonSize{96.0f, WidgetHeight};

UiPlane::UiPlane(Ui::UserInterface& ui):
    Ui::Plane{ui, Ui::Snap::Top|Ui::Snap::Bottom|Ui::Snap::Left|Ui::Snap::Right, 0, 50, 640},
    solveButton{*this, {Ui::Snap::Top|Ui::Snap::Left, Range2D::fromSize(Vector2::yAxis(0), ButtonSize)},
    "Solve", Ui::Style::Default}
{

}
