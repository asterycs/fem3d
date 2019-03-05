#ifndef FEM3D_UI_H
#define FEM3D_UI_H

#include <imgui.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Magnum.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Math/Color.h>

#include <functional>

using namespace Magnum;

class App;

class UI {
public:
    explicit UI(const Vector2i& size);

    void resize(const Vector2i& size);
    void draw();

    void setSolveButtonCallback(std::function<void(bool)> function);
    void setShowVertexMarkersButtonCallback(std::function<void()> function);
    void setChangeGeometryButtonCallback(std::function<void()> function);

    bool wantsTextInput();

    bool handleKeyPressEvent(Platform::Application::KeyEvent& event);
    bool handleKeyReleaseEvent(Platform::Application::KeyEvent& event);

    bool handleMousePressEvent(Platform::Application::MouseEvent& event);
    bool handleMouseReleaseEvent(Platform::Application::MouseEvent& event);
    bool handleMouseMoveEvent(Platform::Application::MouseMoveEvent& event);
    bool handleMouseScrollEvent(Platform::Application::MouseScrollEvent& event);
    bool handleTextInputEvent(Platform::Application::TextInputEvent& event);

private:
    ImGuiIntegration::Context _imgui{NoCreate};
    //float _floatValue;

    bool _showGradient;
    std::function<void(bool)> _solveButtonCallback;
    std::function<void()> _showVertexMarkersButtonCallback;
    std::function<void()> _changeGeometryButtonCallback;
};

#endif //FEM3D_UI_H
