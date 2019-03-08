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
    explicit UI(App& app, const Vector2i& size, const UnsignedInt nScenes);

    void resize(const Vector2i& size);
    void draw();

    bool wantsTextInput();

    bool handleKeyPressEvent(Platform::Application::KeyEvent& event);
    bool handleKeyReleaseEvent(Platform::Application::KeyEvent& event);

    bool handleMousePressEvent(Platform::Application::MouseEvent& event);
    bool handleMouseReleaseEvent(Platform::Application::MouseEvent& event);
    bool handleMouseMoveEvent(Platform::Application::MouseMoveEvent& event);
    bool handleMouseScrollEvent(Platform::Application::MouseScrollEvent& event);
    bool handleTextInputEvent(Platform::Application::TextInputEvent& event);

private:
    std::vector<Vector2> toScreenCoordinates(const std::vector<Vector2i>& pixels);

    ImGuiIntegration::Context _imgui{NoCreate};
    Vector2i _currentSize;

    unsigned int _nScenes;
    unsigned int _currentScene;
    bool _showGradient;
    bool _showVertexMarkers;

    bool _inPinnedVertexLassoMode;
    //bool _inLoadVertexEditMode;
    std::vector<Vector2> _currentLasso;
    Vector2i _lassoPreviousPosition;

    App* _app;
};

#endif //FEM3D_UI_H
