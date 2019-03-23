#ifndef FEM3D_UI_H
#define FEM3D_UI_H

#include <imgui.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Magnum.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Math/Color.h>

#include "Typedefs.h"

#include <functional>

class App;

class UI {
public:
    struct Lasso
    {
      std::vector<Vector2i> pixels;
      std::vector<Vector2> screenCoord;

      void clear()
      {
          pixels.clear();
          screenCoord.clear();
      }
    };

    explicit UI(App& app, const Vector2i& size, const UnsignedInt nScenes);

    void resize(const Vector2i& size);
    void draw();

    bool wantsTextInput();

    bool handleKeyPressEvent(Magnum::Platform::Application::KeyEvent& event);
    bool handleKeyReleaseEvent(Magnum::Platform::Application::KeyEvent& event);

    bool handleMousePressEvent(Magnum::Platform::Application::MouseEvent& event);
    bool handleMouseReleaseEvent(Magnum::Platform::Application::MouseEvent& event);
    bool handleMouseMoveEvent(Magnum::Platform::Application::MouseMoveEvent& event);
    bool handleMouseScrollEvent(Magnum::Platform::Application::MouseScrollEvent& event);
    bool handleTextInputEvent(Magnum::Platform::Application::TextInputEvent& event);

private:
    void drawOptions();
    void drawLasso();
    std::vector<Vector2> toScreenCoordinates(const std::vector<Vector2i>& pixels);

    Magnum::ImGuiIntegration::Context _imgui{Magnum::NoCreate};
    Vector2i _currentSize;

    unsigned int _nScenes;
    unsigned int _currentScene;
    bool _showGradient;
    bool _showVertexMarkers;
    bool _showAbout;

    bool _inPinnedVertexLassoMode;
    //bool _inLoadVertexEditMode;
    Lasso _currentLasso;
    Vector2i _lassoPreviousPosition;

    App& _app;
};

#endif //FEM3D_UI_H
