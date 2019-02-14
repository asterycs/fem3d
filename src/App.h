#ifndef FEM3D_APP_H
#define FEM3D_APP_H

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Interconnect/Receiver.h>

#include <Magnum/Text/Alignment.h>

#include <Magnum/Platform/Sdl2Application.h>

#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/Camera.h>

#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/AbstractShaderProgram.h>

#include <memory>

#include "FEMObject.h"
#include "Typedefs.h"
#include "UI.h"

using namespace Magnum;

class App: public Platform::Application, public Interconnect::Receiver {
public:
    explicit App(const Arguments& arguments);

private:
    void viewportEvent(ViewportEvent& event) override;
    void drawEvent() override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;

    static void solveButtonCallback();

    void drawUi();

    void keyPressEvent(KeyEvent& event) override;
    void textInputEvent(TextInputEvent& event) override;

    Scene3D _scene;
    std::unique_ptr<Object3D> _cameraObject;
    std::unique_ptr<SceneGraph::Camera3D> _camera;
    SceneGraph::DrawableGroup3D _drawables;

    PhongIdShader _phongShader;
    VertexShader _vertexSelectionShader;

    std::unique_ptr<FEMObject> _object;

    GL::Framebuffer _framebuffer;
    GL::Renderbuffer _color, _vertexId, _depth;

    Vector2i _mousePressPosition;
    Vector2i _previousPosition;
    Vector2 _cameraTrackballAngles;

    Containers::Optional<Ui::UserInterface> _ui;
    Containers::Optional<UiPlane> _baseUiPlane;
};

#endif
