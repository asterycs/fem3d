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

#include <imgui.h>
#include <Magnum/ImGuiIntegration/Context.hpp>

#include <memory>

#include "FEMObject3D.h"
#include "Typedefs.h"

using namespace Magnum;

class App : public Platform::Application
{
public:
    explicit App(const Arguments &arguments);

private:
    void viewportEvent(ViewportEvent &event) override;
    void drawEvent() override;
    void mousePressEvent(MouseEvent &event) override;
    void mouseMoveEvent(MouseMoveEvent &event) override;
    void mouseReleaseEvent(MouseEvent &event) override;
    void mouseScrollEvent(MouseScrollEvent &event) override;

    void drawUi();

    void readMeshFiles(const std::vector<std::string>& fnames);

    void keyPressEvent(KeyEvent &event) override;
    void keyReleaseEvent(KeyEvent &event) override;
    void textInputEvent(TextInputEvent &event) override;

    UnsignedInt _currentGeom;
    Scene3D _scene;
    std::unique_ptr<Object3D> _cameraObject;
    std::unique_ptr<SceneGraph::Camera3D> _camera;
    std::vector<SceneGraph::DrawableGroup3D> _drawableGroups;

    PhongIdShader _phongShader;
    VertexShader _vertexSelectionShader;
    CompositionShader _compositionShader;

    std::vector<std::unique_ptr<FEMObject3D>> _objects;

    GL::Framebuffer _framebuffer;
    GL::Renderbuffer _vertexId, _depth;
    GL::Texture2D _color, _transparencyAccumulation, _transparencyRevealage;

    Vector2 _cameraTrackballAngles;

    ImGuiIntegration::Context _imgui{NoCreate};
};

#endif
