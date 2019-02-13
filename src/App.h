#ifndef FEM3D_APP_H
#define FEM3D_APP_H

#include <Magnum/Platform/Sdl2Application.h>

#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/Camera.h>

#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/AbstractShaderProgram.h>

#include "FEMObject.h"
#include "Typedefs.h"

using namespace Magnum;

class App: public Platform::Application {
public:
    explicit App(const Arguments& arguments);

private:
    void viewportEvent(ViewportEvent& event) override;
    void drawEvent() override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;

    Scene3D _scene;
    Object3D* _cameraObject;
    SceneGraph::Camera3D* _camera;
    SceneGraph::DrawableGroup3D _drawables;

    PhongIdShader _phongShader;
    VertexShader _vertexSelectionShader;
    //Magnum::Shaders::MeshVisualizer _wireframeShader{Shaders::MeshVisualizer::Flag::Wireframe};

    enum { ObjectCount = 1 };
    FEMObject* _objects[ObjectCount];

    GL::Framebuffer _framebuffer;
    GL::Renderbuffer _color, _vertexId, _depth;

    Vector2i _mousePressPosition;
    Vector2i _previousPosition;
    Vector2 _cameraTrackballAngles;
};

#endif
