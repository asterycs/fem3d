#include "App.h"

#include <Corrade/Utility/Resource.h>

#include <Magnum/Image.h>
#include <Magnum/PixelFormat.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/RenderbufferFormat.h>

#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/UVSphere.h>

#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/AbstractImporter.h>

#include <Magnum/MeshTools/Transform.h>

#include <sstream>
#include <limits>
#include <random>

#include "Util.h"

using namespace Math::Literals;

App::App(const Arguments& arguments):
        Platform::Application{arguments, Configuration{}
                .setTitle("Finite element")
                .setWindowFlags(Configuration::WindowFlag::Resizable)},
         _framebuffer{GL::defaultFramebuffer.viewport()}
{
#ifndef MAGNUM_TARGET_GLES
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);
#endif

    _color.setStorage(GL::RenderbufferFormat::RGBA8, GL::defaultFramebuffer.viewport().size());
    _vertexId.setStorage(GL::RenderbufferFormat::R32I, GL::defaultFramebuffer.viewport().size());
    _depth.setStorage(GL::RenderbufferFormat::DepthComponent24, GL::defaultFramebuffer.viewport().size());
    _framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, _color)
            .attachRenderbuffer(GL::Framebuffer::ColorAttachment{1}, _vertexId)
            .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, _depth)
            .mapForDraw({{PhongIdShader::ColorOutput, GL::Framebuffer::ColorAttachment{0}},
                         {VertexShader::ColorOutput, GL::Framebuffer::ColorAttachment{0}},
                         {PhongIdShader::ObjectIdOutput, GL::Framebuffer::ColorAttachment{1}},
                         {VertexShader::ObjectIdOutput, GL::Framebuffer::ColorAttachment{1}}});
    CORRADE_INTERNAL_ASSERT(_framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    Utility::Resource rs("fem3d-data");

    const auto str = rs.get("cube.ttg");
    std::vector<Vector3> vertices;
    std::vector<Vector2> uv;
    std::vector<UnsignedInt> triangleIndices;
    std::vector<UnsignedInt> uvIndices;
    std::vector<UnsignedInt> tetrahedronIndices;

    parseTtg(str, vertices, uv, triangleIndices, uvIndices, tetrahedronIndices);

    Vector3 origin,extent;
    computeAABB(vertices, origin, extent);
    MeshTools::transformPointsInPlace(Matrix4::translation(-origin), vertices);

    _object = std::make_unique<FEMObject>(_phongShader, _vertexSelectionShader, vertices, triangleIndices, uv, uvIndices, tetrahedronIndices, _scene, _drawables);

    /* Configure camera */
    _cameraObject = std::make_unique<Object3D>(&_scene);
    _cameraObject->translate(Vector3::zAxis(8.0f));
    _camera = std::make_unique<SceneGraph::Camera3D>(*_cameraObject);
    _camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, 4.0f/3.0f, 0.001f, 100.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());

    // set up ui
    Ui::StyleConfiguration style = Ui::defaultStyleConfiguration();
    _ui.emplace(Math::clamp({640.0f, 480.0f}, {1024.0f, 576.0f}, Vector2(windowSize()/dpiScaling())), windowSize(), framebufferSize(), style, "»");
    _baseUiPlane.emplace(*_ui);
    Interconnect::connect(_baseUiPlane->solveButton, &Ui::Button::tapped, &App::solveButtonCallback);
}

void App::viewportEvent(ViewportEvent& event) {
    GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
}

void App::drawEvent() {
    _framebuffer
            .clearColor(0, Color3{0.0f})
            .clearColor(1, Vector4i{-1})
            .clearDepth(1.0f)
            .bind();
    _camera->draw(_drawables);

    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth)
            .bind();

    // Blit color to main fb
    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0});
    GL::AbstractFramebuffer::blit(_framebuffer, GL::defaultFramebuffer,
                                  {{}, _framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    drawUi();

    swapBuffers();
}


void App::mouseScrollEvent(MouseScrollEvent& event) {
    if(!event.offset().y()) return;

    // Distance to origin
    const Float distance = _cameraObject->transformation().translation().z();

    // Move 15% of the distance back or forward
    _cameraObject->translate(-_camera->cameraMatrix().inverted().backward()*(
            distance*(1.0f - (event.offset().y() > 0 ? 1/0.85f : 0.85f))));

    redraw();
}


void App::mousePressEvent(MouseEvent& event) {
    if (_ui->handlePressEvent(event.position()))
    {
        event.setAccepted();
        redraw();
        return;
    }

    if(event.button() != MouseEvent::Button::Left)
        return;

    _previousPosition = event.position();
    _mousePressPosition = _previousPosition;
    event.setAccepted();

    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{1});
    Image2D data = _framebuffer.read(
            Range2Di::fromSize({event.position().x(), _framebuffer.viewport().sizeY() - event.position().y() - 1}, {1, 1}),
            {PixelFormat::R32I});

    Int selectedVertexId = data.data<Int>()[0];
    _object->togglePinnedVertec(selectedVertexId);

    Debug{} << "selected vertex: " << selectedVertexId;

/*    {
        std::vector<Vector3> newColors{_object->getTetrahedronIndices().size() / 4, Vector3{0.f, 0.f, 1.f}};

        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<UnsignedInt> uni(0, newColors.size() - 1);

        const auto r = uni(rng);
        newColors[r] = {1.f, 0.f, 0.f};
        _object->setTetrahedronColors(newColors);
    }*/
    redraw();
}

void App::mouseMoveEvent(MouseMoveEvent& event) {
    if (_ui->handleMoveEvent(event.position()))
    {
        event.setAccepted();
        redraw();
        return;
    }

    if(!(event.buttons() & MouseMoveEvent::Button::Left))
        return;

    const Vector2i currentPosition = event.position();
    const Vector2i posDiff = currentPosition - _previousPosition;

    if(posDiff.length() < 0.001f)
        return;

    _cameraTrackballAngles[0] -= static_cast<Float>(posDiff[0]) * 0.01f;
    _cameraTrackballAngles[1] -= static_cast<Float>(posDiff[1]) * 0.01f;

    _cameraObject->rotate(Math::Rad(-posDiff[1] * 0.005f), _camera->cameraMatrix().inverted().right().normalized());
    _cameraObject->rotate(Math::Rad(-posDiff[0] * 0.005f), Vector3(0.f, 1.f, 0.f));
    _previousPosition = currentPosition;

    event.setAccepted();
    redraw();
}

void App::mouseReleaseEvent(MouseEvent& event) {
    if (_ui->handleReleaseEvent(event.position()))
    {
        event.setAccepted();
        redraw();
        return;
    }

    if(event.button() != MouseEvent::Button::Left)
        return;

    event.setAccepted();
    redraw();
}

void App::textInputEvent(TextInputEvent &event)
{
   if(isTextInputActive() && _ui->focusedInputWidget() && _ui->focusedInputWidget()->handleTextInput(event))
       redraw();

}

void App::keyPressEvent(KeyEvent& event) {
    if(isTextInputActive() && _ui->focusedInputWidget() && _ui->focusedInputWidget()->handleKeyPress(event))
        redraw();
}

void App::drawUi()
{
    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::One,
                                   GL::Renderer::BlendFunction::OneMinusSourceAlpha);
    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add, GL::Renderer::BlendEquation::Add);
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    _ui->draw();
}

void App::solveButtonCallback()
{
    Debug{} << "I do nothing!";
}
