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

App::App(const Arguments &arguments) :
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
            .mapForDraw({{PhongIdShader::ColorOutput,    GL::Framebuffer::ColorAttachment{0}},
                         {VertexShader::ColorOutput,     GL::Framebuffer::ColorAttachment{0}},
                         {PhongIdShader::ObjectIdOutput, GL::Framebuffer::ColorAttachment{1}},
                         {VertexShader::ObjectIdOutput,  GL::Framebuffer::ColorAttachment{1}}});
    CORRADE_INTERNAL_ASSERT(_framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    /* Configure camera */
    _cameraObject = std::make_unique<Object3D>(&_scene);
    _cameraObject->translate(Vector3::zAxis(8.0f));
    _camera = std::make_unique<SceneGraph::Camera3D>(*_cameraObject);
    _camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::NotPreserved)
            .setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, Vector2{GL::defaultFramebuffer.viewport().size()}.aspectRatio(), 0.001f, 100.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());

    initUi();

    readMeshFile("cube.ttg");
}

void App::initUi()
{
    Ui::StyleConfiguration style = Ui::defaultStyleConfiguration();

    _baseUiPlane.reset();
    _ui.reset();

    _ui = std::make_unique<Ui::UserInterface>(Vector2(GL::defaultFramebuffer.viewport().size()), windowSize(), framebufferSize(), style, "»");
    _baseUiPlane = std::make_unique<UiPlane>(*_ui);

    Interconnect::connect(_baseUiPlane->toggleVertexMarkersButton, &Ui::Button::tapped, *this,
                          &App::toggleVertexMarkersButtonCallback);
    Interconnect::connect(_baseUiPlane->solveButton, &Ui::Button::tapped, *this, &App::solveButtonCallback);
}

void App::readMeshFile(const std::string& fname)
{
    Utility::Resource rs("fem3d-data");

    const auto str = rs.get(fname);
    std::vector<Vector3> vertices;
    std::vector<UnsignedInt> meshElementIndices;
    UnsignedInt dim;

    if (parseTtg(str, vertices, meshElementIndices, dim) && dim == 3)
    {
        Vector3 origin, extent;
        computeAABB(vertices, origin, extent);
        MeshTools::transformPointsInPlace(Matrix4::translation(-origin), vertices);

        std::vector<Vector2> uv;
        std::vector<UnsignedInt> triangleIndices;
        std::vector<UnsignedInt> uvIndices;

        // Expand tetrahedrons to triangles for visualization
        extractTriangleIndices(meshElementIndices, triangleIndices);

        // Create uv data. Not used atm.
        createUVIndices(triangleIndices, uv, uvIndices);

        _object = std::make_unique<FEMObject3D>(_phongShader, _vertexSelectionShader, vertices, triangleIndices, uv,
                                                uvIndices, meshElementIndices, _scene, _drawables);
    } else
    {
        Error{} << "Could not parse mesh file";
    }
}

void App::viewportEvent(ViewportEvent &event)
{
    GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});

    _framebuffer.setViewport({{}, event.framebufferSize()});
    _color.setStorage(GL::RenderbufferFormat::RGBA8, event.framebufferSize());
    _vertexId.setStorage(GL::RenderbufferFormat::R32I, event.framebufferSize());
    _depth.setStorage(GL::RenderbufferFormat::DepthComponent24, event.framebufferSize());

    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, Vector2{event.framebufferSize()}.aspectRatio(), 0.001f, 100.0f))
            .setViewport(event.framebufferSize());

    initUi();
    redraw();
}

void App::drawEvent()
{
    _framebuffer.clearColor(0, Color3{0.0f})
            .clearColor(1, Vector4i{-1})
            .clearDepth(1.0f)
            .bind();
    _camera->draw(_drawables);

    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth)
            .bind();


    // Blit color to main fb
    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0});
    GL::AbstractFramebuffer::blit(_framebuffer, GL::defaultFramebuffer,
                                  {{}, _framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    drawUi();

    swapBuffers();
}


void App::mouseScrollEvent(MouseScrollEvent &event)
{
    if (!event.offset().y()) return;

    // Distance to origin
    const Float distance = _cameraObject->transformation().translation().z();

    // Move 15% of the distance back or forward
    _cameraObject->translate(-_camera->cameraMatrix().inverted().backward() * (
            distance * (1.0f - (event.offset().y() > 0 ? 1 / 0.85f : 0.85f))));

    redraw();
}


void App::mousePressEvent(MouseEvent &event)
{
    if (_ui->handlePressEvent(event.position()))
    {
        event.setAccepted();
        redraw();
        return;
    }

    if (event.button() != MouseEvent::Button::Left)
        return;


    event.setAccepted();

    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{1});
    Image2D data = _framebuffer.read(
            Range2Di::fromSize({event.position().x(), _framebuffer.viewport().sizeY() - event.position().y() - 1},
                               {1, 1}),
            {PixelFormat::R32I});

    Int selectedVertexId = data.data<Int>()[0];

    if (selectedVertexId >= 0)
    {
        _object->togglePinnedVertex(static_cast<UnsignedInt>(selectedVertexId));
        Debug{} << "Toggled vertex number " << selectedVertexId;
    }

    redraw();
}

void App::mouseMoveEvent(MouseMoveEvent &event)
{
    if (_ui->handleMoveEvent(event.position()))
    {
        event.setAccepted();
        redraw();
        return;
    }

    if (!(event.buttons() & MouseMoveEvent::Button::Left))
        return;

    const Vector2i posDiff = event.relativePosition();

    _cameraTrackballAngles[0] -= static_cast<Float>(posDiff[0]) * 0.01f;
    _cameraTrackballAngles[1] -= static_cast<Float>(posDiff[1]) * 0.01f;

    _cameraObject->rotate(Math::Rad(-posDiff[1] * 0.005f), _camera->cameraMatrix().inverted().right().normalized());
    _cameraObject->rotate(Math::Rad(-posDiff[0] * 0.005f), Vector3(0.f, 1.f, 0.f));

    event.setAccepted();
    redraw();
}

void App::mouseReleaseEvent(MouseEvent &event)
{
    if (_ui->handleReleaseEvent(event.position()))
    {
        event.setAccepted();
        redraw();
        return;
    }

    if (event.button() != MouseEvent::Button::Left)
        return;

    event.setAccepted();
    redraw();
}

void App::textInputEvent(TextInputEvent &event)
{
    if (isTextInputActive() && _ui->focusedInputWidget() && _ui->focusedInputWidget()->handleTextInput(event))
        redraw();

}

void App::keyPressEvent(KeyEvent &event)
{
    if (isTextInputActive() && _ui->focusedInputWidget() && _ui->focusedInputWidget()->handleKeyPress(event))
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

void App::toggleVertexMarkersButtonCallback()
{
    _object->toggleVertexMarkers();
}

void App::solveButtonCallback()
{
    _object->solve();
}
