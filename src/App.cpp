#include "App.h"

#include <Magnum/Image.h>

#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/RenderbufferFormat.h>

#include <Magnum/MeshTools/Transform.h>

#include <random>

#include "Util.h"

using namespace Math::Literals;

App::App(const Arguments& arguments)
        :
        Platform::Application{arguments, Configuration{}
                .setTitle("Finite element")
                .setWindowFlags(Configuration::WindowFlag::Resizable)},
        _currentGeom{0},
        _framebuffer{GL::defaultFramebuffer.viewport()},
        _ui{*this, windowSize(), 3}
{
#ifndef MAGNUM_TARGET_GLES
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);
#endif

    // Note about multisampling:
    // Multisampled storage can be set with
    // renderbuffer.setStorageMultisample(8, GL::RenderbufferFormat::RGBA8, GL::defaultFramebuffer.viewport().size());
    // However, OpenGL requires all attached renderbuffers to have the same number of samples.
    // Thus the object picking would need to be done in a separate render pass with single sample renderbuffer.

    _color.setBaseLevel(0)
            .setMaxLevel(0)
            .setImage(0, GL::TextureFormat::RGBA8, ImageView2D{GL::PixelFormat::RGBA, GL::PixelType::UnsignedByte,
                                                               GL::defaultFramebuffer.viewport().size(), nullptr})
            .setMagnificationFilter(GL::SamplerFilter::Nearest)
            .setMinificationFilter(GL::SamplerFilter::Nearest);

    _vertexId.setStorage(GL::RenderbufferFormat::R32I, GL::defaultFramebuffer.viewport().size());
    _depth.setStorage(GL::RenderbufferFormat::DepthComponent24, GL::defaultFramebuffer.viewport().size());

    // Used for Weight blended order-independent transparency: http://jcgt.org/published/0002/02/09/
    _transparencyAccumulation.setBaseLevel(0)
            .setMaxLevel(0)
            .setMagnificationFilter(GL::SamplerFilter::Nearest)
            .setMinificationFilter(GL::SamplerFilter::Nearest)
            .setImage(0, GL::TextureFormat::RGBA16F,
                      ImageView2D{GL::PixelFormat::RGBA, GL::PixelType::Float, GL::defaultFramebuffer.viewport().size(),
                                  nullptr});

    _transparencyRevealage.setBaseLevel(0)
            .setMaxLevel(0)
            .setMagnificationFilter(GL::SamplerFilter::Nearest)
            .setMinificationFilter(GL::SamplerFilter::Nearest)
            .setImage(0, GL::TextureFormat::R8, ImageView2D{GL::PixelFormat::Red, GL::PixelType::UnsignedByte,
                                                            GL::defaultFramebuffer.viewport().size(), nullptr});

    _framebuffer.attachTexture(GL::Framebuffer::ColorAttachment{_phongShader.ColorOutput}, _color, 0)
            .attachRenderbuffer(GL::Framebuffer::ColorAttachment{_phongShader.ObjectIdOutput}, _vertexId)
            .attachTexture(GL::Framebuffer::ColorAttachment{_phongShader.TransparencyAccumulationOutput},
                           _transparencyAccumulation, 0)
            .attachTexture(GL::Framebuffer::ColorAttachment{_phongShader.TransparencyRevealageOutput},
                           _transparencyRevealage, 0)
            .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, _depth)
            .mapForDraw({{PhongIdShader::ColorOutput, GL::Framebuffer::ColorAttachment{_phongShader.ColorOutput}},
                         {PhongIdShader::ObjectIdOutput, GL::Framebuffer::ColorAttachment{_phongShader.ObjectIdOutput}},
                         {PhongIdShader::TransparencyAccumulationOutput,
                          GL::Framebuffer::ColorAttachment{_phongShader.TransparencyAccumulationOutput}},
                         {PhongIdShader::TransparencyRevealageOutput,
                          GL::Framebuffer::ColorAttachment{_phongShader.TransparencyRevealageOutput}}});

    CORRADE_INTERNAL_ASSERT(_framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    // Configure camera
    _cameraObject = std::make_unique<Object3D>(&_scene);
    _cameraObject->translate(Vector3::zAxis(8.0f))
            .rotate(Math::Rad(Constants::pi() * 0.5f), Vector3{1.f, 0.f, 0.f});
    _camera = std::make_unique<SceneGraph::Camera3D>(*_cameraObject);
    _camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::NotPreserved)
            .setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf,
                                                                Vector2{GL::defaultFramebuffer.viewport().size()}.aspectRatio(),
                                                                0.001f, 100.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());

    const std::vector<std::string> fnames{"geom1.ttg", "geom2.ttg", "geom3.ttg"};
    _drawableGroups.resize(fnames.size());
    readMeshFiles(fnames);

    initUi();
}

void App::initUi()
{

}

void App::readMeshFiles(const std::vector<std::string>& fnames)
{
    Utility::Resource rs("fem3d-data");

    for (UnsignedInt i = 0; i < fnames.size(); ++i)
    {
        const auto fname = fnames[i];
        const auto str = rs.get(fname);
        Mesh3D mesh;

        if (parseTtg(str, mesh))
        {
            const AABB<Vector3> aabb = computeAABB(mesh.vertices);
            const Vector3 origin = 0.5f*(aabb.max - aabb.min) + aabb.min;
            MeshTools::transformPointsInPlace(Matrix4::translation(-origin), mesh.vertices);

            _objects.push_back(
                    std::make_unique<FEMObject3D>(_phongShader, _vertexSelectionShader, mesh, _scene, _drawableGroups[i]));
        }
        else
        {
            Error{} << "Could not parse mesh file " << fname;
        }
    }
}

void App::viewportEvent(ViewportEvent& event)
{
    resizeFramebuffers(event.framebufferSize());
    resizeRenderbuffers(event.framebufferSize());
    resizeTextures(event.framebufferSize());
    resizeCamera(event.framebufferSize());

    _ui.resize(event.windowSize());

    redraw();
}

void App::resizeCamera(const Vector2i& size)
{
    _camera->setProjectionMatrix(
                    Matrix4::perspectiveProjection(35.0_degf, Vector2{size}.aspectRatio(), 0.001f,
                                                   100.0f))
            .setViewport(size);
}

void App::resizeTextures(const Vector2i& size)
{
    _color.setImage(0, GL::TextureFormat::RGBA8,
                    ImageView2D{GL::PixelFormat::RGBA, GL::PixelType::UnsignedByte, size, nullptr});
    _transparencyAccumulation.setImage(0, GL::TextureFormat::RGBA16F,
                                       ImageView2D{GL::PixelFormat::RGBA, GL::PixelType::Float, size,
                                                   nullptr});
    _transparencyRevealage.setImage(0, GL::TextureFormat::R8,
                                    ImageView2D{GL::PixelFormat::Red, GL::PixelType::UnsignedByte,
                                                size, nullptr});
}

void App::resizeRenderbuffers(const Vector2i& size)
{
    _vertexId.setStorage(GL::RenderbufferFormat::R32I, size);
    _depth.setStorage(GL::RenderbufferFormat::DepthComponent24, size);
}

void App::resizeFramebuffers(const Vector2i& size)
{
    GL::defaultFramebuffer.setViewport({{}, size});
    _framebuffer.setViewport({{}, size});
}

void App::drawEvent()
{
    if (_ui.wantsTextInput() && !isTextInputActive())
        startTextInput();
    else if (!_ui.wantsTextInput() && isTextInputActive())
        stopTextInput();

    _framebuffer.clearColor(_phongShader.ColorOutput, Vector4{0.0f})
            .clearColor(_phongShader.ObjectIdOutput, Vector4i{-1})
            .clearColor(_phongShader.TransparencyAccumulationOutput, Vector4{0.0f})
            .clearColor(_phongShader.TransparencyRevealageOutput, Vector4{1.f})
            .clearDepth(1.0f)
            .bind();

    _camera->draw(_drawableGroups[_currentGeom]);

    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth)
            .bind();

    // Compose into default framebuffer
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);

    GL::Mesh fullScreenTriangle;
    fullScreenTriangle.setCount(3).setPrimitive(GL::MeshPrimitive::Triangles);

    _compositionShader.setOpaqueTexture(_color);
    _compositionShader.setTransparencyAccumulationTexture(_transparencyAccumulation);
    _compositionShader.setTransparencyRevealageTexture(_transparencyRevealage);
    _compositionShader.setViewportSize(_framebuffer.viewport().size());
    fullScreenTriangle.draw(_compositionShader);

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

    // Blit render target attachments for debugging
    //_framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{_phongShader.TransparencyAccumulationOutput});
    //GL::AbstractFramebuffer::blit(_framebuffer, GL::defaultFramebuffer,
    //                              {{}, _framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    _ui.draw();

    swapBuffers();
    redraw();
}

void App::mouseScrollEvent(MouseScrollEvent& event)
{
    if (_ui.handleMouseScrollEvent(event))
        redraw();
}

void App::zoomCamera(const Float offset)
{
    // Distance to origin
    const Float distance = _cameraObject->transformation().translation().z();

    // Move 15% of the distance back or forward
    _cameraObject->translate(-_camera->cameraMatrix().inverted().backward() * (
            distance * (1.0f - (offset > 0 ? 1 / 0.85f : 0.85f))));
}

void App::mousePressEvent(MouseEvent& event)
{
    if (_ui.handleMousePressEvent(event))
        redraw();
}

void App::handleViewportClick(const Vector2i position)
{
    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{_phongShader.ObjectIdOutput});
    Image2D data = _framebuffer.read(Range2Di::fromSize({position.x(), _framebuffer.viewport().sizeY() - position.y() - 1},
                              {1, 1}), {PixelFormat::R32I});

    Int selectedVertexId = data.data<Int>()[0];

    if (selectedVertexId >= 0)
    {
        _objects[_currentGeom]->togglePinnedVertex(static_cast<UnsignedInt>(selectedVertexId));
        Debug{} << "Toggled vertex number " << selectedVertexId;
    }
}

void App::setVertices(const UI::Lasso& lasso, const bool pinned)
{
    if (lasso.pixels.size() == 0)
        return;

    const auto [min, max] = computeAABB(lasso.pixels);

    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{_phongShader.ObjectIdOutput});
    const Image2D data = _framebuffer.read(
            Range2Di({min.x(), _framebuffer.viewport().sizeY() - max.y() - 1},
                               {max.x(), _framebuffer.viewport().sizeY() - min.y() - 1}), {GL::PixelFormat::RedInteger, GL::PixelType::Int});

    std::set<UnsignedInt> seenIndices;

    const Vector2i size = max - min;
    for (Int i = 0; i < size.x()*size.y(); ++i)
    {
        const Int index = data.data<Int>()[i];

        // -1 contains no vertex
        if (index > -1)
        {
            seenIndices.insert(static_cast<UnsignedInt>(index));
        }
    }

    for (UnsignedInt index : seenIndices)
        _objects[_currentGeom]->setPinnedVertex(index, pinned);
}

void App::mouseMoveEvent(MouseMoveEvent& event)
{
    if (_ui.handleMouseMoveEvent(event))
        redraw();
}

void App::rotateCamera(const Vector2i offset)
{
    const Float cameraMovementSpeed = 0.005f;
    _cameraTrackballAngles[0] += static_cast<Float>(offset[0]) * cameraMovementSpeed;

    // Restrict the up-down angle
    if ((_cameraTrackballAngles[1] < Constants::pi() * 0.5f && offset[1] > 0)
            || (_cameraTrackballAngles[1] > -Constants::pi() * 0.5f && offset[1] < 0))
    {

        _cameraTrackballAngles[1] += static_cast<Float>(offset[1]) * cameraMovementSpeed;
        _cameraObject->rotate(Math::Rad(-static_cast<Float>(offset[1]) * cameraMovementSpeed),
                              _camera->cameraMatrix().inverted().right().normalized());
    }

    _cameraObject->rotate(Math::Rad(-static_cast<Float>(offset[0]) * cameraMovementSpeed), Vector3(0.f, 0.f, 1.f));
}

void App::mouseReleaseEvent(MouseEvent& event)
{
    if (_ui.handleMouseReleaseEvent(event))
        redraw();
}

void App::textInputEvent(TextInputEvent& event)
{
    if (_ui.handleTextInputEvent(event))
        redraw();

}

void App::keyPressEvent(KeyEvent& event)
{
    if (_ui.handleKeyPressEvent(event))
        redraw();
}

void App::keyReleaseEvent(KeyEvent& event)
{
    if (_ui.handleKeyReleaseEvent(event))
        redraw();
}

void App::setVertexMarkersVisibility(bool show)
{
    _objects[_currentGeom]->drawVertexMarkers(show);
}

void App::solveCurrent(bool showGradient)
{
    const auto[U, dU] = _objects[_currentGeom]->solve();

    if (U.size() > 0 && dU.size() > 0)
    {
        std::vector<Float> magnitudes;

        if (showGradient)
            magnitudes = computeNorm(dU);
        else
            magnitudes = U;

        std::vector<Vector3> vertexColors = valuesToHeatGradient(magnitudes);
        _objects[_currentGeom]->setVertexColors(vertexColors);
    }
}

void App::setCurrentGeometry(const UnsignedInt geometry)
{
    _currentGeom = geometry;
}

void App::clearPinnedVertices()
{
    _objects[_currentGeom]->clearPinnedVertices();
}