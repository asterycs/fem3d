#include "UI.h"

#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Context.h>

#include <Magnum/Shaders/Flat.h>

#include "App.h"

using namespace Magnum::Math::Literals;

UI::UI(App& app, const Vector2i& size, const UnsignedInt nScenes)
        :_imgui{NoCreate}, _currentSize{size}, _nScenes{nScenes}, _currentScene{0}, _showGradient{false},
         _showVertexMarkers{true}, _inPinnedVertexLassoMode{false}, _app{&app}
{
    GL::Context::current().resetState(GL::Context::State::EnterExternal);

    ImGui::CreateContext();
    _imgui = ImGuiIntegration::Context{*ImGui::GetCurrentContext(), Vector2{size}, size, size};

    GL::Context::current().resetState(GL::Context::State::ExitExternal);

    draw();
}

void UI::resize(const Vector2i& size)
{
    _imgui.relayout(Vector2{size}, size, size);
    _currentSize = size;
}

void UI::draw()
{
    GL::Context::current().resetState(GL::Context::State::EnterExternal);

    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add, GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                   GL::Renderer::BlendFunction::OneMinusSourceAlpha);

    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);

    _imgui.newFrame();
    drawOptions();
    _imgui.drawFrame();

    GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);

    GL::Context::current().resetState(GL::Context::State::ExitExternal);

    drawLasso();
}

void UI::drawOptions()
{
    ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    //ImGui::SliderFloat("Float", &_floatValue, 0.0f, 1.0f);
    ImGui::Text("%.3f ms/frame (%.1f FPS)",
                1000.0 / Double(ImGui::GetIO().Framerate), Double(ImGui::GetIO().Framerate));

    if (ImGui::Button(_showGradient ? "dU" : "U", ImVec2(110, 20)))
        _showGradient = !_showGradient;

    ImGui::SameLine();
    if (ImGui::Button("Solve", ImVec2(110, 20)))
        _app->solveCurrent(_showGradient);

    if (ImGui::Button(_showVertexMarkers ? "Markers on" : "Markers off", ImVec2(110, 20)))
    {
        _showVertexMarkers = !_showVertexMarkers;
        _app->setVertexMarkersVisibility(_showVertexMarkers);
    }

    if (ImGui::Button("Clear pinned", ImVec2(110, 20)))
    {
        _app->clearPinnedVertices();
    }

    ImGui::SameLine();
    if (ImGui::Button(_inPinnedVertexLassoMode ? "Lasso on" : "Lasso off", ImVec2(110, 20)))
    {
        _inPinnedVertexLassoMode = !_inPinnedVertexLassoMode;
    }

    std::string sceneButtonLabel{"Scene " + std::to_string(_currentScene)};
    if (ImGui::Button(sceneButtonLabel.c_str(), ImVec2(110, 20)))
    {
        ++_currentScene %= _nScenes;
        _app->setCurrentGeometry(_currentScene);
        _app->setVertexMarkersVisibility(_showVertexMarkers);
    }

    ImGui::End();
}

void UI::drawLasso()
{
    GL::Buffer vertices;
    vertices.setData(_currentLasso.screenCoord, GL::BufferUsage::StaticDraw);

    GL::Mesh mesh;
    mesh.setPrimitive(GL::MeshPrimitive::Points)
            .addVertexBuffer(vertices, 0, Shaders::Flat2D::Position{})
            .setCount(_currentLasso.screenCoord.size());

    Shaders::Flat2D shader;
    shader.setColor(0x2f83cc_rgbf)
            .setTransformationProjectionMatrix(Matrix3());

    mesh.draw(shader);
}

bool UI::wantsTextInput()
{
    return ImGui::GetIO().WantTextInput;
}

bool UI::handleKeyPressEvent(Platform::Application::KeyEvent& event)
{
    return _imgui.handleKeyPressEvent(event);
}

bool UI::handleKeyReleaseEvent(Platform::Application::KeyEvent& event)
{
    return _imgui.handleKeyReleaseEvent(event);
}

bool UI::handleMousePressEvent(Platform::Application::MouseEvent& event)
{
    if (!_imgui.handleMousePressEvent(event))
    {
        if (_inPinnedVertexLassoMode)
        {
            _lassoPreviousPosition = event.position();

            return true;
        }
    }

    return false;
}

bool UI::handleMouseReleaseEvent(Platform::Application::MouseEvent& event)
{
    if (_inPinnedVertexLassoMode && _currentLasso.pixels.size() != 0)
    {
        _app->toggleVertices(_currentLasso);
        _currentLasso.clear();
    }

    return _imgui.handleMouseReleaseEvent(event);
}

std::vector<Vector2> UI::toScreenCoordinates(const std::vector<Vector2i>& pixels)
{
    std::vector<Vector2> output;

    std::transform(pixels.begin(), pixels.end(), std::back_inserter(output),
                   [=](const Vector2i p) -> Vector2
                   {
                     return {static_cast<Float>(p.x()) * 2.0f / _currentSize.x() - 1.f,
                             static_cast<Float>(_currentSize.y() - p.y()) * 2.0f / _currentSize.y() - 1.f};
                   });

    return output;
}

std::vector<Vector2i> bresenhamL(const Vector2i a, const Vector2i b);
std::vector<Vector2i> bresenhamL(const Vector2i a, const Vector2i b)
{
    std::vector<Vector2i> output;

    Vector2i delta = b - a;
    Int ys = Math::sign(delta.y());
    delta.y() = abs(delta.y());

    Int err = 2 * delta.y() - delta.x();
    Int y = a.y();

    for (Int x = a.x(); x <= b.x(); ++x)
    {
        output.push_back({x, y});

        if (err > 0)
        {
            y += ys;
            err -= 2 * delta.x();
        }
        err += 2 * delta.y();
    }

    return output;
}

std::vector<Vector2i> bresenhamH(const Vector2i a, const Vector2i b);
std::vector<Vector2i> bresenhamH(const Vector2i a, const Vector2i b)
{
    std::vector<Vector2i> output;

    Vector2i delta = b - a;
    Int xs = Math::sign(delta.x());
    delta.x() = abs(delta.x());

    Int err = 2 * delta.x() - delta.y();
    Int x = a.x();

    for (Int y = a.y(); y <= b.y(); ++y)
    {
        output.push_back({x, y});

        if (err > 0)
        {
            x += xs;
            err -= 2 * delta.y();
        }
        err += 2 * delta.x();
    }

    return output;
}

std::vector<Vector2i> bresenham(const Vector2i a, const Vector2i b);
std::vector<Vector2i> bresenham(const Vector2i a, const Vector2i b)
{
    if (abs(b.y() - a.y()) < abs(b.x() - a.x()))
    {
        if (a.x() > b.x())
            return bresenhamL(b, a);
        else
            return bresenhamL(a, b);
    }
    else
    {
        if (a.y() > b.y())
            return bresenhamH(b, a);
        else
            return bresenhamH(a, b);
    }
}

bool UI::handleMouseMoveEvent(Platform::Application::MouseMoveEvent& event)
{
    if (_imgui.handleMouseMoveEvent(event))
        return true;

    if (_inPinnedVertexLassoMode && event.relativePosition() != Vector2i{0, 0}
            && event.buttons() & Platform::Application::MouseMoveEvent::Button::Left)
    {
        const auto pixels = bresenham(_lassoPreviousPosition, event.position());

        const auto screenCoord = toScreenCoordinates(pixels);
        _currentLasso.screenCoord.insert(_currentLasso.screenCoord.end(), screenCoord.begin(), screenCoord.end());
        _currentLasso.pixels.insert(_currentLasso.pixels.end(), pixels.begin(), pixels.end());

        _lassoPreviousPosition = event.position();

        return true;
    }

    return false;
}

bool UI::handleMouseScrollEvent(Platform::Application::MouseScrollEvent& event)
{
    return _imgui.handleMouseScrollEvent(event);
}

bool UI::handleTextInputEvent(Platform::Application::TextInputEvent& event)
{
    return _imgui.handleTextInputEvent(event);
}