#include "UI.h"

#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Context.h>

using namespace Magnum::Math::Literals;

UI::UI(const Vector2i& size, const UnsignedInt nScenes)
        :_imgui{NoCreate}, _nScenes{nScenes}, _currentScene{0}, _showGradient{false},
         _showVertexMarkers{true}
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

    {
        ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        //ImGui::SliderFloat("Float", &_floatValue, 0.0f, 1.0f);
        ImGui::Text("%.3f ms/frame (%.1f FPS)",
                    1000.0 / Double(ImGui::GetIO().Framerate), Double(ImGui::GetIO().Framerate));

        if (ImGui::Button(_showGradient ? "dU" : "U", ImVec2(110, 20)))
            _showGradient = !_showGradient;

        ImGui::SameLine();
        if (ImGui::Button("Solve", ImVec2(110, 20)))
            _solveButtonCallback(_showGradient);

        if (ImGui::Button(_showVertexMarkers ? "Markers on" : "Markers off", ImVec2(110, 20)))
        {
            _showVertexMarkers = !_showVertexMarkers;
            _showVertexMarkersButtonCallback(_showVertexMarkers);
        }

        if (ImGui::Button("Clear pinned", ImVec2(110, 20)))
        {
            _clearPinnedVerticesCallback();
        }

        std::string sceneButtonLabel{"Scene " + std::to_string(_currentScene)};
        if (ImGui::Button(sceneButtonLabel.c_str(), ImVec2(110, 20)))
        {
            ++_currentScene %= _nScenes;
            _changeGeometryButtonCallback(_currentScene);
            _showVertexMarkersButtonCallback(_showVertexMarkers);
        }

        ImGui::End();
    }

    _imgui.drawFrame();

    GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);

    GL::Context::current().resetState(GL::Context::State::ExitExternal);
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
    return _imgui.handleMousePressEvent(event);
}

bool UI::handleMouseReleaseEvent(Platform::Application::MouseEvent& event)
{
    return _imgui.handleMouseReleaseEvent(event);
}

bool UI::handleMouseMoveEvent(Platform::Application::MouseMoveEvent& event)
{
    return _imgui.handleMouseMoveEvent(event);
}

bool UI::handleMouseScrollEvent(Platform::Application::MouseScrollEvent& event)
{
    return _imgui.handleMouseScrollEvent(event);
}

bool UI::handleTextInputEvent(Platform::Application::TextInputEvent& event)
{
    return _imgui.handleTextInputEvent(event);
}

void UI::setSolveButtonCallback(std::function<void(bool)> function)
{
    _solveButtonCallback = function;
}

void UI::setShowVertexMarkersButtonCallback(std::function<void(bool)> function)
{
    _showVertexMarkersButtonCallback = function;
}

void UI::setChangeGeometryButtonCallback(std::function<void(unsigned int)> function)
{
    _changeGeometryButtonCallback = function;
}

void UI::setClearPinnedVerticesButtonCallback(std::function<void()> function)
{
    _clearPinnedVerticesCallback = function;
}