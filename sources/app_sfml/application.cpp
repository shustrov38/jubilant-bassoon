#include "application.h"

ImWchar const Application::mCyrillicRanges[] =
{
    0x0020, 0x00FF, // Basic Latin + Latin Supplement
    0x0400, 0x044F, // Cyrillic
    0,
};

Application::Application(sf::ContextSettings const& settings)
    : mWindow(sf::VideoMode(WindowWidth, WindowHeight), WindowTitle, WindowStyle, settings)
    , mACV(sf::Vector2f(WindowWidth, WindowHeight))
{
    std::cout << "IMGUI " << IMGUI_VERSION << std::endl;
    std::cout << "SFML " << SFML_VERSION_MAJOR << "." << SFML_VERSION_MINOR << "." << SFML_VERSION_PATCH << std::endl;
    std::cout << "SETTINGS " << settings.majorVersion << "." << settings.minorVersion << std::endl;

    mWindow.setFramerateLimit(FramesPerSecond);
    mView.reset(sf::FloatRect(0, 0, WindowWidth, WindowHeight));

    ImGui::SFML::Init(mWindow, false);
    ImGui::GetIO().Fonts->AddFontFromFileTTF("GeistMono-Regular.otf", 14.f, nullptr, mCyrillicRanges);
    ImGui::SFML::UpdateFontTexture();
}

Application::~Application()
{
    ImGui::SFML::Shutdown();
}

void Application::Run()
{
    sf::Clock clock;
    sf::Time timeSinceLastUpdate = sf::Time::Zero;

    while (mWindow.isOpen()) {
        ProcessEvents();
        timeSinceLastUpdate = clock.restart();

        mWindow.clear();
        ImGui::SFML::Update(mWindow, timeSinceLastUpdate);

        Update(timeSinceLastUpdate);
        Render();

        ImGui::SFML::Render(mWindow);

        mWindow.display();
    }
}

void Application::ProcessEvents()
{
    sf::Event event;
    while (mWindow.pollEvent(event)) {
        ImGui::SFML::ProcessEvent(event);
        switch (event.type) {
            case sf::Event::KeyPressed:
                HandleInput(event.key.code, true);
                break;
            case sf::Event::KeyReleased:
                HandleInput(event.key.code, false);
                break;
            case sf::Event::Closed:
                mWindow.close();
                break;
            default:
                break;
        }
    }
}

void Application::HandleInput(sf::Keyboard::Key key, bool isPressed)
{
}

void Application::Update(sf::Time deltaTime)
{   
    for (int i = 0; i < 1000; ++i) {
        mACV.Update(deltaTime, mView);
    }
}

void Application::Render()
{   
    mWindow.setView(mView);
    ImGui::PushFont(ImGui::GetIO().Fonts->Fonts[0]);

    mACV.Render(mWindow);
    
    ImGui::PopFont();
}