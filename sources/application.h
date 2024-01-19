#pragma once

#include <iostream>

#include <SFML/Graphics.hpp>
#include <SFML/Config.hpp>

#include <imgui-SFML.h>
#include <imgui.h>

#include "acv.h"

class Application {
public:
public:
    explicit Application(sf::ContextSettings const& settings = sf::ContextSettings());
    ~Application();

    void Run();

private:
    void ProcessEvents();
    void HandleInput(sf::Keyboard::Key key, bool isPressed);
    void Update(sf::Time deltaTime);
    void Render();

public:
    static uint32_t constexpr WindowWidth = 1280;
    static uint32_t constexpr WindowHeight = 720;

    static inline std::string const WindowTitle{"Jubilant-Basson."};
    static inline sf::Uint32 const WindowStyle = sf::Style::Close;
    static ImWchar const mCyrillicRanges[];

    static uint32_t constexpr FramesPerSecond = 1000;

private:
    sf::RenderWindow mWindow;
    ACV mACV;
};