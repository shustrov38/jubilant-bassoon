#pragma once

#include "acv_phisics.h"

#include <SFML/Graphics.hpp>

#include <imgui-SFML.h>
#include <imgui.h>

#include "cyclic_buffer.h"

class ACV : public phisics::ACV {
public:
    explicit ACV(sf::Vector2f const& windowSize);
    ~ACV();

    void Update(sf::Time deltaTime, sf::View &view);
    void Render(sf::RenderWindow& window);

private:
    void RenderACV(sf::RenderWindow& window);
    void RenderSegment(sf::RenderWindow& window, size_t segmentIndex);
    void RenderWave(sf::RenderWindow& window);
    void RenderImGui();

    bool IsOnScreen(sf::Vector2f const& coord) const;

    sf::Vector2f ToMathCoord(sf::Vector2f const& coord) const;
    double ToMathCoordX(double x) const;
    double ToMathCoordY(double y) const;

    sf::Vector2f ToWindowCoord(sf::Vector2f const& coord) const;
    double ToWindowCoordX(double x) const;
    double ToWindowCoordY(double y) const;

    sf::Color GetColorForPressure() const;

    static constexpr double ToDegrees(double radians);
    static constexpr double ToRadians(double degrees);

    static float HistoryGetter(void *data, int idx);

private:
    static double constexpr sCenterOfGravityBoxSize = 0.5;

    static double constexpr sVehicleLength = 14;
    static double constexpr sVehicleHeight = 1.4;
    static float  constexpr sScaleFactor = 50;

    static size_t constexpr sWaveResolution = 500;

    static size_t constexpr sHistorySize = 100;
    static size_t constexpr sHistoryPushPeriod = 30;

    sf::Vector2f mWindowSize;
    sf::Vector2f mWindowCenter;

    sf::Vector2f mOffset;

    sf::RectangleShape mVehicle;
    sf::RectangleShape mCenterOfGravity;
    std::array<sf::RectangleShape, N> mAirCushionSegments;

    size_t mUpdateStepsToPushHistory = 0;
    CyclicBuffer<double, sHistorySize> mHistory_Q_in;
    CyclicBuffer<double, sHistorySize> mHistory_Q_out;
    CyclicBuffer<double, sHistorySize> mHistory_phi;
};