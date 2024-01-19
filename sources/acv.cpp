#include "acv.h"

#include "unicode_helpers.h"

ACV::ACV(sf::Vector2f const& windowSize)
    : phisics::ACV()
    , mWindowSize(windowSize)
    , mWindowCenter(windowSize / 2.f)
    , mOffset(0, 0)
{
    mVehicle.setSize(sScaleFactor * sf::Vector2f(sVehicleLength, sVehicleHeight));
    mVehicle.setOrigin(sScaleFactor * sf::Vector2f(sVehicleLength / 2, sVehicleHeight));

    mVehicle.setFillColor(sf::Color::Transparent);
    mVehicle.setOutlineColor(sf::Color(255, 255, 255));
    mVehicle.setOutlineThickness(1);
    mVehicle.setPosition(ToWindowCoord(sf::Vector2f(c.x, c.y)));

    phisics::ACV::Update(0.0);
    sf::Color pressureColor = GetColorForPressure();
    for (size_t i = 0; i < mAirCushionSegments.size(); ++i) {
        mAirCushionSegments[i].setSize(sScaleFactor * sf::Vector2f(delta_L, segments[i].d));
        mAirCushionSegments[i].setOrigin(sScaleFactor * sf::Vector2f(delta_L / 2, 0));

        mAirCushionSegments[i].setFillColor(pressureColor);
        mAirCushionSegments[i].setPosition(ToWindowCoord(sf::Vector2f(segments[i].c.x, segments[i].c.y)));
    }
}

ACV::~ACV()
{
}

void ACV::Update(sf::Time deltaTime)
{
    double const dt = 1e-4;
    phisics::ACV::Update(dt);

    mVehicle.setPosition(ToWindowCoord(sf::Vector2f(c.x, c.y)));

    sf::Color pressureColor = GetColorForPressure();
    for (size_t i = 0; i < mAirCushionSegments.size(); ++i) {
        mAirCushionSegments[i].setFillColor(pressureColor);
        mAirCushionSegments[i].setSize(sScaleFactor * sf::Vector2f(delta_L, segments[i].d));
        mAirCushionSegments[i].setPosition(ToWindowCoord(sf::Vector2f(segments[i].c.x, segments[i].c.y)));
    }

    if (mUpdateStepsToPushHistory == sHistoryPushPeriod) {
        mUpdateStepsToPushHistory = 0;
        
        mHistory_Q_in.Push(Q_in);
        mHistory_Q_out.Push(Q_out);
    }
    ++mUpdateStepsToPushHistory;
}

void ACV::Render(sf::RenderWindow& window)
{
    RenderACV(window);
    RenderWave(window);
    RenderImGui();
}

void ACV::RenderACV(sf::RenderWindow& window)
{
    window.draw(mVehicle);
    for (size_t i = 0; i < mAirCushionSegments.size(); ++i) {
        RenderSegment(window, i);
    }
}
void ACV::RenderSegment(sf::RenderWindow& window, size_t segmentIndex)
{
    window.draw(mAirCushionSegments[segmentIndex]);
}
void ACV::RenderWave(sf::RenderWindow& window)
{
    double le = ToMathCoordX(0);
    double ri = ToMathCoordX(mWindowSize.x);

    double waveStep = std::ceil(ri - le) / sWaveResolution;

    sf::VertexArray wave(sf::LineStrip, waveStep);
    for (double x = le; x < ri; x += waveStep) {
        auto mathCoord = sf::Vector2f(x, phisics::wave::y(x));
        wave.append(sf::Vertex(ToWindowCoord(mathCoord), sf::Color::Blue));
    }

    window.draw(wave);
}
void ACV::RenderImGui()
{
    ImGui::Begin(U8("Параметры ВП"));
    
    ImGui::Text(U8("Избыточное давление %.3lf Па"), p);
    ImGui::Text(U8("Объем %.3lf м^3"), W);

    ImGui::NewLine();

    ImGui::PlotLines(
        U8("Расход в ВП  [Q_in]"),
        HistoryGetter,
        reinterpret_cast<void *>(&mHistory_Q_in),
        mHistory_Q_in.Size(),
        0,
        nullptr,
        0,
        50,
        ImVec2(sHistorySize * 3, 50)
    );

    ImGui::PlotLines(
        U8("Расход из ВП [Q_out]"),
        HistoryGetter,
        reinterpret_cast<void *>(&mHistory_Q_out),
        mHistory_Q_out.Size(),
        0,
        nullptr,
        0,
        50,
        ImVec2(sHistorySize * 3, 50)
    );

    ImGui::End();
}

bool ACV::IsOnScreen(sf::Vector2f const& coord) const
{
    return 0 <= coord.x && coord.x < mWindowSize.x && 0 <= coord.y && coord.y < mWindowSize.y;
}

sf::Vector2f ACV::ToMathCoord(sf::Vector2f const& coord) const
{
    return sf::Vector2f(ToMathCoordX(coord.x), ToMathCoordY(coord.y));
}
double ACV::ToMathCoordX(double x) const
{
    return (x - mWindowCenter.x) / sScaleFactor + mOffset.x;
}
double ACV::ToMathCoordY(double y) const
{
    return -(y - mWindowCenter.y) / sScaleFactor + mOffset.y;
}

sf::Vector2f ACV::ToWindowCoord(sf::Vector2f const& coord) const
{
    return sf::Vector2f(ToWindowCoordX(coord.x), ToWindowCoordY(coord.y));
}
double ACV::ToWindowCoordX(double x) const
{
    return (x - mOffset.x) * sScaleFactor + mWindowCenter.x;
}
double ACV::ToWindowCoordY(double y) const
{
    return -(y - mOffset.y) * sScaleFactor + mWindowCenter.y;
}

sf::Color ACV::GetColorForPressure() const
{
    double min = phisics::ACV::Compressor::p_min;
    double max = phisics::ACV::Compressor::p_max;

    double mid = (min + max) / 2;
    double scale = 255 / (mid - min) / 2;

    if (p <= min) {
        return sf::Color::Green;
    }
    if (p >= max) {
        return sf::Color::Red;
    }

    if (p < mid) {
        uint32_t g = (p - min) * scale;
        return sf::Color(sf::Color::Green.toInteger() | g << 8);
    } else {
        uint32_t r = 255 - (p - mid) * scale;
        return sf::Color(sf::Color::Red.toInteger() | r << 16);
    }
}

float ACV::HistoryGetter(void *data, int idx)
{
    return reinterpret_cast<CyclicBuffer<double, sHistorySize> *>(data)->Get(idx);
}