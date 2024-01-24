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
    mVehicle.setPosition(ToWindowCoord(sf::Vector2f(c.x, c.y)));

    mVehicle.setFillColor(sf::Color::Transparent);
    mVehicle.setOutlineColor(sf::Color::White);
    mVehicle.setOutlineThickness(1);

    mCenterOfGravity.setSize(sScaleFactor * sf::Vector2f(sCenterOfGravityBoxSize, sCenterOfGravityBoxSize));
    mCenterOfGravity.setOrigin(mCenterOfGravity.getSize() / 2.f);
    mCenterOfGravity.setPosition(ToWindowCoord(sf::Vector2f(c.x + l_AC, c.y + sVehicleHeight / 2)));

    mCenterOfGravity.setFillColor(sf::Color::Transparent);
    mCenterOfGravity.setOutlineColor(sf::Color::White);
    mCenterOfGravity.setOutlineThickness(1);

    double const dt = 1e-5;
    phisics::ACV::Update(dt);
    sf::Color pressureColor = GetColorForPressure();
    for (size_t i = 0; i < mAirCushionSegments.size(); ++i) {
        mAirCushionSegments[i].setSize(sScaleFactor * sf::Vector2f(delta_L, std::min(d_max, segments[i].d)));
        mAirCushionSegments[i].setOrigin(sScaleFactor * sf::Vector2f(delta_L / 2, 0));
        mAirCushionSegments[i].setPosition(ToWindowCoord(sf::Vector2f(segments[i].c.x, segments[i].c.y)));

        mAirCushionSegments[i].setFillColor(pressureColor);
    }
}

ACV::~ACV()
{
}

void ACV::Update(sf::Time deltaTime, sf::View &view)
{
    double const dt = 1e-5;
    phisics::ACV::Update(dt);

    double const phiDeg = ToDegrees(phi);

    view.setCenter(ToWindowCoord(sf::Vector2f(c.x, 0)));

    mVehicle.setPosition(ToWindowCoord(sf::Vector2f(c.x, c.y)));
    mVehicle.setRotation(phiDeg);
    
    phisics::Vector cog = {.x = c.x + l_AC, .y = c.y + sVehicleHeight / 2, .z = c.z};
    cog = phisics::detail::RotatePointXY(c, cog, phi);
    mCenterOfGravity.setPosition(ToWindowCoord(sf::Vector2f(cog.x, cog.y)));
    mCenterOfGravity.setRotation(phiDeg);

    sf::Color pressureColor = GetColorForPressure();
    for (size_t i = 0; i < mAirCushionSegments.size(); ++i) {
        mAirCushionSegments[i].setSize(sScaleFactor * sf::Vector2f(delta_L, std::min(d_max, segments[i].d)));
        mAirCushionSegments[i].setPosition(ToWindowCoord(sf::Vector2f(segments[i].c.x, segments[i].c.y)));
        mAirCushionSegments[i].setRotation(phiDeg);

        mAirCushionSegments[i].setFillColor(pressureColor);
    }

    if (mUpdateStepsToPushHistory == sHistoryPushPeriod) {
        mUpdateStepsToPushHistory = 0;
        
        mHistory_Q_in.Push(Q_in);
        mHistory_Q_out.Push(Q_out);

        mHistory_phi.Push(phiDeg);
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
    window.draw(mCenterOfGravity);
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
    double const le = c.x + ToMathCoordX(0);
    double const ri = c.x + ToMathCoordX(mWindowSize.x);

    double const waveStep = std::ceil(ri - le) / sWaveResolution;

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

    ImGui::NewLine();

    ImGui::PlotLines(
        U8("Тангаж [phi]"),
        HistoryGetter,
        reinterpret_cast<void *>(&mHistory_phi),
        mHistory_phi.Size(),
        0,
        nullptr,
        -25,
        25,
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

constexpr double ACV::ToDegrees(double radians)
{
    return radians / M_PI * 180;
}
constexpr double ACV::ToRadians(double degrees)
{
    return degrees / 180 * M_PI;
}

float ACV::HistoryGetter(void *data, int idx)
{
    return reinterpret_cast<CyclicBuffer<double, sHistorySize> *>(data)->Get(idx);
}