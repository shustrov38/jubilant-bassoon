#include "application.h"

int main()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    Application app(settings);
    app.Run();

    return EXIT_SUCCESS;
}