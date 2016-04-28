// clang-format off
//!
//! Animator application.
//!
//! Shows an animation of a mesh by interpolating a set of poses.
//!
//! Usage:
//!   animator [-d|--duration <duration (s)>] [-f|--fps <frames per second>] <pose files>...
//!
// clang-format on


#include <QApplication> 
#include <QCommandLineParser>
#include <QStringList>

#include <iostream>
#include <memory>
#include <tuple>

#include "animator.h"

//!
//! \brief Process command line arguments.
//!
//! \param app QT application
//!
//! \return Poses file paths, animation duration and frame rate
//!
std::tuple<QStringList, float, float>
processArgs(const QApplication &app);

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QApplication::setApplicationName("Animator");
    QApplication::setApplicationVersion("1.0");

    QStringList files;
    float duration;
    float fps;
    std::tie(files, duration, fps) = processArgs(app);

    Animator animator(files, duration, fps);
    animator.show();

    return app.exec();
}

std::tuple<QStringList, float, float>
processArgs(const QApplication &app)
{
    QCommandLineParser parser;
    parser.setApplicationDescription("Animates a mesh by interpolating "
                                     "multiple poses.");
    parser.addHelpOption();
    parser.addVersionOption();

    parser.addPositionalArgument("pose files", "Mesh pose files (2 or more).",
                                 "<pose files...>");

    parser.addOptions({
        {{"d", "duration"},
         "Duration of the animation (default: number of poses minus one).",
         "seconds"},
        {{"f", "fps"}, "Frames per second (default: 25).", "fps", "25"},
    });

    parser.process(app);

    const QStringList files = parser.positionalArguments();
    if (files.size() < 2)
    {
        parser.showHelp(1);
    }

    bool valid;
    float duration = files.size() - 1.0f;
    if (parser.isSet("duration"))
    {
        duration = parser.value("duration").toDouble(&valid);
        if (!valid || duration <= 0.0f)
        {
            parser.showHelp(1);
        }
    }

    float fps = parser.value("fps").toDouble(&valid);
    if (!valid || fps <= 0.0f)
    {
        parser.showHelp(1);
    }

    return std::make_tuple(files, duration, fps);
}