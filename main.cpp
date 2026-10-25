#include <csignal>
#include <iostream>
#include "kinect2pipe.h"

static Kinect2Pipe* g_pipe = nullptr;
static int g_stop_count = 0;

void signalHandler(int signum) {
    if (signum == SIGINT || signum == SIGTERM) {
        g_stop_count++;
        if (g_stop_count > 1) {
            std::cout << "\nForce exiting..." << std::endl;
            exit(0);
        }
        std::cout << "\nStopping kinect2pipe..." << std::endl;
        if (g_pipe != nullptr) {
            g_pipe->stop();
        }
    } else if (signum == SIGUSR1) {
        if (g_pipe != nullptr) {
            g_pipe->toggleNightVision();
        }
    } else if (signum == SIGUSR2) {
        if (g_pipe != nullptr) {
            g_pipe->toggleBoost();
        }
    }
}

/**
 * Main is the entry point for the application. It takes a single argument which should be the path to the v4l2loopback
 * device to watch.
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return Exit status
 */
int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        printf("usage: kinect2pipe <primary v4l2 device> [secondary v4l2 device]\n");
        exit(-1);
    }

    struct sigaction sa{};
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGUSR1, &sa, nullptr);
    sigaction(SIGUSR2, &sa, nullptr);

    g_pipe = new Kinect2Pipe();
    const char* dev2 = (argc == 3) ? argv[2] : nullptr;
    g_pipe->openLoopback(argv[1], dev2);
    g_pipe->run();

    delete g_pipe;
    g_pipe = nullptr;
    return 0;
}