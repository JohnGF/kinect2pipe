#ifndef KINECT2PIPE_KINECT2PIPE_H
#define KINECT2PIPE_KINECT2PIPE_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <termios.h>

using namespace std;
using namespace libfreenect2;

// The base width and height for the Kinect 2 color camera.
#define KINECT2_IMAGE_WIDTH 1920
#define KINECT2_IMAGE_HEIGHT 1080

// The base width and height for the Kinect 2 IR camera.
#define KINECT2_IR_WIDTH 512
#define KINECT2_IR_HEIGHT 424

// Output resolution: 1080p Full HD (1920x1080) for native 1:1 zero-scaling fast path.
#define OUTPUT_WIDTH 1920
#define OUTPUT_HEIGHT 1080

// The YUV420 buffer layout that V4L2 expects: Y first, followed by U, then V.
#define YUV_BUFFER_Y_LEN (OUTPUT_WIDTH * OUTPUT_HEIGHT)
#define YUV_BUFFER_UV_LEN ((OUTPUT_WIDTH / 2) * (OUTPUT_HEIGHT / 2))
#define YUV_BUFFER_LEN (YUV_BUFFER_Y_LEN + (YUV_BUFFER_UV_LEN * 2))

enum VideoMode {
    MODE_RGB_NORMAL = 0,
    MODE_RGB_BOOST = 1,
    MODE_IR_NIGHT_VISION = 2,
    MODE_COUNT = 3
};

class Kinect2Pipe {
public:
    explicit Kinect2Pipe();
    ~Kinect2Pipe();

    void openLoopback(const char* loopbackDev);
    void run();
    void stop();

    void cycleMode();
    void toggleNightVision();
    void toggleBoost();
    void setMode(VideoMode mode);
    void printHelp();

private:
    Freenect2 freenect2;
    Freenect2Device* dev;
    FrameMap frames;
    int v4l2Device;

    struct SwsContext* sws_rgb;
    struct SwsContext* sws_ir;

    uint8_t* srcPtr[4]{};
    int srcStride[4]{};

    uint8_t* srcIrPtr[4]{};
    int srcIrStride[4]{};

    uint8_t* dstPtr[4]{};
    int dstStride[4]{};

    uint8_t* imageBuffer;
    uint8_t* irGrayBuffer;
    uint8_t* boostedRgbBuffer;

    std::thread watcherThread;
    std::thread keyboardThread;

    int inotifyFd;
    int watcherFd;
    int openCount;

    mutex startMutex;
    condition_variable startCondition;

    std::atomic<bool> started;
    std::atomic<bool> shuttingDown;
    std::atomic<VideoMode> currentMode;

    struct termios originalTermios;
    bool termiosSaved;

    bool openV4L2LoopbackDevice(const char* loopbackDev, int width, int height);
    bool openProcMonitorV4L2LoopbackDevice(const char* loopbackDev);
    bool openKinect2Device();
    bool handleRgbFrame(Frame* frame, bool boost);
    bool handleIrFrame(Frame* frame);
    void procMonitorV4L2LoopbackDevice(const char* loopbackDev);
    void keyboardListener();
    void drainInotifyEvents();
    void writeBlankFrame();
};

#endif //KINECT2PIPE_KINECT2PIPE_H
