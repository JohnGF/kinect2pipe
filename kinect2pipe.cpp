#include <iostream>
#include <libfreenect2/logger.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <cmath>
#include <algorithm>
#include <dirent.h>
#include "kinect2pipe.h"

extern "C" {
    #include <libswscale/swscale.h>
}

using namespace std;
using namespace libfreenect2;

Kinect2Pipe::Kinect2Pipe() {
    this->sws_rgb = sws_getContext(
        KINECT2_IMAGE_WIDTH, KINECT2_IMAGE_HEIGHT, AV_PIX_FMT_RGB32,
        OUTPUT_WIDTH, OUTPUT_HEIGHT, AV_PIX_FMT_YUV420P,
        SWS_FAST_BILINEAR, nullptr, nullptr, nullptr
    );

    this->sws_ir = sws_getContext(
        KINECT2_IR_WIDTH, KINECT2_IR_HEIGHT, AV_PIX_FMT_GRAY8,
        OUTPUT_WIDTH, OUTPUT_HEIGHT, AV_PIX_FMT_YUV420P,
        SWS_FAST_BILINEAR, nullptr, nullptr, nullptr
    );

    memset(this->srcPtr, 0, sizeof(uint8_t*) * 4);
    this->srcStride[0] = KINECT2_IMAGE_WIDTH * 4;
    this->srcStride[1] = 0;
    this->srcStride[2] = 0;
    this->srcStride[3] = 0;

    this->irGrayBuffer = (uint8_t*)malloc(KINECT2_IR_WIDTH * KINECT2_IR_HEIGHT);
    this->boostedRgbBuffer = (uint8_t*)malloc(KINECT2_IMAGE_WIDTH * KINECT2_IMAGE_HEIGHT * 4);

    memset(this->srcIrPtr, 0, sizeof(uint8_t*) * 4);
    this->srcIrPtr[0] = this->irGrayBuffer;
    this->srcIrStride[0] = KINECT2_IR_WIDTH;
    this->srcIrStride[1] = 0;
    this->srcIrStride[2] = 0;
    this->srcIrStride[3] = 0;

    this->imageBuffer = (uint8_t*)calloc(1, YUV_BUFFER_LEN);
    this->dstPtr[0] = this->imageBuffer;
    this->dstPtr[1] = this->imageBuffer + YUV_BUFFER_Y_LEN;
    this->dstPtr[2] = this->imageBuffer + YUV_BUFFER_Y_LEN + YUV_BUFFER_UV_LEN;
    this->dstPtr[3] = nullptr;
    this->dstStride[0] = OUTPUT_WIDTH;
    this->dstStride[1] = OUTPUT_WIDTH / 2;
    this->dstStride[2] = OUTPUT_WIDTH / 2;
    this->dstStride[3] = 0;

    this->dev = nullptr;
    this->v4l2Device = 0;
    this->openCount = 0;
    this->started = false;
    this->shuttingDown = false;
    this->currentMode = MODE_RGB_NORMAL;
    this->termiosSaved = false;

    libfreenect2::setGlobalLogger(nullptr);
}

Kinect2Pipe::~Kinect2Pipe() {
    this->stop();

    if (this->keyboardThread.joinable()) {
        this->keyboardThread.join();
    }

    if (this->termiosSaved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &this->originalTermios);
        this->termiosSaved = false;
    }

    if (this->v4l2Device > 0) {
        close(this->v4l2Device);
        this->v4l2Device = 0;
    }

    if (this->sws_rgb) {
        sws_freeContext(this->sws_rgb);
        this->sws_rgb = nullptr;
    }

    if (this->sws_ir) {
        sws_freeContext(this->sws_ir);
        this->sws_ir = nullptr;
    }

    if (this->imageBuffer) {
        free(this->imageBuffer);
        this->imageBuffer = nullptr;
    }

    if (this->irGrayBuffer) {
        free(this->irGrayBuffer);
        this->irGrayBuffer = nullptr;
    }

    if (this->boostedRgbBuffer) {
        free(this->boostedRgbBuffer);
        this->boostedRgbBuffer = nullptr;
    }
}

void Kinect2Pipe::stop() {
    this->shuttingDown = true;
    this->started = false;
    this->startCondition.notify_all();
}

void Kinect2Pipe::printHelp() {
    cout << "\n=================== Kinect2Pipe Controls ===================" << endl;
    cout << "  [Space] / [t] / [n] : Cycle mode" << endl;
    cout << "  [1]                 : Mode 1: RGB Normal (1080p Full HD)" << endl;
    cout << "  [2]                 : Mode 2: RGB Low-Light Boost" << endl;
    cout << "  [3] / [i]           : Mode 3: IR Night Vision (Active Laser)" << endl;
    cout << "  [h]                 : Show this help message" << endl;
    cout << "  [q]                 : Quit" << endl;
    cout << "  Global Shortcut     : pkill -USR1 kinect2pipe (toggles IR)" << endl;
    cout << "  Global Boost Hotkey : pkill -USR2 kinect2pipe (toggles Boost)" << endl;
    cout << "============================================================" << endl;
}

void Kinect2Pipe::setMode(VideoMode mode) {
    this->currentMode = mode;
    switch (mode) {
        case MODE_RGB_NORMAL:
            cout << "\n>>> [Camera Mode] -> RGB Normal (1080p Full HD)" << endl;
            break;
        case MODE_RGB_BOOST:
            cout << "\n>>> [Camera Mode] -> RGB Low-Light Enhanced (Boosted Gamma)" << endl;
            break;
        case MODE_IR_NIGHT_VISION:
            cout << "\n>>> [Camera Mode] -> IR Night Vision (Active Laser Time-of-Flight)" << endl;
            break;
        default:
            break;
    }
}

void Kinect2Pipe::cycleMode() {
    int next = (static_cast<int>(this->currentMode.load()) + 1) % MODE_COUNT;
    setMode(static_cast<VideoMode>(next));
}

void Kinect2Pipe::toggleNightVision() {
    if (this->currentMode.load() == MODE_IR_NIGHT_VISION) {
        setMode(MODE_RGB_NORMAL);
    } else {
        setMode(MODE_IR_NIGHT_VISION);
    }
}

void Kinect2Pipe::toggleBoost() {
    if (this->currentMode.load() == MODE_RGB_BOOST) {
        setMode(MODE_RGB_NORMAL);
    } else {
        setMode(MODE_RGB_BOOST);
    }
}

void Kinect2Pipe::keyboardListener() {
    bool is_tty = isatty(STDIN_FILENO);
    if (is_tty) {
        tcgetattr(STDIN_FILENO, &this->originalTermios);
        this->termiosSaved = true;
        struct termios raw = this->originalTermios;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    printHelp();
    setMode(this->currentMode.load());

    while (!this->shuttingDown) {
        struct pollfd pfd{};
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        int ret = poll(&pfd, 1, 150);

        if (ret > 0 && (pfd.revents & POLLIN)) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                if (c == ' ' || c == 't' || c == 'n' || c == '\t') {
                    cycleMode();
                } else if (c == '1') {
                    setMode(MODE_RGB_NORMAL);
                } else if (c == '2') {
                    setMode(MODE_RGB_BOOST);
                } else if (c == '3' || c == 'i') {
                    setMode(MODE_IR_NIGHT_VISION);
                } else if (c == 'b') {
                    toggleBoost();
                } else if (c == 'h' || c == '?') {
                    printHelp();
                } else if (c == 'q' || c == 'Q') {
                    cout << "\nQuit requested from keyboard." << endl;
                    this->stop();
                    break;
                }
            }
        }
    }

    if (is_tty && this->termiosSaved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &this->originalTermios);
        this->termiosSaved = false;
    }
}

void Kinect2Pipe::openLoopback(const char *loopbackDev) {
    if (!this->openV4L2LoopbackDevice(loopbackDev, OUTPUT_WIDTH, OUTPUT_HEIGHT)) {
        exit(1);
    }
    this->writeBlankFrame();

    this->keyboardThread = thread(&Kinect2Pipe::keyboardListener, this);
}

bool Kinect2Pipe::openV4L2LoopbackDevice(const char* loopbackDev, int width, int height) {
    this->v4l2Device = open(loopbackDev, O_WRONLY | O_NONBLOCK);
    if (this->v4l2Device < 0) {
        cerr << "failed to open v4l2loopback device: " << errno << endl;
        return false;
    }
    struct v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    fmt.fmt.pix.sizeimage = fmt.fmt.pix.width * fmt.fmt.pix.height * 1.5;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = fmt.fmt.pix.width;
    fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    if (ioctl(this->v4l2Device, VIDIOC_S_FMT, &fmt) < 0) {
        cerr << "failed to issue ioctl to v4l2loopback device: " << errno << endl;
        return false;
    }

    return true;
}

bool Kinect2Pipe::openKinect2Device() {
    SyncMultiFrameListener colorListener(Frame::Color);
    SyncMultiFrameListener irListener(Frame::Ir);

    if (freenect2.enumerateDevices() == 0) {
        cerr << "unable to find a kinect2 device to connect to" << endl;
        return false;
    }
    this->dev = freenect2.openDefaultDevice();
    if (!this->dev) {
        cerr << "unable to open kinect2 device" << endl;
        return false;
    }

    this->dev->setColorFrameListener(&colorListener);
    this->dev->setIrAndDepthFrameListener(&irListener);

    VideoMode activeMode = this->currentMode.load();
    bool activeRgb = (activeMode != MODE_IR_NIGHT_VISION);
    bool activeIr = (activeMode == MODE_IR_NIGHT_VISION);

    if (!this->dev->startStreams(activeRgb, activeIr)) {
        cerr << "unable to start kinect2 streams" << endl;
        this->dev->close();
        this->dev = nullptr;
        return false;
    }

    while (this->started && !this->shuttingDown) {
        VideoMode mode = this->currentMode.load();
        bool needRgb = (mode != MODE_IR_NIGHT_VISION);
        bool needIr = (mode == MODE_IR_NIGHT_VISION);

        if (needRgb != activeRgb || needIr != activeIr) {
            cout << "\nSwitching active hardware streams..." << endl;
            this->dev->stop();
            if (!this->dev->startStreams(needRgb, needIr)) {
                cerr << "Failed to switch streams!" << endl;
                break;
            }
            activeRgb = needRgb;
            activeIr = needIr;
            activeMode = mode;
        }

        if (activeMode == MODE_IR_NIGHT_VISION) {
            if (!irListener.waitForNewFrame(frames, 100)) {
                continue;
            }
            Frame* irFrame = frames[Frame::Ir];
            if (irFrame) {
                this->handleIrFrame(irFrame);
            }
            irListener.release(frames);
        } else {
            if (!colorListener.waitForNewFrame(frames, 100)) {
                continue;
            }
            Frame* colorFrame = frames[Frame::Color];
            if (colorFrame) {
                this->handleRgbFrame(colorFrame, mode == MODE_RGB_BOOST);
            }
            colorListener.release(frames);
        }
    }

    this->writeBlankFrame();
    cout << "Stopping Kinect streams and closing device..." << endl;
    this->dev->stop();
    this->dev->close();
    this->dev = nullptr;
    return true;
}

bool Kinect2Pipe::handleRgbFrame(Frame* frame, bool boost) {
    if (boost) {
        static uint8_t lut[256];
        static bool lut_init = false;
        if (!lut_init) {
            for (int i = 0; i < 256; ++i) {
                float norm = i / 255.0f;
                float b = std::pow(norm, 0.65f) * 1.30f * 255.0f;
                lut[i] = static_cast<uint8_t>(b > 255.0f ? 255 : (b < 0.0f ? 0 : b));
            }
            lut_init = true;
        }

        const uint8_t* in = frame->data;
        uint8_t* out = this->boostedRgbBuffer;
        const size_t total = static_cast<size_t>(KINECT2_IMAGE_WIDTH) * KINECT2_IMAGE_HEIGHT * 4;
        for (size_t i = 0; i < total; i += 4) {
            out[i]     = lut[in[i]];     // R
            out[i + 1] = lut[in[i + 1]]; // G
            out[i + 2] = lut[in[i + 2]]; // B
            out[i + 3] = in[i + 3];     // A
        }
        this->srcPtr[0] = this->boostedRgbBuffer;
    } else {
        this->srcPtr[0] = frame->data;
    }

    sws_scale(this->sws_rgb, this->srcPtr, this->srcStride, 0, KINECT2_IMAGE_HEIGHT, this->dstPtr, this->dstStride);
    ssize_t ret = write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN);
    if (ret <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == EPIPE) {
            return true;
        }
    }
    return true;
}

bool Kinect2Pipe::handleIrFrame(Frame* frame) {
    const float* irFloats = reinterpret_cast<const float*>(frame->data);
    const int total = KINECT2_IR_WIDTH * KINECT2_IR_HEIGHT;

    for (int i = 0; i < total; ++i) {
        float val = irFloats[i];
        if (val <= 0.0f) {
            this->irGrayBuffer[i] = 0;
        } else {
            // Sqrt tone curve maps dynamic range of IR laser reflections cleanly to 0..255
            int v = static_cast<int>(std::sqrt(val) * 3.8f);
            this->irGrayBuffer[i] = static_cast<uint8_t>(v > 255 ? 255 : v);
        }
    }

    this->srcIrPtr[0] = this->irGrayBuffer;
    sws_scale(this->sws_ir, this->srcIrPtr, this->srcIrStride, 0, KINECT2_IR_HEIGHT, this->dstPtr, this->dstStride);
    ssize_t ret = write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN);
    if (ret <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == EPIPE) {
            return true;
        }
    }
    return true;
}

bool Kinect2Pipe::openProcMonitorV4L2LoopbackDevice(const char* loopbackDev) {
    this->watcherThread = thread(&Kinect2Pipe::procMonitorV4L2LoopbackDevice, this, loopbackDev);
    this->watcherThread.detach();
    return true;
}

void Kinect2Pipe::procMonitorV4L2LoopbackDevice(const char* loopbackDev) {
    unique_lock<mutex> lk(this->startMutex);
    this->startCondition.wait(lk, [this]{ return !this->started || this->shuttingDown; });
    if (this->shuttingDown) return;

    const string current_pid = to_string(getpid());
    const string target = loopbackDev;

    while (!this->shuttingDown) {
        DIR *proc_dir = opendir("/proc");
        if (!proc_dir) {
            if (this->shuttingDown) return;
            perror("opendir /proc");
            exit(-1);
        }

        struct dirent *entry;
        while (!this->shuttingDown && (entry = readdir(proc_dir)) != nullptr) {
            string pid_str(entry->d_name);
            if (!all_of(pid_str.begin(), pid_str.end(), ::isdigit)) continue;
            if (pid_str == current_pid) continue;

            string fd_dir_path = "/proc/" + pid_str + "/fd";
            DIR *fd_dir = opendir(fd_dir_path.c_str());
            if (!fd_dir) continue;

            struct dirent *fd_entry;
            while (!this->shuttingDown && (fd_entry = readdir(fd_dir)) != nullptr) {
                if (strcmp(fd_entry->d_name, ".") == 0 || strcmp(fd_entry->d_name, "..") == 0)
                    continue;

                string fd_path = fd_dir_path + "/" + fd_entry->d_name;
                char link_target[PATH_MAX];
                ssize_t len = readlink(fd_path.c_str(), link_target, sizeof(link_target) - 1);
                if (len == -1) continue;
                link_target[len] = '\0';

                if (target == link_target) {
                    this->openCount++;
                    if (!this->started) {
                        this->started = true;
                        lk.unlock();
                        this->startCondition.notify_one();
                    }
                }
            }
            closedir(fd_dir);
        }
        closedir(proc_dir);

        if (this->shuttingDown) return;

        if (this->openCount == 0 && this->started) {
            cout << "Closing device since no application is watching..." << endl;
            this->started = false;
            lk.lock();
        }

        this->openCount = 0;
        for (int i = 0; i < 30 && !this->shuttingDown; ++i) {
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
}

void Kinect2Pipe::run() {
    this->started = true;
    cout << "\n>>> Kinect capture stream active. Ready for Discord / OBS / Browser / mpv." << endl;
    if (!this->openKinect2Device()) {
        if (!this->shuttingDown) {
            cerr << "Failed to start Kinect device." << endl;
        }
    }
}

void Kinect2Pipe::writeBlankFrame() {
    memset(this->imageBuffer, 0x10, YUV_BUFFER_Y_LEN);
    memset(this->imageBuffer + YUV_BUFFER_Y_LEN, 0x80, YUV_BUFFER_UV_LEN * 2);
    ssize_t ret = write(this->v4l2Device, this->imageBuffer, YUV_BUFFER_LEN);
    (void)ret;
}
