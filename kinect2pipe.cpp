#include <iostream>
#include <libfreenect2/logger.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <cmath>
#include <algorithm>
#include <dirent.h>
#include <cstring>
#include <ctime>
#include "kinect2pipe.h"

extern "C" {
    #include <libswscale/swscale.h>
}

using namespace std;
using namespace libfreenect2;

Kinect2Pipe::Kinect2Pipe(bool irShmEnabled) {
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
    this->v4l2Device2 = 0;
    this->shmFd = -1;
    this->shmPtr = nullptr;
    this->shmTotalSize = 0;
    this->openCount = 0;
    this->started = false;
    this->shuttingDown = false;
    this->currentMode = MODE_RGB_NORMAL;
    this->termiosSaved = false;
    this->irShmEnabled = irShmEnabled;
    this->irShmFd = -1;
    this->irShmPtr = nullptr;
    this->irShmTotalSize = 0;
    this->pendingDeviceTs = 0;
    this->pendingDeviceSeq = 0;

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

    if (this->v4l2Device2 > 0) {
        close(this->v4l2Device2);
        this->v4l2Device2 = 0;
    }

    this->closeShm();
    this->closeIrShm();

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

void Kinect2Pipe::openLoopback(const char *loopbackDev, const char *loopbackDev2) {
    if (!this->openV4L2LoopbackDevice(loopbackDev, OUTPUT_WIDTH, OUTPUT_HEIGHT, this->v4l2Device)) {
        exit(1);
    }
    if (loopbackDev2 && strlen(loopbackDev2) > 0) {
        if (!this->openV4L2LoopbackDevice(loopbackDev2, OUTPUT_WIDTH, OUTPUT_HEIGHT, this->v4l2Device2)) {
            cerr << "Warning: failed to open secondary v4l2 device: " << loopbackDev2 << endl;
            this->v4l2Device2 = -1;
        } else {
            cout << "Secondary loopback active on " << loopbackDev2 << endl;
        }
    }
    this->initShm();
    if (this->irShmEnabled) {
        this->initIrShm();
    }
    this->writeBlankFrame();

    this->keyboardThread = thread(&Kinect2Pipe::keyboardListener, this);
}

bool Kinect2Pipe::openV4L2LoopbackDevice(const char* loopbackDev, int width, int height, int& outFd) {
    outFd = open(loopbackDev, O_WRONLY | O_NONBLOCK);
    if (outFd < 0) {
        cerr << "failed to open v4l2loopback device: " << loopbackDev << " (errno " << errno << ")" << endl;
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

    if (ioctl(outFd, VIDIOC_S_FMT, &fmt) < 0) {
        cerr << "failed to issue ioctl to v4l2loopback device: " << loopbackDev << " (errno " << errno << ")" << endl;
        close(outFd);
        outFd = -1;
        return false;
    }

    return true;
}

bool Kinect2Pipe::openKinect2Device() {
    SyncMultiFrameListener colorListener(Frame::Color);
    SyncMultiFrameListener irListener(Frame::Ir | Frame::Depth);
    FrameMap irFrames;

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

    // With --ir the IR/depth pipeline always runs so the IR shm segment is fed in parallel with color.
    auto wantRgb = [](VideoMode m) { return m != MODE_IR_NIGHT_VISION; };
    auto wantIr = [this](VideoMode m) { return m == MODE_IR_NIGHT_VISION || (this->irShmEnabled && this->irShmPtr); };

    VideoMode activeMode = this->currentMode.load();
    bool activeRgb = wantRgb(activeMode);
    bool activeIr = wantIr(activeMode);

    if (!this->dev->startStreams(activeRgb, activeIr)) {
        cerr << "unable to start kinect2 streams" << endl;
        this->dev->close();
        this->dev = nullptr;
        return false;
    }

    while (this->started && !this->shuttingDown) {
        VideoMode mode = this->currentMode.load();
        bool needRgb = wantRgb(mode);
        bool needIr = wantIr(mode);

        if (needRgb != activeRgb || needIr != activeIr) {
            cout << "\nSwitching active hardware streams..." << endl;
            this->dev->stop();
            if (!this->dev->startStreams(needRgb, needIr)) {
                cerr << "Failed to switch streams!" << endl;
                break;
            }
            activeRgb = needRgb;
            activeIr = needIr;
        }
        activeMode = mode;

        if (activeMode == MODE_IR_NIGHT_VISION) {
            if (!irListener.waitForNewFrame(irFrames, 100)) {
                continue;
            }
            Frame* irFrame = irFrames[Frame::Ir];
            if (irFrame) {
                this->pendingDeviceTs = irFrame->timestamp;
                this->pendingDeviceSeq = irFrame->sequence;
                this->handleIrFrame(irFrame);
                this->publishIrDepth(irFrame, irFrames[Frame::Depth]);
            }
            irListener.release(irFrames);
        } else {
            if (!colorListener.waitForNewFrame(frames, 100)) {
                continue;
            }
            Frame* colorFrame = frames[Frame::Color];
            if (colorFrame) {
                this->pendingDeviceTs = colorFrame->timestamp;
                this->pendingDeviceSeq = colorFrame->sequence;
                this->handleRgbFrame(colorFrame, mode == MODE_RGB_BOOST);
            }
            colorListener.release(frames);

            // Drain the parallel IR/depth stream without blocking the color path.
            if (activeIr && irListener.hasNewFrame() && irListener.waitForNewFrame(irFrames, 0)) {
                this->publishIrDepth(irFrames[Frame::Ir], irFrames[Frame::Depth]);
                irListener.release(irFrames);
            }
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
    this->writeToOutputs(this->imageBuffer, YUV_BUFFER_LEN);
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
    this->writeToOutputs(this->imageBuffer, YUV_BUFFER_LEN);
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
    this->writeToOutputs(this->imageBuffer, YUV_BUFFER_LEN);
}

void Kinect2Pipe::writeToOutputs(const uint8_t* buffer, size_t size) {
    if (this->v4l2Device > 0) {
        ssize_t ret = write(this->v4l2Device, buffer, size);
        (void)ret;
    }
    if (this->v4l2Device2 > 0) {
        ssize_t ret = write(this->v4l2Device2, buffer, size);
        (void)ret;
    }
    this->updateShm(buffer, size);
}

bool Kinect2Pipe::initShm() {
    this->shmTotalSize = sizeof(KinectSharedMemoryHeader) + YUV_BUFFER_LEN;
    this->shmFd = open(KINECT_SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (this->shmFd < 0) {
        cerr << "Warning: failed to open shm file: " << KINECT_SHM_NAME << " (" << strerror(errno) << ")" << endl;
        this->shmPtr = nullptr;
        return false;
    }

    if (ftruncate(this->shmFd, this->shmTotalSize) != 0) {
        cerr << "Warning: ftruncate shm failed: " << strerror(errno) << endl;
        close(this->shmFd);
        this->shmFd = -1;
        this->shmPtr = nullptr;
        return false;
    }

    this->shmPtr = static_cast<uint8_t*>(mmap(nullptr, this->shmTotalSize, PROT_READ | PROT_WRITE, MAP_SHARED, this->shmFd, 0));
    if (this->shmPtr == MAP_FAILED) {
        cerr << "Warning: mmap shm failed: " << strerror(errno) << endl;
        close(this->shmFd);
        this->shmFd = -1;
        this->shmPtr = nullptr;
        return false;
    }

    auto* hdr = reinterpret_cast<KinectSharedMemoryHeader*>(this->shmPtr);
    memset(hdr, 0, sizeof(KinectSharedMemoryHeader));
    hdr->magic = KINECT_SHM_MAGIC;
    hdr->version = 1;
    hdr->width = OUTPUT_WIDTH;
    hdr->height = OUTPUT_HEIGHT;
    hdr->yuv_bytes = YUV_BUFFER_LEN;
    hdr->mode = static_cast<uint32_t>(this->currentMode.load());
    hdr->sequence = 0;

    cout << "Lock-free shared memory active at " << KINECT_SHM_NAME << " (" << this->shmTotalSize << " bytes)" << endl;
    return true;
}

void Kinect2Pipe::updateShm(const uint8_t* buffer, size_t size) {
    if (!this->shmPtr || size != YUV_BUFFER_LEN) return;

    auto* hdr = reinterpret_cast<KinectSharedMemoryHeader*>(this->shmPtr);
    // Seqlock begin write (odd sequence)
    __atomic_add_fetch(&hdr->sequence, 1, __ATOMIC_RELEASE);

    hdr->mode = static_cast<uint32_t>(this->currentMode.load());
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    hdr->timestamp_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
    hdr->device_ts = this->pendingDeviceTs;
    hdr->device_seq = this->pendingDeviceSeq;
    this->pendingDeviceTs = 0;
    this->pendingDeviceSeq = 0;

    memcpy(this->shmPtr + sizeof(KinectSharedMemoryHeader), buffer, size);

    // Seqlock end write (even sequence)
    __atomic_add_fetch(&hdr->sequence, 1, __ATOMIC_RELEASE);
}

void Kinect2Pipe::closeShm() {
    if (this->shmPtr && this->shmPtr != MAP_FAILED) {
        munmap(this->shmPtr, this->shmTotalSize);
        this->shmPtr = nullptr;
    }
    if (this->shmFd >= 0) {
        close(this->shmFd);
        this->shmFd = -1;
    }
    unlink(KINECT_SHM_NAME);
}

bool Kinect2Pipe::initIrShm() {
    this->irShmTotalSize = sizeof(KinectIrSharedMemoryHeader) + KINECT_IR_PAYLOAD_BYTES;
    this->irShmFd = open(KINECT_IR_SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (this->irShmFd < 0) {
        cerr << "Warning: failed to open IR shm file: " << KINECT_IR_SHM_NAME << " (" << strerror(errno) << ")" << endl;
        return false;
    }
    if (ftruncate(this->irShmFd, this->irShmTotalSize) != 0) {
        cerr << "Warning: ftruncate IR shm failed: " << strerror(errno) << endl;
        close(this->irShmFd);
        this->irShmFd = -1;
        return false;
    }
    void* p = mmap(nullptr, this->irShmTotalSize, PROT_READ | PROT_WRITE, MAP_SHARED, this->irShmFd, 0);
    if (p == MAP_FAILED) {
        cerr << "Warning: mmap IR shm failed: " << strerror(errno) << endl;
        close(this->irShmFd);
        this->irShmFd = -1;
        return false;
    }
    this->irShmPtr = static_cast<uint8_t*>(p);

    auto* hdr = reinterpret_cast<KinectIrSharedMemoryHeader*>(this->irShmPtr);
    memset(this->irShmPtr, 0, this->irShmTotalSize);
    hdr->magic = KINECT_IR_SHM_MAGIC;
    hdr->version = 1;
    hdr->width = KINECT2_IR_WIDTH;
    hdr->height = KINECT2_IR_HEIGHT;
    hdr->payload_bytes = KINECT_IR_PAYLOAD_BYTES;

    cout << "IR + depth shared memory active at " << KINECT_IR_SHM_NAME << " (" << this->irShmTotalSize << " bytes)" << endl;
    return true;
}

void Kinect2Pipe::publishIrDepth(Frame* irFrame, Frame* depthFrame) {
    if (!this->irShmPtr || !irFrame) return;
    if (irFrame->width != KINECT2_IR_WIDTH || irFrame->height != KINECT2_IR_HEIGHT) return;

    auto* hdr = reinterpret_cast<KinectIrSharedMemoryHeader*>(this->irShmPtr);
    auto* irOut = reinterpret_cast<uint16_t*>(this->irShmPtr + sizeof(KinectIrSharedMemoryHeader));
    uint16_t* depthOut = irOut + KINECT_IR_PIXELS;
    const float* irIn = reinterpret_cast<const float*>(irFrame->data);
    const bool hasDepth = depthFrame && depthFrame->width == KINECT2_IR_WIDTH && depthFrame->height == KINECT2_IR_HEIGHT;
    const float* depthIn = hasDepth ? reinterpret_cast<const float*>(depthFrame->data) : nullptr;

    __atomic_add_fetch(&hdr->sequence, 1, __ATOMIC_RELEASE);

    for (int i = 0; i < KINECT_IR_PIXELS; ++i) {
        float v = irIn[i];
        irOut[i] = static_cast<uint16_t>(v <= 0.0f ? 0.0f : (v >= 65535.0f ? 65535.0f : v));
    }
    if (hasDepth) {
        for (int i = 0; i < KINECT_IR_PIXELS; ++i) {
            float d = depthIn[i];  // millimetres, 0 = invalid
            depthOut[i] = static_cast<uint16_t>(d <= 0.0f ? 0.0f : (d >= 65535.0f ? 65535.0f : d));
        }
    } else {
        memset(depthOut, 0, KINECT_IR_PIXELS * sizeof(uint16_t));
    }

    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    hdr->timestamp_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
    hdr->device_ts = irFrame->timestamp;
    hdr->device_seq = irFrame->sequence;
    hdr->flags = hasDepth ? 1u : 0u;

    __atomic_add_fetch(&hdr->sequence, 1, __ATOMIC_RELEASE);
}

void Kinect2Pipe::closeIrShm() {
    if (this->irShmPtr) {
        munmap(this->irShmPtr, this->irShmTotalSize);
        this->irShmPtr = nullptr;
        unlink(KINECT_IR_SHM_NAME);
    }
    if (this->irShmFd >= 0) {
        close(this->irShmFd);
        this->irShmFd = -1;
    }
}
