# kinect2pipe (JohnGF fork)

Turn an old **Kinect v2 (Xbox One Kinect)** into something useful on Linux: a 1080p webcam for
Discord / OBS / browsers, an IR night-vision camera, and a low-latency colour + IR + depth source
for your own code, all from one process.

This is a personal fork of [swedishborgie/kinect2pipe](https://github.com/swedishborgie/kinect2pipe)
(MIT, by Michael Powers). The original README follows below. If this fork helped you get a
Kinect doing something again, a ⭐ is appreciated.

## What this fork adds

- **1080p native output** (1920x1080 YUV420, no scaling). Upstream's output-scaling option is
  not carried over; apps that need 720p can scale on their side.
- **Camera modes**, switchable live: RGB normal, RGB low-light boost (gamma LUT) and IR night
  vision (the Kinect's active IR illuminator, works in the dark).
  Keys in the terminal: `Space`/`t`/`n` cycle, `1` RGB, `2` boost, `3`/`i` IR, `h` help, `q` quit.
  From anywhere: `pkill -USR1 kinect2pipe` toggles IR, `pkill -USR2 kinect2pipe` toggles boost.
- **Only streams while someone watches**: the Kinect starts when a client opens the v4l2loopback
  device and stops when the last one closes (from upstream), and only the colour or IR pipeline
  that is needed runs.
- **Second loopback device** for OBS virtual-camera setups (`kinect_start.sh --obs`).
- **Lock-free shared memory** at `/dev/shm/kinect2pipe_shm`: every frame (YUV420) with a seqlock
  header and `CLOCK_MONOTONIC` timestamp, for programs that want frames without going through V4L2.
- **Parallel IR + depth** with `--ir`: raw 512x424 IR and depth (mm) in
  `/dev/shm/kinect2pipe_ir_shm` while the colour webcam keeps running. Both shared-memory headers
  carry the Kinect's own device timestamp, so colour and IR/depth frames can be paired.
- **`kinect_start.sh`**: builds if needed, loads `v4l2loopback` with the right options, starts the
  pipe. Options: `--obs`, `--ir`; `KINECT2PIPE_CPUS=14,15` pins it to given cores.
- Built with `-O3 -march=native` (build on the machine you run on).

Used by [kinect_eye_tracker](https://github.com/JohnGF/kinect_eye_tracker) (head pose and gaze
to LSL for a BCI platform), which also has a Python reader for both shared-memory segments
(`shm_client.py`).

### Quick start (this fork)

```bash
# libfreenect2 built next to this repo (../libfreenect2/build/lib), or its libs in ./lib
make                      # hand-written Makefile; or use CMake with a system-wide libfreenect2
./kinect_start.sh         # webcam on /dev/video10   (add --ir for IR + depth shared memory)
mpv av://v4l2:/dev/video10
```

### Shared-memory layout

Both segments start with a 64-byte little-endian header; the writer increments `sequence` before
and after each frame (odd = being written), so a reader copies the frame and accepts it only if
`sequence` was even and unchanged.

| Offset | `/dev/shm/kinect2pipe_shm` | `/dev/shm/kinect2pipe_ir_shm` |
|---|---|---|
| 0 | magic `0x4B325050` ("K2PP") | magic `0x4B324952` ("K2IR") |
| 4 | version (1) | version (1) |
| 8, 12 | width, height (1920, 1080) | width, height (512, 424) |
| 16 | payload bytes (YUV420) | payload bytes |
| 20 | mode (0 RGB, 1 boost, 2 IR) | flags (bit 0 = depth present) |
| 24 | sequence (u64) | sequence (u64) |
| 32 | timestamp, CLOCK_MONOTONIC ns (u64) | timestamp, CLOCK_MONOTONIC ns (u64) |
| 40 | Kinect device timestamp (u32, 0.125 ms ticks) | Kinect device timestamp (u32) |
| 44 | Kinect frame sequence (u32) | Kinect frame sequence (u32) |
| 64 | YUV420 frame | uint16 IR[424x512], then uint16 depth_mm[424x512] |

---

# Original README

# kinect2pipe
A small application for piping output from libfreenect2 to a v4l2loopback device so the Kinect 2 can be used as a webcam
in Linux.

The [original Kinect for Xbox driver](https://github.com/torvalds/linux/blob/master/drivers/media/usb/gspca/kinect.c)
for Linux essentially exposes the raw RGB video stream from the Kinect. This worked well for the original Kinect because
the video formats were widely supported by V4L2 clients. The Kinect 2 on the other hand emits JPEG frames. There is
a [Kinect 2 kernel module](https://github.com/yoshimoto/gspca-kinect2) that works well but most clients don't support
the MJPEG format which makes the module of limited use with mainstream applications.

It seemed like a bad idea to do JPEG parsing and colorspace conversions in a kernel module and the v4l2loopback is a
stable kernel module that allows clients to do their work in userspace while still providing a standard V4L2 interface.

This application uses the [libfreenect2](https://github.com/OpenKinect/libfreenect2) library to connect to the Kinect 2
and get RGB color frames and then uses [libswscale](https://ffmpeg.org/libswscale.html) to perform a colorspace
conversion to YUV420 which is a much more generally supported format by v42l clients. Since JPEG decoding is a CPU
intensive operation the application will only start the stream when client applications open a file handle to the
v4l2loopback device and will close the video stream when all handles are closed.

## Compiling

You will need to have installed the `libswscale5` and `libfreenect2` packages in order to compile. Then to compile:

    mkdir build
    cd build
    cmake .. && make

## Installation

If you're using Ubuntu you can grab the packages from
[my PPA](https://launchpad.net/~swedishborgie/+archive/ubuntu/kinect2pipe). If you want to install from source (from
the build directory in the compiling step):

    sudo cp kinect2pipe /usr/bin/kinect2pipe
    cp ../contrib/systemd/kinect2pipe.service ~/.config/systemd/user/
    
## Configuration / Usage

First you'll need a v4l2loopback device which you can create by installing the `v4l2loopback-dkms` package. Then you can
create the loopback device by doing the following:

    sudo modprobe v4l2loopback devices=1 video_nr=0 card_label=Kinect2 exclusive_caps=1

To add the device permanently add the following to `/etc/modules`:

    v4l2loopback
    
Then add the following to `/etc/modprobe.d/v4l2loopback.conf`:

    options v4l2loopback devices=1 video_nr=0 card_label=Kinect2 exclusive_caps=1

In the example above this should create a v4l2loopback device at `/dev/video0`. If you change the device number above
make sure to modify `~/.config/systemd/user/kinect2pipe.service` to reflect the change. Then you can install and start
the service:

    systemctl --user enable kinect2pipe
    systemctl --user start kinect2pipe
    
You can then test everything is working using VLC (or any other V4L2 client):

    vlc v4l2:///dev/video0
    
You should get video output:
![example snapshot](snapshot.png)