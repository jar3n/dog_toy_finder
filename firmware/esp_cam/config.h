// Per-device configuration for the stereo camera.
// CAMERA_SIDE is set by the PlatformIO build flag (-DCAMERA_SIDE="left"/"right");
// when building in the Arduino IDE, define it here before including this file.
#pragma once

#ifndef CAMERA_SIDE
#define CAMERA_SIDE "left"
#endif

// Image orientation: change these if the preview is upside down or mirrored.
#define CAMERA_VFLIP 1     // 1 = flip vertically
#define CAMERA_HMIRROR 0   // 1 = mirror left-right

// JPEG capture size. Keep each frame size next to the width/height reported by
// /status so changing the resolution only takes one edit here.
#define CAMERA_FRAME_SIZE FRAMESIZE_VGA
#define CAMERA_FRAME_WIDTH 640
#define CAMERA_FRAME_HEIGHT 480

// Boards without PSRAM fall back to a smaller JPEG in internal RAM.
#define CAMERA_FALLBACK_FRAME_SIZE FRAMESIZE_QVGA
#define CAMERA_FALLBACK_FRAME_WIDTH 320
#define CAMERA_FALLBACK_FRAME_HEIGHT 240
