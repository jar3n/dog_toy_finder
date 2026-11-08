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