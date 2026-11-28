# ESP32-CAM stereo-vision firmware

Each ESP32-CAM module is one eye of the stereo pair used by the Dog Toy Finder
to find dog toys in 3D. The two modules are independent HTTP servers; the
computer fetches both and pairs the frames.

## Layout

| File | Purpose |
|---|---|
| `esp32_cam_stereo.ino` | Camera + Wi-Fi + HTTP server firmware |
| `platformio.ini` | PlatformIO project with `esp32cam_left` / `esp32cam_right` envs |
| `wifi_config.example.h` | Wi-Fi credentials template (copy to `wifi_config.h`) |
| `README.md` | This file |

## Build

With PlatformIO (recommended):

```bash
cd firmware/esp_cam
cp wifi_config.example.h wifi_config.h   # fill in SSID/password
pio run -e esp32cam_left    # flash the left eye
pio run -e esp32cam_right   # flash the right eye
pio run -e esp32cam_left -t upload
```

With the Arduino IDE: open the sketch, select board "AI Thinker ESP32-CAM",
set the `CAMERA_SIDE` macro at the top of the sketch to `"left"` or `"right"`,
and upload.

## HTTP API

| Endpoint | Returns |
|---|---|
| `/` | Preview page for checking aim and focus |
| `/capture` | One fresh JPEG (VGA). Headers: `X-Camera-Side`, `X-Capture-Millis`, `X-Frame-Seq` |
| `/capture?flash=1` | Same, with the white flash LED on for 150 ms |
| `/stream` | MJPEG stream (`multipart/x-mixed-replace`), ~10 fps |
| `/status` | JSON: side, sensor id, resolution, RSSI, photo count, uptime |
| `/sync` | JSON: side + `millis`, for aligning the two eyes' clocks |

The mDNS names are `esp32cam-left.local` and `esp32cam-right.local`.

## Stereo capture strategy

The two cameras are not hardware-synchronised. The host should:

1. Probe `/sync` on both eyes to estimate the clock offset.
2. Fetch `/capture` from both eyes as fast as possible.
3. Pair frames using `X-Capture-Millis` / `X-Frame-Seq`; discard pairs whose
   capture times differ by more than a few tens of milliseconds.
4. Rectify and compute disparity on the computer (OpenCV / ROS 2 stereo
   pipeline), not on the ESP32.

For tighter sync, route a single GPIO trigger line to both boards and have
`/capture` honour it; that is left as an extension.

## Notes

- The flash LED is on GPIO 4 and doubles as the SD-card data line; do not use
  an SD card together with the flash.
- VGA (640 x 480) JPEG keeps each frame small (20-40 KB). Raise `frame_size`
  in `start_camera()` if more resolution is needed.
- Stereo depth and toy detection run on the computer; this firmware only serves
  images.
