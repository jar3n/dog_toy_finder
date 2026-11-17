// HTTP server implementation for the stereo camera endpoints.
#include "http_server.h"

#include <Arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "camera_driver.h"
#include "camera_pins.h"

#include "esp_camera.h"
#include "esp_http_server.h"

// Shared capture counters.
uint32_t photos_served = 0;
uint32_t frame_seq = 0;
uint32_t last_capture_ms = 0;

static httpd_handle_t server = NULL;

// /  - preview page for checking aim and focus.
static esp_err_t index_handler(httpd_req_t *req) {
    static const char page[] =
        "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width'>"
        "<title>ESP32-CAM %s</title></head><body style='font-family:sans-serif'>"
        "<h2>ESP32-CAM %s eye</h2>"
        "<p><button onclick=\"img.src='/capture?t='+Date.now()\">New photo</button> "
        "<button onclick=\"img.src='/capture?flash=1&t='+Date.now()\">New photo with flash</button></p>"
        "<img id='img' src='/capture' style='max-width:100%%'></body></html>";
    char body[1024];
    snprintf(body, sizeof(body), page, CAMERA_SIDE, CAMERA_SIDE);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, body);
}

// /capture - one fresh JPEG. Headers carry the side and capture time so the
// host can pair left/right frames for stereo.
static esp_err_t capture_handler(httpd_req_t *req) {
    bool flash = false;
    char query[32];
    char value[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "flash", value, sizeof(value)) == ESP_OK) {
        flash = atoi(value) == 1;
    }
    if (flash) {
        digitalWrite(FLASH_LED_GPIO_NUM, HIGH);
        delay(150);   // let auto exposure settle with the light on
    }
    camera_fb_t *fb = take_fresh_photo();
    if (flash) {
        digitalWrite(FLASH_LED_GPIO_NUM, LOW);
    }
    if (!fb) {
        Serial.println("Photo failed.");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    last_capture_ms = millis();
    frame_seq++;
    photos_served++;

    char side_hdr[16];
    char time_hdr[16];
    char seq_hdr[16];
    snprintf(side_hdr, sizeof(side_hdr), "%s", CAMERA_SIDE);
    snprintf(time_hdr, sizeof(time_hdr), "%lu", (unsigned long) last_capture_ms);
    snprintf(seq_hdr, sizeof(seq_hdr), "%lu", (unsigned long) frame_seq);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Camera-Side", side_hdr);
    httpd_resp_set_hdr(req, "X-Capture-Millis", time_hdr);
    httpd_resp_set_hdr(req, "X-Frame-Seq", seq_hdr);

    esp_err_t result = httpd_resp_send(req, (const char *) fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return result;
}

// /stream - MJPEG stream for live vision on the host.
static esp_err_t stream_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Stream frame grab failed.");
            break;
        }
        char part_buf[128];
        size_t hlen = snprintf(part_buf, sizeof(part_buf),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            (unsigned) fb->len);
        esp_err_t res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *) fb->buf, fb->len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, "\r\n", 2);
        }
        esp_camera_fb_return(fb);
        if (res != ESP_OK) {
            // Client disconnected.
            break;
        }
        // ~10 fps keeps the Wi-Fi link and the host decoder comfortable.
        delay(100);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// /status - JSON with side, network and capture counters.
static esp_err_t status_handler(httpd_req_t *req) {
    sensor_t *s = esp_camera_sensor_get();
    char json[256];
    snprintf(json, sizeof(json),
             "{\"side\":\"%s\",\"sensor_id\":\"0x%x\",\"width\":%d,\"height\":%d,"
             "\"rssi_dbm\":%d,\"photos_served\":%lu,\"frame_seq\":%lu,"
             "\"last_capture_ms\":%lu,\"uptime_ms\":%lu}",
             CAMERA_SIDE, s ? s->id.PID : 0,
             (s && s->status.framesize == FRAMESIZE_VGA) ? 640 : 320,
             (s && s->status.framesize == FRAMESIZE_VGA) ? 480 : 240,
             WiFi.RSSI(), (unsigned long) photos_served,
             (unsigned long) frame_seq, (unsigned long) last_capture_ms,
             (unsigned long) millis());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

// /sync - lightweight clock probe used to align the two eyes.
static esp_err_t sync_handler(httpd_req_t *req) {
    char json[96];
    snprintf(json, sizeof(json), "{\"side\":\"%s\",\"millis\":%lu}",
             CAMERA_SIDE, (unsigned long) millis());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

// Start the HTTP server and register all endpoints.
void start_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    if (httpd_start(&server, &config) != ESP_OK) {
        Serial.println("HTTP server failed to start.");
        return;
    }
    httpd_uri_t index_uri   = {"/",        HTTP_GET, index_handler,   NULL};
    httpd_uri_t capture_uri = {"/capture", HTTP_GET, capture_handler, NULL};
    httpd_uri_t stream_uri  = {"/stream",  HTTP_GET, stream_handler,  NULL};
    httpd_uri_t status_uri  = {"/status",  HTTP_GET, status_handler,  NULL};
    httpd_uri_t sync_uri    = {"/sync",    HTTP_GET, sync_handler,    NULL};
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &stream_uri);
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &sync_uri);
}