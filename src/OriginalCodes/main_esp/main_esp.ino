#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define WIFI_SSID     "Lanno"
#define WIFI_PASSWORD "lannoxyz"

// ==========================================
// Camera Pins (Freenove ESP32-S3 + OV7670)
// ==========================================
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM     8
#define SIOC_GPIO_NUM     9
#define Y9_GPIO_NUM      18
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      16
#define Y6_GPIO_NUM      15
#define Y5_GPIO_NUM       7
#define Y4_GPIO_NUM       6
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   12
#define HREF_GPIO_NUM    13
#define PCLK_GPIO_NUM    11
#define LED_PIN          38

// ==========================================
// Globals
// ==========================================
httpd_handle_t camera_http_server = NULL;
bool is_camera_ready = false;

// ==========================================
// HTTP: Index
// ==========================================
static esp_err_t index_handler(httpd_req_t *req) {
    const char* html =
        "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;text-align:center;padding:20px;background:#111;color:white;'>"
        "<h1>ESP32-S3 OV7670 Camera</h1>"
        "<p><a href='/capture'><button style='padding:15px 30px;font-size:18px;"
        "background:#3498db;color:white;border:none;border-radius:8px;cursor:pointer;'>"
        "Capture Frame</button></a></p>"
        "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ==========================================
// HTTP: Capture → JPEG
// ==========================================
static esp_err_t capture_handler(httpd_req_t *req) {
    if (!is_camera_ready) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Frame capture failed!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    uint8_t* jpg_buf = NULL;
    size_t   jpg_len = 0;

    // OV7670 has no hardware JPEG — must software-encode
    // Quality 12 is a good balance: decent image, lower CPU time vs quality=10
    bool ok = fmt2jpg(
        fb->buf, fb->len,
        fb->width, fb->height,
        fb->format,
        12,
        &jpg_buf, &jpg_len
    );

    esp_camera_fb_return(fb);

    if (!ok) {
        Serial.println("JPEG encode failed!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    Serial.printf("Captured JPEG: %u bytes\n", jpg_len);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, (const char*)jpg_buf, jpg_len);
    free(jpg_buf);
    return ESP_OK;
}

// ==========================================
// Start HTTP Server
// ==========================================
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port      = 80;
    config.max_uri_handlers = 8;
    // Increase stack size for HTTP tasks — fmt2jpg is stack-hungry
    config.stack_size       = 8192;

    httpd_uri_t index_uri   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler,   .user_ctx = NULL };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };

    if (httpd_start(&camera_http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_http_server, &index_uri);
        httpd_register_uri_handler(camera_http_server, &capture_uri);
        Serial.println("HTTP Server started.");
    } else {
        Serial.println("HTTP Server failed to start!");
    }
}

// ==========================================
// Setup
// ==========================================
void setup() {
    // --- Disable brownout detector (ESP32-S3 correct method) ---
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0) targets ESP32 original,
    // wrong register on S3. REG_CLR_BIT clears only the enable bit — correct on S3.
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);

    Serial.begin(115200);
    delay(300);
    Serial.println("\nESP32-S3 OV7670 Camera Boot");

    // ----------------------------------------
    // Camera Config
    // OV7670 note:
    //   - No hardware JPEG, must use RGB565 or YUV422 + software encode
    //   - QVGA (320x240) recommended: 4x less data than VGA,
    //     massively reduces fmt2jpg CPU time and peak current draw
    //   - fb_count=2 allows grabbing next frame while sending current one
    // ----------------------------------------
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    config.xclk_freq_hz = 10000000;          // 10 MHz — OV7670 is stable at 10MHz,
                                              // 20MHz can cause glitches on some boards
    config.pixel_format = PIXFORMAT_RGB565;   // OV7670: RGB565 or YUV422 only
    config.frame_size   = FRAMESIZE_QVGA;     // 320x240 — half the CPU load of VGA
    config.fb_count     = 2;                  // Double buffer — smoother capture
    config.fb_location  = CAMERA_FB_IN_PSRAM; // Use PSRAM for frame buffers
    config.grab_mode    = CAMERA_GRAB_LATEST; // Always get newest frame

    if (esp_camera_init(&config) != ESP_OK) {
        is_camera_ready = false;
        Serial.println("Camera Init Failed!");
    } else {
        is_camera_ready = true;
        Serial.println("Camera Ready — QVGA 320x240 RGB565");

        sensor_t* s = esp_camera_sensor_get();
        if (s) {
            s->set_framesize(s, FRAMESIZE_QVGA);
            s->set_brightness(s, 1);
            s->set_contrast(s, 1);
            s->set_saturation(s, 0);
            s->set_whitebal(s, 1);
            s->set_awb_gain(s, 1);
            s->set_exposure_ctrl(s, 1);
            s->set_gain_ctrl(s, 1);
            s->set_hmirror(s, 0);
            s->set_vflip(s, 0);
            Serial.println("Sensor tuned.");
        }
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // ----------------------------------------
    // WiFi
    // ----------------------------------------
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 30) {
        delay(500);
        Serial.print(".");
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.println("IP: " + WiFi.localIP().toString());
        Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
        startCameraServer();
        Serial.println("Open: http://" + WiFi.localIP().toString());
    } else {
        Serial.println("\nWiFi Failed after 30 retries.");
    }

    Serial.printf("PSRAM Total: %d KB\n", ESP.getPsramSize() / 1024);
    Serial.printf("PSRAM Free:  %d KB\n", ESP.getFreePsram() / 1024);
    Serial.printf("Heap Free:   %d KB\n", ESP.getFreeHeap() / 1024);
}

// ==========================================
// Loop
// ==========================================
void loop() {
    static unsigned long last_print = 0;
    if (millis() - last_print > 10000) {
        Serial.printf("PSRAM Free: %d KB | Heap Free: %d KB\n",
            ESP.getFreePsram() / 1024,
            ESP.getFreeHeap() / 1024);
        last_print = millis();
    }
    delay(100);
}
