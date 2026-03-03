#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define WIFI_SSID     "Slowest WiFi in Kiara Plaza 2.4"
#define WIFI_PASSWORD "lannoxyz"

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    42
#define SIOC_GPIO_NUM    41
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

httpd_handle_t camera_http_server = NULL;
bool is_camera_ready = false;

static esp_err_t index_handler(httpd_req_t *req) {
    const char* html =
        "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif; text-align:center; padding:20px;'>"
        "<h1>📷 ESP32 Camera</h1>"
        "<p><a href='/capture'><button style='padding:15px 30px; font-size:18px;'>📸 Capture</button></a></p>"
        "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req) {
    if (!is_camera_ready) { httpd_resp_send_500(req); return ESP_FAIL; }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }

    uint8_t* jpg_buf = NULL;
    size_t   jpg_len = 0;
    bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format, 10, &jpg_buf, &jpg_len);
    esp_camera_fb_return(fb);

    if (!ok) { httpd_resp_send_500(req); return ESP_FAIL; }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char*)jpg_buf, jpg_len);
    free(jpg_buf);
    return ESP_OK;
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri   = { .uri="/",        .method=HTTP_GET, .handler=index_handler,   .user_ctx=NULL };
    httpd_uri_t capture_uri = { .uri="/capture", .method=HTTP_GET, .handler=capture_handler, .user_ctx=NULL };

    if (httpd_start(&camera_http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_http_server, &index_uri);
        httpd_register_uri_handler(camera_http_server, &capture_uri);
        Serial.println("HTTP Server started.");
    }
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    delay(200);

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
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = FRAMESIZE_VGA;
    config.fb_count     = 2;

    if (!psramFound()) {
        Serial.println("No PSRAM! Using QQVGA fallback.");
        config.frame_size  = FRAMESIZE_QQVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
    } else {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode   = CAMERA_GRAB_LATEST;
    }

    if (esp_camera_init(&config) != ESP_OK) {
        is_camera_ready = false;
        Serial.println("Camera Init Failed!");
    } else {
        is_camera_ready = true;
        Serial.println("Camera Ready!");

//        sensor_t* s = esp_camera_sensor_get();
//        if (s) {
//            s->set_brightness(s,    1);
//            s->set_contrast(s,      1);
//            s->set_saturation(s,    0);
//            s->set_sharpness(s,     2);
//            s->set_whitebal(s,      1);
//            s->set_awb_gain(s,      1);
//            s->set_exposure_ctrl(s, 1);
//            s->set_aec2(s,          1);
//            s->set_gain_ctrl(s,     1);
//            s->set_gainceiling(s,   (gainceiling_t)4);
//            s->set_bpc(s,           1);
//            s->set_wpc(s,           1);
//            s->set_raw_gma(s,       1);
//            s->set_lenc(s,          1);
//        }

    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry++ < 20) {
        delay(500); Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
        startCameraServer();
    } else {
        Serial.println("\nWiFi Failed.");
    }

    Serial.println("\n===== System Status =====");
    Serial.printf("Camera     : %s\n",    is_camera_ready ? "OK" : "FAIL");
    Serial.printf("WiFi       : %s\n",    WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "FAIL");
    Serial.printf("PSRAM Total: %d KB\n", ESP.getPsramSize()   / 1024);
    Serial.printf("PSRAM Free : %d KB\n", ESP.getFreePsram()   / 1024);
    Serial.printf("PSRAM Used : %d KB\n", (ESP.getPsramSize()  - ESP.getFreePsram()) / 1024);
    Serial.printf("Heap Total : %d KB\n", ESP.getHeapSize()    / 1024);
    Serial.printf("Heap Free  : %d KB\n", ESP.getFreeHeap()    / 1024);
    Serial.printf("Flash Size : %d MB\n", ESP.getFlashChipSize() / 1024 / 1024);
    Serial.printf("CPU Freq   : %d MHz\n",getCpuFrequencyMhz());
    Serial.println("=========================");
}

void loop() {
    static unsigned long last = 0;
    if (millis() - last > 15000) {
        Serial.printf("[MON] PSRAM Free: %d KB | Heap Free: %d KB | WiFi: %s\n",
            ESP.getFreePsram() / 1024,
            ESP.getFreeHeap()  / 1024,
            WiFi.isConnected() ? "OK" : "LOST");
        last = millis();
    }
    if (!WiFi.isConnected()) { WiFi.reconnect(); delay(5000); }
    delay(100);
}