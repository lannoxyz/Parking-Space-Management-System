#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==========================================
// WiFi
// ==========================================
const char* wifi_ssid     = "Shao Fan's S23 Ultra";
const char* wifi_password = "my9nd3k3zn4d9rz";

// ==========================================
// OLED
// ==========================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define OLED_SDA_PIN 1  
#define OLED_SCL_PIN 2

// ==========================================
// Camera Pins (WROOM Dev Module)
// ==========================================
#define CAM_PIN_SIOD   21
#define CAM_PIN_SIOC   22
#define CAM_PIN_VSYNC  27
#define CAM_PIN_HREF   14
#define CAM_PIN_PCLK   26
#define CAM_PIN_XCLK   25
#define CAM_PIN_D0     32
#define CAM_PIN_D1     33
#define CAM_PIN_D2     16
#define CAM_PIN_D3     17
#define CAM_PIN_D4     5
#define CAM_PIN_D5     4
#define CAM_PIN_D6     18
#define CAM_PIN_D7     13

#define LED_PIN 2  // 内置 LED，用于状态指示

// ==========================================
// Globals
// ==========================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
httpd_handle_t camera_http_server = NULL;
bool is_oled_ready = false;
bool is_camera_ready = false;

// ==========================================
// OLED Update
// ==========================================
void update_oled(String text) {
    if (!is_oled_ready) return;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("UNM Parking");
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.println(text);
    display.display();
}

// ==========================================
// HTTP Handlers
// ==========================================
static esp_err_t index_handler(httpd_req_t *req) {
    const char* html =
    "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>body{font-family:sans-serif; text-align:center; padding:20px;} "
    "button{width:100%; height:60px; font-size:24px; margin:10px 0; border-radius:10px; border:none;} "
    ".btn-cam{background:#3498db; color:white;}</style>"
    "</head><body>"
    "<h1>📷 Exit Camera</h1>"
    "<p><a href='/capture'><button class='btn-cam'>📸 Take Photo</button></a></p>"
    "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req) {
    if (!is_camera_ready) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    uint8_t * jpg_buf = NULL;
    size_t jpg_len = 0;

    bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format, 40, &jpg_buf, &jpg_len);
    esp_camera_fb_return(fb);

    if (!ok) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char*)jpg_buf, jpg_len);
    free(jpg_buf);
    return ESP_OK;
}

// ==========================================
// Start HTTP Server
// ==========================================
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri   = { .uri="/",       .method=HTTP_GET, .handler=index_handler };
    httpd_uri_t capture_uri = { .uri="/capture",.method=HTTP_GET, .handler=capture_handler };

    if (httpd_start(&camera_http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_http_server, &index_uri);
        httpd_register_uri_handler(camera_http_server, &capture_uri);
    }
}

// ==========================================
// Setup
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(200);

    // OLED
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(100000);
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        is_oled_ready = false;
    } else {
        is_oled_ready = true;
        display.clearDisplay();
        display.display();
        delay(100);
        update_oled("Booting...");
    }

    // Camera Init
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn     = -1;
    config.pin_reset    = -1;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        is_camera_ready = false;
        update_oled("Cam Error");
    } else {
        is_camera_ready = true;
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // WiFi
    WiFi.begin(wifi_ssid, wifi_password);
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        retry++;
    }

    if(WiFi.status() == WL_CONNECTED) {
        update_oled("IP: " + WiFi.localIP().toString());
        startCameraServer();
    } else {
        update_oled("WiFi Fail");
    }
}

// ==========================================
// Loop
// ==========================================
void loop() {
    delay(20); // 空循环即可
}