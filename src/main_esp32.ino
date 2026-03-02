#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_arduino_version.h>

// FIX: credentials moved to secrets.h (add secrets.h to .gitignore)
#include "secrets.h"

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
// Servo
// ==========================================
const int servo_entrance_pin = 21;
const int servo_exit_pin     = 47;
const int servo_freq = 50;
const int servo_res  = 16;

const int duty_gate_close = 1638;
const int duty_gate_open  = 4915;

// ==========================================
// Camera (OV7670 / Freenove S3)
// ==========================================
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

// ==========================================
// Globals
// ==========================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
httpd_handle_t camera_http_server = NULL;

bool is_oled_ready   = false;
bool is_camera_ready = false;

bool is_entrance_open = false;
unsigned long entrance_timer = 0;

bool is_exit_open = false;
unsigned long exit_timer = 0;

const unsigned long GATE_TIMEOUT = 5000;

// ==========================================
// Servo Wrapper
// ==========================================
void myservo_write(int pin, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, duty);
#else
    if (pin == servo_entrance_pin) ledcWrite(2, duty);
    if (pin == servo_exit_pin)     ledcWrite(3, duty);
#endif
}

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
    "<style>button{width:100%; height:60px; font-size:24px; margin:10px 0; border-radius:10px; "
    "border:none;} .btn-cam{background:#3498db; color:white;} .btn-ent{background:#2ecc71; color:white;} "
    ".btn-ext{background:#e74c3c; color:white;}</style>"
    "</head><body style='font-family:sans-serif; padding:20px; text-align:center;'>"
    "<h1>🅿️ UNM Parking</h1>"
    "<p><a href='/capture'><button class='btn-cam'>📸 Take Photo</button></a></p>"
    "<p><a href='/action?type=enter'><button class='btn-ent'>⬆️ Open Entrance</button></a></p>"
    "<p><a href='/action?type=exit'><button class='btn-ext'>⬇️ Open Exit</button></a></p>"
    "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ------------------------------------------
// /capture → JPEG output for Python
// ------------------------------------------
static esp_err_t capture_handler(httpd_req_t *req) {
    if (!is_camera_ready) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    uint8_t* jpg_buf = NULL;
    size_t   jpg_len = 0;

    // OV7670 → software JPEG encode required
    bool ok = fmt2jpg(
        fb->buf, fb->len, fb->width, fb->height,
        fb->format, 40, &jpg_buf, &jpg_len
    );

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

// ------------------------------------------
// /action
// ------------------------------------------
static esp_err_t action_handler(httpd_req_t *req) {
    char buf[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "type", param, sizeof(param)) == ESP_OK) {
            if (!strcmp(param, "enter")) {
                myservo_write(servo_entrance_pin, duty_gate_open);
                update_oled("Entrance Open");
                is_entrance_open = true;
                entrance_timer = millis();
            }
            if (!strcmp(param, "exit")) {
                myservo_write(servo_exit_pin, duty_gate_open);
                update_oled("Exit Open");
                is_exit_open = true;
                exit_timer = millis();
            }
        }
    }
    const char* resp = "<script>alert('Action Done!'); window.location.href='/';</script>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ==========================================
// Start HTTP Server
// ==========================================
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler };
    httpd_uri_t action_uri  = { .uri = "/action",  .method = HTTP_GET, .handler = action_handler };

    if (httpd_start(&camera_http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_http_server, &index_uri);
        httpd_register_uri_handler(camera_http_server, &capture_uri);
        httpd_register_uri_handler(camera_http_server, &action_uri);
    }
}

// ==========================================
// Setup
// ==========================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    delay(200);

    // OLED
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(100000);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
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
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        is_camera_ready = false;
        update_oled("Cam Error");
        Serial.println("Camera Init Failed!");
    } else {
        is_camera_ready = true;
        Serial.println("Camera Ready!");
    }

    // LED off
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Servo init
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(servo_entrance_pin, servo_freq, servo_res);
    ledcAttach(servo_exit_pin,     servo_freq, servo_res);
#else
    ledcSetup(2, servo_freq, servo_res);
    ledcSetup(3, servo_freq, servo_res);
    ledcAttachPin(servo_entrance_pin, 2);
    ledcAttachPin(servo_exit_pin,     3);
#endif
    myservo_write(servo_entrance_pin, duty_gate_close);
    myservo_write(servo_exit_pin,     duty_gate_close);

    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        Serial.print(".");
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
        update_oled("IP: " + WiFi.localIP().toString());
        startCameraServer();
    } else {
        Serial.println("\nWiFi Failed after 20 retries.");
        update_oled("WiFi Fail");
    }
}

// ==========================================
// Loop
// ==========================================
void loop() {
    unsigned long now = millis();

    if (is_entrance_open && (now - entrance_timer > GATE_TIMEOUT)) {
        myservo_write(servo_entrance_pin, duty_gate_close);
        update_oled("IP: " + WiFi.localIP().toString());
        is_entrance_open = false;
    }

    if (is_exit_open && (now - exit_timer > GATE_TIMEOUT)) {
        myservo_write(servo_exit_pin, duty_gate_close);
        update_oled("IP: " + WiFi.localIP().toString());
        is_exit_open = false;
    }

    delay(20);
}
