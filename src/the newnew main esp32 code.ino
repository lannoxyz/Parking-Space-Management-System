#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_arduino_version.h> // 引入版本判断库

// ==========================================
// 1. 硬件配置 (Hardware Config)
// ==========================================

// --- WiFi ---
const char* wifi_ssid = "Shao Fan's S23 Ultra";
const char* wifi_password = "my9nd3k3zn4d9rz";

// --- OLED (I2C) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define OLED_SDA_PIN 1  
#define OLED_SCL_PIN 2

// --- 舵机 (Servo) ---
const int servo_entrance_pin = 21;
const int servo_exit_pin = 47; 
const int servo_freq = 50; 
const int servo_res = 16; 
// 16位精度(0-65535)下的 50Hz 占空比
const int duty_gate_close = 1638; // 约 0.5ms (关闭)
const int duty_gate_open = 4915;  // 约 1.5ms (90度开启)

// --- 摄像头 (OV7670 / Freenove S3) ---
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    42
#define SIOC_GPIO_NUM    41
#define Y9_GPIO_NUM      18
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      16
#define Y6_GPIO_NUM      15
#define Y5_GPIO_NUM      7
#define Y4_GPIO_NUM      6
#define Y3_GPIO_NUM      5
#define Y2_GPIO_NUM      4
#define VSYNC_GPIO_NUM   12
#define HREF_GPIO_NUM    13
#define PCLK_GPIO_NUM    11

#define LED_PIN 38

// ==========================================
// 2. 全局变量
// ==========================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
httpd_handle_t camera_http_server = NULL;

bool is_oled_ready = false;
bool is_camera_ready = false;

// 门控计时器
bool is_entrance_open = false;
unsigned long entrance_timer = 0;
bool is_exit_open = false;
unsigned long exit_timer = 0;
const unsigned long GATE_TIMEOUT = 5000; // 5秒自动关门

// ==========================================
// 3. 辅助函数 (兼容性封装)
// ==========================================

// 万能舵机写入函数 (自动判断版本)
void myservo_write(int pin, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    // 3.0+ 版本：直接写引脚
    ledcWrite(pin, duty); 
#else
    // 2.x 版本：写通道 (假设已绑定: 入口=2, 出口=3)
    if(pin == servo_entrance_pin) ledcWrite(2, duty);
    if(pin == servo_exit_pin) ledcWrite(3, duty);
#endif
}

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
// 4. Web Server 处理函数
// ==========================================

// 主页 (解决 "URI Not Exist")
static esp_err_t index_handler(httpd_req_t *req) {
    const char* html = 
    "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>button{width:100%; height:60px; font-size:24px; margin:10px 0; border-radius:10px; border:none;} "
    ".btn-cam{background:#3498db; color:white;} .btn-ent{background:#2ecc71; color:white;} .btn-ext{background:#e74c3c; color:white;}</style>"
    "</head><body style='font-family:sans-serif; padding:20px; text-align:center;'>"
    "<h1>🅿️ UNM Parking</h1>"
    "<p><a href='/capture'><button class='btn-cam'>📸 Take Photo</button></a></p>"
    "<p><a href='/action?type=enter'><button class='btn-ent'>⬆️ Open Entrance</button></a></p>"
    "<p><a href='/action?type=exit'><button class='btn-ext'>⬇️ Open Exit</button></a></p>"
    "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// 拍照
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
    // OV7670 必须软转码 (RGB565 -> JPEG)
    bool converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format, 40, &jpg_buf, &jpg_len);
    esp_camera_fb_return(fb);

    if (!converted) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
    free(jpg_buf);
    return ESP_OK;
}

// 开门动作
static esp_err_t action_handler(httpd_req_t *req) {
    char buf[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "type", param, sizeof(param)) == ESP_OK) {
            if (strcmp(param, "enter") == 0) {
                myservo_write(servo_entrance_pin, duty_gate_open);
                update_oled("Enter Open");
                is_entrance_open = true;
                entrance_timer = millis();
            } else if (strcmp(param, "exit") == 0) {
                myservo_write(servo_exit_pin, duty_gate_open);
                update_oled("Exit Open");
                is_exit_open = true;
                exit_timer = millis();
            }
        }
    }
    // 返回简单的网页提示，并在 1秒后自动跳回主页
    const char* resp = "<script>alert('Action Done!'); window.location.href='/';</script>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };
    httpd_uri_t action_uri = { .uri = "/action", .method = HTTP_GET, .handler = action_handler, .user_ctx = NULL };

    if (httpd_start(&camera_http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_http_server, &index_uri);
        httpd_register_uri_handler(camera_http_server, &capture_uri);
        httpd_register_uri_handler(camera_http_server, &action_uri);
        Serial.println("Web Server Ready!");
    }
}

// ==========================================
// 5. Setup (核心初始化)
// ==========================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // 关闭掉电检测
    delay(500); // 等待电压稳定

    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println("\n--- Booting ---");

    // 1. 初始化 OLED (修复花屏)
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(100000); // ⚠️ 降速到 100kHz 消除杂波
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED Failed");
        is_oled_ready = false;
    } else {
        is_oled_ready = true;
        display.clearDisplay(); // ⚠️ 立即清屏
        display.display();
        delay(100);
        update_oled("System Init...");
    }

    // 2. 初始化摄像头 (⚠️ 必须在舵机之前)
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000; // OV7670 10MHz
    config.pixel_format = PIXFORMAT_RGB565; 
    config.frame_size = FRAMESIZE_QVGA; // 320x240 (开启 PSRAM)
    config.jpeg_quality = 12; 
    config.fb_count = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera Init Failed");
        update_oled("Cam Error!");
        is_camera_ready = false;
    } else {
        is_camera_ready = true;
        Serial.println("Camera OK");
    }

    // 3. 初始化舵机 (兼容性写法)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(servo_entrance_pin, servo_freq, servo_res);
    ledcAttach(servo_exit_pin, servo_freq, servo_res);
#else
    ledcSetup(2, servo_freq, servo_res);
    ledcSetup(3, servo_freq, servo_res);
    ledcAttachPin(servo_entrance_pin, 2);
    ledcAttachPin(servo_exit_pin, 3);
#endif
    myservo_write(servo_entrance_pin, duty_gate_close);
    myservo_write(servo_exit_pin, duty_gate_close);

    // 4. WiFi
    WiFi.begin(wifi_ssid, wifi_password);
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        Serial.print(".");
        retry++;
    }

    if(WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi OK");
        update_oled("IP: " + WiFi.localIP().toString()); // 显示 IP
        startCameraServer();
    } else {
        update_oled("WiFi Fail");
    }
}

// ==========================================
// 6. 主循环
// ==========================================
void loop() {
    unsigned long now = millis();

    // 自动关门逻辑
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
