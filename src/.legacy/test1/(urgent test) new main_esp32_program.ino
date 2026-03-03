#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==========================================
// 1. 定义常量与引脚 (Definitions) - S3 最终版
// ==========================================

// --- WiFi 配置 ---
const char* ssid = "wifi";
const char* password = "wifi password";

// --- OLED 定义 ---
// 使用右上角的 IO1 和 IO2，非常干净
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define OLED_SDA_PIN 1  
#define OLED_SCL_PIN 2

// --- 舵机定义 (PWM) ---
// ⚠️ 修改点：使用 GPIO 21 和 GPIO 47 (右下角相邻的两个引脚)
#define SERVO_ENTRANCE_PIN 21
#define SERVO_EXIT_PIN     47  // 替代危险的 IO20

// 避开摄像头使用的 Timer 0
#define SERVO_TIMER      1 
#define SERVO_CH_ENTRANCE 2
#define SERVO_CH_EXIT     3
#define SERVO_FREQ       50 
#define SERVO_RES        16 

// 16位精度下的占空比 (50Hz)
#define DUTY_GATE_CLOSE 1638 
#define DUTY_GATE_OPEN  4915

// --- 摄像头引脚 (OV7670 / ESP32-S3) ---
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    42 // Camera I2C SDA
#define SIOC_GPIO_NUM    41 // Camera I2C SCL
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
// 2. 全局对象与状态变量 (Globals)
// ==========================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
httpd_handle_t camera_httpd = NULL;

// 状态机变量 (用于替代 delay)
bool is_entrance_open = false;
unsigned long entrance_timer_start = 0;
const unsigned long GATE_OPEN_DURATION = 5000; // 闸机开启 5 秒

bool is_exit_open = false;
unsigned long exit_timer_start = 0;

// ==========================================
// 3. 辅助函数 (Helper Functions)
// ==========================================

// 更新 OLED 显示内容
void update_oled_status(const char* status_text) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("UNM Parking");
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.println(status_text);
    display.display();
}

// ==========================================
// 4. 网络请求处理 (Handlers)
// ==========================================

// 处理图片捕获 (GET /capture)
static esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // OV7670 必须进行格式转换 (RGB565 -> JPEG)
    uint8_t * jpg_buf = NULL;
    size_t jpg_len = 0;
    // ⚠️ 注意：此处转码非常耗时，可能会导致短暂的舵机卡顿
    bool converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format, 60, &jpg_buf, &jpg_len); // 降低质量到60以提速

    esp_camera_fb_return(fb);

    if (!converted) {
        Serial.println("JPEG compression failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
    
    free(jpg_buf);
    return ESP_OK;
}

// 处理控制指令 (GET /action?type=enter 或 ?type=exit)
static esp_err_t action_handler(httpd_req_t *req) {
    char* buf;
    size_t buf_len;
    char param[32];
    
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            
            // 解析 type 参数
            if (httpd_query_key_value(buf, "type", param, sizeof(param)) == ESP_OK) {
                
                // --- 入口逻辑 ---
                if (strcmp(param, "enter") == 0) {
                    Serial.println("Command: Entrance Open");
                    // 立即执行动作
                    ledcWrite(SERVO_CH_ENTRANCE, DUTY_GATE_OPEN);
                    update_oled_status("Welcome!\nFind a slot.");
                    // 设置计时器，不使用 delay
                    is_entrance_open = true;
                    entrance_timer_start = millis();
                } 
                
                // --- 出口逻辑 ---
                else if (strcmp(param, "exit") == 0) {
                    Serial.println("Command: Exit Open");
                    ledcWrite(SERVO_CH_EXIT, DUTY_GATE_OPEN);
                    update_oled_status("Goodbye!\nSee you.");
                    is_exit_open = true;
                    exit_timer_start = millis();
                }
            }
        }
        free(buf);
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// 启动 Web Server
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };
    
    httpd_uri_t action_uri = {
        .uri       = "/action",
        .method    = HTTP_GET,
        .handler   = action_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &action_uri);
        Serial.println("Camera Server Ready");
    }
}

// ==========================================
// 5. Setup 初始化
// ==========================================
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    // --- 1. 初始化 OLED ---
    // 关键：在 display.begin 之前显式启动 Wire，指定 32/33 引脚
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 allocation failed");
        // 这里不要死循环，否则摄像头也起不来，这只是屏幕坏了而已
    } else {
        display.clearDisplay();
        update_oled_status("System Booting...");
    }

    // --- 2. 初始化舵机 PWM ---
    // 使用 ledcSetup (ESP32 Arduino v2.x API)
    ledcSetup(SERVO_CH_ENTRANCE, SERVO_FREQ, SERVO_RES); // Timer 1 自动分配
    ledcSetup(SERVO_CH_EXIT, SERVO_FREQ, SERVO_RES);
    
    ledcAttachPin(SERVO_ENTRANCE_PIN, SERVO_CH_ENTRANCE);
    ledcAttachPin(SERVO_EXIT_PIN, SERVO_CH_EXIT);
    
    // 初始位置：关闭
    ledcWrite(SERVO_CH_ENTRANCE, DUTY_GATE_CLOSE);
    ledcWrite(SERVO_CH_EXIT, DUTY_GATE_CLOSE);

    // --- 3. 初始化 LED ---
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // 假设高电平灭，低电平亮，视板子而定

    // --- 4. 初始化摄像头 ---
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; // ⚠️ 摄像头必须用 Channel 0 / Timer 0
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
    config.xclk_freq_hz = 10000000; // 10MHz XCLK for OV7670
    config.pixel_format = PIXFORMAT_RGB565; // OV7670 不支持 JPEG
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12; 
    config.fb_count = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera init failed!");
        update_oled_status("Cam Error!");
        return;
    }

    // --- 5. WiFi 连接 ---
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");
    
    // 显示 IP 到屏幕，方便调试
    String ip_str = "IP: " + WiFi.localIP().toString();
    update_oled_status(ip_str.c_str());

    // --- 6. 启动服务器 ---
    startCameraServer();
}

// ==========================================
// 6. 主循环 (Non-blocking Logic)
// ==========================================
void loop() {
    unsigned long current_time = millis();

    // 检查入口闸机是否需要关闭
    if (is_entrance_open) {
        if (current_time - entrance_timer_start > GATE_OPEN_DURATION) {
            Serial.println("Timer: Closing Entrance Gate");
            ledcWrite(SERVO_CH_ENTRANCE, DUTY_GATE_CLOSE);
            update_oled_status("IP: " + WiFi.localIP().toString()); // 恢复显示 IP
            is_entrance_open = false;
        }
    }

    // 检查出口闸机是否需要关闭
    if (is_exit_open) {
        if (current_time - exit_timer_start > GATE_OPEN_DURATION) {
            Serial.println("Timer: Closing Exit Gate");
            ledcWrite(SERVO_CH_EXIT, DUTY_GATE_CLOSE);
            update_oled_status("IP: " + WiFi.localIP().toString()); // 恢复显示 IP
            is_exit_open = false;
        }
    }
    
    // 小延迟让 CPU 喘口气
    delay(20); 
}
