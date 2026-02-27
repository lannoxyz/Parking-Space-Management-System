#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==========================================
// 1. 硬件引脚与常量配置 (Hardware Configurations)
// ==========================================

// --- WiFi 配置 ---
const char* wifi_ssid = "your_wifi_ssid";
const char* wifi_password = "your_wifi_password";

// --- OLED 屏幕引脚 (I2C) ---
const int oled_screen_width = 128;
const int oled_screen_height = 64;
const int oled_reset_pin = -1;
const int oled_i2c_addr = 0x3C;
const int oled_sda_pin = 1;  
const int oled_scl_pin = 2;

// --- 舵机引脚与 PWM 参数 ---
const int servo_entrance_pin = 21;
const int servo_exit_pin = 47; 

const int servo_timer_num = 1; 
const int servo_ch_entrance = 2;
const int servo_ch_exit = 3;
const int servo_pwm_freq = 50; 
const int servo_pwm_res = 16; 

// 16位精度下的占空比 (50Hz 控制舵机角度)
const int duty_gate_close = 1638; 
const int duty_gate_open = 4915;

// --- 摄像头引脚 (OV7670 / ESP32-S3) ---
const int cam_pwdn_pin = -1;
const int cam_reset_pin = -1;
const int cam_xclk_pin = 10;
const int cam_siod_pin = 42;
const int cam_sioc_pin = 41;
const int cam_y9_pin = 18;
const int cam_y8_pin = 17;
const int cam_y7_pin = 16;
const int cam_y6_pin = 15;
const int cam_y5_pin = 7;
const int cam_y4_pin = 6;
const int cam_y3_pin = 5;
const int cam_y2_pin = 4;
const int cam_vsync_pin = 12;
const int cam_href_pin = 13;
const int cam_pclk_pin = 11;

const int built_in_led_pin = 38;

// ==========================================
// 2. 全局对象与状态变量 (Globals & States)
// ==========================================

Adafruit_SSD1306 oled_display(oled_screen_width, oled_screen_height, &Wire, oled_reset_pin);
httpd_handle_t camera_http_server = NULL;

// 硬件健康状态标志位
bool is_camera_ready = false;
bool is_oled_ready = false;

// 状态机变量 (用于非阻塞定时关闭闸机)
bool is_entrance_open = false;
unsigned long entrance_timer_start = 0;
const unsigned long gate_open_duration = 5000; // 闸机开启时长 5000ms (5秒)

bool is_exit_open = false;
unsigned long exit_timer_start = 0;

// ==========================================
// 3. 辅助函数 (Helper Functions)
// ==========================================

// 封装的 OLED 更新函数
void update_oled_status(const char* status_text) {
    if (!is_oled_ready) return; // 如果屏幕损坏或未连接，直接跳过，防止死机
    oled_display.clearDisplay();
    oled_display.setTextSize(2);
    oled_display.setTextColor(SSD1306_WHITE);
    oled_display.setCursor(0, 0);
    oled_display.println("UNM Parking");
    oled_display.setTextSize(1);
    oled_display.setCursor(0, 30);
    oled_display.println(status_text);
    oled_display.display();
}

// ==========================================
// 4. 网络请求处理函数 (HTTP Handlers)
// ==========================================

// 处理拍照请求 (GET /capture)
static esp_err_t capture_handler(httpd_req_t *req) {
    if (!is_camera_ready) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 1. 获取帧缓冲
    camera_fb_t * frame_buffer = esp_camera_fb_get();
    if (!frame_buffer) {
        Serial.println("Error: Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 2. 软件转码为 JPEG (OV7670 必须经过此步)
    uint8_t * jpeg_buffer = NULL;
    size_t jpeg_length = 0;
    bool is_converted = fmt2jpg(frame_buffer->buf, frame_buffer->len, frame_buffer->width, frame_buffer->height, frame_buffer->format, 60, &jpeg_buffer, &jpeg_length);

    // 3. 立即释放底层摄像头缓冲，防止内存泄漏
    esp_camera_fb_return(frame_buffer);

    if (!is_converted) {
        Serial.println("Error: JPEG compression failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 4. 发送图像数据给 PC
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_send(req, (const char *)jpeg_buffer, jpeg_length);
    
    // 5. 释放转码缓冲
    free(jpeg_buffer);
    return ESP_OK;
}

// 处理控制指令 (GET /action?type=enter 或 ?type=exit)
static esp_err_t action_handler(httpd_req_t *req) {
    char* query_buffer;
    size_t query_length;
    char param_value[32];
    
    query_length = httpd_req_get_url_query_len(req) + 1;
    if (query_length > 1) {
        query_buffer = (char*)malloc(query_length);
        if (httpd_req_get_url_query_str(req, query_buffer, query_length) == ESP_OK) {
            
            // 解析 type 参数
            if (httpd_query_key_value(query_buffer, "type", param_value, sizeof(param_value)) == ESP_OK) {
                
                // --- 入口开启逻辑 ---
                if (strcmp(param_value, "enter") == 0) {
                    Serial.println("Action: Entrance Open Triggered");
                    ledcWrite(servo_ch_entrance, duty_gate_open);
                    update_oled_status("Welcome to UNM!\nFind a slot.");
                    is_entrance_open = true;
                    entrance_timer_start = millis(); // 记录开启时间戳
                } 
                
                // --- 出口开启逻辑 ---
                else if (strcmp(param_value, "exit") == 0) {
                    Serial.println("Action: Exit Open Triggered");
                    ledcWrite(servo_ch_exit, duty_gate_open);
                    update_oled_status("Goodbye!\nDrive safely.");
                    is_exit_open = true;
                    exit_timer_start = millis();
                }
            }
        }
        free(query_buffer);
    }
    httpd_resp_send(req, "Action Executed", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ==========================================
// 5. 系统初始化 (Setup)
// ==========================================
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println("\n--- UNM Parking Edge Node Booting ---");

    // --- 1. 初始化 OLED ---
    Wire.begin(oled_sda_pin, oled_scl_pin);
    if (!oled_display.begin(SSD1306_SWITCHCAPVCC, oled_i2c_addr)) {
        Serial.println("Warning: OLED init failed, continuing without display.");
        is_oled_ready = false;
    } else {
        is_oled_ready = true;
        oled_display.clearDisplay();
        update_oled_status("Booting up...");
    }

    // --- 2. 初始化舵机 ---
    ledcSetup(servo_ch_entrance, servo_pwm_freq, servo_pwm_res);
    ledcSetup(servo_ch_exit, servo_pwm_freq, servo_pwm_res);
    ledcAttachPin(servo_entrance_pin, servo_ch_entrance);
    ledcAttachPin(servo_exit_pin, servo_ch_exit);
    
    // 强制舵机归位关闭
    ledcWrite(servo_ch_entrance, duty_gate_close);
    ledcWrite(servo_ch_exit, duty_gate_close);

    // --- 3. 初始化摄像头 ---
    camera_config_t cam_config;
    cam_config.ledc_channel = LEDC_CHANNEL_0; 
    cam_config.ledc_timer = LEDC_TIMER_0;
    cam_config.pin_d0 = cam_y2_pin;
    cam_config.pin_d1 = cam_y3_pin;
    cam_config.pin_d2 = cam_y4_pin;
    cam_config.pin_d3 = cam_y5_pin;
    cam_config.pin_d4 = cam_y6_pin;
    cam_config.pin_d5 = cam_y7_pin;
    cam_config.pin_d6 = cam_y8_pin;
    cam_config.pin_d7 = cam_y9_pin;
    cam_config.pin_xclk = cam_xclk_pin;
    cam_config.pin_pclk = cam_pclk_pin;
    cam_config.pin_vsync = cam_vsync_pin;
    cam_config.pin_href = cam_href_pin;
    cam_config.pin_sccb_sda = cam_siod_pin;
    cam_config.pin_sccb_scl = cam_sioc_pin;
    cam_config.pin_pwdn = cam_pwdn_pin;
    cam_config.pin_reset = cam_reset_pin;
    cam_config.xclk_freq_hz = 10000000; 
    cam_config.pixel_format = PIXFORMAT_RGB565; // OV7670 专用
    cam_config.frame_size = FRAMESIZE_QVGA;
    cam_config.jpeg_quality = 12; 
    cam_config.fb_count = 1;

    if (esp_camera_init(&cam_config) != ESP_OK) {
        Serial.println("Critical Warning: Camera init failed! System will run as dummy actuator.");
        update_oled_status("Cam Error!\nCheck Wiring");
        is_camera_ready = false;
        // ⚠️ 取消了此处的 return，确保系统就算瞎了也能连网并驱动舵机
    } else {
        is_camera_ready = true;
        Serial.println("Camera initialized successfully.");
    }

    // --- 4. 连接 WiFi ---
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    
    String ip_address_str = "IP: " + WiFi.localIP().toString();
    update_oled_status(ip_address_str.c_str());
    Serial.println(ip_address_str);

    // --- 5. 启动 HTTP 服务器 ---
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.server_port = 80;

    httpd_uri_t capture_uri_config = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };
    
    httpd_uri_t action_uri_config = {
        .uri       = "/action",
        .method    = HTTP_GET,
        .handler   = action_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&camera_http_server, &server_config) == ESP_OK) {
        httpd_register_uri_handler(camera_http_server, &capture_uri_config);
        httpd_register_uri_handler(camera_http_server, &action_uri_config);
        Serial.println("Web server started successfully.");
    }
}

// ==========================================
// 6. 主循环 (Main Loop)
// ==========================================
void loop() {
    unsigned long current_time = millis();

    // 检查入口闸机是否需要关闭
    if (is_entrance_open) {
        if (current_time - entrance_timer_start > gate_open_duration) {
            Serial.println("Timer Event: Closing Entrance Gate");
            ledcWrite(servo_ch_entrance, duty_gate_close);
            update_oled_status("IP: " + WiFi.localIP().toString()); 
            is_entrance_open = false;
        }
    }

    // 检查出口闸机是否需要关闭
    if (is_exit_open) {
        if (current_time - exit_timer_start > gate_open_duration) {
            Serial.println("Timer Event: Closing Exit Gate");
            ledcWrite(servo_ch_exit, duty_gate_close);
            update_oled_status("IP: " + WiFi.localIP().toString()); 
            is_exit_open = false;
        }
    }
    
    // 让出一点时间片给 FreeRTOS 调度网络任务
    delay(20); 
}
