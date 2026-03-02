#include <Wire.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "driver/ledc.h"
#include "driver/i2s.h"
#include "secrets.h"
#include "ov7670_regs.h"

// ==========================================
// Pin Definitions
// ==========================================
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22
#define CAM_PIN_XCLK    21
#define CAM_PIN_D0       5
#define CAM_PIN_D1      18
#define CAM_PIN_D2      19
#define CAM_PIN_D3      34
#define CAM_PIN_D4      33
#define CAM_PIN_D5      17
#define CAM_PIN_D6      35
#define CAM_PIN_D7      32

// Frame buffer — QCIF RGB565 = 176*144*2 bytes
#define FRAME_W   176
#define FRAME_H   144
#define FB_SIZE   (FRAME_W * FRAME_H * 2)

static uint8_t framebuf[FB_SIZE];
static bool is_camera_ready = false;
httpd_handle_t camera_http_server = NULL;

// ==========================================
// XCLK Generation via LEDC
// ==========================================
void startXCLK() {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_2_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 10000000,  // 10MHz
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = CAM_PIN_XCLK,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 2,  // 50% duty on 2-bit = value 2
        .hpoint     = 0
    };
    ledc_channel_config(&channel);
}

// ==========================================
// SCCB (I2C) Write
// ==========================================
bool sccbWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(OV7670_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

uint8_t sccbRead(uint8_t reg) {
    Wire.beginTransmission(OV7670_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)OV7670_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ==========================================
// OV7670 Init — write register table
// ==========================================
bool initOV7670() {
    Wire.begin(CAM_PIN_SIOD, CAM_PIN_SIOC);
    delay(100);

    // Check sensor is responding
    uint8_t pid = sccbRead(0x0A);  // PID register
    uint8_t ver = sccbRead(0x0B);  // VER register
    Serial.printf("OV7670 PID: 0x%02X  VER: 0x%02X\n", pid, ver);
    if (pid != 0x76) {
        Serial.println("OV7670 not found! Check wiring.");
        return false;
    }

    for (int i = 0; ; i++) {
        uint8_t reg = OV7670_QCIF_RGB565[i][0];
        uint8_t val = OV7670_QCIF_RGB565[i][1];
        if (reg == 0xFF && val == 0xFF) break;   // End
        if (reg == 0xFF && val == 0x00) { delay(100); continue; }  // Delay
        if (!sccbWrite(reg, val)) {
            Serial.printf("SCCB write failed at reg 0x%02X\n", reg);
            return false;
        }
        delay(1);
    }
    Serial.println("OV7670 registers loaded.");
    return true;
}

// ==========================================
// I2S Parallel Init for pixel capture
// ==========================================
void startI2SCamera() {
    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate          = 10000000,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 4,
        .dma_buf_len          = 1024,
        .use_apll             = true,
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    // Map data pins to I2S via GPIO matrix
    i2s_pin_config_t pin_config = {
        .ws_io_num   = CAM_PIN_HREF,
        .bck_io_num  = CAM_PIN_PCLK,
        .data_in_num = CAM_PIN_D0,  // I2S reads D0; D1–D7 mapped below
    };
    i2s_set_pin(I2S_NUM_0, &pin_config);

    // Map remaining data pins via GPIO matrix to I2S input signals
    gpio_matrix_in(CAM_PIN_D0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(CAM_PIN_D1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(CAM_PIN_D2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(CAM_PIN_D3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(CAM_PIN_D4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(CAM_PIN_D5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(CAM_PIN_D6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(CAM_PIN_D7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(CAM_PIN_VSYNC, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(CAM_PIN_HREF,  I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(CAM_PIN_PCLK,  I2S0I_WS_IN_IDX,  false);
}

// ==========================================
// Capture one frame into framebuf
// ==========================================
bool captureFrame() {
    // Wait for VSYNC high (start of frame)
    uint32_t timeout = millis() + 2000;
    while (digitalRead(CAM_PIN_VSYNC) == 0) {
        if (millis() > timeout) { Serial.println("VSYNC timeout"); return false; }
    }
    while (digitalRead(CAM_PIN_VSYNC) == 1) {
        if (millis() > timeout) { Serial.println("VSYNC timeout"); return false; }
    }

    size_t bytes_read = 0;
    size_t total = 0;
    i2s_zero_dma_buffer(I2S_NUM_0);

    while (total < FB_SIZE) {
        esp_err_t ret = i2s_read(I2S_NUM_0,
                                  framebuf + total,
                                  FB_SIZE - total,
                                  &bytes_read,
                                  portMAX_DELAY);
        if (ret != ESP_OK || bytes_read == 0) break;
        total += bytes_read;
    }

    return total == FB_SIZE;
}

// ==========================================
// HTTP Handlers
// ==========================================
static esp_err_t capture_handler(httpd_req_t *req) {
    if (!is_camera_ready) { httpd_resp_send_500(req); return ESP_FAIL; }

    if (!captureFrame()) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    uint8_t* jpg_buf = NULL;
    size_t   jpg_len = 0;

    bool ok = fmt2jpg(framebuf, FB_SIZE, FRAME_W, FRAME_H, PIXFORMAT_RGB565, 40, &jpg_buf, &jpg_len);
    if (!ok) { httpd_resp_send_500(req); return ESP_FAIL; }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char*)jpg_buf, jpg_len);
    free(jpg_buf);
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req) {
    const char* html =
    "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>Exit Camera</title></head><body style='text-align:center; padding:20px;'>"
    "<h1>📷 Exit Camera</h1>"
    "<img id='img' src='/capture' style='max-width:100%'><br><br>"
    "<button onclick=\"document.getElementById('img').src='/capture?t='+Date.now()\">Refresh</button>"
    "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_uri_t index_uri   = { .uri="/",        .method=HTTP_GET, .handler=index_handler };
    httpd_uri_t capture_uri = { .uri="/capture", .method=HTTP_GET, .handler=capture_handler };
    if (httpd_start(&camera_http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_http_server, &index_uri);
        httpd_register_uri_handler(camera_http_server, &capture_uri);
        Serial.println("HTTP server started");
    }
}

// ==========================================
// Setup
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(200);

    startXCLK();
    delay(100);  // Let XCLK stabilise before talking to sensor

    if (!initOV7670()) {
        Serial.println("Camera init failed — halting");
        while (true) delay(1000);
    }

    startI2SCamera();
    is_camera_ready = true;
    Serial.println("Camera Ready!");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500); Serial.print("."); retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
        startCameraServer();
    } else {
        Serial.println("\nWiFi failed. Restarting...");
        delay(3000);
        ESP.restart();
    }
}

// ==========================================
// Loop
// ==========================================
void loop() {
    delay(20);
}
