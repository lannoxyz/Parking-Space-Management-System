#include <Arduino.h>
#include <WiFi.h>
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "driver/i2s.h"
#include "esp_http_server.h"
#include "Wire.h"

// ================= WIFI =================
const char* ssid = "Gin Wu";
const char* password = "1122wweegoh";

// ================ CAMERA PINS ================
#define SIOD 21
#define SIOC 22

#define VSYNC 27
#define HREF  14
#define PCLK  26
#define XCLK  25

#define D0 32
#define D1 33
#define D2 16
#define D3 17
#define D4 5
#define D5 4
#define D6 18
#define D7 13

// =============== IMAGE SETTINGS ===============
#define FRAME_WIDTH  160
#define FRAME_HEIGHT 120

#define I2S_PORT I2S_NUM_0

uint8_t frameBuffer[FRAME_WIDTH * FRAME_HEIGHT * 2];

// =================== OV7670 REGISTER WRITE ===================
void ov7670_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x21);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// =================== CAMERA INIT ===================
void setupOV7670() {
  Wire.begin(SIOD, SIOC);

  delay(100);

  ov7670_write(0x12, 0x80); // reset
  delay(100);

  ov7670_write(0x12, 0x14); // QVGA RGB565
  ov7670_write(0x40, 0xD0); // RGB565
  ov7670_write(0x0C, 0x00);

  Serial.println("OV7670 Initialized");
}

// =================== I2S CAMERA INIT ===================
void setupI2SCamera() {

ledcAttach(XCLK, 20000000, 1);

  i2s_config_t config;
  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  config.sample_rate = 20000000;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
  config.communication_format = I2S_COMM_FORMAT_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 4;
  config.dma_buf_len = 1024;
  config.use_apll = false;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = 0;

  i2s_driver_install(I2S_PORT, &config, 0, NULL);

  i2s_set_pin(I2S_PORT, NULL);
}

// =================== FRAME CAPTURE ===================
void captureFrame() {
  size_t bytes_read;
  i2s_read(I2S_PORT, frameBuffer, sizeof(frameBuffer), &bytes_read, portMAX_DELAY);
}

// =================== STREAM HANDLER ===================
static esp_err_t stream_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

  while (true) {
    captureFrame();

    httpd_resp_send_chunk(req, "--frame\r\n", 9);
    httpd_resp_send_chunk(req, "Content-Type: image/bmp\r\n\r\n", 28);
    httpd_resp_send_chunk(req, (const char*)frameBuffer, sizeof(frameBuffer));
    httpd_resp_send_chunk(req, "\r\n", 2);
  }
  return ESP_OK;
}

// =================== START SERVER ===================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t server = NULL;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t uri_get = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_get);
  }
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  setupOV7670();
  setupI2SCamera();
  startCameraServer();
}

// =================== LOOP ===================
void loop() {
}
