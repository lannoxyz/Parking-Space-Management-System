#include <WiFi.h>
#include "esp_camera.h"

// =============================
// WiFi 设置
// =============================
const char* ssid     = "Shao Fan's S23 Ultra";
const char* password = "my9nd3k3zn4d9rz";

const char* server_ip = "10.143.39.152";   // 你的 PC
const int   server_port = 5001;          // cam.py 的 CAM1 端口

WiFiUDP udp;

// =============================
// CAM1 配置 (320x240 YUV422)
// =============================
#define CAM_ID 1
#define WIDTH  320
#define HEIGHT 240
#define FRAME_BYTES (WIDTH * HEIGHT * 2)
#define CHUNK_SIZE 1400
#define TOTAL_CHUNKS ((FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE)

static uint16_t frame_seq = 0;

// =============================
// 相机 PIN（你 cam1 的版本）
// =============================
camera_config_t config = {
.pin_pwdn  = -1,
.pin_reset = -1,
.pin_xclk = 10,
.pin_sscb_sda = 42,
.pin_sscb_scl = 41,

.pin_d7 = 18,
.pin_d6 = 17,
.pin_d5 = 16,
.pin_d4 = 15,
.pin_d3 = 7,
.pin_d2 = 6,
.pin_d1 = 5,
.pin_d0 = 4,

.pin_vsync = 13,
.pin_href  = 12,
.pin_pclk  = 11,

.xclk_freq_hz = 20000000,
.ledc_timer = LEDC_TIMER_0,
.ledc_channel = LEDC_CHANNEL_0,

.pixel_format = PIXFORMAT_YUV422,
.frame_size = FRAMESIZE_QVGA,
.jpeg_quality = 15,
.fb_count = 1
};

void setup() {
Serial.begin(115200);
delay(2000);

WiFi.begin(ssid, password);
Serial.println("Connecting...");
while (WiFi.status() != WL_CONNECTED) delay(100);

Serial.print("Connected: ");
Serial.println(WiFi.localIP());

if (esp_camera_init(&config) != ESP_OK) {
Serial.println("Camera init failed");
while (1);
}

Serial.println("CAM1 Ready.");
}

void loop() {
// 获取一帧 YUV422 图像
camera_fb_t* fb = esp_camera_fb_get();
if (!fb || fb->len != FRAME_BYTES) {
Serial.println("Bad frame!");
if (fb) esp_camera_fb_return(fb);
return;
}

uint8_t* buf = fb->buf;
uint16_t seq = frame_seq++;

// 分片发送
for (uint16_t idx = 0; idx < TOTAL_CHUNKS; idx++) {

int offset = idx * CHUNK_SIZE;
int remaining = FRAME_BYTES - offset;
int payload_len = remaining > CHUNK_SIZE ? CHUNK_SIZE : remaining;

uint8_t header[7];
header[0] = CAM_ID;
header[1] = (seq >> 8) & 0xFF;
header[2] = (seq     ) & 0xFF;
header[3] = (idx >> 8) & 0xFF;
header[4] = (idx     ) & 0xFF;
header[5] = (TOTAL_CHUNKS >> 8) & 0xFF;
header[6] = (TOTAL_CHUNKS     ) & 0xFF;

udp.beginPacket(server_ip, server_port);
udp.write(header, 7);
udp.write(buf + offset, payload_len);
udp.endPacket();

}

esp_camera_fb_return(fb);
}
