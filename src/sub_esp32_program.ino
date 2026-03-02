// the sub esp32 have the list of components connected below:
// 5 x car detection coils
// 1 x exit side camera

#include "esp_camera.h"
#include <WiFi.h>

// =============================
// WiFi
// =============================
const char* ssid = "Shao Fan's S23 Ultra";
const char* password = "my9nd3k3zn4d9rz";

// =============================
// Camera Pins (你的 WROOM)
// =============================
#define CAM_PIN_SIOD 21
#define CAM_PIN_SIOC 22

#define CAM_PIN_VSYNC 27
#define CAM_PIN_HREF 14
#define CAM_PIN_PCLK 26
#define CAM_PIN_XCLK 25

#define CAM_PIN_D0 32
#define CAM_PIN_D1 33
#define CAM_PIN_D2 16
#define CAM_PIN_D3 17
#define CAM_PIN_D4 5
#define CAM_PIN_D5 4
#define CAM_PIN_D6 18
#define CAM_PIN_D7 13

// =============================
// MJPEG Stream Server
// =============================
#include "esp_http_server.h"

httpd_handle_t stream_httpd = NULL;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=frame";
static const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t jpg_len = 0;
    uint8_t * jpg_buf = NULL;

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera frame failed");
            return ESP_FAIL;
        }

        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 40, &jpg_buf, &jpg_len);
            esp_camera_fb_return(fb);
            if(!jpeg_converted){
                Serial.println("JPEG compression failed");
                continue;
            }
        } else {
            jpg_buf = fb->buf;
            jpg_len = fb->len;
        }

        httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        httpd_resp_send_chunk(req, STREAM_PART, strlen(STREAM_PART));
        httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);

        if(fb->format != PIXFORMAT_JPEG){
            free(jpg_buf);
        }

        esp_camera_fb_return(fb);
    }

    return res;
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;   // distinguish from main esp32

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&stream_httpd, &config) == ESP_OK){
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("MJPEG Stream Ready!");
    }
}

void setup(){
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    delay(300);

    // =============================
    // Camera Config
    // =============================
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;

    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;

    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;

    config.pin_pwdn = -1;
    config.pin_reset = -1;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;  // we need JPEG

    config.frame_size = FRAMESIZE_QVGA;    // 320x240 (smooth)
    config.jpeg_quality = 12;
    config.fb_count = 2;

    if(esp_camera_init(&config) != ESP_OK){
        Serial.println("Camera Init Failed!");
        return;
    }

    // =============================
    // WiFi
    // =============================
    WiFi.begin(ssid, password);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    startCameraServer();
}

void loop(){
    delay(10);
}