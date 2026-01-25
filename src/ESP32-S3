#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h" 

// WIFI
const char* ssid = "wifi";
const char* password = "wifi password";

// LED
#define LED_PIN 38 

// GPIO CAMERA (OV7670)
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

httpd_handle_t camera_httpd = NULL;

// ------------------------------------------------
// 1. PHOTO PROCESSING FUNCTION
// ------------------------------------------------
static esp_err_t capture_handler(httpd_req_t *req) {
    // 1. GET 1 PIC
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Photo failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 2. FORMAT CONVERT TO JPEG
    uint8_t * jpg_buf = NULL; // STORE CONVERTED JPG DATA
    size_t jpg_len = 0; // JPG DATA SIZE
    bool converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format, 80, &jpg_buf, &jpg_len);

    // 3. RELEASE
    esp_camera_fb_return(fb);

    if (!converted) {
        Serial.println("Transcoding failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 4. SEND PIC TO COMPUTER
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
    
    free(jpg_buf);
    return ESP_OK;
}

// ------------------------------------------------
// 2. ACTION CONTROL FUNCTION
// ------------------------------------------------
static esp_err_t action_handler(httpd_req_t *req) {
    char* buf;
    size_t buf_len;
    char param[32];
    
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "val", param, sizeof(param)) == ESP_OK) {
                int val = atoi(param);

                // LED CONTROL
                if(val == 1) {
                    digitalWrite(LED_PIN, HIGH);
                    Serial.println("CAR");
                } else {
                    digitalWrite(LED_PIN, LOW);
                    Serial.println("NO CAR");
                }
            }
        }
        free(buf);
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// REGISTER SERVER
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
        Serial.println("Web server started successfully");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    pinMode(LED_PIN, OUTPUT);

    for(int i=0; i<3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(200);
        digitalWrite(LED_PIN, LOW);  delay(200);
    }

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
    config.xclk_freq_hz = 10000000; 
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QVGA;   
    config.jpeg_quality = 12; 
    config.fb_count = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera initialization failed");
        return;
    }

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connection successful");
    Serial.println("=================================");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); 
    Serial.println("=================================");
    
    startCameraServer();
}

void loop() {
    delay(1000);
}
