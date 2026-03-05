/*  sub_esp.ino  — Control ESP
 *  Hardware:
 *    OLED  : SDA=22, SCL=23  (I2C)
 *    Servo : GPIO 21          (SG90, single gate)
 *    Park  : GPIO 16,17,18,19 (square-wave input sensors)
 *
 *  Parking logic (smart delta detection):
 *    - Every FREQ_SAMPLE_MS ms, count rising edges → instant Hz.
 *    - Maintain a rolling baseline (updates only when delta < FREQ_THRESHOLD).
 *    - If |delta| >= 100 Hz:
 *        delta > 0  → slot becomes Occupied
 *        delta < 0  → slot becomes Vacant
 *    - If |delta| < 100 Hz → no state change, baseline drifts toward current.
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_http_server.h"
#include <ArduinoJson.h>
#include <time.h>

// ---- WiFi ----
#define WIFI_SSID     "Lanno"
#define WIFI_PASSWORD "lannoxyz"
#define NTP_SERVER    "pool.ntp.org"
#define GMT_OFFSET    28800   // UTC+8
#define DST_OFFSET    0

// ---- OLED ----
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oled_ready = false;

// ---- Servo (single gate) ----
#define SERVO_PIN     21
#define SERVO_FREQ    50      // 50 Hz PWM
#define SERVO_RES     16      // 16-bit resolution
#define DUTY_CLOSE    1638    // ~0.5 ms  → 0°
#define DUTY_OPEN     4915    // ~1.5 ms  → 90°
#define GATE_TIMEOUT  5000    // ms gate stays open

bool         gate_open  = false;
unsigned long gate_timer = 0;

// ---- OLED override ----
String        oled_override = "";
unsigned long oled_until    = 0;

// ---- Parking sensors ----
#define PARK_COUNT      4
const int park_pins[PARK_COUNT] = {16, 17, 18, 19};

volatile unsigned long pulse_count[PARK_COUNT] = {0, 0, 0, 0};

#define FREQ_SAMPLE_MS  500       // sample window
#define FREQ_THRESHOLD  100.0f    // Hz — trigger threshold
#define BASELINE_ALPHA  0.15f     // EMA smoothing for baseline drift

float current_freq[PARK_COUNT]  = {0};
float baseline_freq[PARK_COUNT] = {-1};   // -1 = uninitialised
bool  slot_occupied[PARK_COUNT] = {false};

// ---- ISRs ----
void IRAM_ATTR isr0() { pulse_count[0]++; }
void IRAM_ATTR isr1() { pulse_count[1]++; }
void IRAM_ATTR isr2() { pulse_count[2]++; }
void IRAM_ATTR isr3() { pulse_count[3]++; }

httpd_handle_t http_server = NULL;

// ===========================================================
// Servo helpers
// ===========================================================
void servo_set(int duty) { ledcWrite(SERVO_PIN, duty); }

void open_gate() {
    if (gate_open) {
        // Already open — just extend the timer
        gate_timer = millis();
        return;
    }
    servo_set(DUTY_OPEN);
    gate_open  = true;
    gate_timer = millis();
    oled_override = "WELCOME";
    oled_until    = millis() + GATE_TIMEOUT + 500;
    Serial.println("[GATE] Opened");
}

void close_gate() {
    servo_set(DUTY_CLOSE);
    gate_open = false;
    Serial.println("[GATE] Closed");
}

// ===========================================================
// OLED helpers
// ===========================================================
void oled_show_time() {
    if (!oled_ready) return;
    struct tm ti;
    if (!getLocalTime(&ti)) return;
    char t[10], d[20];
    strftime(t, sizeof(t), "%H:%M:%S", &ti);
    strftime(d, sizeof(d), "%d %b %Y", &ti);

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(18, 2);
    display.println("UNMC PARKING");
    display.drawLine(0, 13, 127, 13, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 18);
    display.println(t);
    display.drawLine(0, 38, 127, 38, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(28, 44);
    display.println(d);

    // Slot occupancy row
    display.setTextSize(1);
    display.setCursor(0, 56);
    for (int i = 0; i < PARK_COUNT; i++) {
        display.print(slot_occupied[i] ? "X" : "O");
        display.print(" ");
    }
    display.display();
}

void oled_show_msg(const String &msg) {
    if (!oled_ready) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    int x = max(0, (int)(SCREEN_WIDTH - (int)msg.length() * 12) / 2);
    display.setCursor(x, 22);
    display.println(msg);
    display.display();
}

// ===========================================================
// HTTP handlers
// ===========================================================
static esp_err_t status_handler(httpd_req_t *req) {
    StaticJsonDocument<300> doc;

    JsonArray p = doc.createNestedArray("parking");
    for (int i = 0; i < PARK_COUNT; i++) p.add(slot_occupied[i]);

    // Single servo — return scalar string for simplicity
    doc["servo"] = gate_open ? "Open" : "Closed";

    JsonArray f = doc.createNestedArray("freq");
    for (int i = 0; i < PARK_COUNT; i++) f.add((int)current_freq[i]);

    char buf[300];
    serializeJson(doc, buf, sizeof(buf));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t action_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[16];
        if (httpd_query_key_value(buf, "servo", param, sizeof(param)) == ESP_OK) {
            // Accept "in" or "out" — single gate, same action
            if (!strcmp(param, "in") || !strcmp(param, "out")) {
                open_gate();
            }
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void start_server() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    httpd_uri_t su = {"/status", HTTP_GET, status_handler, NULL};
    httpd_uri_t au = {"/action", HTTP_GET, action_handler, NULL};
    if (httpd_start(&http_server, &cfg) == ESP_OK) {
        httpd_register_uri_handler(http_server, &su);
        httpd_register_uri_handler(http_server, &au);
        Serial.println("[HTTP] Server started");
    }
}

// ===========================================================
// setup
// ===========================================================
void setup() {
    Serial.begin(115200);

    // I2C: SDA=22, SCL=23
    Wire.begin(22, 23);
    Wire.setClock(400000);
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oled_ready = true;
        oled_show_msg("BOOT...");
        Serial.println("[OLED] Ready");
    } else {
        Serial.println("[OLED] Not found");
    }

    // Servo on GPIO21
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
    servo_set(DUTY_CLOSE);
    Serial.println("[SERVO] Initialised on GPIO21");

    // Parking sensor pins + interrupts
    for (int i = 0; i < PARK_COUNT; i++) {
        pinMode(park_pins[i], INPUT);
        baseline_freq[i] = -1;   // mark as uninitialised
    }
    attachInterrupt(digitalPinToInterrupt(park_pins[0]), isr0, RISING);
    attachInterrupt(digitalPinToInterrupt(park_pins[1]), isr1, RISING);
    attachInterrupt(digitalPinToInterrupt(park_pins[2]), isr2, RISING);
    attachInterrupt(digitalPinToInterrupt(park_pins[3]), isr3, RISING);
    Serial.println("[PARK] ISRs attached on GPIO 16,17,18,19");

    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connecting");
    int r = 0;
    while (WiFi.status() != WL_CONNECTED && r++ < 40) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
        configTime(GMT_OFFSET, DST_OFFSET, NTP_SERVER);
        start_server();
        oled_show_msg("READY");
    } else {
        Serial.println("\n[WiFi] Failed");
        oled_show_msg("NO WIFI");
    }
}

// ===========================================================
// loop
// ===========================================================
void loop() {
    unsigned long now = millis();

    // ---- Frequency sampling (every FREQ_SAMPLE_MS) ----
    static unsigned long last_sample = 0;
    if (now - last_sample >= FREQ_SAMPLE_MS) {
        float elapsed_s = (now - last_sample) / 1000.0f;
        last_sample = now;

        for (int i = 0; i < PARK_COUNT; i++) {
            // Atomically snapshot + clear counter
            noInterrupts();
            unsigned long cnt = pulse_count[i];
            pulse_count[i] = 0;
            interrupts();

            current_freq[i] = cnt / elapsed_s;

            // First sample → set baseline, skip decision
            if (baseline_freq[i] < 0) {
                baseline_freq[i] = current_freq[i];
                Serial.printf("[P%d] Baseline init: %.1f Hz\n", i+1, baseline_freq[i]);
                continue;
            }

            float delta = current_freq[i] - baseline_freq[i];

            if (fabsf(delta) >= FREQ_THRESHOLD) {
                // Large change → update occupancy
                if (delta > 0 && !slot_occupied[i]) {
                    slot_occupied[i] = true;
                    Serial.printf("[P%d] OCCUPIED  (freq=%.1f base=%.1f Δ=+%.1f)\n",
                                  i+1, current_freq[i], baseline_freq[i], delta);
                } else if (delta < 0 && slot_occupied[i]) {
                    slot_occupied[i] = false;
                    Serial.printf("[P%d] VACANT    (freq=%.1f base=%.1f Δ=%.1f)\n",
                                  i+1, current_freq[i], baseline_freq[i], delta);
                }
                // Reset baseline to new stable level
                baseline_freq[i] = current_freq[i];
            } else {
                // Small change → let baseline drift slowly (EMA)
                baseline_freq[i] += BASELINE_ALPHA * delta;
            }
        }
    }

    // ---- Gate auto-close ----
    if (gate_open && (now - gate_timer > GATE_TIMEOUT)) {
        close_gate();
    }

    // ---- OLED refresh (every 1 s) ----
    static unsigned long last_oled = 0;
    if (now - last_oled >= 1000) {
        last_oled = now;
        if (now < oled_until) {
            oled_show_msg(oled_override);
        } else {
            oled_override = "";
            oled_show_time();
        }
    }

    delay(10);
}
