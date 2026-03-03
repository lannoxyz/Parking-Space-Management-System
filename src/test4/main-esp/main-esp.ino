/*
 * main_esp32_cam1.ino  –  Entrance Camera + Gate Controller (ESP32-S3)
 * =====================================================================
 * ROOT CAUSE FIXES:
 *
 *  FIX 1 — PID=0xFF  (SCCB never responds, black screen)
 *    CAUSE: GPIO 41 (SCL) = JTAG TDO, GPIO 42 (SDA) = JTAG TMS.
 *    The ESP32-S3 JTAG subsystem actively drives these pins as outputs
 *    → I2C can never pull the line LOW → no ACK ever received → 0xFF.
 *    FIX: call gpio_reset_pin() on both pins to release JTAG and hand
 *         them back to the GPIO matrix before Wire.begin().
 *
 *  FIX 2 — UDP chunk fails (random chunks, ~50% loss on sub-esp)
 *    CAUSE: sending all chunks back-to-back floods the ESP32 WiFi TX
 *    queue. yield() + delay(2) between EVERY chunk lets the WiFi task
 *    drain its queue between packets.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include "driver/gpio.h"    // ← for gpio_reset_pin()

// ─────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────
const char* ssid         = "Shao Fan's S23 Ultra";
const char* password     = "my9nd3k3zn4d9rz";
const char* pc_ip        = "10.143.39.152";   // ← updated from serial log
const int   pc_port_cam1 = 5001;
const int   cmd_port     = 6001;

WiFiUDP udpSend;
WiFiUDP udpCmd;

// ─────────────────────────────────────────────
//  Camera Pins  (OV7670 on ESP32-S3)
//  NOTE: GPIO 41 = JTAG TDO, GPIO 42 = JTAG TMS
//        → released via gpio_reset_pin() in setup()
// ─────────────────────────────────────────────
#define XCLK_PIN   10
#define SIOD_PIN   42    // SDA — also JTAG TMS, must be released first
#define SIOC_PIN   41    // SCL — also JTAG TDO, must be released first
#define VSYNC_PIN  12
#define HREF_PIN   13
#define PCLK_PIN   11

#define D2_PIN      4
#define D3_PIN      5
#define D4_PIN      6
#define D5_PIN      7
#define D6_PIN     15
#define D7_PIN     16
#define D8_PIN     17
#define D9_PIN     18

// ─────────────────────────────────────────────
//  Frame geometry (QVGA, YUV422)
// ─────────────────────────────────────────────
#define WIDTH        320
#define HEIGHT       240
#define FRAME_BYTES  (WIDTH * HEIGHT * 2)   // 153 600
#define CHUNK_SIZE   1400
#define HEADER_SIZE  7
#define CAM_ID       1

static uint8_t  frameBuffer[FRAME_BYTES];
static uint8_t  pktBuffer[HEADER_SIZE + CHUNK_SIZE];
static uint16_t frameSeq   = 0;
static const uint16_t TOTAL_CHUNKS = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

// ─────────────────────────────────────────────
//  Stats
// ─────────────────────────────────────────────
uint32_t frameCount    = 0;
uint32_t droppedFrames = 0;
uint32_t udpFailCount  = 0;
uint32_t lastFpsTime   = 0;
uint32_t lastFpsCount  = 0;
uint32_t lastHeapTime  = 0;

// ─────────────────────────────────────────────
//  Servo / OLED
// ─────────────────────────────────────────────
#define SERVO_ENTRANCE_PIN  21
#define SERVO_EXIT_PIN      47
Servo servoEntrance;
Servo servoExit;

#define OLED_SDA   1
#define OLED_SCL   2
#define OLED_ADDR  0x3C
Adafruit_SSD1306 display(128, 64, &Wire1, -1);

// ─────────────────────────────────────────────
//  Print helpers
// ─────────────────────────────────────────────
void printBanner() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════╗"));
    Serial.println(F("║    ESP32-S3  CAM-1  FIRMWARE          ║"));
    Serial.println(F("║    OV7670  |  QVGA  320x240  YUV422   ║"));
    Serial.println(F("╚═══════════════════════════════════════╝"));
    Serial.println();
}
void printSection(const char* t) {
    Serial.println(F("─────────────────────────────────────────"));
    Serial.println(t);
    Serial.println(F("─────────────────────────────────────────"));
}
void printSignalBar(int rssi) {
    const char *q, *b;
    if      (rssi >= -50) { q="Excellent"; b="[████████]"; }
    else if (rssi >= -60) { q="Good     "; b="[██████░░]"; }
    else if (rssi >= -70) { q="Fair     "; b="[████░░░░]"; }
    else if (rssi >= -80) { q="Poor     "; b="[██░░░░░░]"; }
    else                  { q="Very Poor"; b="[█░░░░░░░]"; }
    Serial.print(F("  Signal   : ")); Serial.print(rssi);
    Serial.print(F(" dBm  ")); Serial.print(b);
    Serial.print(F("  ")); Serial.println(q);
}

// ─────────────────────────────────────────────
//  XCLK  (1-bit → max 40MHz → 20MHz achievable)
// ─────────────────────────────────────────────
#define XCLK_CHANNEL  0
void startXCLK() {
    ledcSetup(XCLK_CHANNEL, 20000000, 1);
    ledcAttachPin(XCLK_PIN, XCLK_CHANNEL);
    ledcWrite(XCLK_CHANNEL, 1);
    Serial.println(F("  [OK] XCLK @ 20 MHz, 50% duty"));
}

// ─────────────────────────────────────────────
//  SCCB helpers
// ─────────────────────────────────────────────
#define OV7670_ADDR  0x21

bool sccbWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(OV7670_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
uint8_t sccbRead(uint8_t reg) {
    Wire.beginTransmission(OV7670_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)OV7670_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ─────────────────────────────────────────────
//  OV7670 init — QVGA (320×240) YUV422
// ─────────────────────────────────────────────
void initOV7670() {
    printSection("[ OV7670 Init ]");

    // Retry soft-reset up to 5 times
    bool resetOk = false;
    for (int i = 0; i < 5; i++) {
        if (sccbWrite(0x12, 0x80)) { resetOk = true; break; }
        Serial.print(F("  [WARN] SCCB soft-reset attempt ")); Serial.print(i+1); Serial.println(F(" failed, retrying..."));
        delay(100);
    }
    if (!resetOk) {
        Serial.println(F("  [FAIL] SCCB soft-reset failed after 5 attempts"));
        Serial.println(F("         gpio_reset_pin was called — if still failing, check:"));
        Serial.println(F("         - 4.7kΩ pull-up on SDA(42) to 3.3V"));
        Serial.println(F("         - 4.7kΩ pull-up on SCL(41) to 3.3V"));
        Serial.println(F("         - OV7670 VCC = 3.3V (NOT 5V)"));
        Serial.println(F("         - XCLK pin physically connected to OV7670"));
    }
    delay(200);

    uint8_t pid = sccbRead(0x0A);
    uint8_t ver = sccbRead(0x0B);
    Serial.print(F("  PID=0x")); Serial.print(pid, HEX);
    Serial.print(F("  VER=0x")); Serial.println(ver, HEX);
    if      (pid == 0x76) Serial.println(F("  [OK] OV7670 detected — JTAG conflict resolved"));
    else if (pid == 0xFF) Serial.println(F("  [FAIL] Still no response — add 4.7k pull-up resistors to 3.3V"));
    else                  Serial.print  (F("  [WARN] Unexpected PID"));

    // QVGA, YUV422 (YUYV)
    sccbWrite(0x12, 0x00);
    sccbWrite(0x0C, 0x00);
    sccbWrite(0x3E, 0x00);
    sccbWrite(0x70, 0x3A);
    sccbWrite(0x71, 0x35);
    sccbWrite(0x72, 0x11);  // H÷2, V÷2 from VGA → QVGA
    sccbWrite(0x73, 0xF0);
    sccbWrite(0xA2, 0x02);
    sccbWrite(0x15, 0x00);
    sccbWrite(0x17, 0x16);
    sccbWrite(0x18, 0x04);
    sccbWrite(0x19, 0x02);
    sccbWrite(0x1A, 0x7A);
    sccbWrite(0x32, 0x80);
    sccbWrite(0x03, 0x0A);
    sccbWrite(0x11, 0x00);
    sccbWrite(0x6B, 0x4A);
    sccbWrite(0x13, 0xE7);
    sccbWrite(0x0E, 0x61);
    sccbWrite(0x0F, 0x4B);
    sccbWrite(0x16, 0x02);
    sccbWrite(0x1E, 0x07);
    sccbWrite(0x21, 0x02);
    sccbWrite(0x22, 0x91);
    sccbWrite(0x29, 0x07);
    sccbWrite(0x33, 0x0B);
    sccbWrite(0x35, 0x0B);
    sccbWrite(0x37, 0x1D);
    sccbWrite(0x38, 0x71);
    sccbWrite(0x39, 0x2A);
    sccbWrite(0x3C, 0x78);
    sccbWrite(0x4D, 0x40);
    sccbWrite(0x4E, 0x20);
    sccbWrite(0x74, 0x10);
    sccbWrite(0x8D, 0x4F);
    sccbWrite(0x8E, 0x00); sccbWrite(0x8F, 0x00);
    sccbWrite(0x90, 0x00); sccbWrite(0x91, 0x00);
    sccbWrite(0x96, 0x00); sccbWrite(0x9A, 0x00);
    sccbWrite(0xB0, 0x84); sccbWrite(0xB1, 0x0C);
    sccbWrite(0xB2, 0x0E); sccbWrite(0xB3, 0x82);
    sccbWrite(0xB8, 0x0A);
    Serial.println(F("  [OK] OV7670 QVGA registers written"));
}

// ─────────────────────────────────────────────
//  Fast parallel pixel read (GPIO register)
// ─────────────────────────────────────────────
inline uint8_t readPixelFast() {
    uint32_t r = REG_READ(GPIO_IN_REG);
    return (uint8_t)(
        ((r >> D9_PIN) & 1) << 7 | ((r >> D8_PIN) & 1) << 6 |
        ((r >> D7_PIN) & 1) << 5 | ((r >> D6_PIN) & 1) << 4 |
        ((r >> D5_PIN) & 1) << 3 | ((r >> D4_PIN) & 1) << 2 |
        ((r >> D3_PIN) & 1) << 1 | ((r >> D2_PIN) & 1)
    );
}

// ─────────────────────────────────────────────
//  Capture one frame
// ─────────────────────────────────────────────
bool captureFrame() {
    uint32_t idx = 0, t;

    t = millis();
    while (!digitalRead(VSYNC_PIN)) {
        if (millis() - t > 500) {
            Serial.println(F("  [WARN] VSYNC rising timeout"));
            droppedFrames++; return false;
        }
    }
    t = millis();
    while (digitalRead(VSYNC_PIN)) {
        if (millis() - t > 500) {
            Serial.println(F("  [WARN] VSYNC falling timeout"));
            droppedFrames++; return false;
        }
    }
    while (idx < FRAME_BYTES) {
        t = millis();
        while (!digitalRead(HREF_PIN)) {
            if (digitalRead(VSYNC_PIN)) return (idx > 0);
            if (millis() - t > 100) { droppedFrames++; return false; }
        }
        while (digitalRead(HREF_PIN) && idx < FRAME_BYTES) {
            while (!digitalRead(PCLK_PIN));
            frameBuffer[idx++] = readPixelFast();
            while (digitalRead(PCLK_PIN));
        }
    }
    return true;
}

// ─────────────────────────────────────────────
//  Send frame as chunked UDP  — FIX 2
//  yield() + delay(2) between EVERY chunk lets the WiFi task
//  drain its TX queue and prevents packet drops.
// ─────────────────────────────────────────────
void sendFrame() {
    uint16_t chunkIdx = 0;
    uint32_t offset   = 0;
    uint8_t  retries;

    while (offset < FRAME_BYTES) {
        uint16_t len = min((uint32_t)CHUNK_SIZE, FRAME_BYTES - offset);

        pktBuffer[0] = CAM_ID;
        pktBuffer[1] = (frameSeq     >> 8) & 0xFF;
        pktBuffer[2] =  frameSeq           & 0xFF;
        pktBuffer[3] = (chunkIdx     >> 8) & 0xFF;
        pktBuffer[4] =  chunkIdx           & 0xFF;
        pktBuffer[5] = (TOTAL_CHUNKS >> 8) & 0xFF;
        pktBuffer[6] =  TOTAL_CHUNKS       & 0xFF;
        memcpy(pktBuffer + HEADER_SIZE, frameBuffer + offset, len);

        // Retry up to 3 times if beginPacket fails
        retries = 0;
        while (retries < 3) {
            if (udpSend.beginPacket(pc_ip, pc_port_cam1)) {
                udpSend.write(pktBuffer, HEADER_SIZE + len);
                if (udpSend.endPacket()) break;   // success
            }
            udpFailCount++;
            retries++;
            delay(2);  // short back-off before retry
        }
        if (retries == 3) {
            // Count only once per failed chunk (not once per retry)
            if (udpFailCount % 100 == 1) {
                Serial.print(F("  [WARN] chunk ")); Serial.print(chunkIdx);
                Serial.print(F(" failed (total: ")); Serial.print(udpFailCount); Serial.println(F(")"));
            }
        }

        offset   += len;
        chunkIdx++;

        // Let WiFi task run between every chunk
        yield();
        delay(2);
    }
    frameSeq++;
}

// ─────────────────────────────────────────────
//  Handle UDP commands from cv.py
// ─────────────────────────────────────────────
void handleCommand() {
    int sz = udpCmd.parsePacket();
    if (!sz) return;
    char buf[64] = {0};
    int  n = udpCmd.read(buf, sizeof(buf) - 1);
    buf[n] = '\0';
    String cmd(buf);
    Serial.print(F("  [CMD] ")); Serial.println(cmd);
    if (cmd.startsWith("SE1:"))        servoEntrance.write(cmd.substring(4).toInt());
    else if (cmd.startsWith("SE2:"))   servoExit.write(cmd.substring(4).toInt());
    else if (cmd.startsWith("OLED:")) {
        String msg = cmd.substring(5);
        display.clearDisplay(); display.setCursor(0,0);
        display.setTextSize(1); display.println(msg); display.display();
    }
}

// ─────────────────────────────────────────────
//  WiFi connect
// ─────────────────────────────────────────────
void connectWiFi() {
    printSection("[ WiFi ]");
    Serial.print(F("  SSID     : ")); Serial.println(ssid);
    Serial.print(F("  Target   : ")); Serial.print(pc_ip); Serial.print(F(":")); Serial.println(pc_port_cam1);
    Serial.println(); Serial.print(F("  Connecting"));
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int att = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(F("."));
        if (++att % 20 == 0) { Serial.println(); Serial.print(F("  Retrying ")); }
        if (att > 60) { Serial.println(); delay(3000); ESP.restart(); }
    }
    Serial.println();
    Serial.println(F("  [OK] Connected!"));
    Serial.print(F("  Local IP : ")); Serial.println(WiFi.localIP());
    printSignalBar(WiFi.RSSI());
    Serial.println();
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);
    printBanner();

    printSection("[ GPIO ]");
    int dataPins[] = {D2_PIN,D3_PIN,D4_PIN,D5_PIN,D6_PIN,D7_PIN,D8_PIN,D9_PIN};
    for (int p : dataPins) pinMode(p, INPUT);
    pinMode(VSYNC_PIN, INPUT);
    pinMode(HREF_PIN,  INPUT);
    pinMode(PCLK_PIN,  INPUT);
    Serial.println(F("  [OK] D2-D9, VSYNC, HREF, PCLK configured"));

    startXCLK();
    delay(500);
    Serial.println(F("  [OK] XCLK stable"));

    // ── FIX 1: Release JTAG before Wire.begin() ──────────────────────
    //  GPIO 41 = JTAG TDO (output), GPIO 42 = JTAG TMS (output)
    //  Without releasing these, the JTAG block drives the pins and
    //  I2C can never pull them LOW → SCCB gets no ACK → PID=0xFF
    // ─────────────────────────────────────────────────────────────────
    printSection("[ SCCB / OV7670 ]");
    Serial.println(F("  Releasing JTAG from GPIO 41 (TDO) and 42 (TMS)..."));
    gpio_reset_pin(GPIO_NUM_41);
    gpio_reset_pin(GPIO_NUM_42);
    Serial.println(F("  [OK] JTAG released — pins returned to GPIO matrix"));

    pinMode(SIOD_PIN, INPUT_PULLUP);
    pinMode(SIOC_PIN, INPUT_PULLUP);
    Wire.begin(SIOD_PIN, SIOC_PIN);
    Wire.setClock(100000);
    Serial.println(F("  [OK] Wire on SDA=42, SCL=41 @ 100kHz"));
    initOV7670();

    connectWiFi();

    printSection("[ UDP ]");
    udpSend.begin(5500);
    udpCmd.begin(cmd_port);
    Serial.print(F("  Sending to : ")); Serial.print(pc_ip); Serial.print(F(":")); Serial.println(pc_port_cam1);
    Serial.print(F("  Cmd port   : ")); Serial.println(cmd_port);
    Serial.print(F("  Frame      : ")); Serial.print(FRAME_BYTES); Serial.println(F(" bytes"));
    Serial.print(F("  Chunks/frm : ")); Serial.print(TOTAL_CHUNKS); Serial.println(F(" × 1400B, 2ms gap each"));
    Serial.println();

    printSection("[ Memory ]");
    Serial.print(F("  Free heap  : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
    Serial.println();

    printSection("[ OLED ]");
    Wire1.begin(OLED_SDA, OLED_SCL);
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1); display.setCursor(0,0);
        display.println("CAM1 READY"); display.println(WiFi.localIP().toString());
        display.display();
        Serial.println(F("  [OK] OLED on Wire1 (pins 1/2)"));
    } else {
        Serial.println(F("  [WARN] OLED not found at 0x3C on Wire1"));
    }

    printSection("[ Servos ]");
    servoEntrance.attach(SERVO_ENTRANCE_PIN); servoEntrance.write(0);
    servoExit.attach(SERVO_EXIT_PIN);         servoExit.write(0);
    Serial.print(F("  Entrance pin ")); Serial.print(SERVO_ENTRANCE_PIN); Serial.println(F(" @ 0deg"));
    Serial.print(F("  Exit     pin ")); Serial.print(SERVO_EXIT_PIN);     Serial.println(F(" @ 0deg"));

    Serial.println();
    Serial.println(F("═════════════════════════════════════════"));
    Serial.println(F("   CAM-1 READY  —  Streaming started    "));
    Serial.println(F("═════════════════════════════════════════"));
    Serial.println();

    lastFpsTime  = millis();
    lastHeapTime = millis();
}

// ─────────────────────────────────────────────
//  Main Loop
// ─────────────────────────────────────────────
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("  [WARN] WiFi lost! Reconnecting..."));
        WiFi.disconnect(); connectWiFi();
    }
    handleCommand();

    if (!captureFrame()) {
        Serial.print(F("  [ERR] Frame dropped (total: "));
        Serial.print(droppedFrames); Serial.println(F(")"));
        delay(10); return;
    }

    sendFrame();
    frameCount++;

    uint32_t now = millis();
    if (now - lastFpsTime >= 5000) {
        float fps = (float)(frameCount - lastFpsCount) / ((now - lastFpsTime) / 1000.0f);
        Serial.print(F("  [STAT] Frames:")); Serial.print(frameCount);
        Serial.print(F("  FPS:"));           Serial.print(fps, 1);
        Serial.print(F("  Dropped:"));       Serial.print(droppedFrames);
        Serial.print(F("  UDP_fail:"));      Serial.println(udpFailCount);
        lastFpsTime  = now; lastFpsCount = frameCount;
    }
    if (now - lastHeapTime >= 30000) {
        Serial.print(F("  [MEM] heap:")); Serial.print(ESP.getFreeHeap());
        Serial.print(F("B  RSSI:"));     Serial.print(WiFi.RSSI()); Serial.println(F("dBm"));
        lastHeapTime = now;
    }
}
