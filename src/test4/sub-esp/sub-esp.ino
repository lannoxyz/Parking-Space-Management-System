/*
 * sub_esp32_cam2.ino  –  Exit Camera (plain ESP32)
 * =================================================
 * ROOT CAUSE FIX — UDP 55% packet loss (random chunks):
 *
 *   CAUSE: Sending all 28 chunks back-to-back saturates the ESP32
 *   WiFi TX queue. The failure is now RANDOM (not fixed at chunk 19)
 *   because the queue fills at different points depending on WiFi load.
 *
 *   FIX: yield() + delay(3) between EVERY chunk gives the WiFi RTOS
 *   task time to dequeue and transmit each packet before the next is
 *   queued. Also added per-chunk retry (up to 3× with back-off) so
 *   transient failures recover without silently dropping chunks.
 *
 *   RESULT: UDP fail count should drop to near zero.
 *           Green image should resolve once complete frames arrive.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// =============================
// WiFi Settings
// =============================
const char* ssid     = "Shao Fan's S23 Ultra";
const char* password = "my9nd3k3zn4d9rz";
const char* pc_ip    = "10.143.39.152";   // ← updated from serial log
int         pc_port  = 5002;

WiFiUDP udp;

// =============================
// OV7670 Pins
// =============================
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

// =============================
// Camera Settings (QQVGA 160×120)
// =============================
#define WIDTH      160
#define HEIGHT     120
#define FRAME_SIZE (WIDTH * HEIGHT * 2)   // 38 400 bytes

// =============================
// Chunked UDP
// =============================
#define CAM_ID      2
#define CHUNK_SIZE  1400
#define HEADER_SIZE 7
static const uint16_t TOTAL_CHUNKS = (FRAME_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE;

uint8_t  frameBuffer[FRAME_SIZE];
uint8_t  pktBuffer[HEADER_SIZE + CHUNK_SIZE];
uint16_t frameSeq = 0;

// =============================
// Stats
// =============================
uint32_t frameCount     = 0;
uint32_t droppedFrames  = 0;
uint32_t udpFailCount   = 0;
uint32_t lastFpsTime    = 0;
uint32_t lastFrameCount = 0;
uint32_t lastHeapReport = 0;

// =============================
// Print helpers
// =============================
void printBanner() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════╗"));
    Serial.println(F("║     ESP32  CAM-2  FIRMWARE       ║"));
    Serial.println(F("║     OV7670  |  QQVGA  160x120    ║"));
    Serial.println(F("╚══════════════════════════════════╝"));
    Serial.println();
}
void printSection(const char* t) {
    Serial.println(F("──────────────────────────────────"));
    Serial.println(t);
    Serial.println(F("──────────────────────────────────"));
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

// =============================
// XCLK  (1-bit → 20MHz)
// =============================
void startXCLK() {
    ledcAttach(CAM_PIN_XCLK, 20000000, 1);
    ledcWrite(CAM_PIN_XCLK, 1);
    Serial.println(F("  [OK] XCLK @ 20 MHz, 50% duty"));
}

// =============================
// SCCB helpers
// =============================
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

// =============================
// OV7670 init — QQVGA (160×120) YUV422
// =============================
void initOV7670() {
    printSection("[ OV7670 Init ]");

    bool resetOk = false;
    for (int i = 0; i < 5; i++) {
        if (sccbWrite(0x12, 0x80)) { resetOk = true; break; }
        Serial.print(F("  [WARN] SCCB attempt ")); Serial.print(i+1); Serial.println(F(" failed, retrying..."));
        delay(100);
    }
    if (!resetOk) {
        Serial.println(F("  [FAIL] SCCB failed after 5 attempts"));
        Serial.println(F("         Add 4.7kΩ pull-up: SDA(26)→3.3V, SCL(27)→3.3V"));
    }
    delay(200);

    uint8_t pid = sccbRead(0x0A);
    uint8_t ver = sccbRead(0x0B);
    Serial.print(F("  PID=0x")); Serial.print(pid, HEX);
    Serial.print(F("  VER=0x")); Serial.print(ver, HEX);
    if      (pid == 0x76) Serial.println(F("  [OK] OV7670 detected"));
    else if (pid == 0xFF) Serial.println(F("  [FAIL] No I2C response — check pull-ups"));
    else                  Serial.println(F("  [WARN] Unexpected PID"));

    sccbWrite(0x12, 0x00);
    sccbWrite(0x0C, 0x00);
    sccbWrite(0x3E, 0x00);
    sccbWrite(0x70, 0x3A);
    sccbWrite(0x71, 0x35);
    sccbWrite(0x72, 0x22);  // H÷4, V÷4 → QQVGA  ← key register
    sccbWrite(0x73, 0xF1);  // PCLK_DIV ÷2 for QQVGA
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
    Serial.println(F("  [OK] OV7670 QQVGA registers written"));
}

// =============================
// Read 8 data pins (D0-D7)
// =============================
inline uint8_t readByte() {
    return ((digitalRead(CAM_PIN_D7) << 7) |
            (digitalRead(CAM_PIN_D6) << 6) |
            (digitalRead(CAM_PIN_D5) << 5) |
            (digitalRead(CAM_PIN_D4) << 4) |
            (digitalRead(CAM_PIN_D3) << 3) |
            (digitalRead(CAM_PIN_D2) << 2) |
            (digitalRead(CAM_PIN_D1) << 1) |
             digitalRead(CAM_PIN_D0));
}

// =============================
// Capture one frame
// =============================
bool captureFrame() {
    int      idx = 0;
    uint32_t t;

    t = millis();
    while (!digitalRead(CAM_PIN_VSYNC)) {
        if (millis() - t > 500) {
            Serial.println(F("  [WARN] VSYNC rising timeout"));
            droppedFrames++; return false;
        }
    }
    t = millis();
    while (digitalRead(CAM_PIN_VSYNC)) {
        if (millis() - t > 500) {
            Serial.println(F("  [WARN] VSYNC falling timeout"));
            droppedFrames++; return false;
        }
    }
    while (idx < FRAME_SIZE) {
        t = millis();
        while (!digitalRead(CAM_PIN_HREF)) {
            if (digitalRead(CAM_PIN_VSYNC)) return (idx > 0);
            if (millis() - t > 100) { droppedFrames++; return false; }
        }
        while (digitalRead(CAM_PIN_HREF) && idx < FRAME_SIZE) {
            while (!digitalRead(CAM_PIN_PCLK));
            frameBuffer[idx++] = readByte();
            while (digitalRead(CAM_PIN_PCLK));
        }
    }
    return true;
}

// =============================
// Send frame as chunked UDP  ← ROOT CAUSE FIX
// yield() + delay(3) between EVERY chunk → WiFi task
// gets time to transmit before next packet is queued.
// Retry up to 3× with back-off for transient failures.
// =============================
void sendFrame() {
    uint16_t chunkIdx = 0;
    uint32_t offset   = 0;
    uint8_t  retries;

    while (offset < FRAME_SIZE) {
        uint16_t len = min((uint32_t)CHUNK_SIZE, (uint32_t)(FRAME_SIZE - offset));

        pktBuffer[0] = CAM_ID;
        pktBuffer[1] = (frameSeq     >> 8) & 0xFF;
        pktBuffer[2] =  frameSeq           & 0xFF;
        pktBuffer[3] = (chunkIdx     >> 8) & 0xFF;
        pktBuffer[4] =  chunkIdx           & 0xFF;
        pktBuffer[5] = (TOTAL_CHUNKS >> 8) & 0xFF;
        pktBuffer[6] =  TOTAL_CHUNKS       & 0xFF;
        memcpy(pktBuffer + HEADER_SIZE, frameBuffer + offset, len);

        retries = 0;
        while (retries < 3) {
            if (udp.beginPacket(pc_ip, pc_port)) {
                udp.write(pktBuffer, HEADER_SIZE + len);
                if (udp.endPacket()) break;   // success
            }
            udpFailCount++;
            retries++;
            delay(5);  // back-off before retry
        }
        if (retries == 3) {
            if (udpFailCount % 100 == 1) {
                Serial.print(F("  [WARN] chunk ")); Serial.print(chunkIdx);
                Serial.print(F(" failed (total: ")); Serial.print(udpFailCount); Serial.println(F(")"));
            }
        }

        offset   += len;
        chunkIdx++;

        // KEY FIX: give WiFi RTOS task time to process between every chunk
        yield();
        delay(5);
    }
    frameSeq++;
}

// =============================
// WiFi Connect
// =============================
void connectWiFi() {
    printSection("[ WiFi ]");
    Serial.print(F("  SSID      : ")); Serial.println(ssid);
    Serial.print(F("  Target IP : ")); Serial.print(pc_ip); Serial.print(F(":")); Serial.println(pc_port);
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
    Serial.print(F("  Local IP  : ")); Serial.println(WiFi.localIP());
    printSignalBar(WiFi.RSSI());
    Serial.println();
}

// =============================
// Setup
// =============================
void setup() {
    Serial.begin(115200);
    delay(1500);
    printBanner();

    printSection("[ GPIO ]");
    pinMode(CAM_PIN_VSYNC, INPUT); pinMode(CAM_PIN_HREF, INPUT); pinMode(CAM_PIN_PCLK, INPUT);
    pinMode(CAM_PIN_D0, INPUT); pinMode(CAM_PIN_D1, INPUT); pinMode(CAM_PIN_D2, INPUT);
    pinMode(CAM_PIN_D3, INPUT); pinMode(CAM_PIN_D4, INPUT); pinMode(CAM_PIN_D5, INPUT);
    pinMode(CAM_PIN_D6, INPUT); pinMode(CAM_PIN_D7, INPUT);
    Serial.println(F("  [OK] D0-D7, VSYNC, HREF, PCLK configured"));

    startXCLK();
    delay(500);
    Serial.println(F("  [OK] XCLK stable"));

    pinMode(CAM_PIN_SIOD, INPUT_PULLUP);
    pinMode(CAM_PIN_SIOC, INPUT_PULLUP);
    Wire.begin(CAM_PIN_SIOD, CAM_PIN_SIOC);
    Wire.setClock(100000);
    Serial.println(F("  [OK] Wire on SDA=26, SCL=27 @ 100kHz"));
    initOV7670();

    connectWiFi();

    printSection("[ UDP ]");
    udp.begin(5002);
    Serial.println(F("  [OK] UDP socket opened"));
    Serial.print(F("  Frame  : ")); Serial.print(FRAME_SIZE); Serial.println(F(" bytes (160x120x2)"));
    Serial.print(F("  Chunks : ")); Serial.print(TOTAL_CHUNKS);
    Serial.println(F(" x 1400B, 3ms gap + retry x3"));
    Serial.println();

    printSection("[ Memory ]");
    Serial.print(F("  Free heap : ")); Serial.print(ESP.getFreeHeap());    Serial.println(F(" bytes"));
    Serial.print(F("  Min heap  : ")); Serial.print(ESP.getMinFreeHeap()); Serial.println(F(" bytes"));
    Serial.println();

    Serial.println(F("══════════════════════════════════"));
    Serial.println(F("   CAM-2 READY  —  Streaming...  "));
    Serial.println(F("══════════════════════════════════"));
    Serial.println();

    lastFpsTime    = millis();
    lastFrameCount = 0;
    lastHeapReport = millis();
}

// =============================
// Loop
// =============================
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("  [WARN] WiFi lost! Reconnecting..."));
        WiFi.disconnect(); connectWiFi();
    }

    if (!captureFrame()) {
        Serial.print(F("  [ERR] Frame dropped (total: "));
        Serial.print(droppedFrames); Serial.println(F(")"));
        delay(10); return;
    }

    sendFrame();
    frameCount++;

    uint32_t now = millis();
    if (now - lastFpsTime >= 5000) {
        float fps = (float)(frameCount - lastFrameCount) / ((now - lastFpsTime) / 1000.0f);
        Serial.print(F("  [STAT] Frames:")); Serial.print(frameCount);
        Serial.print(F("  FPS:"));          Serial.print(fps, 1);
        Serial.print(F("  Dropped:"));      Serial.print(droppedFrames);
        Serial.print(F("  UDP_fail:"));     Serial.println(udpFailCount);
        lastFpsTime    = now;
        lastFrameCount = frameCount;
    }
    if (now - lastHeapReport >= 30000) {
        Serial.print(F("  [MEM] heap:")); Serial.print(ESP.getFreeHeap());
        Serial.print(F("B  RSSI:"));     Serial.print(WiFi.RSSI()); Serial.println(F("dBm"));
        lastHeapReport = now;
    }
    delay(1);
}
