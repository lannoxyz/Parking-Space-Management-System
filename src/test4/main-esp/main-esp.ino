/*
 * main_esp32_cam1.ino  –  Entrance Camera + Gate Controller (ESP32-S3)
 * =====================================================================
 * BUG FIXES applied in this version:
 *
 *  FIX 1 — SCCB PID=0xFF (camera not responding)
 *    Root cause: SDA/SCL lines floating — no pull-up resistors.
 *    OV7670 SCCB requires lines pulled HIGH. External 4.7kΩ to 3.3V
 *    is ideal; this code enables the internal ~45kΩ pull-ups as a
 *    fallback which works reliably at the slow SCCB clock (100kHz).
 *    Also: Wire.setClock(100000) to match SCCB spec.
 *    Also: XCLK stabilisation delay increased 100ms → 500ms
 *          (camera requires ~10ms of stable clock before SCCB works).
 *
 *  FIX 2 — Wire vs Wire1 conflict
 *    display was declared with &Wire but Wire is the camera SCCB bus.
 *    OLED is on GPIO 1/2 which belongs to Wire1.
 *    Fix: declare display with &Wire1.
 *    (This also explains why OLED said "[OK]" — it was accidentally
 *    responding on the camera bus at 0x3C.)
 *
 *  FIX 3 — XCLK 8-bit resolution → 312kHz (previous bug, kept fixed)
 *    1-bit resolution → achievable 20MHz.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ─────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────
const char* ssid         = "Shao Fan's S23 Ultra";
const char* password     = "lannoxyz";
const char* pc_ip        = "172.20.10.4";
const int   pc_port_cam1 = 5001;
const int   cmd_port     = 6001;

WiFiUDP udpSend;
WiFiUDP udpCmd;

// ─────────────────────────────────────────────
//  Camera Pins  (OV7670 on ESP32-S3)
// ─────────────────────────────────────────────
#define XCLK_PIN   10
#define SIOD_PIN   42    // SDA — needs pull-up to 3.3V
#define SIOC_PIN   41    // SCL — needs pull-up to 3.3V
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

// FIX 2: OLED is on Wire1 (GPIO 1/2), not Wire (GPIO 41/42 = camera bus)
#define OLED_SDA   1
#define OLED_SCL   2
#define OLED_ADDR  0x3C
Adafruit_SSD1306 display(128, 64, &Wire1, -1);   // ← was &Wire — WRONG

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
//  XCLK  (1-bit LEDC = max 40MHz → 20MHz OK)
// ─────────────────────────────────────────────
#define XCLK_CHANNEL  0

void startXCLK() {
    ledcSetup(XCLK_CHANNEL, 20000000, 1);
    ledcAttachPin(XCLK_PIN, XCLK_CHANNEL);
    ledcWrite(XCLK_CHANNEL, 1);   // 50% duty
    Serial.println(F("  [OK] XCLK @ 20 MHz, 50% duty (1-bit ch0)"));
}

// ─────────────────────────────────────────────
//  SCCB helpers  (camera I2C on Wire)
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

    // Soft reset
    if (!sccbWrite(0x12, 0x80)) {
        Serial.println(F("  [WARN] SCCB soft-reset failed!"));
        Serial.println(F("         Check: 4.7k pull-ups on SDA(42) and SCL(41) to 3.3V"));
        Serial.println(F("         Check: XCLK connected to OV7670 XCLK pin"));
        Serial.println(F("         Check: OV7670 3.3V supply stable"));
    }
    delay(200);

    // Verify identity
    uint8_t pid = sccbRead(0x0A);
    uint8_t ver = sccbRead(0x0B);
    Serial.print(F("  PID=0x")); Serial.print(pid, HEX);
    Serial.print(F("  VER=0x")); Serial.print(ver, HEX);
    if (pid == 0x76) {
        Serial.println(F("  [OK] OV7670 detected"));
    } else if (pid == 0xFF) {
        Serial.println(F("  [FAIL] No response — I2C bus floating"));
        Serial.println(F("         Internal pull-ups enabled. If still failing,"));
        Serial.println(F("         solder 4.7kΩ resistors: SDA→3.3V, SCL→3.3V"));
    } else {
        Serial.print(F("  [WARN] Unexpected PID"));
    }

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
//  Capture one frame — returns true on success
// ─────────────────────────────────────────────
bool captureFrame() {
    uint32_t idx = 0, t;

    t = millis();
    while (!digitalRead(VSYNC_PIN)) {
        if (millis() - t > 500) {
            Serial.println(F("  [WARN] VSYNC rising timeout — XCLK reaching camera?"));
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
//  Send frame as chunked UDP packets
// ─────────────────────────────────────────────
void sendFrame() {
    uint16_t chunkIdx = 0;
    uint32_t offset   = 0;

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

        if (udpSend.beginPacket(pc_ip, pc_port_cam1)) {
            udpSend.write(pktBuffer, HEADER_SIZE + len);
            if (!udpSend.endPacket()) udpFailCount++;
        } else {
            udpFailCount++;
        }
        offset   += len;
        chunkIdx++;

        // Yield every 8 chunks to prevent WiFi TX buffer saturation
        if (chunkIdx % 8 == 0) delay(1);
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

    if (cmd.startsWith("SE1:")) {
        servoEntrance.write(cmd.substring(4).toInt());
    } else if (cmd.startsWith("SE2:")) {
        servoExit.write(cmd.substring(4).toInt());
    } else if (cmd.startsWith("OLED:")) {
        String msg = cmd.substring(5);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println(msg);
        display.display();
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

    // ── GPIO ──
    printSection("[ GPIO ]");
    int dataPins[] = {D2_PIN,D3_PIN,D4_PIN,D5_PIN,D6_PIN,D7_PIN,D8_PIN,D9_PIN};
    for (int p : dataPins) pinMode(p, INPUT);
    pinMode(VSYNC_PIN, INPUT);
    pinMode(HREF_PIN,  INPUT);
    pinMode(PCLK_PIN,  INPUT);
    Serial.println(F("  [OK] D2-D9, VSYNC, HREF, PCLK configured"));

    // ── XCLK first — camera needs clock before SCCB ──
    startXCLK();
    delay(500);   // FIX 1: was 100ms — 500ms ensures clock is stable
    Serial.println(F("  [OK] XCLK stable delay done"));

    // ── SCCB / OV7670 ──
    printSection("[ OV7670 ]");
    // FIX 1: Enable internal pull-ups on SDA/SCL before Wire.begin()
    //        This compensates for missing external 4.7kΩ resistors.
    //        If SCCB still fails, add physical 4.7kΩ from each pin to 3.3V.
    pinMode(SIOD_PIN, INPUT_PULLUP);
    pinMode(SIOC_PIN, INPUT_PULLUP);
    Wire.begin(SIOD_PIN, SIOC_PIN);
    Wire.setClock(100000);   // 100kHz — SCCB spec maximum
    Serial.println(F("  [OK] Wire on pins SDA=42, SCL=41 @ 100kHz"));
    Serial.println(F("  [NOTE] Internal pull-ups active (~45k). Add 4.7k ext if PID=0xFF"));
    initOV7670();

    // ── WiFi ──
    connectWiFi();

    // ── UDP ──
    printSection("[ UDP ]");
    udpSend.begin(5500);
    udpCmd.begin(cmd_port);
    Serial.print(F("  Sending to : ")); Serial.print(pc_ip); Serial.print(F(":")); Serial.println(pc_port_cam1);
    Serial.print(F("  Cmd port   : ")); Serial.println(cmd_port);
    Serial.print(F("  Frame      : ")); Serial.print(FRAME_BYTES); Serial.println(F(" bytes (320x240x2)"));
    Serial.print(F("  Chunks/frm : ")); Serial.println(TOTAL_CHUNKS);
    Serial.println();

    // ── Memory ──
    printSection("[ Memory ]");
    Serial.print(F("  Free heap  : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
    Serial.println();

    // ── OLED  (FIX 2: Wire1 initialized BEFORE display.begin) ──
    printSection("[ OLED ]");
    Wire1.begin(OLED_SDA, OLED_SCL);   // pins 1, 2
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("CAM1 READY");
        display.println(WiFi.localIP().toString());
        display.display();
        Serial.println(F("  [OK] OLED on Wire1 (pins 1/2) initialized"));
    } else {
        Serial.println(F("  [WARN] OLED not found at 0x3C on Wire1 (pins 1/2)"));
    }

    // ── Servos ──
    printSection("[ Servos ]");
    servoEntrance.attach(SERVO_ENTRANCE_PIN);
    servoExit.attach(SERVO_EXIT_PIN);
    servoEntrance.write(0); servoExit.write(0);
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
        Serial.print(F("  FPS:"));          Serial.print(fps, 1);
        Serial.print(F("  Dropped:"));      Serial.print(droppedFrames);
        Serial.print(F("  UDP_fail:"));     Serial.println(udpFailCount);
        lastFpsTime  = now; lastFpsCount = frameCount;
    }

    if (now - lastHeapTime >= 30000) {
        Serial.print(F("  [MEM] heap:")); Serial.print(ESP.getFreeHeap());
        Serial.print(F("B  RSSI:"));     Serial.print(WiFi.RSSI()); Serial.println(F("dBm"));
        lastHeapTime = now;
    }
}
