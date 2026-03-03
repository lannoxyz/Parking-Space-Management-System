/*
 * main_esp32_cam1.ino  –  Entrance Camera + Gate Controller
 * ===========================================================
 * FIX: replaced ledcAttach() (Core v3.x only) with
 *      ledcSetup() + ledcAttachPin() (Core v2.x compatible)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ─────────────────────────────────────────────
//  WiFi  (Station mode)
// ─────────────────────────────────────────────
const char* ssid          = "Lanno";
const char* password      = "lannoxyz";
const char* pc_ip         = "172.20.10.9";
const int   pc_port_cam1  = 5001;
const int   cmd_port      = 6001;

WiFiUDP udpSend;
WiFiUDP udpCmd;

// ─────────────────────────────────────────────
//  Camera Pins  (OV7670 on ESP32-S3)
// ─────────────────────────────────────────────
#define XCLK_PIN   10
#define SIOD_PIN   42
#define SIOC_PIN   41
#define VSYNC_PIN  12
#define HREF_PIN   13
#define PCLK_PIN   11

#define D2_PIN     4
#define D3_PIN     5
#define D4_PIN     6
#define D5_PIN     7
#define D6_PIN     15
#define D7_PIN     16
#define D8_PIN     17
#define D9_PIN     18

// ─────────────────────────────────────────────
//  Frame geometry
// ─────────────────────────────────────────────
#define WIDTH        320
#define HEIGHT       240
#define FRAME_BYTES  (WIDTH * HEIGHT * 2)   // 153 600
#define CHUNK_SIZE   1400

#define HEADER_SIZE  7
#define CAM_ID       1

static uint8_t  frameBuffer[FRAME_BYTES];
static uint8_t  pktBuffer[HEADER_SIZE + CHUNK_SIZE];
static uint16_t frameSeq = 0;
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
//  Servo
// ─────────────────────────────────────────────
#define SERVO_ENTRANCE_PIN 21
#define SERVO_EXIT_PIN     47
Servo servoEntrance;
Servo servoExit;

// ─────────────────────────────────────────────
//  OLED
// ─────────────────────────────────────────────
#define OLED_SDA  1
#define OLED_SCL  2
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

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

void printSection(const char* title) {
    Serial.println(F("─────────────────────────────────────────"));
    Serial.println(title);
    Serial.println(F("─────────────────────────────────────────"));
}

void printSignalBar(int rssi) {
    const char* quality;
    const char* bar;
    if      (rssi >= -50) { quality = "Excellent"; bar = "[████████]"; }
    else if (rssi >= -60) { quality = "Good     "; bar = "[██████░░]"; }
    else if (rssi >= -70) { quality = "Fair     "; bar = "[████░░░░]"; }
    else if (rssi >= -80) { quality = "Poor     "; bar = "[██░░░░░░]"; }
    else                  { quality = "Very Poor"; bar = "[█░░░░░░░]"; }
    Serial.print(F("  Signal   : "));
    Serial.print(rssi);
    Serial.print(F(" dBm  "));
    Serial.print(bar);
    Serial.print(F("  "));
    Serial.println(quality);
}

// ─────────────────────────────────────────────
//  XCLK — Core v2.x compatible
//  Use ledcSetup() + ledcAttachPin() instead of ledcAttach()
// ─────────────────────────────────────────────
#define XCLK_CHANNEL  0   // LEDC channel 0–15

void startXCLK() {
    ledcSetup(XCLK_CHANNEL, 20000000, 8);   // ch0, 20 MHz, 8-bit resolution
    ledcAttachPin(XCLK_PIN, XCLK_CHANNEL);  // bind GPIO to channel
    ledcWrite(XCLK_CHANNEL, 128);           // 50% duty = 128/255
    Serial.println(F("  [OK] XCLK started @ 20 MHz (50% duty, ch0)"));
}

// ─────────────────────────────────────────────
//  SCCB helpers
// ─────────────────────────────────────────────
#define OV7670_ADDR 0x21

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

// ─────────────────────────────────────────────
//  OV7670 register init  (QVGA, YUV422, ~30 FPS)
// ─────────────────────────────────────────────
void initOV7670() {
    printSection("[ OV7670 Init ]");

    bool ok = sccbWrite(0x12, 0x80);   // Soft reset
    if (!ok) Serial.println(F("  [WARN] SCCB write failed — check wiring!"));
    delay(200);

    uint8_t pid = sccbRead(0x0A);
    Serial.print(F("  OV7670 PID : 0x"));
    Serial.print(pid, HEX);
    if (pid == 0x76) Serial.println(F("  (OK)"));
    else             Serial.println(F("  (unexpected — check wiring/I2C address)"));

    sccbWrite(0x12, 0x00);
    sccbWrite(0x0C, 0x00);
    sccbWrite(0x3E, 0x00);
    sccbWrite(0x70, 0x3A);
    sccbWrite(0x71, 0x35);
    sccbWrite(0x72, 0x11);
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
    sccbWrite(0x8E, 0x00);
    sccbWrite(0x8F, 0x00);
    sccbWrite(0x90, 0x00);
    sccbWrite(0x91, 0x00);
    sccbWrite(0x96, 0x00);
    sccbWrite(0x9A, 0x00);
    sccbWrite(0xB0, 0x84);
    sccbWrite(0xB1, 0x0C);
    sccbWrite(0xB2, 0x0E);
    sccbWrite(0xB3, 0x82);
    sccbWrite(0xB8, 0x0A);

    Serial.println(F("  [OK] OV7670 registers written"));
}

// ─────────────────────────────────────────────
//  Fast pixel read via GPIO input register
// ─────────────────────────────────────────────
inline uint8_t readPixelFast() {
    uint32_t reg = REG_READ(GPIO_IN_REG);
    return (uint8_t)(
        ((reg >> D9_PIN) & 1) << 7 |
        ((reg >> D8_PIN) & 1) << 6 |
        ((reg >> D7_PIN) & 1) << 5 |
        ((reg >> D6_PIN) & 1) << 4 |
        ((reg >> D5_PIN) & 1) << 3 |
        ((reg >> D4_PIN) & 1) << 2 |
        ((reg >> D3_PIN) & 1) << 1 |
        ((reg >> D2_PIN) & 1)
    );
}

// ─────────────────────────────────────────────
//  Capture one frame — returns true on success
// ─────────────────────────────────────────────
bool captureFrame() {
    uint32_t idx = 0;
    uint32_t timeout;

    timeout = millis();
    while (!digitalRead(VSYNC_PIN)) {
        if (millis() - timeout > 200) {
            Serial.println(F("  [WARN] VSYNC timeout (rising)"));
            droppedFrames++; return false;
        }
    }
    timeout = millis();
    while (digitalRead(VSYNC_PIN)) {
        if (millis() - timeout > 200) {
            Serial.println(F("  [WARN] VSYNC timeout (falling)"));
            droppedFrames++; return false;
        }
    }

    while (idx < FRAME_BYTES) {
        timeout = millis();
        while (!digitalRead(HREF_PIN)) {
            if (digitalRead(VSYNC_PIN)) return (idx > 0);
            if (millis() - timeout > 50) { droppedFrames++; return false; }
        }
        while (digitalRead(HREF_PIN) && idx < FRAME_BYTES) {
            while (!digitalRead(PCLK_PIN));
            frameBuffer[idx++] = readPixelFast();
            while ( digitalRead(PCLK_PIN));
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
        uint16_t chunkLen = min((uint32_t)CHUNK_SIZE, FRAME_BYTES - offset);

        pktBuffer[0] = CAM_ID;
        pktBuffer[1] = (frameSeq     >> 8) & 0xFF;
        pktBuffer[2] =  frameSeq           & 0xFF;
        pktBuffer[3] = (chunkIdx     >> 8) & 0xFF;
        pktBuffer[4] =  chunkIdx           & 0xFF;
        pktBuffer[5] = (TOTAL_CHUNKS >> 8) & 0xFF;
        pktBuffer[6] =  TOTAL_CHUNKS       & 0xFF;

        memcpy(pktBuffer + HEADER_SIZE, frameBuffer + offset, chunkLen);

        int ok = udpSend.beginPacket(pc_ip, pc_port_cam1);
        if (ok) {
            udpSend.write(pktBuffer, HEADER_SIZE + chunkLen);
            if (!udpSend.endPacket()) udpFailCount++;
        } else {
            udpFailCount++;
        }
        offset   += chunkLen;
        chunkIdx++;
    }
    frameSeq++;
}

// ─────────────────────────────────────────────
//  Handle commands from cv.py (UDP)
// ─────────────────────────────────────────────
void handleCommand() {
    int pktSize = udpCmd.parsePacket();
    if (!pktSize) return;

    char buf[64] = {0};
    int  len = udpCmd.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';
    String cmd(buf);

    Serial.print(F("  [CMD] Received: ")); Serial.println(cmd);

    if (cmd.startsWith("SE1:")) {
        int angle = cmd.substring(4).toInt();
        servoEntrance.write(angle);
        Serial.print(F("  [SERVO] Entrance -> ")); Serial.println(angle);
    } else if (cmd.startsWith("SE2:")) {
        int angle = cmd.substring(4).toInt();
        servoExit.write(angle);
        Serial.print(F("  [SERVO] Exit -> ")); Serial.println(angle);
    } else if (cmd.startsWith("OLED:")) {
        String msg = cmd.substring(5);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println(msg);
        display.display();
        Serial.print(F("  [OLED] -> ")); Serial.println(msg);
    } else {
        Serial.println(F("  [CMD] Unknown command ignored"));
    }
}

// ─────────────────────────────────────────────
//  WiFi connect with animated feedback
// ─────────────────────────────────────────────
void connectWiFi() {
    printSection("[ WiFi ]");
    Serial.print(F("  SSID      : ")); Serial.println(ssid);
    Serial.print(F("  Target    : ")); Serial.print(pc_ip);
    Serial.print(F(":")); Serial.println(pc_port_cam1);
    Serial.println();
    Serial.print(F("  Connecting"));

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
        attempts++;
        if (attempts % 20 == 0) { Serial.println(); Serial.print(F("  Retrying ")); }
        if (attempts > 60) {
            Serial.println();
            Serial.println(F("  [ERR] WiFi failed. Restarting in 3s..."));
            delay(3000);
            ESP.restart();
        }
    }

    Serial.println();
    Serial.println(F("  [OK] Connected!"));
    Serial.print(F("  Local IP  : ")); Serial.println(WiFi.localIP());
    printSignalBar(WiFi.RSSI());
    Serial.println();
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);   // Let USB-Serial bridge stabilize

    printBanner();

    // ── GPIO ──
    printSection("[ GPIO ]");
    int dataPins[] = {D2_PIN,D3_PIN,D4_PIN,D5_PIN,D6_PIN,D7_PIN,D8_PIN,D9_PIN};
    for (int p : dataPins) pinMode(p, INPUT);
    pinMode(VSYNC_PIN, INPUT);
    pinMode(HREF_PIN,  INPUT);
    pinMode(PCLK_PIN,  INPUT);
    Serial.println(F("  [OK] Data pins D2-D9 configured"));
    Serial.println(F("  [OK] VSYNC / HREF / PCLK configured"));

    // ── XCLK ──
    startXCLK();
    delay(100);

    // ── OV7670 ──
    Wire.begin(SIOD_PIN, SIOC_PIN);
    initOV7670();

    // ── WiFi ──
    connectWiFi();

    // ── UDP ──
    printSection("[ UDP ]");
    udpSend.begin(5500);
    udpCmd.begin(cmd_port);
    Serial.print(F("  Sending to : ")); Serial.print(pc_ip);
    Serial.print(F(":")); Serial.println(pc_port_cam1);
    Serial.print(F("  Cmd port   : ")); Serial.println(cmd_port);
    Serial.print(F("  Frame size : ")); Serial.print(FRAME_BYTES); Serial.println(F(" bytes (320x240x2)"));
    Serial.print(F("  Chunks/frm : ")); Serial.println(TOTAL_CHUNKS);
    Serial.println();

    // ── Memory ──
    printSection("[ Memory ]");
    Serial.print(F("  Free heap  : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
    Serial.println();

    // ── OLED ──
    printSection("[ OLED ]");
    Wire1.begin(OLED_SDA, OLED_SCL);
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("CAM1 READY");
        display.println(WiFi.localIP().toString());
        display.display();
        Serial.println(F("  [OK] OLED initialized"));
    } else {
        Serial.println(F("  [WARN] OLED not found at 0x3C"));
    }

    // ── Servos ──
    printSection("[ Servos ]");
    servoEntrance.attach(SERVO_ENTRANCE_PIN);
    servoExit.attach(SERVO_EXIT_PIN);
    servoEntrance.write(0);
    servoExit.write(0);
    Serial.print(F("  Entrance -> pin ")); Serial.print(SERVO_ENTRANCE_PIN); Serial.println(F(" @ 0deg"));
    Serial.print(F("  Exit     -> pin ")); Serial.print(SERVO_EXIT_PIN);     Serial.println(F(" @ 0deg"));

    // ── Ready ──
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

    // WiFi watchdog
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("  [WARN] WiFi lost! Reconnecting..."));
        WiFi.disconnect();
        connectWiFi();
    }

    handleCommand();

    bool ok = captureFrame();
    if (!ok) {
        Serial.print(F("  [ERR] Frame dropped (total: "));
        Serial.print(droppedFrames);
        Serial.println(F(")"));
        delay(10);
        return;
    }

    sendFrame();
    frameCount++;

    // FPS stats every 5 s
    uint32_t now = millis();
    if (now - lastFpsTime >= 5000) {
        float fps = (float)(frameCount - lastFpsCount) / ((now - lastFpsTime) / 1000.0f);
        Serial.print(F("  [STAT] Frames: ")); Serial.print(frameCount);
        Serial.print(F("  FPS: "));           Serial.print(fps, 1);
        Serial.print(F("  Dropped: "));       Serial.print(droppedFrames);
        Serial.print(F("  UDP fails: "));     Serial.println(udpFailCount);
        lastFpsTime  = now;
        lastFpsCount = frameCount;
    }

    // Heap + RSSI every 30 s
    if (now - lastHeapTime >= 30000) {
        Serial.print(F("  [MEM]  Free heap: ")); Serial.print(ESP.getFreeHeap());
        Serial.print(F(" bytes  RSSI: "));        Serial.print(WiFi.RSSI());
        Serial.println(F(" dBm"));
        lastHeapTime = now;
    }
}
