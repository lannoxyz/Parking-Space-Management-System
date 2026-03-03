#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// =============================
// WiFi Settings
// =============================
const char* ssid     = "Lanno";
const char* password = "lannoxyz";
const char* pc_ip    = "172.20.10.9";
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
// Camera Settings (QQVGA)
// =============================
#define WIDTH      160
#define HEIGHT     120
#define FRAME_SIZE (WIDTH * HEIGHT * 2)  // 38,400 bytes

uint8_t frameBuffer[FRAME_SIZE];

// =============================
// Debug / Stats
// =============================
uint32_t frameCount        = 0;
uint32_t droppedFrames     = 0;
uint32_t udpFailCount      = 0;
uint32_t lastFpsTime       = 0;
uint32_t lastFrameCount    = 0;
uint32_t lastHeapReport    = 0;

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

void printSectionHeader(const char* title) {
    Serial.println(F("──────────────────────────────────"));
    Serial.println(title);
    Serial.println(F("──────────────────────────────────"));
}

void printSignalBar(int rssi) {
    // rssi: -30 (excellent) to -90 (very poor)
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

// =============================
// XCLK via LEDC (Core v3.x)
// =============================
void startXCLK() {
    ledcAttach(CAM_PIN_XCLK, 20000000, 1);
    ledcWrite(CAM_PIN_XCLK, 1);
    Serial.println(F("  [OK] XCLK started @ 20 MHz"));
}

// =============================
// Read 8 data pins (D0–D7)
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
// Capture one frame YUV422
// Returns true on success
// =============================
bool captureFrame() {
    int index = 0;
    uint32_t timeout;

    // Wait for VSYNC rising edge (max 200ms)
    timeout = millis();
    while (!digitalRead(CAM_PIN_VSYNC)) {
        if (millis() - timeout > 200) {
            Serial.println(F("  [WARN] VSYNC timeout (rising)"));
            droppedFrames++;
            return false;
        }
    }

    // Wait for VSYNC falling edge
    timeout = millis();
    while (digitalRead(CAM_PIN_VSYNC)) {
        if (millis() - timeout > 200) {
            Serial.println(F("  [WARN] VSYNC timeout (falling)"));
            droppedFrames++;
            return false;
        }
    }

    while (index < FRAME_SIZE) {

        // Wait for HREF high
        timeout = millis();
        while (!digitalRead(CAM_PIN_HREF)) {
            if (digitalRead(CAM_PIN_VSYNC)) return (index > 0);
            if (millis() - timeout > 50) {
                droppedFrames++;
                return false;
            }
        }

        // Read pixels while HREF is high
        while (digitalRead(CAM_PIN_HREF)) {

            while (!digitalRead(CAM_PIN_PCLK));
            frameBuffer[index++] = readByte();
            while (digitalRead(CAM_PIN_PCLK));

            while (!digitalRead(CAM_PIN_PCLK));
            frameBuffer[index++] = readByte();
            while (digitalRead(CAM_PIN_PCLK));

            if (index >= FRAME_SIZE) break;
        }
    }

    return true;
}

// =============================
// WiFi Connect with animation
// =============================
void connectWiFi() {
    printSectionHeader("[ WiFi ]");
    Serial.print(F("  SSID      : "));
    Serial.println(ssid);
    Serial.print(F("  Target IP : "));
    Serial.print(pc_ip);
    Serial.print(F(":"));
    Serial.println(pc_port);
    Serial.println();
    Serial.print(F("  Connecting"));

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
        attempts++;

        if (attempts % 20 == 0) {
            Serial.println();
            Serial.print(F("  Retrying "));
        }
        if (attempts > 60) {
            Serial.println();
            Serial.println(F("  [ERR] WiFi failed! Restarting in 3s..."));
            delay(3000);
            ESP.restart();
        }
    }

    Serial.println();
    Serial.println(F("  [OK] Connected!"));
    Serial.print(F("  Local IP  : "));
    Serial.println(WiFi.localIP());
    printSignalBar(WiFi.RSSI());
    Serial.println();
}

// =============================
// Setup
// =============================
void setup() {
    Serial.begin(115200);
    delay(1500);  // Let USB-Serial bridge stabilize — fixes garbled output

    printBanner();

    // --- GPIO Setup ---
    printSectionHeader("[ GPIO ]");
    pinMode(CAM_PIN_VSYNC, INPUT);
    pinMode(CAM_PIN_HREF,  INPUT);
    pinMode(CAM_PIN_PCLK,  INPUT);

    pinMode(CAM_PIN_D0, INPUT);
    pinMode(CAM_PIN_D1, INPUT);
    pinMode(CAM_PIN_D2, INPUT);
    pinMode(CAM_PIN_D3, INPUT);
    pinMode(CAM_PIN_D4, INPUT);
    pinMode(CAM_PIN_D5, INPUT);
    pinMode(CAM_PIN_D6, INPUT);
    pinMode(CAM_PIN_D7, INPUT);
    Serial.println(F("  [OK] Data pins D0–D7 configured"));
    Serial.println(F("  [OK] VSYNC / HREF / PCLK configured"));

    // --- XCLK ---
    startXCLK();

    // --- WiFi ---
    connectWiFi();

    // --- UDP ---
    printSectionHeader("[ UDP ]");
    udp.begin(5002);
    Serial.println(F("  [OK] UDP socket opened"));
    Serial.print(F("  Frame size : "));
    Serial.print(FRAME_SIZE);
    Serial.println(F(" bytes  (160 x 120 x 2, YUV422)"));
    Serial.println();

    // --- Memory ---
    printSectionHeader("[ Memory ]");
    Serial.print(F("  Free heap  : "));
    Serial.print(ESP.getFreeHeap());
    Serial.println(F(" bytes"));
    Serial.print(F("  Min heap   : "));
    Serial.print(ESP.getMinFreeHeap());
    Serial.println(F(" bytes"));
    Serial.println();

    // --- Ready ---
    Serial.println(F("══════════════════════════════════"));
    Serial.println(F("   CAM-2 READY  —  Streaming...  "));
    Serial.println(F("══════════════════════════════════"));
    Serial.println();

    lastFpsTime    = millis();
    lastFrameCount = 0;
    lastHeapReport = millis();
}

// =============================
// Loop: Capture & Send
// =============================
void loop() {

    // --- WiFi watchdog ---
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println();
        Serial.println(F("  [WARN] WiFi lost! Reconnecting..."));
        WiFi.disconnect();
        connectWiFi();
    }

    // --- Capture ---
    bool ok = captureFrame();

    if (!ok) {
        Serial.print(F("  [ERR] Frame capture failed  (dropped total: "));
        Serial.print(droppedFrames);
        Serial.println(F(")"));
        delay(10);
        return;
    }

    // --- Send via UDP ---
    int sent = udp.beginPacket(pc_ip, pc_port);
    if (sent) {
        udp.write(frameBuffer, FRAME_SIZE);
        int result = udp.endPacket();
        if (!result) {
            udpFailCount++;
            if (udpFailCount % 10 == 1) {
                Serial.print(F("  [WARN] UDP send failed (total fails: "));
                Serial.print(udpFailCount);
                Serial.println(F(")"));
            }
        }
    } else {
        udpFailCount++;
        Serial.println(F("  [ERR] UDP beginPacket failed"));
    }

    frameCount++;

    // --- FPS report every 5 seconds ---
    uint32_t now = millis();
    if (now - lastFpsTime >= 5000) {
        uint32_t elapsed = now - lastFpsTime;
        uint32_t newFrames = frameCount - lastFrameCount;
        float fps = (float)newFrames / (elapsed / 1000.0f);

        Serial.print(F("  [STAT] Frames: "));
        Serial.print(frameCount);
        Serial.print(F("  |  FPS: "));
        Serial.print(fps, 1);
        Serial.print(F("  |  Dropped: "));
        Serial.print(droppedFrames);
        Serial.print(F("  |  UDP fails: "));
        Serial.println(udpFailCount);

        lastFpsTime    = now;
        lastFrameCount = frameCount;
    }

    // --- Heap report every 30 seconds ---
    if (now - lastHeapReport >= 30000) {
        Serial.print(F("  [MEM]  Free heap: "));
        Serial.print(ESP.getFreeHeap());
        Serial.print(F(" bytes  |  RSSI: "));
        Serial.print(WiFi.RSSI());
        Serial.println(F(" dBm"));
        lastHeapReport = now;
    }

    delay(1);
}
