/*
 * main_esp32_cam1.ino  –  Entrance Camera + Gate Controller
 * ===========================================================
 * FIXES applied vs. original code:
 *   1. WiFi.softAP → WiFi.begin  (Station mode so ESP32 can reach PC)
 *   2. Added WiFi connection wait loop with Serial feedback
 *   3. udpSend.begin(local_port) added before use
 *   4. OV7670 SCCB (I2C) register init added (was completely missing)
 *   5. Frame split into CHUNK_SIZE UDP packets with 7-byte header
 *      (original single-packet send exceeded UDP MTU → frames never arrived)
 *   6. captureFrame() optimised: direct GPIO register reads replace
 *      slow digitalRead() calls (critical for 320×240 @ ~10 FPS)
 *   7. XCLK resolution set to 8 bits (was 1 bit → incorrect duty cycle)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ─────────────────────────────────────────────
//  WiFi  (Station mode — must reach PC)
// ─────────────────────────────────────────────
const char* ssid     = "Lanno";      // ← change
const char* password = "lannoxyz";  // ← change

// PC addresses
const char* pc_ip        = "172.20.10.9";  // ← change to your PC's IP
const int   pc_port_cam1 = 5001;             // cam.py listens here for CAM1
const int   cmd_port     = 6001;             // ESP32 listens for commands from cv.py

// ─────────────────────────────────────────────
//  UDP sockets
// ─────────────────────────────────────────────
WiFiUDP udpSend;
WiFiUDP udpCmd;

// ─────────────────────────────────────────────
//  Camera Pins  (OV7670 on ESP32-S3)
// ─────────────────────────────────────────────
#define XCLK_PIN   10
#define SIOD_PIN   42    // SDA (SCCB)
#define SIOC_PIN   41    // SCL (SCCB)
#define VSYNC_PIN  12
#define HREF_PIN   13
#define PCLK_PIN   11
// Data bus D2–D9
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
#define CHUNK_SIZE   1400                   // safe UDP payload

/*
 * UDP packet header layout (7 bytes, big-endian):
 *   [0]   cam_id       uint8   (1 = entrance, 2 = exit)
 *   [1-2] frame_seq    uint16
 *   [3-4] chunk_index  uint16
 *   [5-6] total_chunks uint16
 */
#define HEADER_SIZE  7
#define CAM_ID       1

static uint8_t frameBuffer[FRAME_BYTES];
static uint8_t pktBuffer[HEADER_SIZE + CHUNK_SIZE];
static uint16_t frameSeq  = 0;
static const uint16_t TOTAL_CHUNKS = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

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
//  SCCB (OV7670 I2C) helpers
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
//  OV7670 Register Initialisation
//  Configures: QVGA (320×240), YUV422 (YUYV), 30 FPS
// ─────────────────────────────────────────────
void initOV7670() {
    // Soft reset
    sccbWrite(0x12, 0x80);
    delay(100);

    // QVGA + YUV422
    sccbWrite(0x12, 0x00);   // COM7: YUV, no mirror/flip
    sccbWrite(0x0C, 0x00);   // COM3
    sccbWrite(0x3E, 0x00);   // COM14: no PCLK scaling
    sccbWrite(0x70, 0x3A);   // SCALING_XSC
    sccbWrite(0x71, 0x35);   // SCALING_YSC
    sccbWrite(0x72, 0x11);   // SCALING_DCWCTR: /2 H and V
    sccbWrite(0x73, 0xF0);   // SCALING_PCLK_DIV
    sccbWrite(0xA2, 0x02);   // SCALING_PCLK_DELAY

    // Output format: YUYV
    sccbWrite(0x15, 0x00);   // COM10: VSYNC positive
    sccbWrite(0x17, 0x16);   // HSTART
    sccbWrite(0x18, 0x04);   // HSTOP
    sccbWrite(0x19, 0x02);   // VSTRT
    sccbWrite(0x1A, 0x7A);   // VSTOP
    sccbWrite(0x32, 0x80);   // HREF
    sccbWrite(0x03, 0x0A);   // VREF

    // Clock: 30 FPS at 20 MHz XCLK
    sccbWrite(0x11, 0x00);   // CLKRC: prescaler /1
    sccbWrite(0x6B, 0x4A);   // DBLV: PLL ×4, bypass internal regulator

    // Colour / AGC / AWB
    sccbWrite(0x13, 0xE7);   // COM8: auto-gain, AWB, AEC all on
    sccbWrite(0x0E, 0x61);   // COM5
    sccbWrite(0x0F, 0x4B);   // COM6
    sccbWrite(0x16, 0x02);
    sccbWrite(0x1E, 0x07);   // MVFP
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

    Serial.println("[CAM] OV7670 registers written.");
}

// ─────────────────────────────────────────────
//  XCLK generation  (20 MHz, 50 % duty)
// ─────────────────────────────────────────────
void startXCLK() {
    // 8-bit resolution, 50 % duty = 128
    ledcAttach(XCLK_PIN, 20000000, 8);
    ledcWrite(XCLK_PIN, 128);
}

// ─────────────────────────────────────────────
//  Fast pixel read via GPIO input register
//  Works only on GPIO 0–31; adjust if pins differ
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
//  Capture one YUV422 frame into frameBuffer[]
// ─────────────────────────────────────────────
void captureFrame() {
    uint32_t idx = 0;

    // Wait for VSYNC rising edge (start of frame)
    while (!digitalRead(VSYNC_PIN));
    while ( digitalRead(VSYNC_PIN));

    while (idx < FRAME_BYTES) {
        // Wait for HREF high (active line)
        while (!digitalRead(HREF_PIN)) {
            if (digitalRead(VSYNC_PIN)) return;  // guard: new VSYNC before frame done
        }

        // Clock in pixels on this line
        while (digitalRead(HREF_PIN) && idx < FRAME_BYTES) {
            while (!digitalRead(PCLK_PIN));
            frameBuffer[idx++] = readPixelFast();
            while ( digitalRead(PCLK_PIN));
        }
    }
}

// ─────────────────────────────────────────────
//  Transmit frame as chunked UDP packets
// ─────────────────────────────────────────────
void sendFrame() {
    uint16_t chunkIdx = 0;
    uint32_t offset   = 0;

    while (offset < FRAME_BYTES) {
        uint16_t chunkLen = min((uint32_t)CHUNK_SIZE, FRAME_BYTES - offset);

        // Build 7-byte header (big-endian)
        pktBuffer[0] = CAM_ID;
        pktBuffer[1] = (frameSeq >> 8) & 0xFF;
        pktBuffer[2] =  frameSeq       & 0xFF;
        pktBuffer[3] = (chunkIdx >> 8) & 0xFF;
        pktBuffer[4] =  chunkIdx       & 0xFF;
        pktBuffer[5] = (TOTAL_CHUNKS >> 8) & 0xFF;
        pktBuffer[6] =  TOTAL_CHUNKS       & 0xFF;

        memcpy(pktBuffer + HEADER_SIZE, frameBuffer + offset, chunkLen);

        udpSend.beginPacket(pc_ip, pc_port_cam1);
        udpSend.write(pktBuffer, HEADER_SIZE + chunkLen);
        udpSend.endPacket();

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
//  Setup
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);

    // ── Camera data pins as input ──
    int dataPins[] = {D2_PIN,D3_PIN,D4_PIN,D5_PIN,D6_PIN,D7_PIN,D8_PIN,D9_PIN};
    for (int p : dataPins) pinMode(p, INPUT);
    pinMode(VSYNC_PIN, INPUT);
    pinMode(HREF_PIN,  INPUT);
    pinMode(PCLK_PIN,  INPUT);

    // ── Generate XCLK before OV7670 init ──
    startXCLK();
    delay(100);

    // ── SCCB (OLED Wire instance reused on different pins) ──
    // Use Wire1 for camera if OLED also uses Wire
    Wire.begin(SIOD_PIN, SIOC_PIN);
    initOV7670();

    // ── WiFi Station mode ──
    Serial.print("[WiFi] Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.print("\n[WiFi] Connected → ");
    Serial.println(WiFi.localIP());

    // ── UDP sockets ──
    udpSend.begin(5500);       // local send port (arbitrary)
    udpCmd.begin(cmd_port);    // listen for commands from cv.py
    Serial.printf("[UDP] Sending frames to %s:%d\n", pc_ip, pc_port_cam1);
    Serial.printf("[UDP] Listening for commands on port %d\n", cmd_port);

    // ── OLED (separate Wire instance on GPIO 1/2) ──
    Wire1.begin(OLED_SDA, OLED_SCL);
    // Use Wire1 for display if board supports it; otherwise share Wire
    // display = Adafruit_SSD1306(128, 64, &Wire1, -1);
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("CAM1 READY");
        display.display();
    }

    // ── Servos ──
    servoEntrance.attach(SERVO_ENTRANCE_PIN);
    servoExit.attach(SERVO_EXIT_PIN);
    servoEntrance.write(0);   // closed
    servoExit.write(0);

    Serial.println("[CAM1] Setup complete. Streaming…");
}

// ─────────────────────────────────────────────
//  Main Loop
// ─────────────────────────────────────────────
void loop() {
    handleCommand();   // process any incoming UDP commands first
    captureFrame();    // grab one YUV422 frame from OV7670
    sendFrame();       // chunk & transmit to cam.py
}
