/*
 * main_esp32_cam1.c  —  ESP32-S3, OV7670, QVGA 320×240 YUV422
 *
 * ROOT CAUSES FIXED:
 *  1. JTAG drives GPIO 41/42 as outputs → SCCB never ACKs → PID=0xFF
 *     Fix: PIN_FUNC_SELECT to force GPIO mode before Wire.begin()
 *  2. (Not applicable to S3 — uses readPixelFast with REG_READ already)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"

// ── WiFi ────────────────────────────────────
const char* ssid         = "Shao Fan's S23 Ultra";
const char* password     = "my9nd3k3zn4d9rz";
const char* pc_ip        = "10.143.39.152";
const int   pc_port_cam1 = 5001;
const int   cmd_port     = 6001;

WiFiUDP udpSend;
WiFiUDP udpCmd;

// ── Camera Pins ──────────────────────────────
#define XCLK_PIN   10
#define SIOD_PIN   42   // JTAG TMS — released via PIN_FUNC_SELECT
#define SIOC_PIN   41   // JTAG TDO — released via PIN_FUNC_SELECT
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

// ── Frame ────────────────────────────────────
#define WIDTH        320
#define HEIGHT       240
#define FRAME_BYTES  (WIDTH * HEIGHT * 2)
#define CHUNK_SIZE   1400
#define HEADER_SIZE  7
#define CAM_ID       1
static const uint16_t TOTAL_CHUNKS = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

static uint8_t  frameBuf[FRAME_BYTES];
static uint8_t  pktBuf[HEADER_SIZE + CHUNK_SIZE];
static uint16_t frameSeq = 0;

// ── Stats ─────────────────────────────────────
uint32_t frameCnt = 0, dropped = 0, udpFail = 0;
uint32_t lastFpsT = 0, lastFpsCnt = 0, lastHeapT = 0;

// ── Peripherals ──────────────────────────────
#define SERVO_ENT  21
#define SERVO_EXIT 47
Servo servoEnt, servoEx;
Adafruit_SSD1306 display(128, 64, &Wire1, -1);
#define OLED_SDA   1
#define OLED_SCL   2
#define OLED_ADDR  0x3C

// ── Helpers ──────────────────────────────────
void sec(const char* t){Serial.println(F("────────────────────────────────────────")); Serial.println(t); Serial.println(F("────────────────────────────────────────"));}
void sig(int r){const char*q,*b;if(r>=-50){q="Excellent";b="[████████]";}else if(r>=-60){q="Good";b="[██████░░]";}else if(r>=-70){q="Fair";b="[████░░░░]";}else if(r>=-80){q="Poor";b="[██░░░░░░]";}else{q="VeryPoor";b="[█░░░░░░░]";}Serial.print("  RSSI: ");Serial.print(r);Serial.print(" dBm ");Serial.print(b);Serial.print("  ");Serial.println(q);}

// ── XCLK ─────────────────────────────────────
void startXCLK() {
    ledcSetup(0, 20000000, 1);
    ledcAttachPin(XCLK_PIN, 0);
    ledcWrite(0, 1);
    Serial.println(F("  [OK] XCLK 20MHz 50% duty"));
}

// ── SCCB ─────────────────────────────────────
#define OV7670_ADDR 0x21
bool sccbW(uint8_t r, uint8_t v){Wire.beginTransmission(OV7670_ADDR);Wire.write(r);Wire.write(v);return Wire.endTransmission()==0;}
uint8_t sccbR(uint8_t r){Wire.beginTransmission(OV7670_ADDR);Wire.write(r);Wire.endTransmission();Wire.requestFrom((uint8_t)OV7670_ADDR,(uint8_t)1);return Wire.available()?Wire.read():0xFF;}

// ── I2C Scanner ──────────────────────────────
void i2cScan() {
    Serial.println(F("  I2C scan:"));
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("    Found 0x")); Serial.println(a, HEX);
            found++;
        }
    }
    if (!found) Serial.println(F("    Nothing found — check wiring + pull-ups"));
}

// ── OV7670 Init ──────────────────────────────
void initOV7670() {
    sec("[ OV7670 ]");

    // Retry soft-reset
    for (int i = 0; i < 3; i++) {
        if (sccbW(0x12, 0x80)) break;
        Serial.print(F("  SCCB reset attempt ")); Serial.print(i+1); Serial.println(F(" failed"));
        delay(50);
    }
    delay(200);

    uint8_t pid = sccbR(0x0A);
    Serial.print(F("  PID=0x")); Serial.print(pid, HEX);
    Serial.print(F("  VER=0x")); Serial.println(sccbR(0x0B), HEX);
    if (pid == 0x76) Serial.println(F("  [OK] OV7670 responding"));
    else {
        Serial.println(F("  [FAIL] No response. Running I2C scan:"));
        i2cScan();
        Serial.println(F("  Continuing anyway with default camera state..."));
    }

    // QVGA YUV422
    sccbW(0x12, 0x00); sccbW(0x0C, 0x00); sccbW(0x3E, 0x00);
    sccbW(0x70, 0x3A); sccbW(0x71, 0x35);
    sccbW(0x72, 0x11); // H÷2 V÷2 → QVGA
    sccbW(0x73, 0xF0); sccbW(0xA2, 0x02);
    sccbW(0x15, 0x00); sccbW(0x17, 0x16); sccbW(0x18, 0x04);
    sccbW(0x19, 0x02); sccbW(0x1A, 0x7A); sccbW(0x32, 0x80); sccbW(0x03, 0x0A);
    sccbW(0x11, 0x00); sccbW(0x6B, 0x4A);
    sccbW(0x13, 0xE7); sccbW(0x0E, 0x61); sccbW(0x0F, 0x4B); sccbW(0x16, 0x02);
    sccbW(0x1E, 0x07); sccbW(0x21, 0x02); sccbW(0x22, 0x91); sccbW(0x29, 0x07);
    sccbW(0x33, 0x0B); sccbW(0x35, 0x0B); sccbW(0x37, 0x1D); sccbW(0x38, 0x71);
    sccbW(0x39, 0x2A); sccbW(0x3C, 0x78); sccbW(0x4D, 0x40); sccbW(0x4E, 0x20);
    sccbW(0x74, 0x10); sccbW(0x8D, 0x4F);
    sccbW(0x8E,0x00);sccbW(0x8F,0x00);sccbW(0x90,0x00);sccbW(0x91,0x00);
    sccbW(0x96,0x00);sccbW(0x9A,0x00);
    sccbW(0xB0,0x84);sccbW(0xB1,0x0C);sccbW(0xB2,0x0E);sccbW(0xB3,0x82);sccbW(0xB8,0x0A);
    Serial.println(F("  [OK] registers written"));
}

// ── Fast pixel read — all S3 data pins < 32 → single GPIO_IN_REG read ──
// D2=4 D3=5 D4=6 D5=7 D6=15 D7=16 D8=17 D9=18 — all in GPIO_IN_REG
inline uint8_t pxFast() {
    uint32_t r = REG_READ(GPIO_IN_REG);
    return (uint8_t)(
        ((r>>D9_PIN)&1)<<7 | ((r>>D8_PIN)&1)<<6 |
        ((r>>D7_PIN)&1)<<5 | ((r>>D6_PIN)&1)<<4 |
        ((r>>D5_PIN)&1)<<3 | ((r>>D4_PIN)&1)<<2 |
        ((r>>D3_PIN)&1)<<1 |  (r>>D2_PIN)&1
    );
}

// Fast PCLK/HREF/VSYNC via same register
#define PCLK_MASK  (1u << PCLK_PIN)
#define HREF_MASK  (1u << HREF_PIN)
#define VSYNC_MASK (1u << VSYNC_PIN)

// ── Capture ──────────────────────────────────
bool captureFrame() {
    uint32_t idx = 0, t;

    t = millis();
    while (!(REG_READ(GPIO_IN_REG) & VSYNC_MASK)) { if (millis()-t>500){dropped++;return false;} }
    t = millis();
    while  (  REG_READ(GPIO_IN_REG) & VSYNC_MASK)  { if (millis()-t>500){dropped++;return false;} }

    while (idx < FRAME_BYTES) {
        t = millis();
        while (!(REG_READ(GPIO_IN_REG) & HREF_MASK)) {
            if (REG_READ(GPIO_IN_REG) & VSYNC_MASK) return idx > 0;
            if (millis()-t > 100) { dropped++; return false; }
        }
        uint32_t gpio;
        while ((gpio = REG_READ(GPIO_IN_REG)) & HREF_MASK) {
            while (!(REG_READ(GPIO_IN_REG) & PCLK_MASK));  // wait rising
            frameBuf[idx++] = pxFast();
            while   (REG_READ(GPIO_IN_REG) & PCLK_MASK);   // wait falling
            if (idx >= FRAME_BYTES) break;
        }
    }
    return true;
}

// ── UDP send ─────────────────────────────────
void sendFrame() {
    uint16_t ci = 0; uint32_t off = 0;
    while (off < FRAME_BYTES) {
        uint16_t len = min((uint32_t)CHUNK_SIZE, FRAME_BYTES - off);
        pktBuf[0]=CAM_ID;
        pktBuf[1]=(frameSeq>>8)&0xFF; pktBuf[2]=frameSeq&0xFF;
        pktBuf[3]=(ci>>8)&0xFF;       pktBuf[4]=ci&0xFF;
        pktBuf[5]=(TOTAL_CHUNKS>>8)&0xFF; pktBuf[6]=TOTAL_CHUNKS&0xFF;
        memcpy(pktBuf+HEADER_SIZE, frameBuf+off, len);

        for (int r=0; r<3; r++) {
            if (udpSend.beginPacket(pc_ip, pc_port_cam1)) {
                udpSend.write(pktBuf, HEADER_SIZE+len);
                if (udpSend.endPacket()) break;
            }
            udpFail++; yield(); delay(2);
        }
        off += len; ci++;
        yield(); delay(2);
    }
    frameSeq++;
}

void handleCmd() {
    if (!udpCmd.parsePacket()) return;
    char buf[64]={0}; int n=udpCmd.read(buf,63); buf[n]=0; String cmd(buf);
    if      (cmd.startsWith("SE1:"))  servoEnt.write(cmd.substring(4).toInt());
    else if (cmd.startsWith("SE2:"))  servoEx.write(cmd.substring(4).toInt());
    else if (cmd.startsWith("OLED:")){display.clearDisplay();display.setCursor(0,0);display.setTextSize(1);display.println(cmd.substring(5));display.display();}
}

void connectWiFi() {
    sec("[ WiFi ]");
    Serial.print("  SSID: "); Serial.println(ssid);
    Serial.print("  Connecting");
    WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
    int a=0;
    while(WiFi.status()!=WL_CONNECTED){delay(500);Serial.print(".");if(++a>60){Serial.println();ESP.restart();}}
    Serial.println();
    Serial.print("  IP: "); Serial.println(WiFi.localIP());
    sig(WiFi.RSSI());
}

void setup() {
    Serial.begin(115200); delay(1500);
    Serial.println(F("\n╔═══════════════════════════════════╗"));
    Serial.println(F(  "║  ESP32-S3  CAM-1  QVGA 320x240   ║"));
    Serial.println(F(  "╚═══════════════════════════════════╝\n"));

    // GPIO
    sec("[ GPIO ]");
    uint8_t dp[]={D2_PIN,D3_PIN,D4_PIN,D5_PIN,D6_PIN,D7_PIN,D8_PIN,D9_PIN};
    for(auto p:dp) pinMode(p, INPUT);
    pinMode(VSYNC_PIN,INPUT); pinMode(HREF_PIN,INPUT); pinMode(PCLK_PIN,INPUT);
    Serial.println(F("  [OK] data + sync pins"));

    // XCLK first — camera needs clock before SCCB
    startXCLK();
    delay(500);

    // ── JTAG RELEASE ─────────────────────────────────────────────────────
    // GPIO 41 = JTAG TDO, GPIO 42 = JTAG TMS on ESP32-S3.
    // The JTAG peripheral drives these as outputs; I2C can never pull them
    // LOW for ACK. We use PIN_FUNC_SELECT to directly write the IO MUX
    // register, forcing both pads to GPIO function regardless of JTAG state.
    // ─────────────────────────────────────────────────────────────────────
    sec("[ JTAG Release + SCCB ]");
    Serial.println(F("  Forcing GPIO 41,42 out of JTAG via IO MUX..."));
    // PIN_FUNC_SELECT sets the pad to function 1 = GPIO on ESP32-S3
    PIN_FUNC_SELECT(IO_MUX_GPIO41_REG, PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(IO_MUX_GPIO42_REG, PIN_FUNC_GPIO);
    gpio_reset_pin(GPIO_NUM_41);
    gpio_reset_pin(GPIO_NUM_42);
    Serial.println(F("  [OK] IO MUX forced to GPIO mode"));

    pinMode(SIOD_PIN, INPUT_PULLUP);
    pinMode(SIOC_PIN, INPUT_PULLUP);
    Wire.begin(SIOD_PIN, SIOC_PIN);
    Wire.setClock(50000);   // 50kHz — generous for any pull-up quality
    Serial.println(F("  Wire @ 50kHz on SDA=42, SCL=41"));
    initOV7670();

    connectWiFi();

    sec("[ UDP ]");
    udpSend.begin(5500); udpCmd.begin(cmd_port);
    Serial.print(F("  -> ")); Serial.print(pc_ip); Serial.print(F(":")); Serial.println(pc_port_cam1);
    Serial.print(F("  ")); Serial.print(TOTAL_CHUNKS); Serial.println(F(" chunks/frame × 1400B + 2ms gap"));

    sec("[ OLED ]");
    Wire1.begin(OLED_SDA, OLED_SCL);
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1); display.setCursor(0,0);
        display.println("CAM1"); display.println(WiFi.localIP().toString()); display.display();
        Serial.println(F("  [OK] OLED"));
    } else Serial.println(F("  [WARN] OLED not found"));

    sec("[ Servos ]");
    servoEnt.attach(SERVO_ENT); servoEnt.write(0);
    servoEx.attach(SERVO_EXIT); servoEx.write(0);
    Serial.print(F("  ent=pin")); Serial.print(SERVO_ENT);
    Serial.print(F(" exit=pin")); Serial.println(SERVO_EXIT);

    Serial.println(F("\n══════════════════════════════════"));
    Serial.println(F(  "   CAM-1 READY — Streaming       "));
    Serial.println(F(  "══════════════════════════════════\n"));
    lastFpsT=lastHeapT=millis();
}

void loop() {
    if(WiFi.status()!=WL_CONNECTED){WiFi.disconnect();connectWiFi();}
    handleCmd();

    if(!captureFrame()){
        if(dropped%50==1){Serial.print(F("  [WARN] dropped: "));Serial.println(dropped);}
        delay(5); return;
    }
    sendFrame(); frameCnt++;

    uint32_t now=millis();
    if(now-lastFpsT>=5000){
        float fps=(float)(frameCnt-lastFpsCnt)/((now-lastFpsT)/1000.0f);
        Serial.print(F("  [STAT] fr:")); Serial.print(frameCnt);
        Serial.print(F(" fps:")); Serial.print(fps,1);
        Serial.print(F(" drop:")); Serial.print(dropped);
        Serial.print(F(" udpFail:")); Serial.println(udpFail);
        lastFpsT=now; lastFpsCnt=frameCnt;
    }
    if(now-lastHeapT>=30000){
        Serial.print(F("  [MEM] heap:")); Serial.print(ESP.getFreeHeap());
        Serial.print(F(" rssi:")); Serial.println(WiFi.RSSI());
        lastHeapT=now;
    }
}
