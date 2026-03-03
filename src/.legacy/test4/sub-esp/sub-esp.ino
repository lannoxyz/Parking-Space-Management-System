/*
 * sub_esp32_cam2.c  —  plain ESP32, OV7670, QQVGA 160×120 YUV422
 *
 * ROOT CAUSE OF GREEN SNOW FIXED:
 *   digitalRead() = 300ns per call × 8 pins = 2400ns per byte.
 *   OV7670 PCLK at 10MHz = 100ns period.
 *   Result: firmware misses ~96% of pixels → garbage/green image.
 *
 *   Fix: REG_READ(GPIO_IN_REG/GPIO_IN1_REG) = ~10ns per call.
 *   Total byte read time ~30ns << 100ns PCLK period → every pixel captured.
 *
 *   Data pins split across two registers:
 *     GPIO_IN_REG  (0-31): D0=5, D1=18, D2=19, D5=17
 *     GPIO_IN1_REG (32+):  D3=34, D4=33, D6=35, D7=32
 *     PCLK=22, HREF=23, VSYNC=25 → all in GPIO_IN_REG
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "soc/gpio_reg.h"

// ── WiFi ────────────────────────────────────
const char* ssid     = "Shao Fan's S23 Ultra";
const char* password = "my9nd3k3zn4d9rz";
const char* pc_ip    = "10.143.39.152";
const int   pc_port  = 5002;

WiFiUDP udp;

// ── Camera Pins ──────────────────────────────
#define CAM_SIOD   26
#define CAM_SIOC   27
#define CAM_VSYNC  25
#define CAM_HREF   23
#define CAM_PCLK   22
#define CAM_XCLK   21

// Data pins — note split between GPIO banks
#define CAM_D0      5   // GPIO_IN_REG
#define CAM_D1     18   // GPIO_IN_REG
#define CAM_D2     19   // GPIO_IN_REG
#define CAM_D3     34   // GPIO_IN1_REG (34-32=2)
#define CAM_D4     33   // GPIO_IN1_REG (33-32=1)
#define CAM_D5     17   // GPIO_IN_REG
#define CAM_D6     35   // GPIO_IN1_REG (35-32=3)
#define CAM_D7     32   // GPIO_IN1_REG (32-32=0)

// Bitmasks for GPIO_IN_REG (0-31)
#define VSYNC_MASK (1u << CAM_VSYNC)
#define HREF_MASK  (1u << CAM_HREF)
#define PCLK_MASK  (1u << CAM_PCLK)
#define D0_MASK    (1u << CAM_D0)
#define D1_MASK    (1u << CAM_D1)
#define D2_MASK    (1u << CAM_D2)
#define D5_MASK    (1u << CAM_D5)

// Bitmasks for GPIO_IN1_REG (32-39, stored in bits 0-7)
#define D7_MASK    (1u << (CAM_D7 - 32))  // bit 0
#define D4_MASK    (1u << (CAM_D4 - 32))  // bit 1
#define D3_MASK    (1u << (CAM_D3 - 32))  // bit 2
#define D6_MASK    (1u << (CAM_D6 - 32))  // bit 3

// ── Frame ────────────────────────────────────
#define WIDTH      160
#define HEIGHT     120
#define FRAME_SIZE (WIDTH * HEIGHT * 2)   // 38400

#define CAM_ID      2
#define CHUNK_SIZE  1400
#define HEADER_SIZE 7
static const uint16_t TOTAL_CHUNKS = (FRAME_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE;

uint8_t  frameBuf[FRAME_SIZE];
uint8_t  pktBuf[HEADER_SIZE + CHUNK_SIZE];
uint16_t frameSeq = 0;

// ── Stats ─────────────────────────────────────
uint32_t frameCnt=0, dropped=0, udpFail=0;
uint32_t lastFpsT=0, lastFpsCnt=0, lastHeapT=0;

// ── Helpers ──────────────────────────────────
void sec(const char* t){Serial.println(F("──────────────────────────────────"));Serial.println(t);Serial.println(F("──────────────────────────────────"));}
void sig(int r){const char*q,*b;if(r>=-50){q="Excellent";b="[████████]";}else if(r>=-60){q="Good";b="[██████░░]";}else if(r>=-70){q="Fair";b="[████░░░░]";}else if(r>=-80){q="Poor";b="[██░░░░░░]";}else{q="VeryPoor";b="[█░░░░░░░]";}Serial.print("  RSSI:");Serial.print(r);Serial.print("dBm ");Serial.print(b);Serial.print(" ");Serial.println(q);}

// ── XCLK ─────────────────────────────────────
void startXCLK() {
    // Core v3.x API
    ledcAttach(CAM_XCLK, 20000000, 1);
    ledcWrite(CAM_XCLK, 1);
    Serial.println(F("  [OK] XCLK 20MHz"));
}

// ── SCCB ─────────────────────────────────────
#define OV7670_ADDR 0x21
bool sccbW(uint8_t r, uint8_t v){Wire.beginTransmission(OV7670_ADDR);Wire.write(r);Wire.write(v);return Wire.endTransmission()==0;}
uint8_t sccbR(uint8_t r){Wire.beginTransmission(OV7670_ADDR);Wire.write(r);Wire.endTransmission();Wire.requestFrom((uint8_t)OV7670_ADDR,(uint8_t)1);return Wire.available()?Wire.read():0xFF;}

void i2cScan(){
    int f=0;
    for(uint8_t a=1;a<127;a++){Wire.beginTransmission(a);if(Wire.endTransmission()==0){Serial.print(F("    Found 0x"));Serial.println(a,HEX);f++;}}
    if(!f) Serial.println(F("    Nothing — check SDA=26/SCL=27 wiring"));
}

// ── OV7670 Init — QQVGA 160×120 YUV422 ─────
void initOV7670() {
    sec("[ OV7670 ]");
    for(int i=0;i<3;i++){if(sccbW(0x12,0x80))break;Serial.print("  reset attempt ");Serial.print(i+1);Serial.println(" failed");delay(50);}
    delay(200);

    uint8_t pid=sccbR(0x0A);
    Serial.print(F("  PID=0x")); Serial.print(pid,HEX);
    Serial.print(F("  VER=0x")); Serial.println(sccbR(0x0B),HEX);
    if(pid==0x76) Serial.println(F("  [OK] OV7670 detected"));
    else{ Serial.println(F("  [FAIL] scanning bus:")); i2cScan(); Serial.println(F("  Continuing with camera defaults..."));}

    // QQVGA YUV422
    sccbW(0x12,0x00); sccbW(0x0C,0x00); sccbW(0x3E,0x00);
    sccbW(0x70,0x3A); sccbW(0x71,0x35);
    sccbW(0x72,0x22); // H÷4 V÷4 → QQVGA  ← KEY
    sccbW(0x73,0xF1); // PCLK÷2 for QQVGA  ← KEY
    sccbW(0xA2,0x02);
    sccbW(0x15,0x00); sccbW(0x17,0x16); sccbW(0x18,0x04);
    sccbW(0x19,0x02); sccbW(0x1A,0x7A); sccbW(0x32,0x80); sccbW(0x03,0x0A);
    sccbW(0x11,0x00); sccbW(0x6B,0x4A);
    sccbW(0x13,0xE7); sccbW(0x0E,0x61); sccbW(0x0F,0x4B); sccbW(0x16,0x02);
    sccbW(0x1E,0x07); sccbW(0x21,0x02); sccbW(0x22,0x91); sccbW(0x29,0x07);
    sccbW(0x33,0x0B); sccbW(0x35,0x0B); sccbW(0x37,0x1D); sccbW(0x38,0x71);
    sccbW(0x39,0x2A); sccbW(0x3C,0x78); sccbW(0x4D,0x40); sccbW(0x4E,0x20);
    sccbW(0x74,0x10); sccbW(0x8D,0x4F);
    sccbW(0x8E,0);sccbW(0x8F,0);sccbW(0x90,0);sccbW(0x91,0);
    sccbW(0x96,0);sccbW(0x9A,0);
    sccbW(0xB0,0x84);sccbW(0xB1,0x0C);sccbW(0xB2,0x0E);sccbW(0xB3,0x82);sccbW(0xB8,0x0A);
    Serial.println(F("  [OK] QQVGA registers written"));
}

// ── FAST pixel read via REG_READ ─────────────
// Two register reads instead of 8 digitalReads:
// Before:  8 × digitalRead = 2400ns  (MISS at 100ns PCLK)
// After:   2 × REG_READ    =   20ns  (fine at 100ns PCLK)
inline uint8_t IRAM_ATTR pxFast() {
    uint32_t lo = REG_READ(GPIO_IN_REG);   // pins 0-31
    uint32_t hi = REG_READ(GPIO_IN1_REG);  // pins 32-39 in bits 0-7
    return (uint8_t)(
        ((hi & D7_MASK) ? 0x80 : 0) |  // D7=GPIO32 → bit0 of hi
        ((hi & D6_MASK) ? 0x40 : 0) |  // D6=GPIO35 → bit3 of hi
        ((lo & D5_MASK) ? 0x20 : 0) |  // D5=GPIO17
        ((hi & D4_MASK) ? 0x10 : 0) |  // D4=GPIO33 → bit1 of hi
        ((hi & D3_MASK) ? 0x08 : 0) |  // D3=GPIO34 → bit2 of hi
        ((lo & D2_MASK) ? 0x04 : 0) |  // D2=GPIO19
        ((lo & D1_MASK) ? 0x02 : 0) |  // D1=GPIO18
        ((lo & D0_MASK) ? 0x01 : 0)    // D0=GPIO5
    );
}

// ── Capture ──────────────────────────────────
bool IRAM_ATTR captureFrame() {
    int idx=0; uint32_t t;

    // Wait VSYNC rising
    t=millis();
    while(!(REG_READ(GPIO_IN_REG) & VSYNC_MASK)){if(millis()-t>500){dropped++;return false;}}
    // Wait VSYNC falling
    t=millis();
    while( (REG_READ(GPIO_IN_REG) & VSYNC_MASK)){if(millis()-t>500){dropped++;return false;}}

    while(idx < FRAME_SIZE) {
        // Wait HREF high
        t=millis();
        while(!(REG_READ(GPIO_IN_REG) & HREF_MASK)){
            if(REG_READ(GPIO_IN_REG) & VSYNC_MASK) return idx>0;
            if(millis()-t>100){dropped++;return false;}
        }
        // Read line — 1 byte per PCLK rising edge
        while((REG_READ(GPIO_IN_REG) & HREF_MASK) && idx<FRAME_SIZE){
            while(!(REG_READ(GPIO_IN_REG) & PCLK_MASK));  // wait rising
            frameBuf[idx++] = pxFast();
            while( (REG_READ(GPIO_IN_REG) & PCLK_MASK));  // wait falling
        }
    }
    return true;
}

// ── UDP send with yield+delay pacing ─────────
void sendFrame(){
    uint16_t ci=0; uint32_t off=0;
    while(off<FRAME_SIZE){
        uint16_t len=min((uint32_t)CHUNK_SIZE,(uint32_t)(FRAME_SIZE-off));
        pktBuf[0]=CAM_ID;
        pktBuf[1]=(frameSeq>>8)&0xFF; pktBuf[2]=frameSeq&0xFF;
        pktBuf[3]=(ci>>8)&0xFF;       pktBuf[4]=ci&0xFF;
        pktBuf[5]=(TOTAL_CHUNKS>>8)&0xFF; pktBuf[6]=TOTAL_CHUNKS&0xFF;
        memcpy(pktBuf+HEADER_SIZE, frameBuf+off, len);

        for(int r=0;r<3;r++){
            if(udp.beginPacket(pc_ip,pc_port)){
                udp.write(pktBuf, HEADER_SIZE+len);
                if(udp.endPacket()) break;
            }
            udpFail++; yield(); delay(2);
        }
        off+=len; ci++;
        yield(); delay(3);  // let WiFi task drain TX queue
    }
    frameSeq++;
}

void connectWiFi(){
    sec("[ WiFi ]");
    Serial.print("  SSID: "); Serial.println(ssid);
    Serial.print("  Connecting");
    WiFi.mode(WIFI_STA); WiFi.begin(ssid,password);
    int a=0;
    while(WiFi.status()!=WL_CONNECTED){delay(500);Serial.print(".");if(++a>60){Serial.println();ESP.restart();}}
    Serial.println();
    Serial.print("  IP: "); Serial.println(WiFi.localIP());
    sig(WiFi.RSSI());
}

void setup(){
    Serial.begin(115200); delay(1500);
    Serial.println(F("\n╔══════════════════════════════════╗"));
    Serial.println(F(  "║  ESP32  CAM-2  QQVGA  160×120   ║"));
    Serial.println(F(  "╚══════════════════════════════════╝\n"));

    sec("[ GPIO ]");
    uint8_t dp[]={CAM_D0,CAM_D1,CAM_D2,CAM_D3,CAM_D4,CAM_D5,CAM_D6,CAM_D7};
    for(auto p:dp) pinMode(p,INPUT);
    pinMode(CAM_VSYNC,INPUT); pinMode(CAM_HREF,INPUT); pinMode(CAM_PCLK,INPUT);
    Serial.println(F("  [OK] GPIO_IN_REG: D0=5 D1=18 D2=19 D5=17"));
    Serial.println(F("       GPIO_IN1_REG: D3=34 D4=33 D6=35 D7=32"));

    startXCLK();
    delay(500);

    sec("[ SCCB ]");
    pinMode(CAM_SIOD,INPUT_PULLUP);
    pinMode(CAM_SIOC,INPUT_PULLUP);
    Wire.begin(CAM_SIOD,CAM_SIOC);
    Wire.setClock(50000);
    Serial.println(F("  Wire @ 50kHz on SDA=26 SCL=27"));
    initOV7670();

    connectWiFi();

    sec("[ UDP ]");
    udp.begin(5002);
    Serial.print(F("  -> ")); Serial.print(pc_ip); Serial.print(F(":")); Serial.println(pc_port);
    Serial.print(F("  ")); Serial.print(TOTAL_CHUNKS); Serial.println(F(" chunks × 1400B + 3ms gap each"));
    Serial.print(F("  Free heap: ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));

    Serial.println(F("\n══════════════════════════════════"));
    Serial.println(F(  "   CAM-2 READY — Streaming       "));
    Serial.println(F(  "══════════════════════════════════\n"));
    lastFpsT=lastHeapT=millis();
}

void loop(){
    if(WiFi.status()!=WL_CONNECTED){WiFi.disconnect();connectWiFi();}

    if(!captureFrame()){
        if(dropped%50==1){Serial.print(F("  [WARN] dropped:"));Serial.println(dropped);}
        delay(5); return;
    }
    sendFrame(); frameCnt++;

    uint32_t now=millis();
    if(now-lastFpsT>=5000){
        float fps=(float)(frameCnt-lastFpsCnt)/((now-lastFpsT)/1000.0f);
        Serial.print(F("  [STAT] fr:")); Serial.print(frameCnt);
        Serial.print(F(" fps:")); Serial.print(fps,1);
        Serial.print(F(" drop:")); Serial.print(dropped);
        Serial.print(F(" fail:")); Serial.println(udpFail);
        lastFpsT=now; lastFpsCnt=frameCnt;
    }
    if(now-lastHeapT>=30000){
        Serial.print(F("  [MEM] heap:")); Serial.print(ESP.getFreeHeap());
        Serial.print(F(" rssi:")); Serial.println(WiFi.RSSI());
        lastHeapT=now;
    }
    delay(1);
}
