#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_http_server.h"
#include <ArduinoJson.h>
#include <time.h>

#define WIFI_SSID "Slowest WiFi in Kiara Plaza 2.4"
#define WIFI_PASSWORD "lannoxyz"

// OLED
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oled_ready = false;

// Servo
#define SV_IN 34
#define SV_OUT 35
#define CLOSE 1638
#define OPEN  4915
#define TIMEOUT 5000
bool sv_in_open=false, sv_out_open=false;
unsigned long sv_in_t=0, sv_out_t=0;
String oled_msg=""; unsigned long oled_until=0;

// Parking
const int ppins[4]={16,17,18,19};
volatile unsigned long pcnt[4]={0};
float freq[4]={0}, base[4]={0};
unsigned long base_ts[4]={0};
bool occupied[4]={false};
void IRAM_ATTR i0(){pcnt[0]++;} void IRAM_ATTR i1(){pcnt[1]++;}
void IRAM_ATTR i2(){pcnt[2]++;} void IRAM_ATTR i3(){pcnt[3]++;}

httpd_handle_t srv=NULL;

// Servo
void sw(int pin,int d){ledcWrite(pin==SV_IN?0:1,d);}
void open_in() {sw(SV_IN,OPEN);  sv_in_open=true;  sv_in_t=millis();  oled_msg="WELCOME"; oled_until=millis()+TIMEOUT+500;}
void open_out(){sw(SV_OUT,OPEN); sv_out_open=true; sv_out_t=millis(); oled_msg="GOODBYE"; oled_until=millis()+TIMEOUT+500;}

// OLED
void oled_time(){
    if(!oled_ready) return;
    struct tm ti; if(!getLocalTime(&ti)) return;
    char t[10],d[20];
    strftime(t,sizeof(t),"%H:%M:%S",&ti);
    strftime(d,sizeof(d),"%d %b %Y",&ti);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(18,2);  display.println("UNMC PARKING");
    display.drawLine(0,13,127,13,SSD1306_WHITE);
    display.setTextSize(2); display.setCursor(10,18); display.println(t);
    display.drawLine(0,38,127,38,SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(28,44); display.println(d);
    display.display();
}
void oled_show(String m){
    if(!oled_ready) return;
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE); display.setTextSize(2);
    display.setCursor(max(0,(int)(128-(int)m.length()*12)/2),22);
    display.println(m); display.display();
}

// HTTP
static esp_err_t h_status(httpd_req_t *req){
    StaticJsonDocument<256> doc;
    JsonArray p=doc.createNestedArray("parking"); for(int i=0;i<4;i++) p.add(occupied[i]);
    JsonArray s=doc.createNestedArray("servo");   s.add(sv_in_open?"Open":"Closed"); s.add(sv_out_open?"Open":"Closed");
    JsonArray f=doc.createNestedArray("freq");    for(int i=0;i<4;i++) f.add((int)freq[i]);
    char buf[256]; serializeJson(doc,buf,sizeof(buf));
    httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
    httpd_resp_send(req,buf,HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static esp_err_t h_action(httpd_req_t *req){
    char buf[64]; char p[16];
    if(httpd_req_get_url_query_str(req,buf,sizeof(buf))==ESP_OK)
        if(httpd_query_key_value(buf,"servo",p,sizeof(p))==ESP_OK){
            if(!strcmp(p,"in"))  open_in();
            if(!strcmp(p,"out")) open_out();
        }
    httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
    httpd_resp_send(req,"{\"ok\":true}",HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void setup(){
    Serial.begin(115200);
    Wire.begin(12,14); Wire.setClock(400000);
    if(display.begin(SSD1306_SWITCHCAPVCC,0x3C)){oled_ready=true; oled_show("BOOT...");}

    ledcSetup(0,50,16); ledcSetup(1,50,16);
    ledcAttachPin(SV_IN,0); ledcAttachPin(SV_OUT,1);
    sw(SV_IN,CLOSE); sw(SV_OUT,CLOSE);

    for(int i=0;i<4;i++){pinMode(ppins[i],INPUT); base_ts[i]=millis();}
    attachInterrupt(digitalPinToInterrupt(ppins[0]),i0,RISING);
    attachInterrupt(digitalPinToInterrupt(ppins[1]),i1,RISING);
    attachInterrupt(digitalPinToInterrupt(ppins[2]),i2,RISING);
    attachInterrupt(digitalPinToInterrupt(ppins[3]),i3,RISING);

    WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
    int r=0; while(WiFi.status()!=WL_CONNECTED&&r++<30) delay(500);
    if(WiFi.status()==WL_CONNECTED){
        configTime(28800,0,"pool.ntp.org");
        httpd_config_t cfg=HTTPD_DEFAULT_CONFIG(); cfg.server_port=80;
        httpd_uri_t su={"/status",HTTP_GET,h_status,NULL};
        httpd_uri_t au={"/action",HTTP_GET,h_action,NULL};
        if(httpd_start(&srv,&cfg)==ESP_OK){
            httpd_register_uri_handler(srv,&su);
            httpd_register_uri_handler(srv,&au);
        }
        Serial.println("IP: "+WiFi.localIP().toString());
        oled_show("READY");
    } else oled_show("NO WIFI");
}

void loop(){
    unsigned long now=millis();
    static unsigned long ls=0;
    if(now-ls>=500){
        for(int i=0;i<4;i++){
            noInterrupts(); unsigned long c=pcnt[i]; pcnt[i]=0; interrupts();
            freq[i]=c*2.0f;
            if(base[i]==0){base[i]=freq[i]; base_ts[i]=now;}
            if(now-base_ts[i]>=3000){
                float d=freq[i]-base[i];
                if(d> 100&&!occupied[i]){occupied[i]=true;  Serial.printf("[P%d] IN\n", i+1);}
                if(d<-100&& occupied[i]){occupied[i]=false; Serial.printf("[P%d] OUT\n",i+1);}
                base[i]=freq[i]; base_ts[i]=now;
            }
        }
        ls=now;
    }
    if(sv_in_open  &&now-sv_in_t >TIMEOUT){sw(SV_IN, CLOSE);sv_in_open =false;}
    if(sv_out_open &&now-sv_out_t>TIMEOUT){sw(SV_OUT,CLOSE);sv_out_open=false;}
    static unsigned long lo=0;
    if(now-lo>=1000){
        if(now<oled_until) oled_show(oled_msg);
        else{oled_msg=""; oled_time();}
        lo=now;
    }
    delay(10);
}