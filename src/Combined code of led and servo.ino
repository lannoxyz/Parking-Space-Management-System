#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define SDA 32
#define SCL 33

#define SERVO1_PIN   25
#define SERVO2_PIN  26
#define SERVO_CH1    0
#define SERVO_CH2    1
#define SERVO_FREQ  50      // 50 Hz for servo
#define SERVO_RES   16      // 16-bit resolution

// Duty values calculated for 16-bit @ 50 Hz
#define DUTY_0_DEG   1638   // ~500 µs
#define DUTY_90_DEG  4915   // ~1500 µs
#define DUTY_180_DEG 7864   // ~2400 µs

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);


void setup() {
  // Configure PWM channel
  ledcSetup(SERVO_CH1, SERVO_FREQ, SERVO_RES);
  ledcSetup(SERVO_CH2, SERVO_FREQ, SERVO_RES);

  // Attach GPIO to PWM channel
  ledcAttachPin(SERVO1_PIN, SERVO_CH1);
  ledcAttachPin(SERVO2_PIN, SERVO_CH1);

  // Move servo to 0°
  ledcWrite(SERVO_CH1, DUTY_0_DEG);
  ledcWrite(SERVO_CH2, DUTY_0_DEG);

  Wire.begin(SDA, SCL);   // SDA, SCL

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (true); // OLED not found
  }

  display.clearDisplay();
  display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(0, 0);
   //display welcome for 2s
   display.println("University of Nottingham\nTime");
   display.display();
}


void loop ()
{
  if(in == TRUE){//function in outputs 1 or true
    ledcWrite(SERVO_CH1, DUTY_90_DEG); //open gate
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(0, 0);
   //display welcome for 2s
   display.println("Welcome");
   display.display();
   delay(2000);

   //display parking info for 5s
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(0, 0);
   display.println("Parking info");
   display.display();
   delay(5000);
   display.clearDisplay();
   ledcWrite(SERVO_CH1, DUTY_0_DEG); //gate opens for 7s

  }else if(in == FALSE){
    delay(4000);
    ledcWrite(SERVO_CH1, DUTY_0_DEG); //the wait for 4 seconds and close to 0 degree but dont really understand this part
  }

  if(exit == TRUE){
   ledcWrite(SERVO_CH2, DUTY_90_DEG); //open gate
    delay(5000);
    ledcWrite(SERVO_CH2, DUTY_0_DEG);
  } else if(exit == FALSE){
    delay(4000);
    ledcWrite(SERVO_CH1, DUTY_0_DEG);

}
