#define SERVO_PIN   25
#define SERVO_CH    0
#define SERVO_FREQ  50      // 50 Hz for servo
#define SERVO_RES   16      // 16-bit resolution

// Duty values calculated for 16-bit @ 50 Hz
#define DUTY_0_DEG   1638   // ~500 µs
#define DUTY_90_DEG  4915   // ~1500 µs
#define DUTY_180_DEG 7864   // ~2400 µs

void setup() {
  // Configure PWM channel
  ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);

  // Attach GPIO to PWM channel
  ledcAttachPin(SERVO_PIN, SERVO_CH);

  // Move servo to 0°
  ledcWrite(SERVO_CH, DUTY_0_DEG);
}

void loop() {
  delay(1000);
  // Move servo to 90°
  ledcWrite(SERVO_CH, DUTY_90_DEG);

  delay(1000);
  // Move servo to 180°
  ledcWrite(SERVO_CH, DUTY_180_DEG);

  delay(1000);
    // Move servo to 0°
  ledcWrite(SERVO_CH, DUTY_0_DEG);
}
