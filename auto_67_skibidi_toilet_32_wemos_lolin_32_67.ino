/*********
 EV & AV module kit — Autonomous speed controller
 Board: Wemos LOLIN32 (ESP32)
 Driver: Cytron Dual 3A
 Sensors: MPU6050 (I2C via i2cdevlib), INA219 (I2C), Light (ADC1)
 Behavior: move straight; loop conditions & adjust PWM: uphill=100, downhill=60, tunnel dark=60, general=80
*********/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_INA219.h>
#include <MPU6050.h>   // i2cdevlib version (MPU6050_Base)

// ---------- Pins ----------
const int motorLeft_A  = 13;
const int motorLeft_B  = 12;
const int motorRight_A = 25;
const int motorRight_B = 26;
const int LIGHT_PIN    = 36;   // << use an ADC1 pin: 32/33/34/35/36/39

// ---------- Servo (keep wheels straight) --------
Servo steer;
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin = 17;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
int servoPin = 7;
#else
int servoPin = 16;
#endif

// ---------- Sensors ----------
Adafruit_INA219 ina219;
MPU6050 mpu(0x68);  // i2cdevlib object at default address

// ---------- Mission speeds ----------
const int SPD_GENERAL   = 80;
const int SPD_UPHILL    = 100;
const int SPD_DOWNHILL  = 60;
const int SPD_TUNNEL_N  = 80;
const int SPD_TUNNEL_D  = 60;

// ---------- Thresholds ----------
float PITCH_THRESH_DEG = 7.0;  // tune after prints
int   DARK_THRESHOLD   = 200;  // light raw (0..4095). Lower = darker

// ---------- Current guard (optional) ----------
float   CURRENT_LIMIT_A = 2.0;     // Cytron 3A/channel — start conservative
uint16_t OVERCURR_MS    = 150;

// ---------- Smooth ramp ----------
const int RAMP_UP_PER_TICK   = 6;   // PWM steps per control tick (~50 ms)
const int RAMP_DOWN_PER_TICK = 12;

// ---------- Motor helpers ----------
inline void Forward_L(int speed){ analogWrite(motorLeft_A,  speed); analogWrite(motorLeft_B, 0); }
inline void Forward_R(int speed){ analogWrite(motorRight_A, speed); analogWrite(motorRight_B,0); }
inline void Stop_L(){ analogWrite(motorLeft_A, 0); analogWrite(motorLeft_B, 0); }
inline void Stop_R(){ analogWrite(motorRight_A,0); analogWrite(motorRight_B,0); }

// ---------- Read pitch (deg) from accel via i2cdevlib ----------
float readPitchDeg() {
  int16_t axr, ayr, azr, gxr, gyr, gzr;
  mpu.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);   // reads accel+gyro registers

  // NOTE: scale depends on accel FS. We'll set FS=±2g, so 16384 LSB/g.
  const float ACC_LSB_PER_G = 16384.0f; // for ±2g
  float ax = axr / ACC_LSB_PER_G;
  float ay = ayr / ACC_LSB_PER_G;
  float az = azr / ACC_LSB_PER_G;

  // Pitch from accel (y-plane tilt)
  float pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  return pitch;
}

// ---------- Light ----------
int readLightRaw() { return analogRead(LIGHT_PIN); }

// ---------- Current ----------
float readCurrentA() { return ina219.getCurrent_mA() / 1000.0f; }

// ---------- Decide target speed from environment ----------
int decideEnvSpeed(bool debug=false){
  int   light = readLightRaw();
  bool  isDark = (light < DARK_THRESHOLD);

  float pitch = readPitchDeg();
  bool  uphill   = (pitch >  PITCH_THRESH_DEG);
  bool  downhill = (pitch < -PITCH_THRESH_DEG);

  int spd;
  if (isDark)         spd = SPD_TUNNEL_D;
  else if (uphill)    spd = SPD_UPHILL;
  else if (downhill)  spd = SPD_DOWNHILL;
  else                spd = SPD_GENERAL;

  if (debug){
    Serial.printf("[ENV] light=%d dark=%d pitch=%.1f => spd=%d\n",
                  light, isDark, pitch, spd);
  }
  return spd;
}

// ---------- Ramp toward target ----------
int ramp(int current, int target){
  if (target > current){
    int next = current + RAMP_UP_PER_TICK;
    return (next > target) ? target : next;
  }else if (target < current){
    int next = current - RAMP_DOWN_PER_TICK;
    return (next < target) ? target : next;
  }
  return current;
}

void setup(){
  Serial.begin(115200);

  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);

  steer.setPeriodHertz(50);
  steer.attach(servoPin, 1000, 2000);
  steer.write(90); // straight

  pinMode(LIGHT_PIN, INPUT);
  analogSetPinAttenuation(LIGHT_PIN, ADC_11db);   // ~0–3.3 V span on ESP32

  Wire.begin();
  if (ina219.begin()) Serial.println("[INA219] OK"); else Serial.println("[INA219] NOT FOUND");

  // ---- i2cdevlib MPU6050 init ----
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("[MPU6050] OK");
    // Set accelerometer ±2g, gyro ±2000 °/s to match our math
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    // Optional: quick gyro calibration (keep still). Function name is capitalized in this lib:
    mpu.CalibrateGyro(6);  // 6 samples; increase if you want longer calibration
  } else {
    Serial.println("[MPU6050] NOT FOUND");
  }

  Serial.println("Autonomous mode: driving straight and adapting speed.");
}

void loop(){
  static uint32_t last = 0;
  static int pwmCmd = 0;     // current commanded PWM
  if (millis() - last < 50) return; // ~20 Hz control loop
  last = millis();

  // 1) Decide environment speed
  int target = decideEnvSpeed(true);      // set to false after you calibrate
  target = constrain(target, 0, 255);

  // 2) (Optional) current guard — enable if INA219 calibrated well
  // float ia = readCurrentA();
  // static uint32_t overSince = 0;
  // if (ia > CURRENT_LIMIT_A) { if (!overSince) overSince = millis(); }
  // else overSince = 0;
  // if (overSince && millis() - overSince > OVERCURR_MS) target = (target * 80) / 100;

  // 3) Ramp smoothly
  pwmCmd = ramp(pwmCmd, target);

  // 4) Keep steering straight & drive forward
  steer.write(90);
  Forward_L(pwmCmd);
  Forward_R(pwmCmd);
}
