/*********
 Electric (EV) & Autonomous vehicle (AV) module kit
 By FABLab Bangkok
 (Max-speed version)
*********/

#include <Ps3Controller.h>
#include <ESP32Servo.h>
int player = 3;   // Variables for changing ID player from 1 into 3
int battery = 0;  // Variable of battery

// Define channels for each motor
const int motorLeft_A = 13;
const int motorLeft_B = 12;
const int motorRight_A = 25;
const int motorRight_B = 26;

// Variables for left Analog values
int leftX = 0;
int leftY = 0;
int min_lx = -50;
int max_lx = 50;
int min_ly = -10;
int max_ly = 10;

// Variables for right Analog values
int rightX = 0;
int rightY = 0;
int min_rx = -50;
int max_rx = 50;
int min_ry = -10;
int max_ry = 10;

//-----------------------------------------------------------------------------------------
Servo myservo;
int pos = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin = 17;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
int servoPin = 7;
#else
int servoPin = 16;
#endif

void setup() {
  Serial.begin(115200);
  Ps3.begin("bb:5a:5a:a0:06:59");
  Serial.println("Ready.");

  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);

  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 1000, 2000);
  myservo.write(90);
}

void Forward_L(int speed) {
  analogWrite(motorLeft_A, speed);
  analogWrite(motorLeft_B, 0);
}
void Backward_L(int speed) {
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, speed);
}
void Forward_R(int speed) {
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}
void Backward_R(int speed) {
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, speed);
}
void Stop_R(int speed) {
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 0);
}
void Stop_L(int speed) {
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 0);
}

void loop() {
  static uint32_t lastSend = 0;
  static bool wasConnected = false;
  const bool connected = Ps3.isConnected();

  if (connected && !wasConnected) {
    Ps3.setPlayer(player);
    Serial.println("PS3 Connected");
  }
  wasConnected = connected;

  if (Ps3.isConnected() && millis() - lastSend > 50) {
    battery = Ps3.data.status.battery;
    Serial.print("Battery: ");
    if (battery == ps3_status_battery_charging) Serial.println("charging");
    else if (battery == ps3_status_battery_full) Serial.println("FULL");
    else if (battery == ps3_status_battery_high) Serial.println("HIGH");
    else if (battery == ps3_status_battery_low) Serial.println("LOW");
    else if (battery == ps3_status_battery_dying) Serial.println("DYING");
    else if (battery == ps3_status_battery_shutdown) Serial.println("SHUTDOWN");
    else Serial.println("UNDEFINED");

    leftX = Ps3.data.analog.stick.lx;
    leftY = Ps3.data.analog.stick.ly;
    rightX = Ps3.data.analog.stick.rx;
    rightY = Ps3.data.analog.stick.ry;

    Serial.print(leftX); Serial.print(", ");
    Serial.print(leftY); Serial.print(", ");
    Serial.print(rightX); Serial.print(", ");
    Serial.println(rightY);

    if (Ps3.data.button.up) {
      Serial.println("forward");
      Forward_L(230);
      Forward_R(230);
    }
    if (Ps3.data.button.down) {
      Serial.println("backward");
      Backward_L(230);
      Backward_R(230);
    }
    if (Ps3.data.button.left) {
      myservo.write(60);
      Serial.println("left");
    }
    if (Ps3.data.button.right) {
      myservo.write(130);
      Serial.println("right");
    }
    if (Ps3.data.button.select) {
      Serial.println("AI mode");
      Forward_L(0);
      Forward_R(0);
    }

    if (!Ps3.data.button.right && !Ps3.data.button.left && !Ps3.data.button.down && !Ps3.data.button.up) {
      Serial.println("stop");
      myservo.write(90);
      Stop_R(0);
      Stop_L(0);
    }
  }
}
