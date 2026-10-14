#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#define JOY_X   34
#define JOY_Y   35
#define JOY_BTN 32
#define BTN_GRIP 27
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17
#define CH_SHOULDER 0
#define CH_ELBOW 1
#define CH_WRIST 2
#define CH_GRIP 3

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

AccelStepper myStepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

#define JOY_X_CENTER 2000
#define JOY_Y_CENTER 1840
#define DEAD_ZONE    100
#define GRIPPER_OPEN_ANGLE  90
#define GRIPPER_CLOSE_ANGLE 40
int mode = 0;
bool gripperOpen = true;
float shoulderAngle = 90;
float elbowAngle = 90;
float wristAngle = 90;
bool lastModeBtn = false;
bool lastGripBtn = false;
void setServoAngle(uint8_t channel, float angle) {
    uint16_t pulseLength = map(angle, 0, 180, 102, 492);
    pca.setPWM(channel, 0, pulseLength);
}
const float MAX_DEG_PER_LOOP = 1.5;
float axisVelocity(int raw, int center) {
    int offset = raw - center;
    if (abs(offset) <= DEAD_ZONE) return 0.0;
  float span = (offset > 0) ? (4095 - center - DEAD_ZONE) : (center - DEAD_ZONE);
    float amount = (abs(offset) - DEAD_ZONE) / span;   // 0.0 to 1.0
    return (offset > 0 ? 1.0 : -1.0) * amount * MAX_DEG_PER_LOOP;
}
void setup() {
  Serial.print("Y raw: ");
Serial.println(analogRead(JOY_Y));
    Serial.begin(115200);
    pinMode(JOY_BTN, INPUT_PULLUP);
    pinMode(BTN_GRIP, INPUT_PULLUP);
    myStepper.setMaxSpeed(400);
    myStepper.setAcceleration(500);
    myStepper.moveTo(0);
    Wire.begin(21, 22);
    pca.begin();
    pca.setPWMFreq(50);
    setServoAngle(CH_SHOULDER, shoulderAngle);
    setServoAngle(CH_ELBOW,    elbowAngle);
    setServoAngle(CH_WRIST,    wristAngle);
    setServoAngle(CH_GRIP,     GRIPPER_OPEN_ANGLE);
}
void loop() {
  if (digitalRead(JOY_BTN) == LOW && !lastModeBtn) {
    mode = 1 - mode;
    lastModeBtn = true;
  }
  else if (digitalRead(JOY_BTN) == HIGH) {
    lastModeBtn = false;
  }
  if (digitalRead(BTN_GRIP) == LOW && !lastGripBtn) {
    gripperOpen = !gripperOpen;
    lastGripBtn = true;
    setServoAngle(CH_GRIP, gripperOpen ? GRIPPER_OPEN_ANGLE : GRIPPER_CLOSE_ANGLE);
  }
  else if (digitalRead(BTN_GRIP) == HIGH) {
    lastGripBtn = false;
  } 
  int x   = analogRead(JOY_X);
  int y   = analogRead(JOY_Y);
  if (mode == 0) {
    if (x < JOY_X_CENTER - DEAD_ZONE) {
        int speed = map(x,
                        JOY_X_CENTER - DEAD_ZONE,
                        0,
                        0, 800);
        myStepper.setSpeed(-speed);
        myStepper.runSpeed();
    }
    else if (x > JOY_X_CENTER + DEAD_ZONE) {
        int speed = map(x,
                        JOY_X_CENTER + DEAD_ZONE,
                        4095,
                        0, 800);
        myStepper.setSpeed(speed);
        myStepper.runSpeed();
    }
    else {
        myStepper.setSpeed(0);
        myStepper.runSpeed();
    }
    shoulderAngle += axisVelocity(y, JOY_Y_CENTER);
    shoulderAngle = constrain(shoulderAngle, 0, 180);
    setServoAngle(CH_SHOULDER, shoulderAngle);
  }
  else if (mode == 1) {
    elbowAngle += axisVelocity(x, JOY_X_CENTER);
    elbowAngle = constrain(elbowAngle, 0, 180);
    setServoAngle(CH_ELBOW, elbowAngle);

    wristAngle += axisVelocity(y, JOY_Y_CENTER);
    wristAngle = constrain(wristAngle, 0, 180);
    setServoAngle(CH_WRIST, wristAngle);
  }
}
