/*
 * sketch.ino - Bun direct tread + mast servo driver (Arduino UNO Q).
 *
 * Receives RPC calls from the host via Arduino_RouterBridge:
 *
 *     Bridge.call("drive", motor_code, dir_code, power, duration_ms)
 *     Bridge.call("drive_pair", left_speed, right_speed)
 *     Bridge.call("drive_triple", left_speed, right_speed, mast_speed)
 *     Bridge.call("drive_mast", mast_speed)
 *
 * Speeds are signed integers in [-100, 100].
 *
 * Motor assignment:
 *     Left tread:  D6/D7
 *     Right tread: D8/D9
 *     Mast servo:  D10 continuous servo, 90 neutral, 30/150 motion
 */

#include <Arduino_RouterBridge.h>
#include "/home/arduino/Arduino/libraries/Arduino_HardwareServo/src/Arduino_HardwareServo.h"
#include "/home/arduino/Arduino/libraries/Arduino_HardwareServo/src/HardwareServo.cpp"
#include "/home/arduino/Arduino/libraries/Arduino_HardwareServo/src/HardwareServo_zephyr.cpp"

const int LEFT_A  = 6;
const int LEFT_B  = 7;
const int RIGHT_A = 8;
const int RIGHT_B = 9;
const int MAST_PIN = 10;

const int SERVO_NEUTRAL = 90;
const int SERVO_DOWN = 30;
const int SERVO_UP = 150;

const uint32_t CMD_TIMEOUT_MS = 500;
const unsigned long PWM_PERIOD_US = 5000; // 200 Hz software PWM
const unsigned long KICK_MS = 120;        // full-power startup kick

HardwareServo mast;

volatile int cmd_left = 0;
volatile int cmd_right = 0;
volatile int cmd_mast = 0;
volatile bool cmd_pending = false;
volatile unsigned long lastCmdMs = 0;
bool raw_pins_active = false;

int applied_mast = 0;

struct Tread {
  int pinA;
  int pinB;
  int speed;
  unsigned long kickUntilMs;
};

Tread leftTread = {LEFT_A, LEFT_B, 0, 0};
Tread rightTread = {RIGHT_A, RIGHT_B, 0, 0};

static inline int clampSpeed(int speed) {
  return max(-100, min(100, speed));
}

void driveHBridgeRaw(int pinA, int pinB, int speed, bool on) {
  if (!on || speed == 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    return;
  }

  if (speed > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }
}

void setTread(Tread &tread, int speed) {
  speed = clampSpeed(speed);
  bool startingOrReversing =
      (tread.speed == 0) || ((tread.speed > 0) != (speed > 0));

  if (speed == 0) {
    tread.speed = 0;
    tread.kickUntilMs = 0;
    driveHBridgeRaw(tread.pinA, tread.pinB, 0, false);
    return;
  }

  tread.speed = speed;
  if (startingOrReversing && abs(speed) < 100) {
    tread.kickUntilMs = millis() + KICK_MS;
  }
}

void serviceTread(Tread &tread) {
  int magnitude = abs(tread.speed);
  if (magnitude <= 0) {
    driveHBridgeRaw(tread.pinA, tread.pinB, 0, false);
    return;
  }

  if (magnitude >= 100 || millis() < tread.kickUntilMs) {
    driveHBridgeRaw(tread.pinA, tread.pinB, tread.speed, true);
    return;
  }

  unsigned long phase = micros() % PWM_PERIOD_US;
  unsigned long onTime = (PWM_PERIOD_US * (unsigned long)magnitude) / 100;
  driveHBridgeRaw(tread.pinA, tread.pinB, tread.speed, phase < onTime);
}

void driveMastServo(int speed) {
  if (speed > 0) {
    mast.write(SERVO_UP);
  } else if (speed < 0) {
    mast.write(SERVO_DOWN);
  } else {
    mast.write(SERVO_NEUTRAL);
  }
}

void applyCmd(int left, int right, int mastSpeed) {
  raw_pins_active = false;
  setTread(leftTread, left);
  setTread(rightTread, right);

  if (!mast.attached()) {
    mast.attach(MAST_PIN);
  }

  if (mastSpeed != applied_mast) {
    driveMastServo(mastSpeed);
    applied_mast = mastSpeed;
  }
}

void queueDrive(int left, int right, int mastSpeed) {
  cmd_left = clampSpeed(left);
  cmd_right = clampSpeed(right);
  cmd_mast = clampSpeed(mastSpeed);
  cmd_pending = true;
  lastCmdMs = millis();
}

void drive_pair(int left_speed, int right_speed) {
  queueDrive(left_speed, right_speed, 0);
}

void drive_triple(int left_speed, int right_speed, int mast_speed) {
  queueDrive(left_speed, right_speed, mast_speed);
}

void drive_mast(int mast_speed) {
  queueDrive(0, 0, mast_speed);
}

void drive_mast_candidate(int channel_code, int speed) {
  // Backwards-compatible with mastScan.py while we transition away from
  // DRV8912 channel probing. Any nonzero channel drives the D10 mast servo.
  if (channel_code == 0) {
    queueDrive(0, 0, 0);
  } else {
    queueDrive(0, 0, speed);
  }
}

void drive_pins(int d6, int d7, int d8, int d9, int d10) {
  // Raw diagnostic escape hatch. Values are clamped to 0/1 and applied
  // directly to D6-D10 so we can probe motor-driver wiring.
  raw_pins_active = true;
  cmd_pending = false;
  lastCmdMs = millis();

  leftTread.speed = 0;
  rightTread.speed = 0;
  applied_mast = 0;

  if (mast.attached()) {
    mast.detach();
  }

  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  pinMode(MAST_PIN, OUTPUT);

  digitalWrite(LEFT_A, d6 != 0 ? HIGH : LOW);
  digitalWrite(LEFT_B, d7 != 0 ? HIGH : LOW);
  digitalWrite(RIGHT_A, d8 != 0 ? HIGH : LOW);
  digitalWrite(RIGHT_B, d9 != 0 ? HIGH : LOW);
  digitalWrite(MAST_PIN, d10 != 0 ? HIGH : LOW);
}

void drive(int motor_code, int dir_code, int power, int duration_ms) {
  power = max(0, min(100, power));
  int speed = dir_code * power;

  int left = 0;
  int right = 0;
  int mastSpeed = 0;

  if (motor_code == 0 || motor_code == 2 || motor_code == 4) left = speed;
  if (motor_code == 1 || motor_code == 2 || motor_code == 4) right = speed;
  if (motor_code == 3 || motor_code == 4) mastSpeed = speed;

  queueDrive(left, right, mastSpeed);
}

void stopAll() {
  raw_pins_active = false;
  queueDrive(0, 0, 0);
  applyCmd(0, 0, 0);
}

void setup() {
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  mast.attach(MAST_PIN);
  mast.write(SERVO_NEUTRAL);

  stopAll();

  lastCmdMs = millis();

  Monitor.begin();
  Bridge.begin();
  Bridge.provide_safe("drive", drive);
  Bridge.provide_safe("drive_pair", drive_pair);
  Bridge.provide_safe("drive_triple", drive_triple);
  Bridge.provide_safe("drive_mast", drive_mast);
  Bridge.provide_safe("drive_mast_candidate", drive_mast_candidate);
  Bridge.provide_safe("drive_pins", drive_pins);
}

void loop() {
  if (raw_pins_active) {
    if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
      digitalWrite(LEFT_A, LOW);
      digitalWrite(LEFT_B, LOW);
      digitalWrite(RIGHT_A, LOW);
      digitalWrite(RIGHT_B, LOW);
      digitalWrite(MAST_PIN, LOW);
      raw_pins_active = false;
    }
    return;
  }

  if (cmd_pending) {
    cmd_pending = false;
    applyCmd(cmd_left, cmd_right, cmd_mast);
  }

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    if (leftTread.speed != 0 || rightTread.speed != 0 || applied_mast != 0) {
      applyCmd(0, 0, 0);
    }
  }

  serviceTread(leftTread);
  serviceTread(rightTread);
}
