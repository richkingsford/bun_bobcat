#include "Arduino_RouterBridge.h"

// Bun robot bridge-controlled tread driver
// Left tread:  pins 6 / 7
// Right tread: pins 8 / 9
//
// Python sends parsed values:
//   Bridge.call("drive", motorCode, dirCode, power, durationMs)
//
// motorCode:
//   0 = left
//   1 = right
//   2 = all
//
// dirCode:
//   1  = forward
//   -1 = backward
//   0  = stop
//
// durationMs:
//   0 = keep going until another command
//   >0 = automatically stop after that many milliseconds

const int LEFT_A  = 6;
const int LEFT_B  = 7;
const int RIGHT_A = 8;
const int RIGHT_B = 9;

const unsigned long PWM_PERIOD_US = 5000; // 200 Hz software PWM
const unsigned long KICK_MS = 120;        // full-power startup kick

struct Motor {
  int pinA;
  int pinB;
  int dir;                 // 1 forward, -1 backward, 0 stop
  int power;               // 0-100
  unsigned long stopAtMs;  // 0 means no timed stop
  unsigned long kickUntil;
};

Motor leftMotor  = { LEFT_A, LEFT_B, 0, 0, 0, 0 };
Motor rightMotor = { RIGHT_A, RIGHT_B, 0, 0, 0, 0 };

void setup() {
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  stopAll();

  Monitor.begin();
  Bridge.begin();

  // Python calls this function.
  Bridge.provide_safe("drive", drive);

  Monitor.println("Bun Bridge tread control ready.");
}

void loop() {
  unsigned long now = millis();

  checkTimedStop(leftMotor, now);
  checkTimedStop(rightMotor, now);

  serviceMotor(leftMotor);
  serviceMotor(rightMotor);
}

void drive(int motorCode, int dirCode, int power, int durationMs) {
  power = constrain(power, 0, 100);

  Monitor.print("drive motorCode=");
  Monitor.print(motorCode);
  Monitor.print(" dirCode=");
  Monitor.print(dirCode);
  Monitor.print(" power=");
  Monitor.print(power);
  Monitor.print(" durationMs=");
  Monitor.println(durationMs);

  if (dirCode != 1 && dirCode != -1 && dirCode != 0) {
    Monitor.println("Bad dirCode.");
    return;
  }

  if (durationMs < 0) {
    durationMs = 0;
  }

  if (motorCode == 0) {
    setMotor(leftMotor, dirCode, power, durationMs);
  } else if (motorCode == 1) {
    setMotor(rightMotor, dirCode, power, durationMs);
  } else if (motorCode == 2) {
    setMotor(leftMotor, dirCode, power, durationMs);
    setMotor(rightMotor, dirCode, power, durationMs);
  } else {
    Monitor.println("Bad motorCode.");
  }
}

void setMotor(Motor &m, int dir, int power, int durationMs) {
  bool startingOrChanging = (m.dir != dir || m.power == 0);

  if (dir == 0 || power == 0) {
    m.dir = 0;
    m.power = 0;
    m.stopAtMs = 0;
    m.kickUntil = 0;
    stopMotor(m);
    return;
  }

  m.dir = dir;
  m.power = power;

  if (durationMs > 0) {
    m.stopAtMs = millis() + (unsigned long)durationMs;
  } else {
    m.stopAtMs = 0;
  }

  if (startingOrChanging && power < 100) {
    m.kickUntil = millis() + KICK_MS;
  } else {
    m.kickUntil = 0;
  }
}

void checkTimedStop(Motor &m, unsigned long now) {
  if (m.stopAtMs != 0 && timeReached(now, m.stopAtMs)) {
    m.dir = 0;
    m.power = 0;
    m.stopAtMs = 0;
    m.kickUntil = 0;
    stopMotor(m);
  }
}

// Handles millis() rollover safely.
bool timeReached(unsigned long now, unsigned long target) {
  return (long)(now - target) >= 0;
}

void serviceMotor(Motor &m) {
  if (m.dir == 0 || m.power <= 0) {
    stopMotor(m);
    return;
  }

  bool fullPowerKick = millis() < m.kickUntil;

  if (m.power >= 100 || fullPowerKick) {
    driveMotorRaw(m, true);
    return;
  }

  unsigned long phase = micros() % PWM_PERIOD_US;
  unsigned long onTime = (PWM_PERIOD_US * m.power) / 100;
  bool pwmOn = phase < onTime;

  driveMotorRaw(m, pwmOn);
}

void driveMotorRaw(Motor &m, bool on) {
  if (!on) {
    stopMotor(m);
    return;
  }

  if (m.dir == 1) {
    digitalWrite(m.pinA, HIGH);
    digitalWrite(m.pinB, LOW);
  } else if (m.dir == -1) {
    digitalWrite(m.pinA, LOW);
    digitalWrite(m.pinB, HIGH);
  } else {
    stopMotor(m);
  }
}

void stopMotor(Motor &m) {
  digitalWrite(m.pinA, LOW);
  digitalWrite(m.pinB, LOW);
}

void stopAll() {
  leftMotor.dir = 0;
  leftMotor.power = 0;
  leftMotor.stopAtMs = 0;
  leftMotor.kickUntil = 0;

  rightMotor.dir = 0;
  rightMotor.power = 0;
  rightMotor.stopAtMs = 0;
  rightMotor.kickUntil = 0;

  stopMotor(leftMotor);
  stopMotor(rightMotor);
}