#include "Arduino_RouterBridge.h"

// Bun robot bridge-controlled tread driver + continuous-rotation MG995 mast servo.
//
// Python sends:
//   Bridge.call("drive", motorCode, dirCode, power, durationMs)
//   Bridge.call("mast", dirCode, durationMs)
//
// drive motorCode:
//   0 = left tread, pins 6/7
//   1 = right tread, pins 8/9
//   2 = both treads
//
// mast dirCode:
//   1  = one servo direction, same as Servo.write(150)
//   -1 = other servo direction, same as Servo.write(30)
//   0  = neutral/stop, same as Servo.write(90)

const int LEFT_A = 6;
const int LEFT_B = 7;
const int RIGHT_A = 8;
const int RIGHT_B = 9;
const int MAST_PIN = 10;

const unsigned long PWM_PERIOD_US = 5000; // 200 Hz software PWM for tread power.
const unsigned long KICK_MS = 120;        // Full-power startup kick.

const unsigned long SERVO_PERIOD_US = 20000;
const int SERVO_MIN_US = 544;
const int SERVO_MAX_US = 2400;
const int MAST_DOWN_ANGLE = 30;
const int MAST_STOP_ANGLE = 90;
const int MAST_UP_ANGLE = 150;

struct Motor {
  int pinA;
  int pinB;
  int dir;
  int power;
  unsigned long stopAtMs;
  unsigned long kickUntil;
};

Motor leftMotor = {LEFT_A, LEFT_B, 0, 0, 0, 0};
Motor rightMotor = {RIGHT_A, RIGHT_B, 0, 0, 0, 0};

int mastDir = 0;
int mastPulseUs = 0;
unsigned long mastStopAt = 0;
unsigned long nextServoPulseAt = 0;

void setup() {
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  pinMode(MAST_PIN, OUTPUT);

  stopAll();
  setMast(0, 0);

  Monitor.begin();
  Bridge.begin();

  Bridge.provide_safe("drive", drive);
  Bridge.provide_safe("mast", mastControl);

  Monitor.println("Bun bridge tread + MG995 mast servo ready.");
}

void loop() {
  unsigned long now = millis();

  checkTimedStop(leftMotor, now);
  checkTimedStop(rightMotor, now);
  checkMastTimedStop(now);

  serviceMotor(leftMotor);
  serviceMotor(rightMotor);
  serviceMastServo();
}

void mastControl(int dirCode, int durationMs) {
  if (dirCode != 1 && dirCode != -1 && dirCode != 0) {
    Monitor.println("Bad mast dirCode.");
    return;
  }

  if (durationMs < 0) {
    durationMs = 0;
  }

  Monitor.print("mast dir=");
  Monitor.print(dirCode);
  Monitor.print(" durationMs=");
  Monitor.println(durationMs);

  setMast(dirCode, durationMs);
}

void setMast(int dirCode, int durationMs) {
  mastDir = dirCode;

  if (dirCode == 1) {
    mastPulseUs = servoAngleToPulseUs(MAST_UP_ANGLE);
  } else if (dirCode == -1) {
    mastPulseUs = servoAngleToPulseUs(MAST_DOWN_ANGLE);
  } else {
    mastPulseUs = servoAngleToPulseUs(MAST_STOP_ANGLE);
  }

  if (durationMs > 0 && dirCode != 0) {
    mastStopAt = millis() + (unsigned long)durationMs;
  } else {
    mastStopAt = 0;
  }
}

int servoAngleToPulseUs(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
}

void checkMastTimedStop(unsigned long now) {
  if (mastStopAt != 0 && timeReached(now, mastStopAt)) {
    setMast(0, 0);
  }
}

void serviceMastServo() {
  unsigned long now = micros();
  if (nextServoPulseAt != 0 && !timeReached(now, nextServoPulseAt)) {
    return;
  }

  digitalWrite(MAST_PIN, HIGH);
  delayMicroseconds(mastPulseUs);
  digitalWrite(MAST_PIN, LOW);
  nextServoPulseAt = micros() + SERVO_PERIOD_US;
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
  driveMotorRaw(m, phase < onTime);
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
