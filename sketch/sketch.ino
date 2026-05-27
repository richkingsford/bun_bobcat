/*
 * sketch.ino - Bun dumb serial receiver (Arduino Uno Q).
 *
 * Pure motor receiver. All control logic (vision, PD, trajectory) lives in
 * host_controller.py on the Linux host. This sketch listens on Serial at
 * 115200 baud for ASCII frames:
 *
 *     <L,R>\n
 *
 * where L and R are signed decimal integers in [-100, 100]:
 *     > 0   forward at |x|%
 *     < 0   reverse at |x|%
 *     == 0  coast
 *
 * Values outside [-100, 100] are clamped. Malformed frames are dropped
 * silently; the 500 ms watchdog will coast the motors if they keep coming.
 *
 * Motor wiring (unchanged from the App Lab import — verified working on
 * this physical Bun; do NOT swap to DRV8912 SPI, this robot uses simple
 * H-bridge digital pins):
 *
 *     Left tread:  D6 (forward) / D7 (reverse)
 *     Right tread: D8 (forward) / D9 (reverse)
 *     Mast servo:  D10 continuous servo, held at SERVO_NEUTRAL (the
 *                  serial protocol is intentionally drive-only; if mast
 *                  control returns, extend the frame to <L,R,M> rather
 *                  than re-introducing an RPC path).
 *
 * Motor control:
 *     - Software PWM at PWM_PERIOD_US (200 Hz) for proportional speed.
 *     - KICK_MS of full-power drive on direction change / startup to
 *       overcome stiction; the kick is suppressed when |speed| == 100.
 *
 * Safety:
 *     - 500 ms watchdog: if no valid frame arrives within CMD_TIMEOUT_MS,
 *       both treads coast. The mast is already held at neutral so no
 *       extra action is needed there.
 */

#include "/home/arduino/Arduino/libraries/Arduino_HardwareServo/src/Arduino_HardwareServo.h"
#include "/home/arduino/Arduino/libraries/Arduino_HardwareServo/src/HardwareServo.cpp"
#include "/home/arduino/Arduino/libraries/Arduino_HardwareServo/src/HardwareServo_zephyr.cpp"

// ---------- Wiring (matches the existing physical robot) ----------------
const int LEFT_A   = 6;
const int LEFT_B   = 7;
const int RIGHT_A  = 8;
const int RIGHT_B  = 9;
const int MAST_PIN = 10;

const int SERVO_NEUTRAL = 90;

// ---------- Motor service constants -------------------------------------
const uint32_t      CMD_TIMEOUT_MS = 500;    // watchdog window
const unsigned long PWM_PERIOD_US  = 5000;   // 200 Hz software PWM
const unsigned long KICK_MS        = 120;    // stiction-breaking kick window

// ---------- Serial / parser ---------------------------------------------
const uint32_t SERIAL_BAUD  = 115200;
const size_t   RX_BUF_MAX   = 32;

char     rxBuf[RX_BUF_MAX];
size_t   rxLen     = 0;
bool     inFrame   = false;
uint32_t lastCmdMs = 0;

HardwareServo mast;

struct Tread {
  int pinA;
  int pinB;
  int speed;
  unsigned long kickUntilMs;
};

Tread leftTread  = {LEFT_A,  LEFT_B,  0, 0};
Tread rightTread = {RIGHT_A, RIGHT_B, 0, 0};


// ------------------------------------------------------------------------
// Motor primitives (carried over from the RPC sketch; this is the part that
// is known-working on the physical robot — only the front-end changes).
// ------------------------------------------------------------------------
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
    // Kick-start: gear stiction won't yield at low duty unless we briefly
    // pin the bridge at full power. Skipped at 100% (already full).
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

  unsigned long phase  = micros() % PWM_PERIOD_US;
  unsigned long onTime = (PWM_PERIOD_US * (unsigned long)magnitude) / 100;
  driveHBridgeRaw(tread.pinA, tread.pinB, tread.speed, phase < onTime);
}

void applyDrive(int left, int right) {
  setTread(leftTread,  left);
  setTread(rightTread, right);
  // Mast is held at SERVO_NEUTRAL; nothing to update here.
}


// ------------------------------------------------------------------------
// Frame parser: '<' opens a frame, '>' closes it. Characters outside a
// frame are discarded. Inside a frame: split on the first comma, atoi
// each half.
// ------------------------------------------------------------------------
void handleFrame(char *body) {
  char *comma = strchr(body, ',');
  if (!comma) {
    return;  // malformed; drop silently, watchdog will catch sustained gaps
  }
  *comma = '\0';
  int l = clampSpeed(atoi(body));
  int r = clampSpeed(atoi(comma + 1));
  applyDrive(l, r);
  lastCmdMs = millis();
}

void serialPoll() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '<') {                  // frame start (also resets buffer)
      inFrame = true;
      rxLen   = 0;
      continue;
    }

    if (!inFrame) {
      continue;                      // noise outside frames is discarded
    }

    if (c == '>') {                  // frame end
      inFrame = false;
      if (rxLen < RX_BUF_MAX) {
        rxBuf[rxLen] = '\0';
        handleFrame(rxBuf);
      }
      rxLen = 0;
      continue;
    }

    if (rxLen < RX_BUF_MAX - 1) {
      rxBuf[rxLen++] = c;
    } else {
      // Frame too long — abort it, wait for next '<'.
      inFrame = false;
      rxLen   = 0;
    }
  }
}


// ------------------------------------------------------------------------
// Watchdog: coast both treads if no valid frame arrives within
// CMD_TIMEOUT_MS. Cheaper-than-correct path: we only re-issue a coast
// when the treads aren't already coasted.
// ------------------------------------------------------------------------
void watchdog() {
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    if (leftTread.speed != 0 || rightTread.speed != 0) {
      applyDrive(0, 0);
    }
  }
}


// ------------------------------------------------------------------------
// Arduino entry points
// ------------------------------------------------------------------------
void setup() {
  pinMode(LEFT_A,  OUTPUT);
  pinMode(LEFT_B,  OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  mast.attach(MAST_PIN);
  mast.write(SERVO_NEUTRAL);

  applyDrive(0, 0);

  Serial.begin(SERIAL_BAUD);
  lastCmdMs = millis();

  Serial.println("Bun dumb-serial receiver ready. Send <L,R> at 115200.");
}

void loop() {
  serialPoll();
  watchdog();
  serviceTread(leftTread);
  serviceTread(rightTread);
}
