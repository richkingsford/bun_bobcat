/*
 * sketch.ino — Bun dumb serial motor receiver (Arduino Uno Q).
 *
 * The Uno Q is now a pure motor driver. All control logic (vision, PD,
 * trajectory) runs on the host. Host streams ASCII frames over USB serial
 * at 115200 baud:
 *
 *     <L,R>\n
 *
 * where L and R are signed decimal integers in [-100, 100]:
 *     > 0  -> forward
 *     < 0  -> reverse
 *     == 0 -> coast (both half-bridges off / high-Z)
 *
 * Magnitude is sign-mapped here per spec (DRV8912 bit patterns are
 * direction-only). If proportional speed is wanted later, write the
 * DRV8912 PWM_DUTY registers before each OPx update.
 *
 * Wiring & SPI configuration are unchanged from the verified wiggle test:
 *
 *     CS_PIN = D10  -> DRV8912 nSCS
 *     D13          -> SCLK
 *     D11          -> SDI / MOSI
 *     D12          -> SDO / MISO
 *
 *     SPI: 1 MHz, MSBFIRST, SPI_MODE1   (safely below DRV8912's 5 MHz max)
 *     Frame: 16-bit, 2 us idle between frames
 *     write: bit14=0, addr in bits 13:8, data in bits 7:0
 *     read : bit14=1, addr in bits 13:8
 *
 * Motor assignment:
 *     Motor 1 = LEFT  -> OUT1/OUT2 = HB1/HB2 (REG_OP1, bits 3:0)
 *     Motor 2 = RIGHT -> OUT3/OUT4 = HB3/HB4 (REG_OP1, bits 7:4)
 *
 * Because both motors share REG_OP1, we cache the byte and only write when
 * the combined state actually changes — keeps the SPI bus quiet at idle.
 *
 * Watchdog: if no valid <L,R> frame arrives for CMD_TIMEOUT_MS, coast both
 * motors. Prevents runaway if USB drops or the host crashes.
 */

#include <SPI.h>

// ---------- Wiring (unchanged from wiggle-test sketch) -------------------
const int      CS_PIN     = 10;
const uint32_t DRV_SPI_HZ = 1000000;   // 1 MHz

// ---------- DRV8912 register map (subset) --------------------------------
const uint8_t REG_IC_STAT = 0x00;
const uint8_t REG_CONFIG  = 0x07;
const uint8_t REG_OP1     = 0x08;      // HB1-HB4 (Motors 1 & 2)
const uint8_t REG_OP2     = 0x09;
const uint8_t REG_OP3     = 0x0A;

// ---------- Motor command bit patterns within REG_OP1 --------------------
//   Motor 1 forward = 0x06 (HB1 high-side, HB2 low-side)
//   Motor 1 reverse = 0x09 (HB1 low-side,  HB2 high-side)
//   Motor 2 forward = 0x60 (HB3 high-side, HB4 low-side)
//   Motor 2 reverse = 0x90 (HB3 low-side,  HB4 high-side)
//   coast           = 0x00
const uint8_t LEFT_COAST  = 0x00;
const uint8_t LEFT_FWD    = 0x06;
const uint8_t LEFT_REV    = 0x09;
const uint8_t RIGHT_COAST = 0x00;
const uint8_t RIGHT_FWD   = 0x60;
const uint8_t RIGHT_REV   = 0x90;

// ---------- Serial / parser ---------------------------------------------
const uint32_t SERIAL_BAUD    = 115200;
const uint32_t CMD_TIMEOUT_MS = 500;   // watchdog: coast if host goes silent

const size_t RX_BUF_MAX = 32;
char     rxBuf[RX_BUF_MAX];
size_t   rxLen = 0;
bool     inFrame = false;
uint32_t lastCmdMs = 0;

// 0xFF is impossible (would forward+reverse both motors) -> forces first write
uint8_t op1Cache = 0xFF;


// ------------------------------------------------------------------------
// DRV8912 SPI helpers (bit-identical to the verified wiggle test)
// ------------------------------------------------------------------------
uint16_t drvTransfer(uint16_t frame) {
  SPI.beginTransaction(SPISettings(DRV_SPI_HZ, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_PIN, LOW);
  uint8_t hi = SPI.transfer((uint8_t)(frame >> 8));
  uint8_t lo = SPI.transfer((uint8_t)(frame & 0xFF));
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(2);                // DRV8912 CS-high min between frames
  return ((uint16_t)hi << 8) | lo;
}

uint16_t writeReg(uint8_t addr, uint8_t data) {
  return drvTransfer(((uint16_t)(addr & 0x3F) << 8) | data);
}

uint16_t readReg(uint8_t addr) {
  return drvTransfer(0x4000 | ((uint16_t)(addr & 0x3F) << 8));
}

void clearFaults() {
  writeReg(REG_CONFIG, 0x01);          // CLR_FLT bit
  delay(5);
}

void allCoast() {
  writeReg(REG_OP1, 0x00);
  writeReg(REG_OP2, 0x00);
  writeReg(REG_OP3, 0x00);
  op1Cache = 0x00;
}


// ------------------------------------------------------------------------
// Sign -> bit-pattern mapping (sign-only, per spec)
// ------------------------------------------------------------------------
static inline uint8_t leftBits(int speed) {
  if (speed > 0) return LEFT_FWD;
  if (speed < 0) return LEFT_REV;
  return LEFT_COAST;
}

static inline uint8_t rightBits(int speed) {
  if (speed > 0) return RIGHT_FWD;
  if (speed < 0) return RIGHT_REV;
  return RIGHT_COAST;
}

void applyCmd(int leftSpeed, int rightSpeed) {
  uint8_t op1 = leftBits(leftSpeed) | rightBits(rightSpeed);
  if (op1 != op1Cache) {
    writeReg(REG_OP1, op1);
    op1Cache = op1;
  }
}


// ------------------------------------------------------------------------
// Frame parser
//
// '<' starts a frame, '>' ends it. Whitespace outside frames is ignored.
// Inside a frame: split on ',' and atoi() the two halves.
// ------------------------------------------------------------------------
void handleFrame(char *body) {
  char *comma = strchr(body, ',');
  if (!comma) {
    Serial.println("E:nocomma");
    return;
  }
  *comma = '\0';
  int l = atoi(body);
  int r = atoi(comma + 1);

  // Clamp; runaway parser shouldn't push absurd values downstream.
  if (l >  100) l =  100;
  if (l < -100) l = -100;
  if (r >  100) r =  100;
  if (r < -100) r = -100;

  applyCmd(l, r);
  lastCmdMs = millis();
}

void serialPoll() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '<') {                    // frame start (resets buffer)
      inFrame = true;
      rxLen = 0;
      continue;
    }

    if (!inFrame) {
      continue;                        // discard noise outside frames
    }

    if (c == '>') {                    // frame end
      inFrame = false;
      if (rxLen < RX_BUF_MAX) {
        rxBuf[rxLen] = '\0';
        handleFrame(rxBuf);
      } else {
        Serial.println("E:overflow");
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
      Serial.println("E:overflow");
    }
  }
}


// ------------------------------------------------------------------------
// Watchdog: coast both motors if the host stops talking for CMD_TIMEOUT_MS.
// ------------------------------------------------------------------------
void watchdog() {
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    if (op1Cache != 0x00) {
      writeReg(REG_OP1, 0x00);
      op1Cache = 0x00;
    }
  }
}


// ------------------------------------------------------------------------
// Arduino entry points
// ------------------------------------------------------------------------
void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  Serial.begin(SERIAL_BAUD);

  delay(500);                          // let DRV8912 settle
  allCoast();
  clearFaults();

  uint16_t configResp = readReg(REG_CONFIG);
  Serial.print("CONFIG read: 0x");
  Serial.println(configResp, HEX);
  if ((configResp & 0xC000) != 0xC000) {
    Serial.println("WARN: SPI response invalid. Check SDO/MISO, SCLK, "
                   "SDI/MOSI, CS, GND.");
  }

  lastCmdMs = millis();
  Serial.println("Bun dumb-receiver ready. Send <L,R> at 115200.");
}

void loop() {
  serialPoll();
  watchdog();
}
