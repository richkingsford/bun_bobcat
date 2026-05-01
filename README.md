#include <SPI.h>
#include <Arduino_RouterBridge.h>

// Your wiring:
const int CS_PIN = 10;   // UNO Q D10 -> DRV8912 J8 pin 3 nSCS
// D13 -> SCLK
// D11 -> SDI / MOSI
// D12 -> SDO / MISO

const uint32_t DRV_SPI_HZ = 1000000; // 1 MHz, safely below DRV8912 5 MHz max

// DRV8912 registers
const uint8_t REG_IC_STAT = 0x00;
const uint8_t REG_CONFIG  = 0x07;
const uint8_t REG_OP1     = 0x08; // HB1-HB4
const uint8_t REG_OP2     = 0x09; // HB5-HB8
const uint8_t REG_OP3     = 0x0A; // HB9-HB12

uint16_t drvTransfer(uint16_t frame) {
  SPI.beginTransaction(SPISettings(DRV_SPI_HZ, MSBFIRST, SPI_MODE1));

  digitalWrite(CS_PIN, LOW);
  uint8_t hi = SPI.transfer((uint8_t)(frame >> 8));
  uint8_t lo = SPI.transfer((uint8_t)(frame & 0xFF));
  digitalWrite(CS_PIN, HIGH);

  SPI.endTransaction();

  delayMicroseconds(2); // DRV8912 needs CS high time between 16-bit frames
  return ((uint16_t)hi << 8) | lo;
}

// DRV8912 format:
// write: bit14 = 0, address in bits 13:8, data in bits 7:0
// read:  bit14 = 1, address in bits 13:8
uint16_t writeReg(uint8_t addr, uint8_t data) {
  return drvTransfer(((uint16_t)(addr & 0x3F) << 8) | data);
}

uint16_t readReg(uint8_t addr) {
  return drvTransfer(0x4000 | ((uint16_t)(addr & 0x3F) << 8));
}

void clearFaults() {
  writeReg(REG_CONFIG, 0x01); // CLR_FLT bit
  delay(5);
}

void allCoast() {
  writeReg(REG_OP1, 0x00);
  writeReg(REG_OP2, 0x00);
  writeReg(REG_OP3, 0x00);
}

void printResponse(const char *label, uint16_t r) {
  Monitor.print(label);
  Monitor.print(" response: status=0x");
  Monitor.print((r >> 8) & 0xFF, HEX);
  Monitor.print(" data=0x");
  Monitor.println(r & 0xFF, HEX);
}

// Motor 1 = OUT1/OUT2 = HB1/HB2, controlled by OP_CTRL_1
void motor1Forward() { writeReg(REG_OP1, 0x06); } // HB1 high-side + HB2 low-side
void motor1Reverse() { writeReg(REG_OP1, 0x09); } // HB1 low-side  + HB2 high-side

// Motor 2 = OUT3/OUT4 = HB3/HB4, controlled by OP_CTRL_1
void motor2Forward() { writeReg(REG_OP1, 0x60); } // HB3 high-side + HB4 low-side
void motor2Reverse() { writeReg(REG_OP1, 0x90); } // HB3 low-side  + HB4 high-side

// Motor 3 = OUT5/OUT6 = HB5/HB6, controlled by OP_CTRL_2
void motor3Forward() { writeReg(REG_OP2, 0x06); } // HB5 high-side + HB6 low-side
void motor3Reverse() { writeReg(REG_OP2, 0x09); } // HB5 low-side  + HB6 high-side

void wiggleMotor(const char *name, void (*forwardFn)(), void (*reverseFn)()) {
  Monitor.print("Wiggle ");
  Monitor.println(name);

  clearFaults();
  allCoast();
  delay(300);

  forwardFn();
  delay(250);

  allCoast();
  delay(400);

  reverseFn();
  delay(250);

  allCoast();
  delay(800);
}

void setup() {
  Bridge.begin();
  Monitor.begin();

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  delay(500);

  Monitor.println("DRV8912 3-motor wiggle test starting...");

  allCoast();
  clearFaults();

  uint16_t configResp = readReg(REG_CONFIG);
  printResponse("CONFIG read", configResp);

  if ((configResp & 0xC000) != 0xC000) {
    Monitor.println("WARNING: SPI response does not look valid. Check SDO/MISO, SCLK, SDI/MOSI, CS, GND.");
  }

  delay(1000);
}

void loop() {
  wiggleMotor("Motor 1 OUT1/OUT2", motor1Forward, motor1Reverse);
  wiggleMotor("Motor 2 OUT3/OUT4", motor2Forward, motor2Reverse);
  wiggleMotor("Motor 3 OUT5/OUT6", motor3Forward, motor3Reverse);

  Monitor.println("Cycle complete. Waiting...");
  delay(3000);
}