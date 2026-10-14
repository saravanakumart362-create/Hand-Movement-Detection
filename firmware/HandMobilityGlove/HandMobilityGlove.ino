/*
  Hand Mobility Assessment Glove
  Arduino-based real-time hand movement detection and alert system for coma patients.

  What it does
    Reads an MPU-6050 IMU on the back of the hand 50 times a second over I2C,
    works out how much the hand is moving and how it is tilted, and sounds a
    buzzer and lights an LED when the movement stays above a threshold.
    Each movement is labelled short, long or shaking, and small or big.
    A big movement beeps faster than a small one.

  Parts
    Arduino Uno or Nano (ATmega328P)
    MPU-6050 breakout (GY-521) on the back of the hand, I2C address 0x68
    Optional second MPU-6050 on a finger, with AD0 tied to 3.3 V, address 0x69
    Active buzzer, LED with a 220 ohm resistor, push button (optional)

  Wiring
    MPU-6050 VCC -> 5V    (the GY-521 board has its own 3.3 V regulator)
    MPU-6050 GND -> GND
    MPU-6050 SDA -> A4
    MPU-6050 SCL -> A5
    Buzzer +     -> D8    Buzzer - -> GND
    LED anode    -> D13 through 220 ohm, cathode -> GND (the on-board LED also lights)
    Button       -> between D2 and GND (caregiver button, optional)

  Serial, 115200 baud
    One line per sample, readable by the Serial Plotter and by the demo page (index.html):
      ax:..,ay:..,az:..,gx:..,gy:..,gz:..,am:..,gm:..,roll:..,pitch:..,st:..,ty:..,lv:..
    ax ay az    acceleration in g            gx gy gz  rotation in deg/s, bias removed
    am          acceleration change in g     gm        rotation rate in deg/s
    roll pitch  tilt of the hand in degrees
    st          0 monitoring, 1 alert, 2 paused for care, 3 calibrating, 4 sensor fault
    ty          movement type during an alert: 1 short, 2 long, 3 shaking (0 = no alert)
    lv          movement strength during an alert: 1 small, 2 big (0 = no alert)
    Lines that start with # are messages.

    Commands, one per line:
      A0.030  acceleration-change threshold in g
      G3.0    rotation threshold in deg/s
      C3      readings in a row needed to confirm movement
      H3000   how long the alert stays on after movement stops, in ms
      P30000  length of the care pause, in ms
      K       same as pressing the caregiver button
      R       recalibrate
      S       save the settings now (they are also saved 2 s after the last change)
      D       go back to the default settings below
      ?       print the current settings
*/

#include <Wire.h>
#include <EEPROM.h>

// ------------------------------- Pins -------------------------------
const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_LED    = 13;
const uint8_t PIN_BUTTON = 2;

// Set to 1 for a passive piezo, which needs a tone rather than a steady HIGH.
#define BUZZER_IS_PASSIVE 0
const unsigned int BUZZER_TONE_HZ = 2000;

// ------------------------ Detection settings ------------------------
// The values a new glove starts with. Tune them in the demo page, then paste the lines it
// shows here. Settings changed from the demo page or by command are saved in the Arduino's
// memory and used instead of these, until these lines are edited and uploaded again.
const float    DEFAULT_ACCEL_THRESHOLD_G  = 0.030f;   // acceleration change that counts as movement
const float    DEFAULT_GYRO_THRESHOLD_DPS = 3.0f;     // rotation rate that counts as movement
const uint8_t  DEFAULT_CONFIRM_SAMPLES    = 3;        // readings in a row above a threshold (3 = 60 ms)
const uint16_t DEFAULT_ALERT_HOLD_MS      = 3000;     // alert stays on this long after movement stops
const uint32_t DEFAULT_CARE_PAUSE_MS      = 30000UL;  // caregiver button pauses monitoring for this long

float    accelThresholdG  = DEFAULT_ACCEL_THRESHOLD_G;
float    gyroThresholdDps = DEFAULT_GYRO_THRESHOLD_DPS;
uint8_t  confirmSamples   = DEFAULT_CONFIRM_SAMPLES;
uint16_t alertHoldMs      = DEFAULT_ALERT_HOLD_MS;
uint32_t carePauseMs      = DEFAULT_CARE_PAUSE_MS;

// ------------------- Movement type and strength -------------------
const uint16_t LONG_MOVEMENT_MS   = 500;     // movement that lasts this long is "long"
const uint16_t SHAKING_MIN_MS     = 1000;    // shaking lasts at least this long...
const uint8_t  SHAKING_REVERSALS  = 6;       // ...and changes direction at least this many times
const float    BIG_MOVEMENT_RATIO = 4.0f;    // a peak of 4 times a threshold or more is "big"

const uint8_t TYPE_NONE = 0, TYPE_SHORT = 1, TYPE_LONG = 2, TYPE_SHAKING = 3;
const uint8_t LEVEL_NONE = 0, LEVEL_SMALL = 1, LEVEL_BIG = 2;

// ---------------------- Sampling and filtering ----------------------
const uint16_t SAMPLE_PERIOD_MS = 20;             // 50 Hz
const float    GRAVITY_ALPHA    = 0.04f;          // slow low-pass that follows gravity
const float    SMOOTH_ALPHA     = 0.25f;          // fast low-pass on the two motion values
const uint8_t  CALIBRATION_SAMPLES = 100;         // 2 s with the hand still
const float    CALIBRATION_MAX_SPREAD_DPS = 4.0f; // more gyro spread than this means the hand moved
const uint8_t  MAX_READ_ERRORS  = 10;             // failed reads in a row before a sensor fault

// ----------------------------- MPU-6050 -----------------------------
const uint8_t MPU_ADDR_HAND    = 0x68;
const uint8_t MPU_ADDR_FINGER  = 0x69;
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;
const float   ACCEL_LSB_PER_G  = 16384.0f;        // at +/-2 g
const float   GYRO_LSB_PER_DPS = 131.0f;          // at +/-250 deg/s

const uint8_t STATE_MONITORING  = 0;
const uint8_t STATE_ALERT       = 1;
const uint8_t STATE_PAUSED      = 2;
const uint8_t STATE_CALIBRATING = 3;
const uint8_t STATE_FAULT       = 4;

struct Imu {
  uint8_t address;
  bool    present;
  uint8_t readErrors;
  float   accel[3];       // g
  float   gyroRaw[3];     // deg/s as read
  float   gyro[3];        // deg/s with the bias removed
  float   gyroBias[3];
  float   gravity[3];     // slow-moving estimate of where gravity points
  float   accelMotion;    // smoothed acceleration change, g
  float   gyroMotion;     // smoothed rotation rate, deg/s
  int8_t  lastSign[6];    // last clear direction on each axis: 3 acceleration, 3 rotation
  uint8_t reversals[6];   // direction changes on each axis during the current alert
  float   sum[6];         // calibration running totals
  float   gyroMin[3];
  float   gyroMax[3];
};

// The settings as they are kept in the Arduino's permanent memory (EEPROM).
struct SavedSettings {
  uint16_t magic;         // marks a block this sketch wrote
  uint16_t stamp;         // fingerprint of the defaults above when it was saved
  float    accelThresholdG;
  float    gyroThresholdDps;
  uint8_t  confirmSamples;
  uint16_t alertHoldMs;
  uint32_t carePauseMs;
};
const uint16_t SETTINGS_MAGIC = 0x4847;
const uint16_t SETTINGS_SAVE_DELAY_MS = 2000;     // wait for changes to stop before writing

Imu imus[2];
const uint8_t IMU_COUNT = 2;
bool fingerFitted = false;

uint8_t  state = STATE_CALIBRATING;
uint8_t  calibrationCount = 0;
uint8_t  confirmCount = 0;
float    accelMotion = 0;     // the larger of the sensors' values
float    gyroMotion = 0;
float    rollDeg = 0;         // tilt of the hand
float    pitchDeg = 0;
uint16_t eventCount = 0;
uint8_t  eventType = TYPE_NONE;
uint8_t  eventLevel = LEVEL_NONE;
float    eventPeakAccel = 0;
float    eventPeakGyro = 0;
bool     settingsDirty = false;
uint32_t settingsChangedMs = 0;
uint32_t lastSampleMs = 0;
uint32_t lastMoveMs = 0;
uint32_t alertStartMs = 0;
uint32_t pauseStartMs = 0;
uint32_t lastRetryMs = 0;

// ------------------------------- I2C --------------------------------
bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, length) != length) return false;
  for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

// Wakes the sensor and sets its ranges. Returns false if nothing answers at that address.
bool imuBegin(Imu &imu) {
  uint8_t id;
  if (!readRegisters(imu.address, REG_WHO_AM_I, &id, 1)) return false;
  if (!writeRegister(imu.address, REG_PWR_MGMT_1, 0x01)) return false;   // wake, clock from the gyro
  delay(20);
  bool ok = writeRegister(imu.address, REG_SMPLRT_DIV, 19);              // 1 kHz / (1 + 19) = 50 Hz
  ok = writeRegister(imu.address, REG_CONFIG, 0x04) && ok;               // built-in low-pass, about 20 Hz
  ok = writeRegister(imu.address, REG_GYRO_CONFIG, 0x00) && ok;          // +/-250 deg/s
  ok = writeRegister(imu.address, REG_ACCEL_CONFIG, 0x00) && ok;         // +/-2 g
  imu.readErrors = 0;
  return ok;
}

// Reads acceleration and rotation in one 14-byte burst (the two temperature bytes are skipped).
bool imuRead(Imu &imu) {
  uint8_t raw[14];
  if (!readRegisters(imu.address, REG_ACCEL_XOUT_H, raw, 14)) {
    if (imu.readErrors < 255) imu.readErrors++;
    return false;
  }
  imu.readErrors = 0;
  for (uint8_t i = 0; i < 3; i++) {
    int16_t a = (int16_t)(((uint16_t)raw[2 * i] << 8) | raw[2 * i + 1]);
    int16_t g = (int16_t)(((uint16_t)raw[8 + 2 * i] << 8) | raw[9 + 2 * i]);
    imu.accel[i]   = a / ACCEL_LSB_PER_G;
    imu.gyroRaw[i] = g / GYRO_LSB_PER_DPS;
    imu.gyro[i]    = imu.gyroRaw[i] - imu.gyroBias[i];
  }
  return true;
}

// ----------------------------- Settings -----------------------------
// Changes whenever the default values at the top are edited, so that an uploaded sketch
// with new defaults is not overridden by settings saved under the old ones.
uint16_t defaultsStamp() {
  uint32_t mix = (uint32_t)(DEFAULT_ACCEL_THRESHOLD_G * 10000.0f) * 31UL
               + (uint32_t)(DEFAULT_GYRO_THRESHOLD_DPS * 100.0f) * 17UL
               + DEFAULT_CONFIRM_SAMPLES * 7UL
               + DEFAULT_ALERT_HOLD_MS * 3UL
               + DEFAULT_CARE_PAUSE_MS;
  return (uint16_t)(mix ^ (mix >> 16));
}

void useDefaultSettings() {
  accelThresholdG  = DEFAULT_ACCEL_THRESHOLD_G;
  gyroThresholdDps = DEFAULT_GYRO_THRESHOLD_DPS;
  confirmSamples   = DEFAULT_CONFIRM_SAMPLES;
  alertHoldMs      = DEFAULT_ALERT_HOLD_MS;
  carePauseMs      = DEFAULT_CARE_PAUSE_MS;
}

// Reads the saved settings, and keeps them only if this sketch wrote them and they make sense.
void loadSettings() {
  SavedSettings saved;
  EEPROM.get(0, saved);
  bool usable = saved.magic == SETTINGS_MAGIC && saved.stamp == defaultsStamp()
    && saved.accelThresholdG >= 0.005f && saved.accelThresholdG <= 1.0f
    && saved.gyroThresholdDps >= 0.5f && saved.gyroThresholdDps <= 100.0f
    && saved.confirmSamples >= 1 && saved.confirmSamples <= 50
    && saved.alertHoldMs >= 500 && saved.alertHoldMs <= 60000U
    && saved.carePauseMs >= 1000UL && saved.carePauseMs <= 600000UL;
  if (!usable) {
    Serial.println(F("# settings: using the defaults in the sketch"));
    return;
  }
  accelThresholdG  = saved.accelThresholdG;
  gyroThresholdDps = saved.gyroThresholdDps;
  confirmSamples   = saved.confirmSamples;
  alertHoldMs      = saved.alertHoldMs;
  carePauseMs      = saved.carePauseMs;
  Serial.println(F("# settings: loaded from memory"));
}

// EEPROM.put only rewrites the bytes that changed, which spares the memory.
void saveSettings() {
  SavedSettings saved;
  saved.magic = SETTINGS_MAGIC;
  saved.stamp = defaultsStamp();
  saved.accelThresholdG  = accelThresholdG;
  saved.gyroThresholdDps = gyroThresholdDps;
  saved.confirmSamples   = confirmSamples;
  saved.alertHoldMs      = alertHoldMs;
  saved.carePauseMs      = carePauseMs;
  EEPROM.put(0, saved);
  settingsDirty = false;
  Serial.println(F("# settings saved"));
}

// ---------------------------- Calibration ----------------------------
// With the hand still, the average gyro reading is its bias and the average
// accelerometer reading is gravity.
void startCalibration() {
  state = STATE_CALIBRATING;
  calibrationCount = 0;
  confirmCount = 0;
  accelMotion = 0;
  gyroMotion = 0;
  eventType = TYPE_NONE;
  eventLevel = LEVEL_NONE;
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    Imu &imu = imus[n];
    imu.accelMotion = 0;
    imu.gyroMotion = 0;
    for (uint8_t i = 0; i < 6; i++) imu.sum[i] = 0;
    for (uint8_t i = 0; i < 3; i++) { imu.gyroMin[i] = 1e9f; imu.gyroMax[i] = -1e9f; }
  }
  Serial.println(F("# calibrating: keep the hand still"));
}

void calibrationStep() {
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    Imu &imu = imus[n];
    if (!imu.present) continue;
    for (uint8_t i = 0; i < 3; i++) {
      imu.sum[i]     += imu.accel[i];
      imu.sum[3 + i] += imu.gyroRaw[i];
      imu.gyroMin[i] = min(imu.gyroMin[i], imu.gyroRaw[i]);
      imu.gyroMax[i] = max(imu.gyroMax[i], imu.gyroRaw[i]);
      // Follow gravity from the first reading, so the tilt angles are right while calibrating.
      if (calibrationCount == 0) imu.gravity[i] = imu.accel[i];
      else imu.gravity[i] += GRAVITY_ALPHA * (imu.accel[i] - imu.gravity[i]);
    }
  }
  if (++calibrationCount < CALIBRATION_SAMPLES) return;

  float spread = 0;
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    if (!imus[n].present) continue;
    for (uint8_t i = 0; i < 3; i++) spread = max(spread, imus[n].gyroMax[i] - imus[n].gyroMin[i]);
  }
  if (spread > CALIBRATION_MAX_SPREAD_DPS) {
    Serial.println(F("# the hand moved, calibrating again"));
    startCalibration();
    return;
  }
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    Imu &imu = imus[n];
    if (!imu.present) continue;
    for (uint8_t i = 0; i < 3; i++) {
      imu.gravity[i]  = imu.sum[i] / CALIBRATION_SAMPLES;
      imu.gyroBias[i] = imu.sum[3 + i] / CALIBRATION_SAMPLES;
    }
  }
  state = STATE_MONITORING;
  Serial.println(F("# ready"));
}

// ----------------------------- Detection -----------------------------
// Counts a change of direction on one axis. Values inside the deadband are too small to be
// movement and are ignored, so sensor noise around zero does not count.
void countReversal(Imu &imu, uint8_t channel, float value, float deadband) {
  int8_t sign = value > deadband ? 1 : (value < -deadband ? -1 : 0);
  if (sign == 0) return;
  if (imu.lastSign[channel] != 0 && sign != imu.lastSign[channel] && imu.reversals[channel] < 255) {
    imu.reversals[channel]++;
  }
  imu.lastSign[channel] = sign;
}

// Acceleration change: how far the reading is from the gravity estimate. A hand that is
// tilted but still reads zero, because the estimate catches up with the new tilt.
// Rotation rate: the size of the gyro reading across all three axes.
void updateMotion(Imu &imu) {
  float change[3];
  for (uint8_t i = 0; i < 3; i++) change[i] = imu.accel[i] - imu.gravity[i];
  float size = sqrt(change[0] * change[0] + change[1] * change[1] + change[2] * change[2]);
  for (uint8_t i = 0; i < 3; i++) imu.gravity[i] += GRAVITY_ALPHA * change[i];
  float rotation = sqrt(imu.gyro[0] * imu.gyro[0] + imu.gyro[1] * imu.gyro[1] + imu.gyro[2] * imu.gyro[2]);
  imu.accelMotion += SMOOTH_ALPHA * (size - imu.accelMotion);
  imu.gyroMotion  += SMOOTH_ALPHA * (rotation - imu.gyroMotion);
  for (uint8_t i = 0; i < 3; i++) {
    countReversal(imu, i, change[i], accelThresholdG * 0.5f);
    countReversal(imu, 3 + i, imu.gyro[i], gyroThresholdDps * 0.5f);
  }
}

// Roll and pitch of the hand, in degrees, from where gravity points on the hand sensor.
// Roll is the tilt to the side; pitch is the fingers lifting or dropping.
void updateTilt() {
  const float *g = imus[0].gravity;
  rollDeg  = atan2(-g[0], g[2]) * RAD_TO_DEG;
  pitchDeg = atan2(g[1], sqrt(g[0] * g[0] + g[2] * g[2])) * RAD_TO_DEG;
}

void startEvent(uint32_t now) {
  state = STATE_ALERT;
  alertStartMs = now;
  eventCount++;
  eventType = TYPE_SHORT;
  eventLevel = LEVEL_SMALL;
  eventPeakAccel = 0;
  eventPeakGyro = 0;
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    for (uint8_t c = 0; c < 6; c++) { imus[n].lastSign[c] = 0; imus[n].reversals[c] = 0; }
  }
  Serial.print(F("# movement event "));
  Serial.println(eventCount);
}

// Labels the movement so far. Short: under half a second. Long: half a second or more.
// Shaking: at least a second, with the direction changing 6 times or more.
// Big: either value reached 4 times its threshold. A label only ever moves up.
void updateEvent() {
  eventPeakAccel = max(eventPeakAccel, accelMotion);
  eventPeakGyro  = max(eventPeakGyro, gyroMotion);
  float ratio = max(accelMotion / accelThresholdG, gyroMotion / gyroThresholdDps);
  if (ratio >= BIG_MOVEMENT_RATIO) eventLevel = LEVEL_BIG;

  uint8_t reversals = 0;
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    if (!imus[n].present) continue;
    for (uint8_t c = 0; c < 6; c++) reversals = max(reversals, imus[n].reversals[c]);
  }
  uint32_t lasted = lastMoveMs - alertStartMs;
  if (reversals >= SHAKING_REVERSALS && lasted >= SHAKING_MIN_MS) eventType = TYPE_SHAKING;
  else if (lasted >= LONG_MOVEMENT_MS) eventType = TYPE_LONG;
  else eventType = TYPE_SHORT;
}

// Prints a one-line summary of the alert that just ended, then clears its labels.
void endEvent() {
  Serial.print(F("# event "));
  Serial.print(eventCount);
  Serial.print(F(" ended: "));
  Serial.print(eventType == TYPE_SHAKING ? F("shaking") : eventType == TYPE_LONG ? F("long movement") : F("short movement"));
  Serial.print(eventLevel == LEVEL_BIG ? F(", big, lasted ") : F(", small, lasted "));
  Serial.print((lastMoveMs - alertStartMs) / 1000.0f, 1);
  Serial.print(F(" s, peaks "));
  Serial.print(eventPeakAccel, 3);
  Serial.print(F(" g and "));
  Serial.print(eventPeakGyro, 1);
  Serial.println(F(" deg/s"));
  eventType = TYPE_NONE;
  eventLevel = LEVEL_NONE;
}

void detect(uint32_t now) {
  accelMotion = 0;
  gyroMotion = 0;
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    if (!imus[n].present) continue;
    updateMotion(imus[n]);
    accelMotion = max(accelMotion, imus[n].accelMotion);
    gyroMotion  = max(gyroMotion, imus[n].gyroMotion);
  }

  bool over = accelMotion > accelThresholdG || gyroMotion > gyroThresholdDps;
  if (!over) confirmCount = 0;
  else if (confirmCount < 255) confirmCount++;
  bool moving = confirmCount >= confirmSamples;

  if (state == STATE_PAUSED) {
    if (now - pauseStartMs >= carePauseMs) {
      state = STATE_MONITORING;
      Serial.println(F("# monitoring resumed"));
    }
  } else if (moving) {
    lastMoveMs = now;
    if (state != STATE_ALERT) startEvent(now);
  } else if (state == STATE_ALERT && now - lastMoveMs >= alertHoldMs) {
    endEvent();
    state = STATE_MONITORING;
  }
  if (state == STATE_ALERT) updateEvent();
}

// The caregiver button silences an alert and pauses monitoring; pressed again, it resumes.
void caregiverButton(uint32_t now) {
  if (state == STATE_CALIBRATING || state == STATE_FAULT) return;
  if (state == STATE_PAUSED) {
    state = STATE_MONITORING;
    Serial.println(F("# monitoring resumed"));
  } else {
    if (state == STATE_ALERT) endEvent();
    state = STATE_PAUSED;
    pauseStartMs = now;
    Serial.println(F("# paused for care"));
  }
}

// --------------------------- Sensor faults ---------------------------
void enterFault() {
  if (state == STATE_ALERT) endEvent();
  state = STATE_FAULT;
  lastRetryMs = millis();
  Serial.println(F("# sensor fault: check the IMU wiring (VCC, GND, SDA to A4, SCL to A5)"));
  Serial.println(F("st:4"));
}

void recoverFromFault(uint32_t now) {
  if (now - lastRetryMs < 1000) return;
  lastRetryMs = now;
  bool ok = imuBegin(imus[0]);
  if (ok && fingerFitted) ok = imuBegin(imus[1]);
  if (!ok) {
    Serial.println(F("st:4"));
    return;
  }
  imus[0].present = true;
  imus[1].present = fingerFitted;
  Serial.println(F("# sensor answering again"));
  startCalibration();
}

// ------------------------------ Outputs ------------------------------
void setBuzzer(bool on) {
  static bool isOn = false;
  if (on == isOn) return;
  isOn = on;
#if BUZZER_IS_PASSIVE
  if (on) tone(PIN_BUZZER, BUZZER_TONE_HZ); else noTone(PIN_BUZZER);
#else
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
#endif
}

void driveOutputs(uint32_t now) {
  bool led = false, buzzer = false;
  switch (state) {
    case STATE_ALERT:                         // LED on; slow beeps for small, fast beeps for big
      led = true;
      if (eventLevel == LEVEL_BIG) buzzer = (now - alertStartMs) % 250UL < 125UL;
      else buzzer = (now - alertStartMs) % 800UL < 150UL;
      break;
    case STATE_CALIBRATING:                   // LED on
      led = true;
      break;
    case STATE_PAUSED:                        // short blink every 2 s
      led = now % 2000UL < 100UL;
      break;
    case STATE_FAULT:                         // fast blink, chirp every 2 s
      led = now % 200UL < 100UL;
      buzzer = now % 2000UL < 100UL;
      break;
  }
  digitalWrite(PIN_LED, led ? HIGH : LOW);
  setBuzzer(buzzer);
}

// ------------------------------ Serial -------------------------------
void printSample() {
  const Imu &hand = imus[0];
  Serial.print(F("ax:"));     Serial.print(hand.accel[0], 3);
  Serial.print(F(",ay:"));    Serial.print(hand.accel[1], 3);
  Serial.print(F(",az:"));    Serial.print(hand.accel[2], 3);
  Serial.print(F(",gx:"));    Serial.print(hand.gyro[0], 2);
  Serial.print(F(",gy:"));    Serial.print(hand.gyro[1], 2);
  Serial.print(F(",gz:"));    Serial.print(hand.gyro[2], 2);
  Serial.print(F(",am:"));    Serial.print(accelMotion, 3);
  Serial.print(F(",gm:"));    Serial.print(gyroMotion, 2);
  Serial.print(F(",roll:"));  Serial.print(rollDeg, 1);
  Serial.print(F(",pitch:")); Serial.print(pitchDeg, 1);
  Serial.print(F(",st:"));    Serial.print(state);
  Serial.print(F(",ty:"));    Serial.print(eventType);
  Serial.print(F(",lv:"));    Serial.println(eventLevel);
}

void printSettings() {
  Serial.print(F("# settings A=")); Serial.print(accelThresholdG, 3);
  Serial.print(F(" G=")); Serial.print(gyroThresholdDps, 1);
  Serial.print(F(" C=")); Serial.print(confirmSamples);
  Serial.print(F(" H=")); Serial.print(alertHoldMs);
  Serial.print(F(" P=")); Serial.println(carePauseMs);
}

void handleCommand(const char *line, uint32_t now) {
  float value = atof(line + 1);
  switch (line[0]) {
    case 'A': accelThresholdG  = constrain(value, 0.005f, 1.0f); break;
    case 'G': gyroThresholdDps = constrain(value, 0.5f, 100.0f); break;
    case 'C': confirmSamples   = (uint8_t)constrain(value, 1.0f, 50.0f); break;
    case 'H': alertHoldMs      = (uint16_t)constrain(value, 500.0f, 60000.0f); break;
    case 'P': carePauseMs      = (uint32_t)constrain(value, 1000.0f, 600000.0f); break;
    case 'D': useDefaultSettings(); break;
    case 'S': saveSettings(); return;
    case 'K': caregiverButton(now); return;
    case 'R': if (state != STATE_FAULT) startCalibration(); return;
    case '?': printSettings(); return;
    default: return;
  }
  settingsDirty = true;                       // saved once the changes have stopped for 2 s
  settingsChangedMs = now;
  printSettings();
}

void pollSerial(uint32_t now) {
  static char line[16];
  static uint8_t length = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (length > 0) {
        line[length] = '\0';
        handleCommand(line, now);
        length = 0;
      }
    } else if (length < sizeof(line) - 1) {
      line[length++] = c;
    }
  }
}

void pollButton(uint32_t now) {
  static bool lastReading = HIGH, pressed = false;
  static uint32_t changedAt = 0;
  bool reading = digitalRead(PIN_BUTTON);
  if (reading != lastReading) {
    lastReading = reading;
    changedAt = now;
  }
  if (now - changedAt < 40) return;           // wait out contact bounce
  bool isPressed = reading == LOW;
  if (isPressed && !pressed) caregiverButton(now);
  pressed = isPressed;
}

// --------------------------- Setup and loop ---------------------------
void sampleTick(uint32_t now) {
  if (state == STATE_FAULT) {
    recoverFromFault(now);
    return;
  }
  bool allRead = true;
  for (uint8_t n = 0; n < IMU_COUNT; n++) {
    if (!imus[n].present) continue;
    if (imuRead(imus[n])) continue;
    allRead = false;
    if (imus[n].readErrors >= MAX_READ_ERRORS) {
      enterFault();
      return;
    }
  }
  if (!allRead) return;                       // skip this sample rather than reuse an old reading

  if (state == STATE_CALIBRATING) calibrationStep();
  else detect(now);
  updateTilt();
  printSample();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println(F("# Hand Mobility Assessment Glove"));
  loadSettings();

  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);            // a loose wire must not hang the loop
#endif

  setBuzzer(true);                            // short beep so a dead buzzer is noticed at power-up
  delay(120);
  setBuzzer(false);

  imus[0].address = MPU_ADDR_HAND;
  imus[1].address = MPU_ADDR_FINGER;
  imus[0].present = imuBegin(imus[0]);
  fingerFitted = imuBegin(imus[1]);
  imus[1].present = fingerFitted;

  if (!imus[0].present) {
    enterFault();
  } else {
    Serial.println(fingerFitted ? F("# sensors: hand 0x68, finger 0x69") : F("# sensor: hand 0x68"));
    printSettings();
    startCalibration();
  }
  lastSampleMs = millis();
}

void loop() {
  uint32_t now = millis();
  pollSerial(now);
  pollButton(now);
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs += SAMPLE_PERIOD_MS;
    if (now - lastSampleMs >= SAMPLE_PERIOD_MS) lastSampleMs = now;   // fell behind: do not try to catch up
    sampleTick(now);
  }
  if (settingsDirty && now - settingsChangedMs >= SETTINGS_SAVE_DELAY_MS) saveSettings();
  driveOutputs(now);
}
