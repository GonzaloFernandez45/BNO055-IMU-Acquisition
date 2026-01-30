// IMU Serial Test (Nano ESP32 + BNO055)
// Purpose: verify calibration restore/save (NVS), neutral reference (quaternion) and automatic anti-drift re-anchor.
// FINAL: roll & pitch from Euclidean (fixed mapping), yaw from yawHeading.
//  Output is via Serial & BLE.
//
// Serial output (CSV):
//   t_ms,roll,pitch,yawHeading
//
// Debug lines are prefixed with '#', so you can ignore them in a CSV plotter.
// Commands:
//   n  -> set current posture as NEUTRAL (reference)

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#include <Preferences.h> // ESP32 NVS storage (flash key-value)
#include <ArduinoBLE.h> // BLE peripheral stack

bool imuReady = false;

// -------------------- Sample rate --------------------
#define SAMPLE_DELAY_MS 20 // 50 Hz

// -------------------- Core objects --------------------
// myIMU: Adafruit driver for BNO055 (I2C)
// prefs: access to ESP32 NVS (flash) to persist offsets across reboots

Adafruit_BNO055 myIMU = Adafruit_BNO055();
Preferences prefs;

// -------------------- BLE (Nordic UART Service - NUS) --------------------
// We emulate a UART-over-BLE service:  
// UUID corresponds to the Nordic UART Service: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" it is the identifier of the BLE (UART) Service
// - TX characteristic: ESP32 -> phone (Notify)
// - RX characteristic: phone -> ESP32 (Write)
// This lets you stream CSV lines and also send simple commands like 'n'.

BLEService nusService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

// TX: ESP32 -> Phone (Notify)
BLEStringCharacteristic nusTxChar(
  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
  BLENotify,
  80
);

// RX: Phone -> ESP32 (Write)
BLECharacteristic nusRxChar(
  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
  BLEWrite,
  20
);

// -------------------- Neutral reference --------------------
// qRef is the "neutral posture" quaternion.
// We output relative orientation: qRel = qRef* (conjugate) * q.
// hasReference ensures we don't compute relative angles before setting qRef.
imu::Quaternion qRef;
bool hasReference = false;

// -------------------- Still detection (LPF + hysteresis) --------------------
// We estimate "stillness" using gyro magnitude (rotation rate).
// 1) compute gyro magnitude g = sqrt(wx^2+wy^2+wz^2)
// 2) low-pass filter it: gFilt = alpha*g + (1-alpha)*gFilt
// 3) hysteresis thresholds:
//    - enter stillness when gFilt < gyroStillEnter
//    - exit stillness when gFilt > gyroStillExit

float gFilt = 0.0f;
const float gAlpha = 0.15f;

const float gyroStillEnter = 0.25f; //enter still
const float gyroStillExit  = 0.60f; // exit still

bool isStill = false;

unsigned long stillStartMs = 0;
bool stillRunning = false;

const unsigned long stillTimeMs = 2000; // must be still for 2s to "confirm"

// -------------------- BNO055 remap (optional) --------------------
// The BNO055 supports axis remapping via registers.
// We write AXIS_MAP_CONFIG (0x41) and AXIS_MAP_SIGN (0x42) on page 0.
// This lets you adapt roll/pitch/yaw to different mounting (wrist/shoulder/etc.).

uint8_t TARGET_CFG;
uint8_t TARGET_SIGN;

#define BNO055_ADDR               0x28
#define BNO055_AXIS_MAP_CONFIG    0x41
#define BNO055_AXIS_MAP_SIGN      0x42
#define BNO055_PAGE_ID            0x07

// Write one byte to a BNO055 register (low-level I2C helper)
static void bnoWrite8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read one byte from a BNO055 register (low-level I2C helper)
uint8_t i2cRead8(uint8_t reg)
{
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

// Print current remap registers to Serial (for debugging)
void printRemap(const char* tag)
{
  // remap registers live on PAGE 0
  bnoWrite8(BNO055_PAGE_ID, 0x00);
  delay(10);

  uint8_t cfg  = i2cRead8(BNO055_AXIS_MAP_CONFIG);
  uint8_t sign = i2cRead8(BNO055_AXIS_MAP_SIGN);

  Serial.print(tag);
  Serial.print(" CONFIG=0x");
  Serial.print(cfg, HEX);
  Serial.print(" SIGN=0x");
  Serial.println(sign, HEX);
}

// Different physical placements can require different axis mapping/sign
enum Placement {
  PLACEMENT_DEFAULT,
  PLACEMENT_WRIST,
  PLACEMENT_SHOULDER
};

Placement placement = PLACEMENT_DEFAULT;

// Apply a remap by:
// 1) switching sensor to CONFIG mode (required before changing registers)
// 2) writing AXIS_MAP_CONFIG and AXIS_MAP_SIGN

void applyRemap(uint8_t cfg, uint8_t sign)
{
  myIMU.setMode(OPERATION_MODE_CONFIG);
  delay(100);

  bnoWrite8(BNO055_PAGE_ID, 0x00);
  delay(10);

  bnoWrite8(BNO055_AXIS_MAP_CONFIG, cfg);
  delay(10);
  bnoWrite8(BNO055_AXIS_MAP_SIGN, sign);
  delay(10);
}

// -------------------- Calibration save/restore (ESP32 NVS) --------------------
// BNO055 has offset registers but no EEPROM: offsets are lost on power cycle.
// We store offsets blob in ESP32 NVS (flash), keyed by sensor_id so we don’t load
// offsets for a different BNO055 unit accidentally.

static const char *NVS_NAMESPACE  = "bno055";
static const char *KEY_SENSOR_ID  = "sid";
static const char *KEY_CALIB_BLOB = "cal";

// Try to load offsets from NVS into calibOut; return true if valid and matching sensor_id
static bool loadCalibrationFromNVS(adafruit_bno055_offsets_t &calibOut)
{
  prefs.begin(NVS_NAMESPACE, true);

  uint32_t savedId = prefs.getUInt(KEY_SENSOR_ID, 0);

  sensor_t sensor;
  myIMU.getSensor(&sensor);
  uint32_t currentId = (uint32_t)sensor.sensor_id;

  size_t blobSize = prefs.getBytesLength(KEY_CALIB_BLOB);

  // If no data or wrong size, consider it invalid
  if (savedId == 0 || blobSize != sizeof(adafruit_bno055_offsets_t)) {
    prefs.end();
    return false;
  }

  // If stored offsets belong to another sensor, ignore them
  if (savedId != currentId) {
    prefs.end();
    return false;
  }

  // Load the blob (raw data) into calibOut
  prefs.getBytes(KEY_CALIB_BLOB, &calibOut, sizeof(adafruit_bno055_offsets_t));
  prefs.end();
  return true;
}

// Save offsets + sensor_id to NVS
static void saveCalibrationToNVS(const adafruit_bno055_offsets_t &calibIn)
{
  prefs.begin(NVS_NAMESPACE, false);

  sensor_t sensor;
  myIMU.getSensor(&sensor);
  uint32_t currentId = (uint32_t)sensor.sensor_id;

  prefs.putUInt(KEY_SENSOR_ID, currentId);
  prefs.putBytes(KEY_CALIB_BLOB, &calibIn, sizeof(adafruit_bno055_offsets_t));

  prefs.end();
}

// -------------------- Quaternion helpers --------------------

// Clamp a float to [lo, hi] to avoid domain errors (e.g., asin input slightly > 1)
static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Compute "angle" of quaternion (0..180 deg) from its w component.
// Used here as a rough magnitude of relative rotation (drift) in qRel.
static float quatAngleDeg(const imu::Quaternion &q)
{
  float w = q.w();

   // numerical guard for acos
  if (w > 1.0f) w = 1.0f;
  if (w < -1.0f) w = -1.0f;

  float angleRad = 2.0f * acos(w);
  return angleRad * 180.0f / (float)M_PI;
}


// -------- Version : EuclideanSpace (non-normalised + singularities) --------
// This conversion handles singularities (gimbal lock regions) in a stable way
// using the "test" variable and special cases.
// NOTE: naming used by EuclideanSpace:
// roll=bank, pitch = heading, yaw = attitude

static void quatToEulerDeg_Euclid(const imu::Quaternion &q, float &rollDeg, float &pitchDeg, float &yawDeg)
{
  float qw = q.w();
  float qx = q.x();
  float qy = q.y();
  float qz = q.z();

  float sqw = qw * qw;
  float sqx = qx * qx;
  float sqy = qy * qy;
  float sqz = qz * qz;

  float unit = sqx + sqy + sqz + sqw;
  float test = qx * qy + qz * qw;

  float heading = 0.0f;
  float attitude = 0.0f;
  float bank = 0.0f;

  if (test > 0.499f * unit) {
    heading = 2.0f * atan2f(qx, qw);
    attitude = (float)M_PI / 2.0f;
    bank = 0.0f;
  } else if (test < -0.499f * unit) {
    heading = -2.0f * atan2f(qx, qw);
    attitude = -(float)M_PI / 2.0f;
    bank = 0.0f;
  } else {
    heading = atan2f(2.0f * qy * qw - 2.0f * qx * qz, sqx - sqy - sqz + sqw);

    float s = 2.0f * test / unit;
    s = clampf(s, -1.0f, 1.0f);
    attitude = asinf(s);

    bank = atan2f(2.0f * qx * qw - 2.0f * qy * qz, -sqx + sqy - sqz + sqw);
  }

  yawDeg   = attitude  * 180.0f / (float)M_PI;
  pitchDeg = heading * 180.0f / (float)M_PI;
  rollDeg  = bank     * 180.0f / (float)M_PI;
}

void setup()
{
  // -------------------- Serial + I2C init --------------------

  Serial.begin(115200);
  Wire.begin();

  delay(1000);
  
  // Debug header (helps when logging)
  Serial.println("# IMU Serial Test (two quaternion->Euler versions)");
  Serial.println("# CSV: t_ms,rollY,pitchY,yawEulerY,rollE,pitchE,yawEulerE,yawHeading,sys,g");
  Serial.println("# Command: 'n' sets NEUTRAL reference");

  // -------------------- BNO055 init --------------------
  // If begin() fails, likely wiring/power/I2C addr issue.
  if (!myIMU.begin()) {
    Serial.println("# ERROR: BNO055 not detected. Check wiring.");
    while (1) { delay(200); }
  }

  delay(500);

  // -------------------- Optional axis remap --------------------
  // Choose remap constants based on placement.
  // (These values are your chosen register configs for each mounting.)

  switch (placement) {
    case PLACEMENT_DEFAULT:
      TARGET_CFG  = 0x24;
      TARGET_SIGN = 0x00;
      Serial.print("WROTE  CONFIG=0x");
      Serial.print(TARGET_CFG, HEX);
      Serial.print(" SIGN=0x");
      Serial.println(TARGET_SIGN, HEX);
      applyRemap(TARGET_CFG, TARGET_SIGN);
      break;

    case PLACEMENT_WRIST:
      TARGET_CFG  = 0x24;
      TARGET_SIGN = 0x03;
      Serial.print("WROTE  CONFIG=0x");
      Serial.print(TARGET_CFG, HEX);
      Serial.print(" SIGN=0x");
      Serial.println(TARGET_SIGN, HEX);
      applyRemap(TARGET_CFG, TARGET_SIGN);
      break;

    case PLACEMENT_SHOULDER:
      TARGET_CFG  = 0x21;
      TARGET_SIGN = 0x01;
      Serial.print("WROTE  CONFIG=0x");
      Serial.print(TARGET_CFG, HEX);
      Serial.print(" SIGN=0x");
      Serial.print(TARGET_SIGN, HEX);
      Serial.println();
      applyRemap(TARGET_CFG, TARGET_SIGN);
      break;
  }

  printRemap("READ@CONFIG");

  // -------------------- Fusion mode setup --------------------
  // Use NDOF_FMC_OFF: NDOF fusion with "Fast Magnetometer Calibration" off, to reduce undesired magnetometer-driven jumps in indoor environments.

  myIMU.setMode(OPERATION_MODE_NDOF_FMC_OFF);
  delay(200);

  // These mode toggles are kept from your debug sequence:
  // switching to CONFIG allows reading/writing certain registers reliably.

  myIMU.setMode(OPERATION_MODE_CONFIG);
  delay(100);
  printRemap("READ@AFTER_NDOF");
  myIMU.setMode(OPERATION_MODE_NDOF_FMC_OFF);
  delay(100);

  myIMU.setMode(OPERATION_MODE_NDOF_FMC_OFF);
  delay(100);

  // -------------------- Restore calibration offsets (if available) --------------------
  // We load offsets from ESP32 NVS and write them into the BNO055 at boot, so the sensor starts closer to calibrated state.

  myIMU.setMode(OPERATION_MODE_CONFIG);
  delay(200);

  adafruit_bno055_offsets_t calib;
  bool restored = loadCalibrationFromNVS(calib);
  if (restored) {
    myIMU.setSensorOffsets(calib);
    Serial.println("# Offsets restored from NVS");
  } else {
    Serial.println("# No valid offsets in NVS (first run or different sensor)");
  }

  // Back to fusion mode after writing offsets
  myIMU.setMode(OPERATION_MODE_NDOF_FMC_OFF);
  delay(200);

  Serial.println("# Ready. Calibrate until Sys/G/A/M reach 3. Offsets will be saved automatically.");

  // -------------------- BLE init + advertising --------------------
  // Start BLE peripheral, expose NUS-like service and characteristics, advertise so a phone app can connect and receive notifications.

  if (!BLE.begin()) {
    Serial.println("# ERROR: BLE.begin() failed");
    while (1) { delay(100); }
  }

  BLE.setLocalName("NanoBNO055");
  BLE.setDeviceName("NanoBNO055");
  BLE.setAdvertisedService(nusService);

  nusService.addCharacteristic(nusTxChar);
  nusService.addCharacteristic(nusRxChar);

  BLE.addService(nusService);

  nusTxChar.writeValue("0,0,0,0");
  BLE.advertise();

  Serial.println("# BLE NUS advertising as 'NanoBNO055'");
}

void loop()
{
  // -------------------- BLE housekeeping --------------------
  // Keeps BLE stack responsive (connections, writes, notifications).

  BLE.poll();

  // -------------------- Read calibration status --------------------
  // Four values in [0..3]: system, gyro, accel, mag.

  uint8_t calSys = 0;
  uint8_t calGyro = 0;
  uint8_t calAccel = 0;
  uint8_t calMag = 0;
  myIMU.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

  // -------------------- Read raw absolute orientation (quaternion) --------------------
  // This is the fused orientation from the BNO055.

  imu::Quaternion q = myIMU.getQuat();

  // -------------------- Gate output until fully calibrated --------------------
  // We wait until all cal fields hit 3, then:
  // - mark imuReady
  // - auto-set neutral reference to current pose

  if (!imuReady) {
    if (calGyro == 3 && calAccel == 3 && calMag == 3 && calSys == 3) {
      imuReady = true;

      // set the neutral to "current orientation"
      qRef = myIMU.getQuat();
      hasReference = true;

      // reset filter state
      gFilt = 0.0f;

      Serial.println("# Neutral reference set automatically after calibration");
      Serial.println("# IMU calibrated. Orientation output ENABLED.");
    } else {
      // During calibration, print progress and do not output angles yet.
      Serial.print("# Calibrating... ");
      Serial.print("Sys=");
      Serial.print(calSys);
      Serial.print(" G=");
      Serial.print(calGyro);
      Serial.print(" A=");
      Serial.print(calAccel);
      Serial.print(" M=");
      Serial.println(calMag);

      delay(200);
      return;
    }
  }

  // -------------------- Manual neutral set via Serial --------------------
  // You can press 'n' in Serial Monitor to redefine neutral posture.
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'n' || c == 'N') {
      qRef = q;
      hasReference = true;
      Serial.println("# NEUTRAL set (qRef updated)");
    }
  }

  // -------------------- Manual neutral set via BLE RX --------------------
  // If phone sends 'n', we redefine qRef from current orientation.
  if (BLE.connected()) {
    if (nusRxChar.written()) {
      uint8_t rxBuf[20];
      int n = nusRxChar.valueLength();
      if (n > 20) n = 20;

      nusRxChar.readValue(rxBuf, n);

      for (int i = 0; i < n; i++) {
        if (rxBuf[i] == 'n' || rxBuf[i] == 'N') {
          qRef = q;
          hasReference = true;
          Serial.println("# NEUTRAL set (from BLE RX)");
        }
      }
    }
  }

  // -------------------- Safety: ensure neutral exists --------------------
  // If for any reason neutral wasn't set yet, set it on first loop.

  if (!hasReference) {
    qRef = q;
    hasReference = true;
    Serial.println("# NEUTRAL auto-set on first reading");
  }

  // -------------------- Compute relative orientation --------------------
  // qRel rotates from neutral frame to current frame.
  // Using qRef.conjugate() effectively "removes" neutral orientation.

  imu::Quaternion qRel = qRef.conjugate() * q;

  // -------------------- Compute yaw heading from projected forward vector --------------------
  // Goal: stable yaw-like angle that behaves nicely across ±180 deg.
  // Steps:
  // 1) define a body-frame forward vector (x-axis)
  // 2) rotate it into world frame using qRel
  // 3) project onto XY plane and compute atan2(hy, hx)

  imu::Vector<3> fwdBody(1.0f, 0.0f, 0.0f);
  imu::Vector<3> fwdWorld = qRel.rotateVector(fwdBody);

  float hx = fwdWorld.x();
  float hy = fwdWorld.y();

  float yawHeadingDeg = atan2f(hy, hx) * 180.0f / (float)M_PI;

  // Wrap to [-180, 180]
  if (yawHeadingDeg > 180.0f) yawHeadingDeg -= 360.0f;
  if (yawHeadingDeg < -180.0f) yawHeadingDeg += 360.0f;

  // -------------------- Stillness detection (gyro magnitude + LPF + hysteresis) --------------------
  // Read gyro vector (rad/s), compute magnitude, low-pass filter it,
  // then apply enter/exit thresholds + dwell time to confirm stillness.

  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float g = sqrtf(gyro.x() * gyro.x() + gyro.y() * gyro.y() + gyro.z() * gyro.z());

  // First-order LPF: smooths noise spikes
  gFilt = gAlpha * g + (1.0f - gAlpha) * gFilt;

  unsigned long now = millis();

  // Hysteresis state machine:
  // - if moving and filtered gyro drops below enter threshold: start timer
  // - if still and filtered gyro rises above exit threshold: cancel timer

  if (!isStill) {
    if (gFilt < gyroStillEnter) {
      isStill = true;
      stillRunning = true;
      stillStartMs = now;
      Serial.println("# Stillness timer started");
    }
  } else {
    if (gFilt > gyroStillExit) {
      isStill = false;
      if (stillRunning) Serial.println("# Stillness timer canceled (movement detected)");
      stillRunning = false;
    }
  }

  // If the still timer runs long enough, "confirm still"
  // and compute an indicative drift angle from qRel.

  if (stillRunning) {
    if (now - stillStartMs >= stillTimeMs) {
      float driftDeg = quatAngleDeg(qRel);
      Serial.print("# Still confirmed, driftDeg=");
      Serial.println(driftDeg, 2);
      stillRunning = false;
    }
  }

   // -------------------- Compute roll & pitch from Euclid conversion --------------------
  // You keep roll/pitch from EuclideanSpace conversion (good singularity behavior),
  // and yaw from yawHeadingDeg (your preferred yaw behavior, works for [-180,180]).
  float rollE = 0.0f, pitchE = 0.0f, yawTmp = 0.0f;
  quatToEulerDeg_Euclid(qRel, rollE, pitchE, yawTmp);

  // Keep your sign convention flips (mounting convention)
  pitchE = -pitchE;
  rollE  = -rollE;

  // Final outputs
  float rollFinal  = rollE;
  float pitchFinal = pitchE;
  float yawFinal   = yawHeadingDeg;

  // -------------------- BLE output (short CSV) --------------------
  // Send "t_ms,roll,pitch,yaw" via NUS TX characteristic as notifications.
  if (BLE.connected()) {
    char buffer[80];
    unsigned long t = millis();
    // t,roll,pitch,yaw
    snprintf(buffer, sizeof(buffer), "%lu,%.2f,%.2f,%.2f",
             t, rollFinal, pitchFinal, yawFinal);
    nusTxChar.writeValue(buffer);
  }

  // -------------------- Serial CSV output --------------------
  // Here you output: sys,gyro,accel,mag,roll,pitch,yaw
  // (So you can log calibration quality alongside angles.)
  
  Serial.print(calSys);
  Serial.print(",");
  Serial.print(calGyro);
  Serial.print(",");
  Serial.print(calAccel);
  Serial.print(",");
  Serial.print(calMag);
  Serial.print(",");
  Serial.print(rollFinal, 2);
  Serial.print(",");
  Serial.print(pitchFinal, 2);
  Serial.print(",");
  Serial.println(yawFinal, 2);
  

  // -------------------- Save offsets once per boot --------------------
  // When the IMU becomes fully calibrated, capture offsets and persist them in NVS.
  // savedThisBoot prevents repeated flash writes in the same run.

  static bool savedThisBoot = false;
  if (!savedThisBoot) {
    if (myIMU.isFullyCalibrated()) {
      adafruit_bno055_offsets_t newCalib;
      myIMU.getSensorOffsets(newCalib);
      saveCalibrationToNVS(newCalib);
      savedThisBoot = true;
      Serial.println("# Offsets saved to NVS (flash)");
    }
  }

  // -------------------- Fixed loop timing --------------------
  delay(SAMPLE_DELAY_MS);
}

