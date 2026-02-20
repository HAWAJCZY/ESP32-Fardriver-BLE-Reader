/****************************************************************************
 *
 * ESP32 Fardriver BLE → RealDash Gateway
 *
 * Dual-core design:
 *   Core 0 – BLE task : scans for and connects to the Fardriver controller,
 *             receives BLE notifications and decodes packet data.
 *   Core 1 – RealDash task : exposes a Bluetooth Classic SPP device called
 *             "Fardriver_Classic_Link" and streams RealDash type-44 CAN
 *             frames to the RealDash app at ~30 Hz.
 *
 * Requirements:
 *   - ESP32 board package ≥ 2.0 (Arduino-ESP32 / ESP-IDF v4.x)
 *   - Both CONFIG_BT_ENABLED and CONFIG_BT_CLASSIC_ENABLED must be set
 *     (default for most ESP32 Arduino board definitions).
 *
 ****************************************************************************/

// ─── 1. LIBRARIES ────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "BluetoothSerial.h"

// ─── 2. CONFIGURATION ────────────────────────────────────────────────────────

// BLE UUIDs for the Fardriver controller
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffec-0000-1000-8000-00805f9b34fb"

// Bluetooth Classic device name visible to RealDash
#define REALDASH_BT_NAME    "Fardriver_Classic_Link"

// CAN frame ID used in the RealDash XML mapping
#define REALDASH_CAN_ID     0x3200UL

// Frame send interval in milliseconds (30 ms ≈ 33 Hz, within 20–50 ms range)
#define REALDASH_INTERVAL_MS 30

// Motor / wheel parameters – adjust to match your hardware
#define MOTOR_POLE_PAIRS      20
#define WHEEL_CIRCUMFERENCE_M 1.416f

// ─── 3. DATA STRUCTURES ──────────────────────────────────────────────────────

struct FardriverData {
  float   voltage        = 0.0f;
  float   lineCurrent    = 0.0f;
  float   rpm            = 0.0f;
  int16_t rawRpm         = 0;
  int     gear           = 0;
  int     controllerTemp = 0;
  int     motorTemp      = 0;
  int     soc            = 0;
};

// ─── 4. GLOBAL OBJECTS & STATE ────────────────────────────────────────────────

BluetoothSerial SerialBT;

static FardriverData   controllerData;
static SemaphoreHandle_t dataMutex = nullptr;

static BLEAdvertisedDevice*     myDevice    = nullptr;
static BLEClient*               pClient     = nullptr;
static BLERemoteCharacteristic* pRemoteChar = nullptr;
static volatile bool bleConnected = false;
static volatile bool needsToScan  = false;

// Fardriver address lookup table.
// Index = (packet[1] & 0x3F).  Each entry is the controller memory address
// that the corresponding packet carries.  Only a handful of addresses are
// decoded below (0xE2, 0xE8, 0xD6, 0xF4); the rest are ignored.
static const uint8_t flash_read_addr[55] = {
  0xE2, 0xE8, 0xEE, 0xE4, 0x06, 0x0C, 0x12, 0xE2, 0xE8, 0xEE, 0x18, 0x1E, 0x24, 0x2A,
  0xE2, 0xE8, 0xEE, 0x30, 0x5D, 0x63, 0x69, 0xE2, 0xE8, 0xEE, 0x7C, 0x82, 0x88, 0x8E,
  0xE2, 0xE8, 0xEE, 0x94, 0x9A, 0xA0, 0xA6, 0xE2, 0xE8, 0xEE, 0xAC, 0xB2, 0xB8, 0xBE,
  0xE2, 0xE8, 0xEE, 0xC4, 0xCA, 0xD0, 0xE2, 0xE8, 0xEE, 0xD6, 0xDC, 0xF4, 0xFA
};

// ─── 5. FUNCTION PROTOTYPES ──────────────────────────────────────────────────

bool verifyCRC(uint8_t* data, uint16_t length);
void processPacket(uint8_t* pData, size_t length);
bool connectToServer();
void startScan();
void sendRealDashFrame(uint32_t canId, const uint8_t* data, uint8_t len);
void bleTask(void* pvParameters);
void realdashTask(void* pvParameters);

// ─── 6. BLE CALLBACKS ────────────────────────────────────────────────────────

static void notifyCallback(BLERemoteCharacteristic* /*pChar*/,
                           uint8_t* pData, size_t length, bool /*isNotify*/) {
  processPacket(pData, length);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* /*pclient*/) override {
    bleConnected = true;
    needsToScan  = false;
    Serial.println("[BLE] Connected to Fardriver");
  }
  void onDisconnect(BLEClient* /*pclient*/) override {
    bleConnected = false;
    if (myDevice != nullptr) {
      delete myDevice;
      myDevice = nullptr;
    }
    needsToScan = true;
    Serial.println("[BLE] Disconnected – will rescan");
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      if (myDevice == nullptr) {
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        BLEDevice::getScan()->stop();
        needsToScan = false;
        Serial.println("[BLE] Fardriver found");
      }
    }
  }
};

// ─── 7. CRC VERIFICATION ─────────────────────────────────────────────────────
// Seed: 0x7F3C, polynomial: 0xA001 (CRC-16/ARC variant)
bool verifyCRC(uint8_t* data, uint16_t length) {
  if (length != 16) return false;
  uint16_t crc = 0x7F3C;
  for (int i = 0; i < 14; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc == ((uint16_t)(data[15] << 8) | data[14]);
}

// ─── 8. PACKET PROCESSING ────────────────────────────────────────────────────
void processPacket(uint8_t* packet, size_t length) {
  if (!verifyCRC(packet, length)) return;

  uint8_t id = packet[1] & 0x3F;
  if (id >= sizeof(flash_read_addr)) return;

  uint8_t address = flash_read_addr[id];
  uint8_t* d      = &packet[2];

  // Work on a local copy; write back under mutex at the end
  FardriverData local;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    local = controllerData;
    xSemaphoreGive(dataMutex);
  } else {
    return;
  }

  switch (address) {
    case 0xE2:
      local.gear   = ((d[0] >> 2) & 0b11) + 1;
      local.rawRpm = (int16_t)((d[7] << 8) | d[6]);
      break;

    case 0xE8: {
      // Fardriver controllers typically operate in the 30–100 V range.
      // Values outside 0–200 V are treated as corrupt and discarded.
      float v = ((uint16_t)(d[1] << 8) | d[0]) / 10.0f;
      if (v >= 0.0f && v <= 200.0f) local.voltage = v;

      // Line current: positive = discharge, negative = regenerative braking.
      // Range -100 A to 300 A covers real-world regen and max drive current.
      float c = (int16_t)((d[5] << 8) | d[4]) / 4.0f;
      if (c >= -100.0f && c <= 300.0f) local.lineCurrent = c;
      break;
    }

    case 0xD6: {
      // Controller (driver board) safe operating range: -20 °C to 150 °C.
      int t = (int16_t)((d[11] << 8) | d[10]);
      if (t > -20 && t < 150) local.controllerTemp = t;
      break;
    }

    case 0xF4: {
      // Hub/mid-drive motor safe operating range: -20 °C to 200 °C.
      int mt = (int16_t)((d[1] << 8) | d[0]);
      if (mt > -20 && mt < 200) local.motorTemp = mt;
      local.soc = d[3];
      break;
    }

    default:
      return; // Unrecognised address – skip derived calculations
  }

  // Derived RPM (clamp negative rawRpm to 0)
  int16_t displayRawRpm = (local.rawRpm < 0) ? 0 : local.rawRpm;
  local.rpm = displayRawRpm * 4.0f / MOTOR_POLE_PAIRS;

  // Write updated values back
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    controllerData = local;
    xSemaphoreGive(dataMutex);
  }
}

// ─── 9. BLE HELPER FUNCTIONS ─────────────────────────────────────────────────

void startScan() {
  if (myDevice != nullptr) {
    delete myDevice;
    myDevice = nullptr;
  }
  Serial.println("[BLE] Scanning...");
  BLEDevice::getScan()->start(0, nullptr, false);
}

bool connectToServer() {
  if (!pClient->connect(myDevice)) return false;

  BLERemoteService* pSvc = pClient->getService(BLEUUID(SERVICE_UUID));
  if (!pSvc) {
    pClient->disconnect();
    return false;
  }

  pRemoteChar = pSvc->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (!pRemoteChar) {
    pClient->disconnect();
    return false;
  }

  if (pRemoteChar->canNotify()) {
    pRemoteChar->registerForNotify(notifyCallback);
  }
  return true;
}

// ─── 10. REALDASH FRAME SENDER ───────────────────────────────────────────────
// Type-44 frame layout (16 bytes total):
//   [0x44][0x33][0x22][0x11]  – fixed header
//   [CAN ID bytes 0-3 LE]     – 4-byte little-endian CAN ID
//   [data bytes 0-7]          – 8 payload bytes
void sendRealDashFrame(uint32_t canId, const uint8_t* data, uint8_t len) {
  uint8_t frame[16];
  frame[0] = 0x44;
  frame[1] = 0x33;
  frame[2] = 0x22;
  frame[3] = 0x11;
  frame[4] = (uint8_t)( canId        & 0xFF);
  frame[5] = (uint8_t)((canId >>  8) & 0xFF);
  frame[6] = (uint8_t)((canId >> 16) & 0xFF);
  frame[7] = (uint8_t)((canId >> 24) & 0xFF);
  uint8_t payloadLen = (len > 8) ? 8 : len;
  memcpy(&frame[8], data, payloadLen);
  if (payloadLen < 8) memset(&frame[8 + payloadLen], 0, 8 - payloadLen);
  SerialBT.write(frame, 16);
}

// ─── 11. RTOS TASKS ──────────────────────────────────────────────────────────

// BLE task – pinned to Core 0
// Handles scanning, connecting, and reconnection to the Fardriver controller.
// The BLE stack (including notify callbacks) also runs on Core 0.
void bleTask(void* /*pvParameters*/) {
  for (;;) {
    if (needsToScan && !bleConnected) {
      startScan();
      needsToScan = false;
    }

    if (myDevice != nullptr && !bleConnected) {
      if (connectToServer()) {
        Serial.println("[BLE] Connection established");
      } else {
        Serial.println("[BLE] Connection failed, will retry...");
        delete myDevice;
        myDevice   = nullptr;
        needsToScan = true;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// RealDash task – pinned to Core 1
// Reads the shared FardriverData snapshot and sends one RealDash CAN frame
// every REALDASH_INTERVAL_MS milliseconds.
//
// CAN frame 0x3200 payload layout (8 bytes):
//   Bytes 0–1 : RPM            uint16 LE
//   Bytes 2–3 : Voltage × 10   uint16 LE  (divide by 10 in realdash.xml)
//   Byte  4   : Gear            uint8
//   Byte  5   : Controller Temp int8   [°C]
//   Byte  6   : Motor Temp      int8   [°C]
//   Byte  7   : SOC             uint8  [%]
void realdashTask(void* /*pvParameters*/) {
  uint8_t payload[8];

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(REALDASH_INTERVAL_MS));

    if (!SerialBT.hasClient()) continue;

    FardriverData snap;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;
    snap = controllerData;
    xSemaphoreGive(dataMutex);

    uint16_t rpmVal  = (uint16_t)constrain((int)snap.rpm, 0, 65535);
    uint16_t voltVal = (uint16_t)constrain((int)(snap.voltage * 10.0f), 0, 65535);

    payload[0] = (uint8_t)( rpmVal        & 0xFF);
    payload[1] = (uint8_t)((rpmVal  >> 8) & 0xFF);
    payload[2] = (uint8_t)( voltVal       & 0xFF);
    payload[3] = (uint8_t)((voltVal >> 8) & 0xFF);
    payload[4] = (uint8_t)constrain(snap.gear, 0, 255);
    payload[5] = (int8_t) constrain(snap.controllerTemp, -128, 127);
    payload[6] = (int8_t) constrain(snap.motorTemp,      -128, 127);
    payload[7] = (uint8_t)constrain(snap.soc, 0, 100);

    sendRealDashFrame(REALDASH_CAN_ID, payload, 8);
  }
}

// ─── 12. SETUP ───────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("[GW] ESP32 Fardriver Gateway starting...");

  dataMutex = xSemaphoreCreateMutex();

  // Initialise Bluetooth Classic first so the BT controller starts in
  // dual-mode (BTDM).  BLEDevice::init() then adds the BLE stack on top.
  SerialBT.begin(REALDASH_BT_NAME);
  Serial.println("[BT] Classic SPP ready: " REALDASH_BT_NAME);

  // Initialise BLE client
  BLEDevice::init("ESP32_Fardriver_GW");
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  needsToScan = true;

  // BLE task on Core 0 (priority 2 – higher than RealDash to serve BLE stack)
  xTaskCreatePinnedToCore(bleTask,      "BLE_Task", 8192, nullptr, 2, nullptr, 0);
  // RealDash task on Core 1 (priority 1)
  xTaskCreatePinnedToCore(realdashTask, "RD_Task",  4096, nullptr, 1, nullptr, 1);
}

// ─── 13. MAIN LOOP ───────────────────────────────────────────────────────────
// All work is done in FreeRTOS tasks; the Arduino loop is intentionally idle.
void loop() {
  vTaskDelay(portMAX_DELAY);
}
