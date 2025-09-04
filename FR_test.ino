#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Wire.h>
#include <MPU9250.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <HardwareSerial.h>

HardwareSerial SerialPort(1);

// Beacon configuration
static const char* TARGET_UUID = "8f1c4b2a-9e3d-47c1-b5f0-66789a123456";
static const uint16_t TARGET_MAJOR = 4223;
static const uint16_t TARGET_MINORS[] = {12, 14, 16, 18, 20};
static const int NUM_BEACONS = 5;
static const int SCAN_TIME = 1; // seconds

// Array to store RSSI values for each minor
int rssiValues[NUM_BEACONS];

// Compass configuration
MPU9250 mpu;
float declination = 0.0; // Set your local magnetic declination in degrees (e.g., 2.5)
float offsetCorrection = 50.0; // Experimentally adjust this value to correct the heading
const unsigned long printInterval = 1000; // ms

// Simple moving average filter for yaw
const int filterSize = 10;
float yawBuffer[filterSize];
int filterIndex = 0;
bool bufferFilled = false;

float filterYaw(float newYaw) {
  yawBuffer[filterIndex] = newYaw;
  filterIndex = (filterIndex + 1) % filterSize;
  if (filterIndex == 0) bufferFilled = true;
  float sum = 0;
  int count = bufferFilled ? filterSize : filterIndex;
  for (int i = 0; i < count; i++) sum += yawBuffer[i];
  return sum / count;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Check if the device has iBeacon data
    if (advertisedDevice.haveManufacturerData()) {
      String manufacturerData = advertisedDevice.getManufacturerData();
      const uint8_t* data = (uint8_t*)manufacturerData.c_str();
      size_t length = manufacturerData.length();

      // Check for iBeacon pattern
      if (length >= 25 && data[0] == 0x00 && data[1] == 0x4C && data[2] == 0x02 && data[3] == 0x15) {
        // Extract UUID
        char uuid[37];
        snprintf(uuid, sizeof(uuid),
                 "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                 data[4], data[5], data[6], data[7],
                 data[8], data[9], data[10], data[11],
                 data[12], data[13], data[14], data[15],
                 data[16], data[17], data[18], data[19]);

        // Extract major and minor
        uint16_t major = (data[20] << 8) | data[21];
        uint16_t minor = (data[22] << 8) | data[23];
          
        if (strcasecmp(uuid, TARGET_UUID) == 0 && major == TARGET_MAJOR) {
            // Check if minor matches any target minor
            for (int i = 0; i < NUM_BEACONS; i++) {
              if (minor == TARGET_MINORS[i]) {
                rssiValues[i] = advertisedDevice.getRSSI();
                break;
              }
            }
          }
        }
      }
    }
};

void taskBLE(void *pvParameters) {
  BLEScan* pBLEScan = BLEDevice::getScan();
  for (;;) {
    // Reset RSSI values
    for (int i = 0; i < NUM_BEACONS; i++) {
      rssiValues[i] = -200; // -200 indicates no detection
    }

    // Start scanning
    pBLEScan->start(SCAN_TIME, false);

    // Print RSSI values in the required format
    for (int i = 0; i < NUM_BEACONS; i++) {
      Serial.print(rssiValues[i]);
      SerialPort.print(i * 2 + 12); SerialPort.print(": ");
      SerialPort.print(rssiValues[i]);
      if (i < NUM_BEACONS - 1) {
        Serial.print("\t");
        SerialPort.print("\t");
      }
    }
    Serial.println();
    SerialPort.println();

    // Clear scan results to prepare for next scan
    pBLEScan->clearResults();
  }
}

void taskCompass(void *pvParameters) {
  unsigned long lastPrint = 0;
  for (;;) {
    if (mpu.update()) {
      float yaw = mpu.getYaw(); // -180 to 180
      float filteredYaw = filterYaw(yaw);

      // Adjust yaw reading with fixed offset correction
      float heading = filteredYaw - 90 + offsetCorrection;
      // Normalize heading to 0-360 range
      if (heading < 0) heading += 360.0;
      if (heading >= 360) heading -= 360.0;

      String direction;
      if (heading >= 337.5 || heading < 22.5) direction = "N";
      else if (heading < 67.5) direction = "NE";
      else if (heading < 112.5) direction = "E";
      else if (heading < 157.5) direction = "SE";
      else if (heading < 202.5) direction = "S";
      else if (heading < 247.5) direction = "SW";
      else if (heading < 292.5) direction = "W";
      else direction = "NW";

      unsigned long now = millis();
      if (now - lastPrint >= printInterval) {
        Serial.print("Yaw: "); Serial.print(filteredYaw, 1);
        //SerialPort.print("Yaw: "); SerialPort.print(filteredYaw, 1);
        Serial.print(" | Heading: "); Serial.print(heading, 1);
        SerialPort.print("Heading: "); SerialPort.print(heading, 1); SerialPort.print("\t");
        Serial.print(" deg | Direction: "); Serial.println(direction);
        //SerialPort.print(" deg | Direction: "); SerialPort.println(direction);
        lastPrint = now;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to avoid hogging CPU
  }
}

void setup() {
  Serial.begin(115200);
  // ESP32-S3
  SerialPort.begin(9600, SERIAL_8N1, 18, 17, false, 15, 16); // RX, TX, invert=false, RTS, CTS    
  pinMode(48, OUTPUT);
  Serial.println("Starting BLE Beacon Scanner and Compass...");
  rgbLedWrite(48, 0, 0, 64);

  // Initialize BLE
  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P9); // Max power for better scanning
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  rgbLedWrite(48, 0, 0, 0);

  // Initialize MPU9250
  Wire.begin(8, 9);
  delay(2000);

  // Enable I2C bypass mode to access AK8963 directly
  Serial.println("Enabling I2C bypass mode...");
  Wire.beginTransmission(0x68);
  Wire.write(0x37); // INT_PIN_CFG register
  Wire.write(0x02); // Set bypass enable bit
  if (Wire.endTransmission() != 0) {
    Serial.println("Failed to enable bypass mode!");
    while (1);
  }

  // Optional: customize settings for better noise filtering
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  mpu.setup(0x68, setting); // Default I2C address
  mpu.selectFilter(QuatFilterSel::MADGWICK); // or MAHONY
  mpu.setFilterIterations(10); // More iterations = smoother

  // Set your local magnetic declination (degrees)
  mpu.setMagneticDeclination(declination);

  Serial.println("Calibrating Accel/Gyro (keep still)...");
  rgbLedWrite(48, 64, 0, 0);
  delay(2000);
  mpu.calibrateAccelGyro();
  Serial.println("Calibrating Magnetometer (move in figure 8)...");
  rgbLedWrite(48, 0, 64, 0);
  delay(2000);
  mpu.calibrateMag();
  Serial.println("Calibration done.");
  rgbLedWrite(48, 0, 0, 0);
  delay(1000);

  // Create FreeRTOS tasks
  xTaskCreate(taskBLE, "BLE Task", 4096, NULL, 1, NULL);
  xTaskCreate(taskCompass, "Compass Task", 4096, NULL, 1, NULL);
}

void loop() {
  // Empty loop as tasks handle the functionality
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}