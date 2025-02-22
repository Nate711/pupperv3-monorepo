
#include "rt/rt_bno055.h"

#include <string>
// #include <i2c/smbus.h>

#include <chrono>  // for std::chrono::milliseconds
#include <iostream>
#include <thread>  // for std::this_thread::sleep_for

BNO055::BNO055(int i2c_device_number, uint32_t micros_between_reports) {
  // the name
  std::string device_name = std::string("/dev/i2c-") + std::to_string(i2c_device_number);
  printf("[BNO055] Using device name %s\n", device_name.c_str());

  // open the device
  m_fd = open(device_name.c_str(), O_RDWR);
  if (m_fd < 0) {
    printf("[BNO055] failed to open: %s\n", strerror(errno));
    m_okay = false;
  } else {
    m_okay = true;
  }

  if (m_okay) {
    if (ioctl(m_fd, I2C_SLAVE, I2C_ADDR) < 0) {
      m_okay = false;
      printf("[BNO055] failed to set slave: %s\n", strerror(errno));
    }
  }
  printf("Connected\r\n");
  softReset();
  printf("Reset\r\n");
  shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST;  // Request the product ID and reset info
  shtpData[1] = 0;                               // Reserved

  // Transmit packet on channel 2, 2 bytes
  sendPacket<2>(CHANNEL_CONTROL);
  printf("Sent first packet\r\n");

  // Now we wait for response
  if (receivePacket() == true) {
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
      printf("Correct Product ID\r\n");
    }
  }

  std::cout << "Enabling rot vec + gyro. Micros between reports: " << micros_between_reports
            << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  enableRotationVector(micros_between_reports);
  // IMPORTANT: NOT ENABLING ACCELEROMETER
  enableGyro(micros_between_reports);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

BNO055::~BNO055() {
  close(m_fd);
}

void BNO055::softReset(void) {
  shtpData[0] = 1;  // Reset

  // Attempt to start communication with sensor
  sendPacket<1>(CHANNEL_EXECUTABLE);  // Transmit packet on channel 1, 1 byte

  // Read all incoming data and flush it
  std::chrono::milliseconds duration(200);  // duration of 50 milliseconds
  std::this_thread::sleep_for(duration);    // pause for the specified duration
  while (receivePacket() == true);
  printf("First receive finished\r\n");
  std::this_thread::sleep_for(duration);  // pause for the specified duration
  while (receivePacket() == true);
  printf("second receive finished\r\n");
}

bool BNO055::receivePacket(void) {
  constexpr int headerLength = 4;
  uint8_t packetHeader[headerLength];

  // if (write(m_fd, &packetHeader, headerLength) != headerLength)
  // {
  //     printf("[BNO055] failed to write for receivePacket: %s\n", strerror(errno));
  //     return false;
  // }
  int response = read(m_fd, &packetHeader, headerLength);
  // printf("%i\r\n",response);
  if (response != headerLength) {
    printf("[BNO055] failed to read header for receivePacket: %s\n", strerror(errno));
    return false;
  }

  // Store the header info
  shtpHeader[0] = packetHeader[0];
  shtpHeader[1] = packetHeader[1];
  shtpHeader[2] = packetHeader[2];
  shtpHeader[3] = packetHeader[3];

  // Calculate the number of data bytes in this packet
  int16_t dataLength = ((uint16_t)packetHeader[1] << 8 | packetHeader[0]);
  dataLength &= ~(1 << 15);  // Clear the MSbit

  if (dataLength == 0) {
    // Packet is empty
    return false;
  }
  dataLength -= headerLength;

  getData(dataLength);

  return true;  // We're done!
}

bool BNO055::getData(uint16_t bytesRemaining) {
  uint16_t dataSpot = 0;  // Start at the beginning of shtpData array

  // Setup a series of chunked 32 byte reads
  while (bytesRemaining > 0) {
    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4)) {
      numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);
    }
    std::vector<uint8_t> readBuffer(numberOfBytesToRead + 4);

    // if (write(m_fd, &readBuffer, 4) != 4)
    // {
    //     printf("[BNO055] failed to write for getData: %s\n", strerror(errno));
    //     return false;
    // }
    int response = read(m_fd, readBuffer.data(), numberOfBytesToRead + 4);
    if (response != numberOfBytesToRead + 4) {
      printf("[BNO055] failed to read bytes for getData: %s\n", strerror(errno));
      return false;
    }

    for (uint8_t x = 4; x < numberOfBytesToRead + 4; x++) {
      uint8_t incoming = readBuffer[x];
      if (dataSpot < MAX_PACKET_SIZE) {
        shtpData[dataSpot++] = incoming;  // Store data into the shtpData array
      } else {
        // Do nothing with the data
      }
    }

    bytesRemaining -= numberOfBytesToRead;
  }
  return true;  // Done!
}

bool BNO055::waitForI2C() {
  uint8_t buffer[1];
  int timeoutCounter = 100;

  while (timeoutCounter > 0) {
    int bytesRead = read(m_fd, &buffer, 1);
    if (bytesRead == -1) {
      printf("[BNO055] failed to read for waitForI2C: %s\n", strerror(errno));
      return false;
    } else if (bytesRead == 1) {
      return true;
    }

    timeoutCounter--;
  }

  return false;
}

void BNO055::setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports,
                               uint32_t specificConfig) {
  shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;  // Set feature command. Reference page 55
  shtpData[1] = reportID;  // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpData[2] = 0;         // Feature flags
  shtpData[3] = 0;         // Change sensitivity (LSB)
  shtpData[4] = 0;         // Change sensitivity (MSB)
  shtpData[5] = (microsBetweenReports >> 0) & 0xFF;   // Report interval (LSB) in microseconds.
  shtpData[6] = (microsBetweenReports >> 8) & 0xFF;   // Report interval
  shtpData[7] = (microsBetweenReports >> 16) & 0xFF;  // Report interval
  shtpData[8] = (microsBetweenReports >> 24) & 0xFF;  // Report interval (MSB)
  shtpData[9] = 0;                                    // Batch Interval (LSB)
  shtpData[10] = 0;                                   // Batch Interval
  shtpData[11] = 0;                                   // Batch Interval
  shtpData[12] = 0;                                   // Batch Interval (MSB)
  shtpData[13] = (specificConfig >> 0) & 0xFF;        // Sensor-specific config (LSB)
  shtpData[14] = (specificConfig >> 8) & 0xFF;        // Sensor-specific config
  shtpData[15] = (specificConfig >> 16) & 0xFF;       // Sensor-specific config
  shtpData[16] = (specificConfig >> 24) & 0xFF;       // Sensor-specific config (MSB)

  // Transmit packet on channel 2, 17 bytes
  sendPacket<17>(CHANNEL_CONTROL);
}
void BNO055::enableRotationVector(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}
void BNO055::enableGyro(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}
void BNO055::enableLinearAccelerometer(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}
void BNO055::enableAccelerometer(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

void BNO055::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports) {
  setFeatureCommand(reportID, timeBetweenReports, 0);  // No specific config
}
bool BNO055::dataAvailable(void) {
  if (receivePacket() == true) {
    // Check to see if this packet is a sensor reporting its data to us
    // printf("here\n");
    if (shtpHeader[2] == CHANNEL_REPORTS) {
      //   printf("got channel report. shtpData[0]=%d\n", static_cast<int>(shtpData[0]));
      if (shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP) {
        // printf("got base timestamp\n");
        parseInputReport();  // This will update the rawAccelX, etc variables depending on which
                             // feature report is found
        return (true);
      }
    } else if (shtpHeader[2] == CHANNEL_CONTROL) {
      //   printf("got channel control\n");
      parseCommandReport();  // This will update responses to commands, calibrationStatus, etc.
      return (true);
    } else {
      printf("IMU got wack data\n");
    }
  }

  return (false);
}
float BNO055::getQuatI() {
  float quat = qToFloat(rawQuatI, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion J
float BNO055::getQuatJ() {
  float quat = qToFloat(rawQuatJ, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion K
float BNO055::getQuatK() {
  float quat = qToFloat(rawQuatK, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion Real
float BNO055::getQuatReal() {
  float quat = qToFloat(rawQuatReal, rotationVector_Q1);
  return (quat);
}

// Return the acceleration component
float BNO055::getLinAccelX() {
  float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getLinAccelY() {
  float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getLinAccelZ() {
  float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
  return (accel);
}
// Return the acceleration component
float BNO055::getAccelX() {
  float accel = qToFloat(rawAccelX, accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getAccelY() {
  float accel = qToFloat(rawAccelY, accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getAccelZ() {
  float accel = qToFloat(rawAccelZ, accelerometer_Q1);
  return (accel);
}

// Return the gyro component
float BNO055::getGyroX() {
  float gyro = qToFloat(rawGyroX, gyro_Q1);
  return (gyro);
}

// Return the gyro component
float BNO055::getGyroY() {
  float gyro = qToFloat(rawGyroY, gyro_Q1);
  return (gyro);
}

// Return the gyro component
float BNO055::getGyroZ() {
  float gyro = qToFloat(rawGyroZ, gyro_Q1);
  return (gyro);
}

uint32_t BNO055::getTimestamp() {
  return timeStamp;
}

float BNO055::qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return (qFloat);
}
// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
// shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
// shtpData[5 + 1]: Sequence number (See 6.5.18.2)
// shtpData[5 + 2]: Status
// shtpData[3]: Delay
// shtpData[4:5]: i/accel x/gyro x/etc
// shtpData[6:7]: j/accel y/gyro y/etc
// shtpData[8:9]: k/accel z/gyro z/etc
// shtpData[10:11]: real/gyro temp/etc
// shtpData[12:13]: Accuracy estimate
void BNO055::parseInputReport(void) {
  // Calculate the number of data bytes in this packet
  int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
  dataLength &= ~(
      1
      << 15);  // Clear the MSbit. This bit indicates if this package is a continuation of the last.
  // Ignore it for now. TODO catch this as an error and exit

  dataLength -= 4;  // Remove the header bytes from the data count

  timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | (shtpData[3] << (8 * 2)) |
              (shtpData[2] << (8 * 1)) | (shtpData[1] << (8 * 0));

  uint8_t sequence_number = shtpData[5 + 1];  // Sequence number

  // Is showing a new message only on updated data. And consecutive sequence numbers so none missing
  //   std::cout << "seq #: " << static_cast<int>(sequence_number)
  //             << " type: " << static_cast<int>(shtpData[5]) << std::endl;

  uint8_t status = shtpData[5 + 2] & 0x03;  // Get status bits
  uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
  uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
  uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
  uint16_t data4 = 0;
  uint16_t data5 = 0;

  if (dataLength - 5 > 9) {
    data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
  }
  if (dataLength - 5 > 11) {
    data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
  }

  // Store these generic values to their proper global variable
  if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER) {
    accelAccuracy = status;
    rawAccelX = data1;
    rawAccelY = data2;
    rawAccelZ = data3;
  } else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION) {
    accelLinAccuracy = status;
    rawLinAccelX = data1;
    rawLinAccelY = data2;
    rawLinAccelZ = data3;
  } else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE) {
    gyroAccuracy = status;
    rawGyroX = data1;
    rawGyroY = data2;
    rawGyroZ = data3;
  } else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD) {
    magAccuracy = status;
    rawMagX = data1;
    rawMagY = data2;
    rawMagZ = data3;
  } else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
             shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    quatAccuracy = status;
    rawQuatI = data1;
    rawQuatJ = data2;
    rawQuatK = data3;
    rawQuatReal = data4;
    rawQuatRadianAccuracy = data5;  // Only available on rotation vector, not game rot vector
  } else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER) {
    stepCount = data3;  // Bytes 8/9
  } else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
    stabilityClassifier = shtpData[5 + 4];  // Byte 4 only
  } else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER) {
    activityClassifier = shtpData[5 + 5];  // Most likely state

    // Load activity classification confidences into the array
    for (uint8_t x = 0; x < 9; x++) {  // Hardcoded to max of 9. TODO - bring in array size
      _activityConfidences[x] =
          shtpData[5 + 6 + x];  // 5 bytes of timestamp, byte 6 is first confidence byte
    }
  } else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE) {
    // The BNO080 responds with this report to command requests. It's up to use to remember which
    // command we issued.
    uint8_t command = shtpData[5 + 2];  // This is the Command byte of the response

    if (command == COMMAND_ME_CALIBRATE) {
      printf("ME Cal report found!\r\n");
      calibrationStatus = shtpData[5 + 5];  // R0 - Status (0 = success, non-zero = fail)
    }
  } else {
    // This sensor report ID is unhandled.
    // See reference manual to add additional feature reports as needed
  }

  // TODO additional feature reports may be strung together. Parse them all.
}

// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0]: The Report ID
// shtpData[1]: Sequence number (See 6.5.18.2)
// shtpData[2]: Command
// shtpData[3]: Command Sequence Number
// shtpData[4]: Response Sequence Number
// shtpData[5 + 0]: R0
// shtpData[5 + 1]: R1
// shtpData[5 + 2]: R2
// shtpData[5 + 3]: R3
// shtpData[5 + 4]: R4
// shtpData[5 + 5]: R5
// shtpData[5 + 6]: R6
// shtpData[5 + 7]: R7
// shtpData[5 + 8]: R8
void BNO055::parseCommandReport(void) {
  if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE) {
    // The BNO080 responds with this report to command requests. It's up to use to remember which
    // command we issued.
    uint8_t command = shtpData[2];  // This is the Command byte of the response

    if (command == COMMAND_ME_CALIBRATE) {
      calibrationStatus = shtpData[5 + 0];  // R0 - Status (0 = success, non-zero = fail)
    }
  } else {
    // This sensor report ID is unhandled.
    // See reference manual to add additional feature reports as needed
  }

  // TODO additional feature reports may be strung together. Parse them all.
}

BNO055::Output BNO055::sample() {
  // Typically has two messages available in one call to sample()
  while (dataAvailable() == true) {
    latest_packet_timestamp = std::chrono::high_resolution_clock::now();
  }
  Output result;
  result.quat.x() = getQuatI();
  result.quat.y() = getQuatJ();
  result.quat.z() = getQuatK();
  result.quat.w() = getQuatReal();

  // Accelerometer currently not enabled
  //   result.acc(0) = getAccelX();
  //   result.acc(1) = getAccelY();
  //   result.acc(2) = getAccelZ();

  result.gyro(0) = getGyroX();
  result.gyro(1) = getGyroY();
  result.gyro(2) = getGyroZ();

  // Record time data read from IMU
  result.packet_timestamp = latest_packet_timestamp;
  result.measurement_timestamp = latest_packet_timestamp - std::chrono::microseconds(timeStamp);

  return result;
}
