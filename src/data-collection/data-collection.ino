/*
  IMU Capture

  This example uses the on-board IMU to start reading acceleration and gyroscope
  data from on-board IMU and prints it to the Serial Monitor for one second
  when the significant motion is detected.

  You can also use the Serial Plotter to graph the data.

  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.

  Created by Don Coleman, Sandeep Mistry
  Modified by Dominic Pajak, Sandeep Mistry

  This example code is in the public domain.
*/

// BNO055 instantiation
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#define BNO_RST_PIN 5
#define BNO_SDA_PIN 26
#define BNO_SCL_PIN 33
#define SAMPLE_RATE 100 // Hz
volatile bool isIMUCalibrated = false; // IMU Calibration flag
Adafruit_BNO055 bno = Adafruit_BNO055(0x28); // Create BNO object with I2C addr 0x28

const float accelerationThreshold = 2.5; // threshold of significant in G's
const int numSamples = 250;

int samplesRead = numSamples;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    initIMU();

    // print the header
    Serial.println("aX,aY,aZ,gX,gY,gZ");
}

void loop() {
    float aX, aY, aZ, gX, gY, gZ;

    // wait for significant motion
    while (samplesRead == numSamples) {
        // read the acceleration data
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);    // - m/s^2

        // check if it's above the threshold
        if (accel.magnitude() >= accelerationThreshold) {
            // reset the sample read count
            samplesRead = 0;
            break;
        }
    }

    // Check if the all the required samples have been read since the last time the significant motion was detected
    while (samplesRead < numSamples) {
        // Read the acceleration and gyroscope data
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // - m/s^2
        aX = accel.x();
        aY = accel.y();
        aZ = accel.z();

        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // - rad/s
        gX = gyro.x();
        gY = gyro.y();
        gZ = gyro.z();

        samplesRead++;

        // print the data in CSV format
        Serial.print(aX, 3);
        Serial.print(',');
        Serial.print(aY, 3);
        Serial.print(',');
        Serial.print(aZ, 3);
        Serial.print(',');
        Serial.print(gX, 3);
        Serial.print(',');
        Serial.print(gY, 3);
        Serial.print(',');
        Serial.print(gZ, 3);
        Serial.println();

        if (samplesRead == numSamples) {
            // add an empty line if it's the last sample
            Serial.println();
        }

        delay(1/SAMPLE_RATE);
    }
}

// =========================
// === UTILITY FUNCTIONS ===
// =========================

void initIMU() {
    // Serial.print("Initializing IMU...");
    Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN); // Initialize I2C bus with correct wires
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055");
        while (true);
    }
    bno.setExtCrystalUse(true);

    // Serial.println("done!"); // DEBUG
}


