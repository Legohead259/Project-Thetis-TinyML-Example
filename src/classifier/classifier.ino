/*
  IMU Classifier
  This example uses the on-board IMU to start reading acceleration and gyroscope
  data from on-board IMU, once enough samples are read, it then uses a
  TensorFlow Lite (Micro) model to try to classify the movement as a known gesture.
  Note: The direct use of C/C++ pointers, namespaces, and dynamic memory is generally
        discouraged in Arduino examples, and in the future the TensorFlowLite library
        might change to make the sketch simpler.
  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.
  Created by Don Coleman, Sandeep Mistry
  Modified by Dominic Pajak, Sandeep Mistry
  This example code is in the public domain.
*/

#include <TensorFlowLite_ESP32.h>

// #include <TensorFlowLite.h>
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/experimental/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "model.h"

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
const int numSamples = 119;

int samplesRead = numSamples;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::ops::micro::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize];

// array to map gesture index to a name
const char* GESTURES[] = {
  "forward-back",
  "left-right"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

void setup() {
    Serial.begin(9600);
    while (!Serial);

    // initialize the IMU
    initIMU();

    // print out the samples rates of the IMUs
    Serial.print("Accelerometer sample rate = ");
    Serial.print(SAMPLE_RATE);
    Serial.println(" Hz");
    Serial.print("Gyroscope sample rate = ");
    Serial.print(SAMPLE_RATE);
    Serial.println(" Hz");

    Serial.println();

    // get the TFL representation of the model byte array
    tflModel = tflite::GetModel(model);
    if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("Model schema mismatch!");
        while (1);
    }

    // Create an interpreter to run the model
    tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

    // Allocate memory for the model's input and output tensors
    tflInterpreter->AllocateTensors();

    // Get pointers for the model's input and output tensors
    tflInputTensor = tflInterpreter->input(0);
    tflOutputTensor = tflInterpreter->output(0);

    Serial.println("Finished setup");
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
            Serial.println("Beginning sampling...");
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

        // normalize the IMU data between 0 to 1 and store in the model's
        // input tensor
        tflInputTensor->data.f[samplesRead * 6 + 0] = (aX + 4.0) / 8.0;
        tflInputTensor->data.f[samplesRead * 6 + 1] = (aY + 4.0) / 8.0;
        tflInputTensor->data.f[samplesRead * 6 + 2] = (aZ + 4.0) / 8.0;
        tflInputTensor->data.f[samplesRead * 6 + 3] = (gX + 2000.0) / 4000.0;
        tflInputTensor->data.f[samplesRead * 6 + 4] = (gY + 2000.0) / 4000.0;
        tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ + 2000.0) / 4000.0;

        samplesRead++;

        if (samplesRead == numSamples) {
            // Run inferencing
            TfLiteStatus invokeStatus = tflInterpreter->Invoke();
            if (invokeStatus != kTfLiteOk) {
                Serial.println("Invoke failed!");
                while (true); // Block further code execution
            }

            // Loop through the output tensor values from the model
            for (int i = 0; i < NUM_GESTURES; i++) {
                Serial.print(GESTURES[i]);
                Serial.print(": ");
                Serial.println(tflOutputTensor->data.f[i], 6);
            }
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
