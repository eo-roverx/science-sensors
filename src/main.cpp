#include <Arduino.h>
#include <Wire.h>

// ICM20948 SparkFun IMU
#include <ICM20948_WE.h>
#define IMU_I2C_ADDRESS 0x69
ICM20948_WE IMUSensor = ICM20948_WE(IMU_I2C_ADDRESS);

// LTR390 Adafruit UV Sensor
#include <LTR390.h>
#define UV_I2C_ADDRESS 0x53
LTR390 UVSensor = LTR390(UV_I2C_ADDRESS);

// AGS02MA Adafruit Gas Sensor
#include <AGS02MA.h>
#define GAS_I2C_ADDRESS 26
AGS02MA GasSensor = AGS02MA(GAS_I2C_ADDRESS);

void configureAGS02MAGasSensor(AGS02MA gasObject) {
    Serial.println("Initializing Gas Sensor");

    delay(100);

    while (!gasObject.begin()) {
        Serial.println("Gas Sensor initialization unsuccessful");
        delay(1e2);
    }

    gasObject.setPPBMode();

    Serial.println("Gas Sensor initialization successful");
}

void readAGS02MAGasSensor(AGS02MA gasObject) {
    Serial.print("Gas Sensor Value: ");
    Serial.println(gasObject.readPPB());
}

void readGasAnalogSensor() {
    long value = analogRead(A0);

    Serial.print("Gas Sensor Value: ");
    Serial.println(value);
}

void configureUV(LTR390 uvObject) {
    Serial.println("Initializing UV Sensor");

    delay(100);

    while (!uvObject.init()) {
        Serial.println("UV Sensor initialization unsuccessful");
        delay(1e2);
    }

    Serial.println("UV Sensor initialization successful");

    uvObject.setMode(LTR390_MODE_ALS);
    uvObject.setGain(LTR390_GAIN_3);
    uvObject.setResolution(LTR390_RESOLUTION_18BIT);
}

void readUVData(LTR390 uvObject) {
    if (uvObject.newDataAvailable()) {
        if (uvObject.getMode() == LTR390_MODE_ALS) {
            Serial.print("Ambient Light Lux: ");
            Serial.println(uvObject.getLux());
            uvObject.setGain(LTR390_GAIN_18);                 // Recommended for UVI - x18
            uvObject.setResolution(LTR390_RESOLUTION_20BIT);  // Recommended for UVI - 20-bit
            uvObject.setMode(LTR390_MODE_UVS);
        } else if (uvObject.getMode() == LTR390_MODE_UVS) {
            Serial.print("UV Index: ");
            Serial.println(uvObject.getUVI());
            uvObject.setGain(LTR390_GAIN_3);                  // Recommended for Lux - x3
            uvObject.setResolution(LTR390_RESOLUTION_18BIT);  // Recommended for Lux - 18-bit
            uvObject.setMode(LTR390_MODE_ALS);
        }
    }
}

void configureIMU(ICM20948_WE imuObject) {
    Serial.println("Initializing IMU");

    delay(100);

    while (!IMUSensor.init()) {
        Serial.println("IMU initialization unsuccessful");
        delay(1e2);
    }

    Serial.println("IMU initialization successful");

    imuObject.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16640.0, 16560.0);

    Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
    delay(1000);
    imuObject.autoOffsets();
    Serial.println("Done!");

    imuObject.setAccRange(ICM20948_ACC_RANGE_2G);

    imuObject.setAccDLPF(ICM20948_DLPF_6);

    imuObject.setAccSampleRateDivider(10);
}

void readIMUData(ICM20948_WE imuObject) {
    imuObject.readSensor();
    xyzFloat accRaw = imuObject.getAccRawValues();
    xyzFloat corrAccRaw = imuObject.getCorrectedAccRawValues();
    xyzFloat gVal = imuObject.getGValues();
    float resultantG = imuObject.getResultantG(gVal);

    Serial.println("Raw acceleration values (x,y,z):");
    Serial.print(accRaw.x);
    Serial.print("   ");
    Serial.print(accRaw.y);
    Serial.print("   ");
    Serial.println(accRaw.z);
    Serial.println("Corrected raw acceleration values (x,y,z):");
    Serial.print(corrAccRaw.x);
    Serial.print("   ");
    Serial.print(corrAccRaw.y);
    Serial.print("   ");
    Serial.println(corrAccRaw.z);
    Serial.println("g-values (x,y,z):");
    Serial.print(gVal.x);
    Serial.print("   ");
    Serial.print(gVal.y);
    Serial.print("   ");
    Serial.println(gVal.z);
    Serial.print("Resultant g: ");
    Serial.println(resultantG);
    Serial.println("*************************************");
}

void setup() {
    Wire.begin();
    Serial.begin(115200);
    configureAGS02MAGasSensor(GasSensor);
}

void loop() {
    readAGS02MAGasSensor(GasSensor);
    delay(100);
}