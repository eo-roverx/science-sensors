#include <Arduino.h>
#include <Wire.h>

// SparkFun IMU
#include <ICM20948_WE.h>
#define IMU_I2C_ADDRESS 0x69
ICM20948_WE imu = ICM20948_WE(IMU_I2C_ADDRESS);

void configureIMU(ICM20948_WE imuObject) {
    Serial.println("Initializing IMU");

    delay(100);

    while (!imu.init()) {
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
    configureIMU(imu);
}

void loop() {
    readIMUData(imu);
    delay(100);
}