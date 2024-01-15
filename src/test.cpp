// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <BME280I2C.h>

// BME280I2C bme;  // Default : forced mode, standby time = 1000 ms

// void printBME280Data(Stream* client);

// void setup() {
//     Serial.begin(115200);

//     while (!Serial) {
//     }  // Wait

//     Wire.begin();

//     while (!bme.begin()) {
//         Serial.println("Could not find BME280 sensor!");
//         delay(1000);
//     }
// }

// void loop() {
//     printBME280Data(&Serial);
//     delay(500);
// }

// void printBME280Data(
//     Stream* client) {
//     float temp(NAN), hum(NAN), pres(NAN);

//     BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
//     BME280::PresUnit presUnit(BME280::PresUnit_Pa);

//     bme.read(pres, temp, hum, tempUnit, presUnit);

//     client->print("Temp: ");
//     client->print(temp);
//     client->print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
//     client->print("\t\tHumidity: ");
//     client->print(hum);
//     client->print("% RH");
//     client->print("\t\tPressure: ");
//     client->print(pres);
//     client->print("Pa");

//     float altitude = 44330 * (1.0 - pow((pres / 100) / 1013, 0.1903));
//     client->print("% Alt: ");
//     client->println(altitude);

//     delay(1000);
// }