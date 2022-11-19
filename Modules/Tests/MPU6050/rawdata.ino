#include <Wire.h>

const int MPU = 0x68; // I2C address of the MPU-6050

int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void get6050();

void setup() {
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.begin(9600);
}

void loop() {
    get6050();
    Serial.print(AcX);
    Serial.print("");
    Serial.print(AcY);
    Serial.print("");
    Serial.print(AcZ);
    Serial.println();
    delay(15);
}

void get6050() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
}