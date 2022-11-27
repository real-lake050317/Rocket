

#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>

MPU6050 accelgyro;
Adafruit_BMP085 bmp;
//HMC5883L_Simple Compass;

int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!bmp.begin()) {
    Serial.println("Sensor not found");
    while (1) {}
  }

  // initialize mpu6050
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connected" : "Failed to connect MPU6050");
  
  accelgyro.setI2CBypassEnabled(true);
  
  //Compass.SetDeclination(23, 35, 'E');
  //Compass.SetSamplingMode(COMPASS_SINGLE);
  //Compass.SetScale(COMPASS_SCALE_130);
  //Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}

void loop() {
  
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");
  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(101300));
  Serial.println(" meters");
  
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  Serial.print("a/g:\t");
  Serial.print("ax: ");Serial.print(ax); Serial.print("\n");
  Serial.print("ay: ");Serial.print(ay); Serial.print("\n");
  Serial.print("az: ");Serial.print(az); Serial.print("\n");
  Serial.print("gx: ");Serial.print(gx); Serial.print("\n");
  Serial.print("gy: ");Serial.print(gy); Serial.print("\n");
  Serial.print("gz: ");Serial.println(gz);
  //float heading = Compass.GetHeadingDegrees();
  //Serial.print("Heading: \t");
  //Serial.println( heading );
  Serial.println("----------------------------------------------------------------------");
  delay(5000);
}
