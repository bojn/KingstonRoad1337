#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>


unsigned char counter = 0;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    //    Serial.println("Initializing I2C devices...");
    accelgyro.initialize(MPU6050_GYRO_FS_2000);
  
    // verify connection
    // Serial.println("Testing device connections...");
    // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop()
{
    // read raw accel/gyro measurements from device
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    // display tab-separated accel/gyro x/y/z values
    
  if (Serial.available()) {
    unsigned char numToSend = Serial.read();
    Serial.write(numToSend); // send ack
    
    for (unsigned char c = 0; c < numToSend; c++) {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
    }
  }
}


