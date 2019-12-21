/******************************************/
// code for writing offset values to the
// offset register of mpu60X0. these values
// were obtained using IMU_ZERO sketch of
// mpu6050 library by jroberg
// https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
// compiled for arduino uno
/*****************************************/
/**************change log*****************/
// 98/08/11 first release
/*****************************************/
#include <Wire.h>

/////offsets[6] = {AccelX,AccelY,AccelY,gyroX,gyroY,gyroZ }
long offsets[6] = {   900   ,   2115  ,   1369  ,  -15 ,  54  , 36 };

int addr = 0x68; //AD0 pin low (default)
//int addr = 0x69; //AD0 pin high
uint8_t gyro_range = 0b00000000; //250 deg/sec
uint8_t accel_range = 0b00000000; //2g

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(addr);
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  //  Wire.beginTransmission(addr); //I2C address of the MPU
  //  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  //  Wire.write(gyro_range); //Setting gyroscope range to 1000deg/s
  //  Wire.endTransmission();
  //  Wire.beginTransmission(addr); //I2C address of the MPU
  //  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  //  Wire.write(accel_range); //Setting accelorameter range to 8g
  //  Wire.endTransmission();
  Serial.println("Sensor set-up done");
  delay(500);

  Serial.println("Writing below offset values to corresponding register");
  Serial.println("accelX  accelY  accelZ  gyroX gyroY gyroZ");
  for (int i = 0; i < 6; i++) {
    Serial.print(offsets[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.println("enter any character if you confirm");
  delay(2000);
  while (!Serial.available());

  /****** Converting offset ranges ******/
  // According to the "MPU hardware offset registers
  // application note (rev. 1.0)" accelerometer and
  // gyroscope offsets need to be in the 8g and 1000dps
  // ranges respectivley. Since the offset values obtained
  // from IMU_ZERO by jroberg are in 2g and 250dps range,
  // these values need to be converted before Writing them
  // to the corresponding registers.
  for (int i = 0; i < 3; i++) { //converting accel offset values
    offsets[i] = (offsets[i] / 16384L) * 4096L;
  }

  for (int j = 3; j < 6; j++) { //converting gyro offset values
    offsets[j] = (offsets[j] / 131L) * 32.8L;
  }

  // array for holding data to send
  // to the device
  byte data[6] = {0, 0, 0, 0, 0, 0};

  /****** writing accel offsets******/
  // according to the datasheet:"Initial values contain the OTP values of the Accel
  //factory trim. Therefore at bootup there will be a non-zero value in these registers.
  //Users will need to first read the register and apply the biases to that value."
  
  //Serial.println("reading data from accel registers");
  long accel_reg_bias[3];
  long mask = 0x0001;
  byte mask_bit[3] = {0, 0, 0};
  Wire.beginTransmission(addr);
  Wire.write(0x06); //XG_OFFS_USRH hardware offset registers(sec.7.1)
  Wire.endTransmission();
  Wire.requestFrom(addr, 6); //request 6 bytes from slave
  accel_reg_bias[0] = Wire.read() << 8 | Wire.read();
  accel_reg_bias[1] = Wire.read() << 8 | Wire.read();
  accel_reg_bias[2] = Wire.read() << 8 | Wire.read();
  //bit 0 on the low byte of each accel axis is for temp compensation
  //and needs to be preserved
  for (int i = 0; i < 3; i++) {
    if (accel_reg_bias[i] & mask) {
      mask_bit[i] = 0x01;
    }
  }
  accel_reg_bias[0] -= offsets[0];
  accel_reg_bias[1] -= offsets[1];
  accel_reg_bias[2] -= offsets[2];
  
  data[0] = (accel_reg_bias[0] >> 8) & 0xff;
  data[1] =  accel_reg_bias[0] & 0xff;
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_reg_bias[1] >> 8) & 0xff;
  data[3] =  accel_reg_bias[1] & 0xff;
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_reg_bias[2] >> 8) & 0xff;
  data[5] =  accel_reg_bias[2] & 0xff;
  data[5] = data[5] | mask_bit[2];
  delay(500);
  Serial.println("writing data to accel registers");
  Wire.beginTransmission(addr);
  Wire.write(0x06); //XG_OFFS_USRH-MPU Hardware Offset Registers(sec7.1)
  Wire.write(&data[0], 6);
  Wire.endTransmission();
//  Wire.beginTransmission(addr);
//  Wire.write(0x08); //YG_OFFS_USRH-MPU Hardware Offset Registers(sec7.1)
//  Wire.write(&data[2], 2);
//  Wire.endTransmission();
//  Wire.beginTransmission(addr);
//  Wire.write(0x0A); //ZG_OFFS_USRH-MPU Hardware Offset Registers(sec7.1)
//  Wire.write(&data[4], 2);
//  Wire.endTransmission();
  //Serial.println("done");
  //delay(500);
  
  /****** writing gyro offsets******/
  //inverting gyro offset values. this is done
  // according to the datasheet. sec 6.3
  // MPU Hardware Offset Registers
  for (int i = 3; i < 6; i++) {
    offsets[i] = (-offsets[i]);
  }
  data[0] = (offsets[3] >> 8) & 0xff;
  data[1] = (offsets[3]) & 0xff;
  data[2] = (offsets[4] >> 8) & 0xff;
  data[3] = (offsets[4]) & 0xff;
  data[4] = (offsets[5] >> 8) & 0xff;
  data[5] = (offsets[5]) & 0xff;
  //Serial.println("writing data to gyro registers");
  Wire.beginTransmission(addr);
  Wire.write(0x13); //XG_OFFS_USRH-MPU Hardware Offset Registers(sec6.1)
  Wire.write(&data[0], 6);
  Wire.endTransmission();
//  Wire.beginTransmission(addr);
//  Wire.write(0x15); //YG_OFFS_USRH-MPU Hardware Offset Registers(sec6.1)
//  Wire.write(&data[2], 2);
//  Wire.endTransmission();
//  Wire.beginTransmission(addr);
//  Wire.write(0x17); //ZG_OFFS_USRH-MPU Hardware Offset Registers(sec6.1)
//  Wire.write(&data[4], 2);
//  Wire.endTransmission();
//  Serial.println("done!");
//  delay(1000);
  //reading registers data to confirm succesful operation
  Wire.beginTransmission(addr);
  Wire.write(0x06); //XG_OFFS_USRH hardware offset registers(sec.7.1)
  Wire.endTransmission();
  Wire.requestFrom(addr, 6); //request 6 bytes from slave
  accel_reg_bias[0] = Wire.read() << 8 | Wire.read();
  accel_reg_bias[1] = Wire.read() << 8 | Wire.read();
  accel_reg_bias[2] = Wire.read() << 8 | Wire.read();

  long gyro_reg_bias[3];
  Wire.beginTransmission(addr);
  Wire.write(0x13); //XG_OFFS_USRH hardware offset registers(sec.7.1)
  Wire.endTransmission();
  Wire.requestFrom(addr, 6); //request 6 bytes from slave
  gyro_reg_bias[0] = Wire.read() << 8 | Wire.read();
  gyro_reg_bias[1] = Wire.read() << 8 | Wire.read();
  gyro_reg_bias[2] = Wire.read() << 8 | Wire.read();

  for (int k = 0; k < 3; k++) {
    Serial.print(accel_reg_bias[k]);
    Serial.print("\t");
  }
  for (int j = 0; j < 3; j++) {
    Serial.print(gyro_reg_bias[j]);
    Serial.print("\t");
  }
  Serial.println();
  Wire.endTransmission();
  Wire.beginTransmission(addr); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(gyro_range); //Setting gyroscope range
  Wire.endTransmission();
  Wire.beginTransmission(addr); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(accel_range); //Setting accelorameter range
  Wire.endTransmission();
}

void loop() {
  Wire.beginTransmission(addr); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(addr, 6); //Request 6 bytes from slave. Accel Registers (3B - 40)
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  Wire.beginTransmission(addr); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(addr, 6); //Request 6 bytes from slave. Gyro Registers (43 - 48)
  gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into gyroX
  gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into gyroY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into gyroZ
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
  Serial.print(gForceX);
  Serial.print("\t");
  Serial.print(gForceY);
  Serial.print("\t");
  Serial.print(gForceZ);
  Serial.print("\t");
  Serial.print(rotX);
  Serial.print("\t");
  Serial.print(rotY);
  Serial.print("\t");
  Serial.println(rotZ);

}
