
#include <Wire.h>

#define MPU_ADDRESS 0x68

void writeRegister(uint8_t,uint8_t);
uint8_t readRegister(uint8_t);

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  //writeRegister(26,0);
  //writeRegister(28,0);
  //writeRegister(29,0x08);
  //writeRegister(35,0x08);
  //writeRegister(106,0x50);
  //writeRegister(107, 0x00);
  //writeRegister(108, 0x00);
}

void loop()
{
  uint8_t high = 0;
  uint8_t low = 0;
  uint16_t val = 0;
  
  //writeRegister(35,0x08);
  r//eadRegister(60);
  high = readRegister(63);
  low = readRegister(64);
  val = (high<<8)+low;
  Serial.println((int16_t)val);
  //Serial.println("test");
}

void writeRegister(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(MPU_ADDRESS,1);
  
  uint8_t ret = 0;
  while(Wire.available())
  {
    ret = Wire.read();
  }
    
  return ret;
}
  
  
