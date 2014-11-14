#include <SoftwareSerial.h>
#include <SPI.h>

//Sensor's memory register addresses:
const int ledpin = 19;
const int chipSelectPin = 7;

   int dataReadyL = 0;
   int dataReadyH = 0;
   byte acc_X[2] = {0,0};
   long acc_full = 0;
   byte testh = 1;
   byte testl = 1;
   byte test = 0;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Connect TXO of OpenLog to pin 3, RXI to pin 2
//SoftwareSerial(rxPin, txPin)

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

int ledPin = 19;


void setup() {                
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);


  // start the SPI library:
  SPI.begin();

  // initalize the  data ready and chip select pins:
  //pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);

  //Configure SCP1000 for low noise configuration:
  writeRegister(26, 0x00); //Config reg
  delay(10);
  writeRegister(28, 0x00); //Accel config
    delay(10);
  writeRegister(29, 0x08); //Config2
  delay(10);
  writeRegister(35, 0x08); //FIFO enable
  delay(10);
  writeRegister(106, 0x50); //User control 0101 0000
  delay(10); 
  writeRegister(107, 0x00);//Power managment 0000 0000
  delay(10);
//  writeRegister(108, 0x1F); //Disable all axis and gryo except ACC_X 0001 1111
  writeRegister(108, 0x00); //Disable all axis and gryo except ACC_X 0001 1111

  // give the sensor time to set up:
  delay(100);
}


void loop() {
  // read the value from the sensor:
  // turn the ledPin on
   //writeRegister(29, 0x08); //Config2
   delay(100);
   digitalWrite(ledpin, HIGH);

    // don't do anything until the data ready pin is high:
    //Read the temperature data
    //testh = readRegister(59, 1);
    //delay(10);
    //testl = readRegister(60, 1);
    //delay(10);
    //acc_full = (testh << 16) | testl;
    //test = readRegister(0x75, 0); //Reg 117
    dataReadyL = readRegister(0x73, 1); //Read from FIFO_CNT_L 
    dataReadyH = readRegister(0x72, 1); //Read from FIF0_CNT 0x72, latch new value
        /*Serial.print("ReadyL=");
    Serial.print(dataReadyH);
        Serial.print("  ReadyH=");
    Serial.println(dataReadyL);*/
    //Serial.print("Test=");
    //Serial.println(acc_full);
    
    if (dataReadyL | dataReadyH) { //If there is data to be read in the FIFO
        //digitalWrite(ledpin, HIGH);
         //acc_X[0] = readRegister(0x74, 1); //Read from 0x74 FIFO_R_W ACC_X_H
         //acc_X[1] = readRegister(0x74, 1); //Read from 0x74 FIFO_R_W ACC_X_L
         //acc_full = ((acc_X[0] << 15) | acc_X[1]);
         delay(10);
         //digitalWrite(ledpin, LOW);
         acc_full = readRegister(0x74, 2); //Read from 0x74 FIFO_R_W ACC_X_H
    } 
    else{
         digitalWrite(ledpin, HIGH);
         delay(100);
         digitalWrite(ledpin, LOW);
         delay(100);
    }
    // convert the temperature to celsius and display it:
    /*Serial.print("ACC_X[0]=");
   Serial.print(acc_X[0]);
   Serial.print("   ACC_X[1]=");
   Serial.println(acc_X[1]);*/
    Serial.print("Acc_X ");
    Serial.println(acc_full);
  
    //Serial.print("   Test =");
    //Serial.println(test);
}

//Read from or write to register from the SCP1000:
unsigned int readRegister(byte thisRegister, int bytesToRead ) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  //Serial.print(thisRegister, BIN);
  //Serial.print("\t");
  // now combine the address and the command into one byte
  byte dataToSend = thisRegister | 0x80;
  //Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // return the result:
  return(result);
}


//Sends a write command to SCP1000

void writeRegister(byte thisRegister, byte thisValue) {

  // SCP1000 expects the register address in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  //thisRegister = thisRegister << 2;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}