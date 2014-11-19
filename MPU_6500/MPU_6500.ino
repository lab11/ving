#include <SoftwareSerial.h>
#include <SPI.h>
#include <Time.h>  

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

//Sensor's memory register addresses:
const int ledpin = 19;
const int chipSelectPin = 7;
const int resetOpenLog = 2;
SoftwareSerial OpenLog(4, 3); //Soft RX on 4, Soft TX out on 3

   int dataReadyL = 0;
   int dataReadyH = 0;
   byte acc_X[2] = {0,0};
   uint16_t acc_full = 0;
   uint8_t testh = 1;
   uint8_t testl = 1;
   byte test = 0;
   uint16_t time;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Connect TXO of OpenLog to pin 3, RXI to pin 2
//SoftwareSerial(rxPin, txPin)

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

int ledPin = 19;


void setup() {                
  pinMode(ledpin, OUTPUT);
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

  Serial.println("SPI initialized");
  // give the sensor time to set up:

  delay(100);
}


void loop() {
  // read the value from the sensor:
  // turn the ledPin on
   //writeRegister(29, 0x08); //Config2
   delay(100);
   digitalWrite(ledpin, HIGH);

    testh = readRegister(59, 1);
    testl = readRegister(60, 1);
    //delay(10);
    acc_full = ((testh << 8) | testl);
    //test = readRegister(0x75, 0); //Reg 117
    //dataReadyL = readRegister(0x73, 1); //Read from FIFO_CNT_L 
    //dataReadyH = readRegister(0x72, 1); //Read from FIF0_CNT 0x72, latch new value
    /*Serial.print("ReadyL=");
    Serial.print(dataReadyH);
    Serial.print("  ReadyH=");
    Serial.println(dataReadyL);*/
   
    
    /*if (dataReadyL | dataReadyH) { //If there is data to be read in the FIFO
        //digitalWrite(ledpin, HIGH);
         //acc_X[0] = readRegister(0x74, 1); //Read from 0x74 FIFO_R_W ACC_X_H
         //acc_X[1] = readRegister(0x74, 1); //Read from 0x74 FIFO_R_W ACC_X_L
         //acc_full = ((acc_X[0] << 8) | acc_X[1]);
         delay(10);
         //digitalWrite(ledpin, LOW);

           acc_full = readRegister(0x74, 2); //Read from 0x74 FIFO_R_W ACC_X_H
    } 
    else{
         digitalWrite(ledpin, HIGH);
         delay(100);
         digitalWrite(ledpin, LOW);
         delay(100);
    }*/
    // convert the temperature to celsius and display it:
   
   
    /*Serial.print("ACC_X[0]=");
   Serial.print(acc_X[0]);
   Serial.print("   ACC_X[1]=");
   Serial.println(acc_X[1]);*/
    //Serial.print("   Test =");
    Serial.println((int)(acc_full));
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

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void processSyncMessage() {
  // if time sync available from serial port, update time and return true
  while(Serial.available() >=  TIME_MSG_LEN ){  // time message consists of header & 10 ASCII digits
    char c = Serial.read() ; 
    Serial.print(c);  
    if( c == TIME_HEADER ) {       
      time_t pctime = 0;
      for(int i=0; i < TIME_MSG_LEN -1; i++){   
        c = Serial.read();          
        if( c >= '0' && c <= '9'){   
          pctime = (10 * pctime) + (c - '0') ; // convert digits to a number    
        }
      }   
      setTime(pctime);   // Sync Arduino clock to the time received on the serial port
    }  
  }
}