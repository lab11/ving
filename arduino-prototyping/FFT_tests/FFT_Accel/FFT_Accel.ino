
#define ARM_MATH_CM4
#define MPU_ADDRESS 0x68

#include <arm_math.h>
#include <Wire.h>
void writeRegister(uint8_t,uint8_t);
uint8_t readRegister(uint8_t);

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION 
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int SAMPLE_RATE_HZ = 1000;             // Sample rate of the audio in hertz.
const int FFT_SIZE = 64;              //256 Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.

const int READ_RES = 16;               // Bits of resolution for the ADC.

const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).



////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////
const int MAX_CHARS = 65;              // Max size of the input command buffer
char commandBuffer[MAX_CHARS];

IntervalTimer samplingTimer;
int sampleCounter = 0;

q31_t samples_fix[FFT_SIZE*2];
q31_t samples_hold[FFT_SIZE*2];
q31_t magnitudes_fix[FFT_SIZE];
q31_t max_finder[FFT_SIZE];

volatile uint8_t high = 0;
volatile uint8_t low = 0;
q31_t val = 0;
q31_t max_val = 0;
uint32_t max_index = 0;
volatile int dom_freq_h = 0;
volatile int dom_freq_l = 0;
arm_cfft_radix4_instance_q31 fft_inst_fix;
volatile boolean hasSampled = false;
volatile boolean isDone = false;

////////////////////////////////////////////////////////////////////////////////
// MAIN SKETCH FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set up serial port.
  Serial.begin(115200);
  Wire.begin();
  
  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
  
  // Begin sampling audio
  //samplingBegin();
  
    int status = arm_cfft_radix4_init_q31(&fft_inst_fix, FFT_SIZE, 0, 1); //Selects forward transform and bit reversal of output
    if (status == ARM_MATH_ARGUMENT_ERROR){
        Serial.print(status);
        Serial.println("FFT Init ERROR!");
    }
     memset(commandBuffer, 0, sizeof(commandBuffer));
    
    samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);

}

void loop() {
  
  
  // Calculate FFT if a full sample is available.
  if (hasSampled) {
    
    // Run FFT on sample data.

    //int first = micros();
    
    //Calculate the FFT of samples and upscale output by 6 bits to retain q31_t data format
    arm_cfft_radix4_q31(&fft_inst_fix, samples_hold);

    // Calculate magnitude of complex numbers outq31put by the FFT.
    arm_cmplx_mag_q31(samples_hold, magnitudes_fix, FFT_SIZE);
    
    //int secon = micros();
    
    //Serial.print("Time to run FFT: ");
    //Serial.println(secon-first);
    
    
    //Serial.print("Sample: ");
    //Serial.println((magnitudes_fix[0] >> (30-READ_RES)), DEC);
   
    for (int i = 0; i < FFT_SIZE/2; ++i) {
      max_finder[i] = magnitudes_fix[i];
    }
    
    max_finder[0] = 0;
    arm_max_q31(max_finder, FFT_SIZE, &max_val, &max_index);
    
    dom_freq_l = (SAMPLE_RATE_HZ/FFT_SIZE)*max_index;
    dom_freq_h = dom_freq_l + (SAMPLE_RATE_HZ/FFT_SIZE);
    
    //Serial.print("Dominent Freq: ");
    Serial.print(dom_freq_l);
    Serial.print(" - ");
    Serial.print(dom_freq_h);
    Serial.print("  Val: ");
    //Serial.println(((max_val >> (30-READ_RES))/32767.0)*1.8);
    Serial.println(max_val);
    
    hasSampled = false;
    isDone = false;
    
    //high = readRegister(63);
    //low = readRegister(64);
    //val = (high<<8)+low;
    //val = (val << 30-READ_RES);
    //Serial.println(val, BIN);
   
  }
  
  //parserLoop();

}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  
  if(!hasSampled) {
    
  //Serial.println("sampled");
  // Read from the I2C and store the sample data
   //Serial.print("a");
   //Input samples and convert to q31_t data bit (signbit.31resolution bits)
   high = readRegister(63);
   low = readRegister(64);
   val = (q31_t)((high<<8)+low);
   //Serial.println((int16_t)((high<<8)+low));
   val = (val << 30-READ_RES);
 
   //samples_fix[sampleCounter] = val;
   
   //shfit the array left by 1
   shiftLeft();
   samples_fix[FFT_SIZE*2-2] = val;
   samples_fix[FFT_SIZE*2-1] = (q31_t)0;
   copyArray();
   //add new val to the back
   sampleCounter += 2;
   
   //hasSampled = true;
   
   if(sampleCounter > FFT_SIZE*2)  {
     hasSampled = true;
     isDone = true;
     sampleCounter = 0;
   }
   
  }
   
   
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only havGETe real data, set this coefficient to zero.
  //samples_fix[sampleCounter+1] = (q31_t)0;  
 
  // Update sample buffer position and stop after the buffer is filled
  //sampleCounter += 2;
  //if (sampleCounter >= FFT_SIZE*2) {
   // samplingTimer.end();
  //}
}

void copyArray() {
  for(int i = 0; i < FFT_SIZE*2; i++) {
    samples_hold[i] = samples_fix[i];
  }
  
}

void shiftLeft() {
  
  /*for(int i = 0; i < FFT_SIZE*2; i+=2)
  {
     samples_hold[i] = samples_fix[i+1];
     samples_hold[i+1] = (q31_t)0; 
  }*/
  
  
  for(int i = 0; i < FFT_SIZE*2; i++)
  {
    samples_fix[i] = samples_fix[i+2];
    //samples_fix[i+1] = (q31_t)0;
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
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
  
  
////////////////////////////////////////////////////////////////////////////////
// COMMAND PARSING FUNCTIONS
// These functions allow parsing simple commands input on the serial port.
// Commands allow reading and writing variables that control the device.
//
// All commands must end with a semicolon character.
// 
// Example commands are:
// GET SAMPLE_RATE_HZ;
// - Get the sample rate of the device.
// SET SAMPLE_RATE_HZ 400;
// - Set the sample rate of the device to 400 hertz.
// 
////////////////////////////////////////////////////////////////////////////////

void parserLoop() {
  // Process any incoming characters from the serial port
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Add any characters that aren't the end of a command (semicolon) to the input buffer.
    if (c != ';') {
      c = toupper(c);
      strncat(commandBuffer, &c, 1);
    }
    else
    {
      // Parse the command because an end of command token was encountered.
      parseCommand(commandBuffer);
      // Clear the input buffer
      memset(commandBuffer, 0, sizeof(commandBuffer));
    }
  }
}

// Macro used in parseCommand function to simplify parsing get and set commands for a variable
#define GET_AND_SET(variableName) \
  else if (strcmp(command, "GET " #variableName) == 0) { \
    Serial.println(variableName); \
  } \
  else if (strstr(command, "SET " #variableName " ") != NULL) { \
    variableName = (typeof(variableName)) atof(command+(sizeof("SET " #variableName " ")-1)); \
  }

void parseCommand(char* command) {
  if (strcmp(command, "GET MAGNITUDES") == 0) {
    for (int i = 0; i < FFT_SIZE; ++i) {
      Serial.println(magnitudes_fix[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE*2; i+=1) {
      Serial.println(samples_fix[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }
}


