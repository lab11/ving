// Audio Spectrum Display
// Copyright 2013 Tony DiCola (tony@tonydicola.com)

// This code is part of the guide at http://learn.adafruit.com/fft-fun-with-fourier-transforms/

#define ARM_MATH_CM4
#define MPU_ADDRESS 0x68

#include <arm_math.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
void writeRegister(uint8_t,uint8_t);
uint8_t readRegister(uint8_t);

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION 
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int SAMPLE_RATE_HZ = 1000;             // Sample rate of the audio in hertz.
float SPECTRUM_MIN_DB = 30.0;          // Audio intensity (in decibels) that maps to low LED brightness.
float SPECTRUM_MAX_DB = 60.0;          // Audio intensity (in decibels) that maps to high LED brightness.
int LEDS_ENABLED = 1;                  // Control if the LED's should display the spectrum or not.  1 is true, 0 is false.
                                       // Useful for turning the LED display on and off with commands from the serial port.
const int FFT_SIZE = 64;              //256 Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.
                                       
const int AUDIO_INPUT_PIN = 14;        // Input pin for audio data.
const int READ_RES = 16;               // Bits of resolution for the ADC.

const int ANALOG_READ_AVERAGING = 4;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).
const int NEO_PIXEL_PIN = 3;           // Output pin for neo pixels.
const int NEO_PIXEL_COUNT = 4;         // Number of neo pixels.  You should be able to increase this without
                                       // any other changes to the program.
const int MAX_CHARS = 65;              // Max size of the input command buffer


////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
char commandBuffer[MAX_CHARS];
float frequencyWindow[NEO_PIXEL_COUNT+1];
float hues[NEO_PIXEL_COUNT];

IntervalTimer samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
float mag_f[FFT_SIZE];
int sampleCounter = 0;

q31_t samples_fix[FFT_SIZE*2];
//q31_t samples_h_fix[FFT_SIZE*2];
q31_t magnitudes_fix[FFT_SIZE];
q31_t max_finder[FFT_SIZE];

uint8_t high = 0;
uint8_t low = 0;
q31_t val = 0;
int holding = 0;
q31_t max_val = 0;
uint32_t max_index = 0;
float32_t test = 0;
int dom_freq_h = 0;
int dom_freq_l = 0;

////////////////////////////////////////////////////////////////////////////////
// MAIN SKETCH FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set up serial port.
  Serial.begin(38400);
  Wire.begin();
  
  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(READ_RES);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  
  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
  
  // Initialize neo pixel library and turn off the LEDs
  pixels.begin();
  pixels.show(); 
  
  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));
  
  // Initialize spectrum display
  spectrumSetup();
  
  // Begin sampling audio
  samplingBegin();
}

void loop() {
  Serial.print("here");
  high = readRegister(63);
  low = readRegister(64);
  Serial.print("here");
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    //Serial.print("Sampling Completed");
    /*arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);*/
    
    //////////////////////////////////////////////////////////////////////////////////
    
    //unsigned long beg = micros();
    arm_cfft_radix4_instance_q31 fft_inst_fix;
    int status = arm_cfft_radix4_init_q31(&fft_inst_fix, FFT_SIZE, 0, 1); //Selects forward transform and bit reversal of output
    if (status == ARM_MATH_ARGUMENT_ERROR){
        Serial.print(status);
        Serial.println("FFT Init ERROR!");
    }
    
    //Calculate the FFT of samples and upscale output by 6 bits to retain q31_t data format
    
    //Serial.println(samples_fix[0], DEC);
    arm_cfft_radix4_q31(&fft_inst_fix, samples_fix);
    /*Serial.print("After: ");
    Serial.println(samples_fix[0], DEC);
    
    //Serial.println(samples_fix[0]);
    Serial.print("Before: ");
    Serial.println(samples_fix[0], HEX);*/
    
    

    
    // Calculate magnitude of complex numbers outq31put by the FFT.
    arm_cmplx_mag_q31(samples_fix, magnitudes_fix, FFT_SIZE);
    
    //Serial.print("Sample: ");
    //Serial.println((magnitudes_fix[0] >> (30-READ_RES)), DEC);
   
    for (int i = 0; i < FFT_SIZE/2; ++i) {
      max_finder[i] = magnitudes_fix[i];
    }
    max_finder[0] = 0;
    arm_max_q31(max_finder, FFT_SIZE, &max_val, &max_index);
    
    dom_freq_l = (SAMPLE_RATE_HZ/FFT_SIZE)*max_index;
    dom_freq_h = dom_freq_l + (SAMPLE_RATE_HZ/FFT_SIZE);
    
    /*Serial.print("Dominent Freq: ");
    Serial.print(dom_freq_l);
    Serial.print(" - ");
    Serial.print(dom_freq_h);
    Serial.print("  Val: ");
    Serial.println(((max_val >> (30-READ_RES))/32767.0)*1.8);*/
    
    /*Serial.print("Max val: ");
    Serial.print(max_val);
    Serial.print("   Max index:  ");
    Serial.println(max_index);  */
    
    //arm_float_to_q31(
    //arm_q31_to_float(magnitudes_fix, mag_f, FFT_SIZE);
    //unsigned long sto  = micros();
    //unsigned long delta = sto - beg;
    //Serial.println(delta);    
    
    for (int i = 0; i < FFT_SIZE; ++i) {
      //magnitudes[i] = (magnitudes_fix[i] >> 15);
      magnitudes[i] = (magnitudes_fix[i] >> 12);
      //Serial.println(magnitudes[i]);
     // magnitudes[i] = (float)(magnitudes_fix[i] >> 10);
    }
    
    /////////////////////////////////////////////////////////////////////////////////
  
    if (LEDS_ENABLED == 1)
    {
      spectrumLoop();
    }
  
    // Restart audio sampling.
    samplingBegin();
    
    //high = readRegister(63);
    //low = readRegister(64);
    //val = (high<<8)+low;
    //val = (val << 30-READ_RES);
    //Serial.println(val, BIN);
   
  }

  // Parse any pending commands.
  parserLoop();
}
////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < FFT_SIZE/2; ++i) {
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}

// Convert from HSV values (in floating point 0 to 1.0) to RGB colors usable
// by neo pixel functions.
uint32_t pixelHSVtoRGBColor(float hue, float saturation, float value) {
  // Implemented from algorithm at http://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV
  float chroma = value * saturation;
  float h1 = float(hue)/60.0;
  float x = chroma*(1.0-fabs(fmod(h1, 2.0)-1.0));
  float r = 0;
  float g = 0;
  float b = 0;
  if (h1 < 1.0) {
    r = chroma;
    g = x;
  }
  else if (h1 < 2.0) {
    r = x;
    g = chroma;
  }
  else if (h1 < 3.0) {
    g = chroma;
    b = x;
  }
  else if (h1 < 4.0) {
    g = x;
    b = chroma;
  }
  else if (h1 < 5.0) {
    r = x;
    b = chroma;
  }
  else // h1 <= 6.0
  {
    r = chroma;
    b = x;
  }
  float m = value - chroma;
  r += m;
  g += m;
  b += m;
  return pixels.Color(int(255*r), int(255*g), int(255*b));
}


////////////////////////////////////////////////////////////////////////////////
// SPECTRUM DISPLAY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

void spectrumSetup() {
  // Set the frequency window values by evenly dividing the possible frequency
  // spectrum across the number of neo pixels.
  float windowSize = (SAMPLE_RATE_HZ / 2.0) / float(NEO_PIXEL_COUNT);
  for (int i = 0; i < NEO_PIXEL_COUNT+1; ++i) {
    frequencyWindow[i] = i*windowSize;
  }
  // Evenly spread hues across all pixels.
  for (int i = 0; i < NEO_PIXEL_COUNT; ++i) {
    hues[i] = 360.0*(float(i)/float(NEO_PIXEL_COUNT-1));
  }
}

void spectrumLoop() {
  // Update each LED based on the intensity of the audio 
  // in the associated frequency window.
  float intensity, otherMean;
  for (int i = 0; i < NEO_PIXEL_COUNT; ++i) {
    windowMean(magnitudes, 
               frequencyToBin(frequencyWindow[i]),
               frequencyToBin(frequencyWindow[i+1]),
               &intensity,
               &otherMean);
    // Convert intensity to decibels.
    intensity = 20.0*log10(intensity);
    // Scale the intensity and clamp between 0 and 1.0.
    intensity -= SPECTRUM_MIN_DB;
    intensity = intensity < 0.0 ? 0.0 : intensity;
    intensity /= (SPECTRUM_MAX_DB-SPECTRUM_MIN_DB);
    intensity = intensity > 1.0 ? 1.0 : intensity;
    pixels.setPixelColor(i, pixelHSVtoRGBColor(hues[i], 1.0, intensity));
  }
  pixels.show();
}


////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
   high = readRegister(63);
   low = readRegister(64);
   val = (q31_t)((high<<8)+low);
   Serial.println(val);
   //val = (val << 30-READ_RES);
   samples_fix[sampleCounter] = val;
   
   //Input samples and convert to q31_t data bit (signbit.31resolution bits)
   //test = ((float32_t)((analogRead(AUDIO_INPUT_PIN)) & 0xFFFF)/(0xFFFF/2)-1);
   //Serial.print("Before cast:");
   //Serial.println(holding);
   holding = analogRead(AUDIO_INPUT_PIN);
   samples[sampleCounter] = holding;
   
   //holding = ((float32_t)(holding/65535));
   //arm_float_to_q31()
   //samples_fix[sampleCounter] = (q31_t)(holding << (31-READ_RES));
   //Serial.print("After cast:");
   //Serial.println(samples_fix[sampleCounter], BIN);
  
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only havGETe real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  samples_fix[sampleCounter+1] = (q31_t)0;  
 
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
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
      Serial.println(mag_f[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE*2; i+=1) {
      Serial.println(samples[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }
  

  GET_AND_SET(SAMPLE_RATE_HZ)
  GET_AND_SET(LEDS_ENABLED)
  GET_AND_SET(SPECTRUM_MIN_DB)
  GET_AND_SET(SPECTRUM_MAX_DB)
  
  // Update spectrum display values if sample rate was changed.
  if (strstr(command, "SET SAMPLE_RATE_HZ ") != NULL) {
    spectrumSetup();
  }
  
  // Turn off the LEDs if the state changed.
  if (LEDS_ENABLED == 0) {
    for (int i = 0; i < NEO_PIXEL_COUNT; ++i) {
      pixels.setPixelColor(i, 0);
    }
    pixels.show();
  }
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
  

