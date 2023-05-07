
//  Created by bobolink
//  twitter: @wm6h
//  rev: 20180304

/*
   Two core ESP-32 Arduino real-time audio demo.
   Tested on Espressif ESP32 Dev board
   Rev. 1 silicon
   Real-Time Samples at 8Ksps for voice audio range (< 4KHz).
   Not for music.
   Compatible with Arduino IDE
*/

/*
 * Generates an old-time telephone dial tone
 * With two tones (350 and 440 Hz).
 * 
 * this example points out that
 * a computed value must be scaled properly to stay within the
 * +0.1 to -0.1 range as to not overload the 8-bit DAC.
 * When the two numbers within range are added, 
 * they will exceed +/- 1.0 and must be divided by two. 
 * Three numbers added must be divided by 3, etc.    
 */

#include <Arduino.h>
#include <driver/adc.h>

int LED_BUILTIN = 5;

const uint16_t  N = 1024; // should be a power of 2 for FFTs
// Determines how long the Application Processor can run for 
// real-time applications
// or how long the Application has to write the samples to
// a ring buffer for non-real time applications.
// Latency, input to output, is a function of N.

// we create complex numbers here for convenience
// could be done in frame processing
volatile double realPing[N];
volatile double imagPing[N];
volatile double realPong[N];
volatile double imagPong[N];

// Do frame processing in floats for better dynamic range.
// At last stage, scale floats to range -1.0 to +1.0
// then convert to unsigned char. Mult by 127 and
// adding 128 to map +1.0 -- -1.0 to 0-255 for
// output to 8-bit unsigned DAC

// we create analytic IQ input samples
// but we only output real numbers
volatile unsigned char outRealPing[N];
volatile unsigned char outRealPong[N];

const float Pi = 3.14159;
float theta1  = 0.0;
float theta2  = 0.0;
float sampleRateHz = 8000.0;

float testFreqHz1 = 2525.0;
//float testFreqHz1 = 350.0;

//float testFreqHz2 = 440.0;
float testFreqHz2 = 2475.0;

float theta_increment1 = 2.0 * Pi * (testFreqHz1 / sampleRateHz);
float theta_increment2 = 2.0 * Pi * (testFreqHz2 / sampleRateHz);

int FrameCNT = 0;

float volume = 1.0;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// this is Core 1's variable.
// Core 0 will operate on the other buffer
volatile boolean pingCore1 = true;

volatile boolean sampling = false;
volatile boolean outputEnable = true;

TaskHandle_t Task1;
SemaphoreHandle_t newFrame;

esp_err_t result;

void IRAM_ATTR onTimer()
{

  portENTER_CRITICAL_ISR(&timerMux);
  sampling = true;
  portEXIT_CRITICAL_ISR(&timerMux);

}

const byte interruptPin = 15;



enum STATES { PTT_IDLE = 0, KEY_ON1, KEY_ON2, PTT_ON, KEY_OFF1, KEY_OFF2 };

volatile int currentKeyValue = 0;
volatile STATES currentKeyState = PTT_IDLE; //LOW ptt switch off
volatile STATES nextKeyState = currentKeyState; 
float outVal = 0.0;


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterrupt() 
{
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  
  portENTER_CRITICAL_ISR(&mux);
  //interruptCounter++;
  portEXIT_CRITICAL_ISR(&mux);
  Serial.println("An interrupt has occurred.");
  delay(500);

}

void zeroDACbuffers(void)
{
      // zero the DAC buffers
      for (int i = 0; i < N; ++i)
      {
        outRealPing[i] = 0.0;
        outRealPong[i] = 0.0;
      }
  
}

void outputMicAudio(void)
{
  zeroDACbuffers();
}


// output the current tone 2525.0 or 2475.0
// tone must be set before calling
void outputTone(void)
{

    for (int i = 0; i < N; ++i)
    {
      // the point of this example is to point out that
      // a computed value must be scaled properly to stay within the
      // +0.1 to -0.1 range to not overload the 8-bit DAC.
      // When the two numbers are added, they exceed +/- 1.0
      // and must be divided by two (mult by 0.5). Three numbers added must
      // be divided by 3, etc.    
      outVal = (sinf(theta1) + sinf(theta2)) * 0.5 * volume;

      theta1 += theta_increment1;
      if (theta1 > 2.0 * Pi)
      {
        theta1 -= 2.0 * Pi;
      }

      theta2 += theta_increment2;
      if (theta2 > 2.0 * Pi)
      {
        theta2 -= 2.0 * Pi;
      }
      // NOTE:
      // this variable belongs to core 1 -- use the other buffer
      // If core 1 is ping, we are pong
      if (pingCore1 == true)
      {
        outRealPong[i] = (unsigned char) (outVal * 127.0 + 128.0);
      }
      else
      {
        outRealPing[i] = (unsigned char) (outVal * 127.0 + 128.0);
      }

    }// end frame processing of N samples
  
}

void setup()
{

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  //pinMode(interruptPin, INPUT_PULLUP);
  pinMode(interruptPin,INPUT);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

  newFrame = xSemaphoreCreateMutex();


  xTaskCreatePinnedToCore(
    frameProcessing,
    "FrameProcessing",
    1000,
    NULL,
    1,
    &Task1,
    0);

  delay(500);  // needed to start-up task1


  zeroDACbuffers();
  

 // this almost matches the output resolution
  // all channels  GPIOs 32-39
  result = adc1_config_width(ADC_WIDTH_9Bit);
  // complete with Apple type error message
  if (result != ESP_OK)
  {
    Serial.println("Error, an unknown error occurred");
  }

  // this might allow 3.3VDC on the mic
  // pin 34
  result = adc1_config_channel_atten( ADC1_CHANNEL_6, ADC_ATTEN_11db);
  if (result != ESP_OK)
  {
    Serial.println("Error, an unknown error occurred");
  }

  hw_timer_t* timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 125, true); // sampling frequency 8kHz
  timerAlarmEnable(timer);

}

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1
//                   This Task runs on Core: 1
// Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1
// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

// ||||||||||||||||||
//  Sample Service
// ||||||||||||||||||
void loop()
{
  for (int i = 0; i < N; ++i)
  {
    while (!sampling);

//    portENTER_CRITICAL(&timerMux);
    sampling = false;
//    portEXIT_CRITICAL(&timerMux);

    //digitalWrite(LED_BUILTIN, HIGH);
    if (pingCore1 == true)
    {
      // configured for 9 bits with an 11 dB pad
      // drop one bit to give 8 to match the output
      realPing[i] = (float) (((byte)((adc1_get_voltage(ADC1_CHANNEL_6) >> 1) & 0xFF)) - 128.0);
      imagPing[i] = 0.0;
      // DAC output is 8-bit unsigned. Maximum (255) corresponds
      // for VDD_A 3.3V, to 2.59V
      if (outputEnable == true)
      { // GPIO pin 25 of the ESP32.
        // This is the output of DAC1
        dacWrite(25, outRealPing[i]);
      }
    }
    else
    {
      // drop one bit to give 8 to match the output
      realPong[i] = (float) (((byte)((adc1_get_voltage(ADC1_CHANNEL_6) >> 1) & 0xFF)) - 128.0);
      imagPong[i] = 0.0;
      // DAC output is 8-bit unsigned. Maximum (255) corresponds
      // for VDD_A 3.3V, to 2.59V
      if (outputEnable == true)
      { // for VDD_A 3.3V, to 2.59V
        dacWrite(25, outRealPong[i]);
      }
    } // end single sample processing
      //digitalWrite(LED_BUILTIN, LOW);
  } // end N samples processing

  // swap working buffer
  pingCore1 = !pingCore1;
  // give the old buffer to frame processing
  xSemaphoreGive(newFrame);
} // end sample service task

// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// 1111111111111111111111111111111111111111111111111111111111111
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/



// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0
//                    This Task runs on Core: 0
// Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0
// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Frame processing
// Runs every N samples

void frameProcessing( void* parameter )
{
  for (;;) // this is required, normally put in by
  {        // Arduino IDE

    //float outVal;

    outputEnable = true;

    // "Arf! Arf!"
    // pet the watchdog.
    vTaskDelay(10);
    
    // wait for the ping-pong buffer swap
    // indicating frame processing should begin
    // Core 1 has the timer resolution and is in charge of buffers
    xSemaphoreTake(newFrame, portMAX_DELAY);


    currentKeyValue = digitalRead(interruptPin);
    
    // state logic
    // +++++++++++++++++++++++++++++++++++++++++++++++
    if(currentKeyValue == 1) // PTT pressed
    {
      if (currentKeyState == PTT_IDLE) // *leave IDLE
      {
         nextKeyState = KEY_ON1;
      }     
    }
    else // PTT not pressed
    {
      if (currentKeyState == PTT_ON)
      {
         nextKeyState = KEY_OFF1; // *return to IDLE
      }
      else
      {
         nextKeyState = PTT_IDLE; //explicit  
      }
    }

    
    // tone logic
    // +++++++++++++++++++++++++++++++++++++++++++++++
    switch (currentKeyState) 
    {
      case PTT_IDLE: //PTT off, No Tone, No mic audio
            digitalWrite(LED_BUILTIN, LOW); // PTT OFF
            zeroDACbuffers();
            break;
      case KEY_ON1:
            theta_increment1 = 2.0 * Pi * (testFreqHz1 * 0.000125);
            theta_increment2 = 0.0;            
            outputTone(); // first frame 128 msec.
            digitalWrite(LED_BUILTIN, HIGH); // PTT ON
            nextKeyState = KEY_ON2;            
            break;
      case KEY_ON2:
            outputTone(); // second frame 256 msec.
            nextKeyState = PTT_ON; 
            break;
      case PTT_ON:
            outputMicAudio();
            break;
      case KEY_OFF1:
            theta_increment1 = 0.0;           
            theta_increment2 = 2.0 * Pi * (testFreqHz2 * 0.000125); 
            outputTone(); // first frame 128 msec.
            nextKeyState = KEY_OFF2;            
            break;
      case KEY_OFF2:
            outputTone(); // first frame 256 msec.
            nextKeyState = PTT_IDLE;            
            break;           
      default: 
            break;
    }
    // +++++++++++++++++++++++++++++++++++++++++++++++
    
    currentKeyState = nextKeyState;
              
 } // end Arduino task loop
}
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// 0000000000000000000000000000000000000000000000000000000000000
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
