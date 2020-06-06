#include "font.h"
#include"Catch.h"
#include <Wire.h>
#include "Protocentral_MAX30205.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_NeoPixel.h>



MAX30205 tempSensor;
MAX30105 particleSensor;

#define PIN            6
#define NUMPIXELS      16
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int delayval = 500;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[50]; //infrared LED sensor data
uint16_t redBuffer[50];  //red LED sensor data
#else
uint32_t irBuffer[50]; //infrared LED sensor data
uint32_t redBuffer[50];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

const int GSR = A2;
int threshold = 0;
int sensorValue;

void setup() {

  strip.begin();
  strip.show();


  Lcd_Init();
  LCD_Clear(WHITE);
  BACK_COLOR = WHITE;

  Wire.begin();
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  long sum = 0;
  for (int i = 0; i < 500; i++)
  {
    sensorValue = analogRead(GSR);
    sum += sensorValue;
  }
  threshold = sum / 500;
  Serial.print("threshold =");
  Serial.println(threshold);


  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 50; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  LCD_ShowString(10, 0, "Systeam Starting....", BLUE);

  Serial.begin(115200);
  LCD_Clear(WHITE);
  tempSensor.begin();   // set continuos mode, active mode
 /* LCD_ShowString(10, 0, "HeartRate:", BLUE);
  LCD_ShowString(10, 20, "SPO2:", BLUE);
  LCD_ShowString(10, 40, "Temperature:", BLUE);*/

  /*
      LCD_ShowString(10,0,"LCD_W:",RED);  LCD_ShowNum(70,0,LCD_W,3,RED);
      LCD_ShowString(10,20,"LCD_H:",RED);LCD_ShowNum(70,20,LCD_H,2,RED);
    }*/

}

void loop() {

  int tempThreshold = 37.5;
  int SPO2Threshold = 92;
  int HRThreshold = 100;
  int color = 0;

  bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  Serial.println("System Start!");
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 50; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 25; i < 50; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

  /*    if (validHeartRate == 0) {
        heartRate = 0;
        LCD_ShowNum(110, 0, heartRate, 3, BLACK);
      }
      else if (heartRate > HRThreshold || heartRate < 30) {
        LCD_ShowNum(110, 0, heartRate, 3, BLUE);
      }
      else {
        LCD_ShowNum(110, 0, heartRate, 3, RED);
      }

      if (validSPO2 == 0) {
        spo2 = 0;
        LCD_ShowNum(110, 20, spo2, 3, BLACK);
      }
      else if (spo2 < SPO2Threshold) {
        LCD_ShowNum(110, 20, spo2, 3, BLUE);
      }
      else {
        LCD_ShowNum(110, 20, spo2, 3, RED);
      }*/

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

      int temp1;
      sensorValue = analogRead(GSR);
      Serial.print("sensorValue=");
      Serial.println(sensorValue);
      temp1 = threshold - sensorValue;
      if (abs(temp1) > 60)
      {
        sensorValue = analogRead(GSR);
        temp1 = threshold - sensorValue;
        if (abs(temp1) > 60) {
          Serial.println("Emotion Changes Detected!");
        }
      }

      float temp = tempSensor.getTemperature(); // read temperature for every 100ms
      if (temp > tempThreshold || temp < 27) {
        LCD_ShowNum(110, 40, temp, 2, BLUE); LCD_ShowNum1(130, 40, temp, 2, BLUE);
      }
      else {
        LCD_ShowNum(110, 40, temp, 2, RED); LCD_ShowNum1(130, 40, temp, 2, RED);
      }

      int color_temp = (100 - spo2) * 100 + heartRate * 5.5 + sensorValue * 0.5;
      int color_new = map(color_temp, 0, 1000, 0, 255);
      Serial.print(temp , 2);
      Serial.println("'c" );

      if (validHeartRate == 0 || validSPO2 == 0) {
        strip.clear();
        strip.show();
      }
      else if (color_new >= color) {
        for (int j = 0; j <= color_new - color; j++) {
          for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel(color + j));
          }
          strip.show();
          delay(20);
        }
        color = color_new;
        Serial.println(strip.getPixelColor(5), HEX);
      }
      else {
        for (int j = 0; j <=  color - color_new; j++) {
          for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel(color - j));
          }
          strip.show();
          delay(20);
        }
        color = color_new;
        Serial.println(strip.getPixelColor(5), HEX);
      }
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}


// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
