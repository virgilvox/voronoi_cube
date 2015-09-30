
#include "neopixel/neopixel.h"
#include "Adafruit_Simple_AHRS/Adafruit_Simple_AHRS.h"
#include "Adafruit_LSM9DS0/Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0/Adafruit_LSM9DS0.h"


Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

SYSTEM_MODE(AUTOMATIC);

/* ======================= prototypes =============================== */

void colorAll(uint32_t c, uint8_t wait);
void colorWipe(uint32_t c, uint8_t wait);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);
uint32_t Wheel(byte WheelPos);

#define PIXEL_COUNT 12
#define PIXEL_PIN D2
#define PIXEL_TYPE WS2812B


Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;

  lsm.getSensor(&accel, &mag, &gyro, &temp);

  delay(500);
}

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  
}

void setup(void)
{
    
  strip.begin();
  strip.show(); 
  
  cubeSet();
  
  Spark.function("color", color); 

 
  if(!lsm.begin())
  {
  
    while(1);
  }
  
  displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

}


String values;
String heading= String("{\"heading\":");
String pitch = String(",\"pitch\":");
String roll = String(",\"roll\":" );



void loop(void)
{
 
  

   sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
      
      values = String(heading + String(orientation.heading) + pitch + String(orientation.pitch) + roll + String(orientation.roll) + "}");
      delay(1200);
 
      Spark.publish("voronoi_accel", String(values));
  
  }
  
 
}


int color(String command){
    
    if(command == "red"){
        colorWipe(strip.Color(255, 0, 0), 50); // Red
    }else if(command == "blue"){
        colorWipe(strip.Color(0, 0, 255), 50);
    }else if(command == "green"){
        colorWipe(strip.Color(0, 255, 0), 50);
    }else if(command == "rainbow"){
        rainbow(20);
    }else if(command == "clear"){
         colorWipe(strip.Color(0, 0, 0), 50);
    }else if(command == "cube"){
        cubeSet();
    }
    
}

// Set all pixels in the strip to a solid color, then wait (ms)
void colorAll(uint32_t c, uint8_t wait) {
  uint16_t i;
  
  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

// Fill the dots one after the other with a color, wait (ms) after each one
void colorWipe(uint32_t c, uint8_t wait) {
    
    
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void cubeSet(){
    side(strip.Color(255, 0, 0),1);
    
     delay(100);
     strip.show();
    side(strip.Color(0, 255, 0),2);
    
      delay(100);
    strip.show();
    side(strip.Color(0, 0, 255),3);
    
     delay(100);
    strip.show();
    side(strip.Color(255, 255, 0),4);
    
     delay(100);
    strip.show();
    side(strip.Color(0, 255, 255),5);
    
     delay(100);
    strip.show();
    side(strip.Color(255, 0, 255),6);
    
    delay(100);
    strip.show();
}

void side(uint32_t c, int section) {
    
  section = (section - 1)*2;    
  
    strip.setPixelColor(section, c);
    strip.setPixelColor((section+1), c);
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout, then wait (ms)
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) { // 1 cycle of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

