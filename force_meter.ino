/**********************************************
 *  Force meter sketch for Arduino UNO
 *  shows force value (in kgf)
 *  By Andrey Likhobabin andreylikhobabin@ya.ru
 *  2019
***********************************************/
#include <Arduino.h>
#include <TM1637Display.h>
#include "HX711.h"


/**********************************************
 *  Pinout
***********************************************/
// HX711 circuit wiring
#define DOUT_PIN 5
#define SCK_PIN 6

// TM1637 module connection pins (Digital Pins)
#define TM1637_CLK_PIN 3
#define TM1637_DIO_PIN 4

#define TARE_BUTTON_PIN 2 // should be ONLY 2 becasue of interrupt


/**********************************************
 *  Global value
***********************************************/
HX711 scale;
TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);

bool tare_pressed = false;

long ADC_tare_value = 0;            // ADC value when tenzo unloaded
const float scale_factor = 23300.0; // how many ADC values in one kilo

// #define SERIAL_OUTPUT  // enable serial output with force values


void setup() 
{
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TARE_BUTTON_PIN), tare_button_func, FALLING);
  
  #ifdef SERIAL_OUTPUT
    Serial.begin(38400);
  #endif

  scale.begin(DOUT_PIN, SCK_PIN);
  scale.set_gain(64);
  ADC_tare_value = scale.read();

  display.setBrightness(0x0f, true);
  display.clear();

}


void loop() 
{
  // ADC reading
  const int num_of_meas = 1;
  long ADC_sum = 0;
  for (int i=0; i<num_of_meas; i++)
  {
    ADC_sum += scale.read();
  }
  float ADC_mean = ADC_sum/num_of_meas;

  // zerroing ADC (pressing Tare button from interrupt)  
  if (tare_pressed)
  {
    ADC_tare_value = ADC_mean;
    tare_pressed = false;
  }
  
  // convert ADC to force,kg
  float force_kg = (ADC_mean - ADC_tare_value) / scale_factor;

  // display force
  float display_value = force_kg;
  if (display_value < -50)
    display_value = -50;
  if (display_value > 50)
    display_value = 50;

  int value_x10_int = int(display_value*10);
  display.showNumberDecEx(value_x10_int, (0x80 >> 2), false);

  #ifdef SERIAL_OUTPUT
    Serial.print(millis()/1000.0, 3);
    Serial.print("\t");
    Serial.println(force_kg, 3);
  #endif

}


void tare_button_func()
{
  tare_pressed = true;
}
