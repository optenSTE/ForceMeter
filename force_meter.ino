/**********************************************
 *  Force meter sketch for Arduino UNO
 *  shows force value (in kgf) on display
 *  By Andrey Likhobabin andreylikhobabin@ya.ru
 *  2019
***********************************************/
#include <Arduino.h>
#include <TM1637Display.h>  // https://github.com/avishorp/TM1637
#include "HX711.h"          // https://github.com/bogde/HX711


/**********************************************
 *  Pinout configuration
***********************************************/
// HX711 - ADC converter https://ru.aliexpress.com/item/33046037411.html?spm=a2g0s.9042311.0.0.5f8a33edzvRB08
#define HX711_DOUT_PIN 5
#define HX711_SCK_PIN 6

// TM1637 - Display module https://www.aliexpress.com/item/32797695630.html?spm=a2g0s.9042311.0.0.27424c4dyNtc2z
#define TM1637_CLK_PIN 3
#define TM1637_DIO_PIN 4

#define TARE_BUTTON_PIN 2 // should be ONLY 2 becasue of interrupt

// Relay for triggering when force in defined limits
#define RELAY_PIN 13


/**********************************************
 *  Constants
***********************************************/
#define SERIAL_OUTPUT  // enable serial output with force values

#define ADC_GAIN 64 // hx711 gain factor - 32(channel B), 64(channel A), 128(channel A)

// force limits
#define MAX_FORCE 50
#define MIN_FORCE -50

// force trigger configuration
// trigger releses when 
//    (f_cur > (FORCE_TRIGGER_THRESHOLD - FORCE_TRIGGER_GAP))
//  and
//    (f_cur < (FORCE_TRIGGER_THRESHOLD + FORCE_TRIGGER_GAP))
//  during FORCE_TRIGGER_TIME_MS
// where f_cur is current averaged force
#define FORCE_TRIGGER_THRESHOLD 160
#define FORCE_TRIGGER_GAP 1
#define FORCE_TRIGGER_TIME_MS 3000

#define FORCE_TRIGGER_RELAY_HIGH_TIME_MS 1000 // time period for high state of relay when force trigger released

const long SCALE_FACTOR = 23300;          // how many ADC values in one kilo
const unsigned long ADC_AVG_TIME_MS = 200;// during this time adc-measurmetnts will be averaged
const unsigned long DELAY_MS = 300;       // delays, used in different places
const uint8_t DISPLAY_BRIGHTNESS = 0xff;  // display brightness

// sumbols for display
const uint8_t DISPLAY_TARE[] = { SEG_G, SEG_G, SEG_G, SEG_G }; // '----'
const uint8_t DISPLAY_ERROR[] = { SEG_A | SEG_F | SEG_E | SEG_D | SEG_G, SEG_G | SEG_E, SEG_G | SEG_E, SEG_G | SEG_E }; // 'Err'
const uint8_t DISPLAY_TRIGGER[] = {SEG_A | SEG_F | SEG_E | SEG_G, SEG_F | SEG_E | SEG_D, SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, SEG_A | SEG_C | SEG_D | SEG_F | SEG_G }; // 'FLAS'

const int LAST_MEASURMENTS_SIZE = 10; // Max_force mode - display shows max value from last measurments

/**********************************************
 *  Global variables
***********************************************/
HX711 scale_sensor;
TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);

bool tare_pressed = false;

long adc_tare_value = 0;            // ADC value when tenzo unloaded
bool display_blink_state_on = true;

unsigned long force_trigger_start = 0;    // time when first force trigger conidions released

float last_measurments[LAST_MEASURMENTS_SIZE];



void setup() 
{
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TARE_BUTTON_PIN), tare_button_func, FALLING);
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  #ifdef SERIAL_OUTPUT
    Serial.begin(38400);
  #endif

  scale_sensor.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  scale_sensor.set_gain(ADC_GAIN);
  adc_tare_value = scale_sensor.read();

  display.setBrightness(DISPLAY_BRIGHTNESS, true);
  display.clear();

}


void loop() 
{
  bool blink_mode = false;
  bool scale_sensor_error = false;

  // ADC reading
  long adc_sum = 0;
  int num_of_reading = 0;
  unsigned long start = millis();
  while ((millis() - start) < ADC_AVG_TIME_MS)
  {
    if (scale_sensor.wait_ready_timeout(ADC_AVG_TIME_MS))
    {
      adc_sum += scale_sensor.read();
      num_of_reading++;
    }
    else
    {
      scale_sensor_error = true;
      break;
    }
  }
  float adc_avg = adc_sum/num_of_reading;

  // zerroing ADC (pressing Tare button from interrupt)  
  if (tare_pressed)
  {
    adc_tare_value = adc_avg;
    tare_pressed = false;
    
    // show minuses for a short time
    display.setSegments(DISPLAY_TARE);
    delay(DELAY_MS);
  }
  
  // convert ADC to force,kg
  float force_kg = (adc_avg - adc_tare_value) / SCALE_FACTOR;
  
  // check conditions for force trigger
  if ((force_kg > (FORCE_TRIGGER_THRESHOLD - FORCE_TRIGGER_GAP)) and (force_kg < (FORCE_TRIGGER_THRESHOLD + FORCE_TRIGGER_GAP)))
  {
    // force is in limit, let's chech time
    blink_mode = true;
    if (force_trigger_start == 0)
      force_trigger_start = millis();
    else
    {
      unsigned long force_trigger_duration = millis() - force_trigger_start; // current time period for force trigger conidions
      if (force_trigger_duration > FORCE_TRIGGER_TIME_MS)
      {
        // trigger is released - let's release relay
        digitalWrite(RELAY_PIN, HIGH);
        display.setBrightness(DISPLAY_BRIGHTNESS, true);  // Turn on
        display.setSegments(DISPLAY_TRIGGER);
        delay(FORCE_TRIGGER_RELAY_HIGH_TIME_MS);
        digitalWrite(RELAY_PIN, LOW);
        force_trigger_start = 0;
      }
    }
  }
  else
    force_trigger_start = 0;
    
  
  // display force
  if (scale_sensor_error)
  {
    display.setSegments(DISPLAY_ERROR);
    blink_mode = true;
    delay(DELAY_MS);
  }
  else
  {
    float display_value = force_kg;
    
    // check for valid limits
    if (display_value < MIN_FORCE)
      display_value = MIN_FORCE;
    if (display_value > MAX_FORCE)
      display_value = MAX_FORCE;
    if ((display_value == MAX_FORCE) or (display_value == MIN_FORCE))
      blink_mode = true;
    
    // shift stored values to left (zero index gone)
    for (int i=0; i<LAST_MEASURMENTS_SIZE-1; i++)
      last_measurments[i] = last_measurments[i+1];
    // store new value to the last index
    last_measurments[LAST_MEASURMENTS_SIZE-1] = display_value;
    
    // find max
    float max_value = MIN_FORCE;
    for (int i=0; i<LAST_MEASURMENTS_SIZE; i++)
    {
      if (last_measurments[i] > max_value)
        max_value = last_measurments[i];
    }
    
    display_value = max_value;
  
    int value_x10_int = int(display_value*10);
    display.showNumberDecEx(value_x10_int, (0x80 >> 2), false);
  }
  if (blink_mode)
  {
    display.setBrightness(DISPLAY_BRIGHTNESS, display_blink_state_on);  // Turn off/on
    display_blink_state_on = !display_blink_state_on;
  }
  else
    display.setBrightness(DISPLAY_BRIGHTNESS, true);  // Turn on
  
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
