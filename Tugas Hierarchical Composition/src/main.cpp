#include <Arduino.h>
#include <TM1637.h>
#include <EEPROM.h>

#include "button_fsm.h"
#include "device_fsm.h"

// Plant constants
#define tau 1.2
#define T 0.008

// EEPROM address
#define KP_ADDRESS 4
#define KI_ADDRESS 5
#define KD_ADDRESS 6

// Display object
TM1637 display(12,11);

// Output structs
output_struct plus_res = {0,0,0}, minus_res = {0,0,0}, set_res = {0,0,0};
output device_res;

// FSMs
button_fsm plus, minus;        // plus and minus button, only edge detecting needed
button_fsm set;                // set and hold button
device_fsm device;             // controller

// Button outputs
volatile bool single_plus = 0, single_minus = 0, single_set = 0;
volatile bool hold_set = 0;
volatile bool auto_repeat_plus = 0, auto_repeat_minus = 0;

// PID params
float kp, ki, kd, prev_kp, prev_ki, prev_kd;

// Control params
volatile float prev_input = 0, cur_input = 0, prev_output = 0, cur_output = 0;

// Condition to check that the operational mode is on
bool running = 0;

void setup() {
  // Calculate Threshold
  /**
   * 
   * -> button execution threshold
   * Config: prescaler: 8
   *         frequency: 2 MHz -> 1 tick = 1/2M = 500ns
   *         interrupt will be invoked every 500ns * PERIOD
   *         lets say ISR is invoked every 100 us, then PERIOD = 200
   * 
   * -> display config
   * Config: prescaler: 1024
   *         frequency: 15625 Hz
   *         let's say the period would be 2500 ticks -> T = 2500/15625 = 160ms -> good enough!
  */     

  // Initialize FSM
  plus.setThresOne(100);       // clear debouncing, 10ms delay
  plus.setThresTwo(10000);
  plus.setThresThree(2500);
  plus.setThresFour(10000);

  minus.setThresOne(100);       // clear debouncing, 10ms delay
  minus.setThresTwo(10000);
  minus.setThresThree(2500);
  minus.setThresFour(10000);

  set.setThresOne(100);       // clear debouncing, 10 ms delay
  set.setThresTwo(10000);     // 1 s delay before autorepeat
  set.setThresFour(10000);    // 1 s delay after autorepeat before hold
  set.setThresThree(2500);    // 250 ms delay on autorepeat

  device.updateTarget(10.);

  // Initialize LED to show mode
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  // Initialize Serial
  Serial.begin(115200);

  // Initialize FSM pins
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

  // Initialize Timer Interurpt   
  cli();
  // Display
  TCCR1A = 0;
  TCCR1B = 0; TCCR1B |= 0b101;  // pres 8
  TIMSK1 = 0; TIMSK1 |= 0b1;    // enable ovf int
  TCNT1 = 65525-2500;           // 

  // Button handler
  TCCR2A = 0;
  TCCR2B = 0; TCCR2B |= 0b010;  // pres 8
  TIMSK2 = 0; TIMSK2 |= 0b1;    // enable ovf int
  TCNT2 = 255 - 200;            // max counter - PERIOD
  sei();

  // Initialize display
  display.init();
  display.set(7);

  // Read Kp, Ki, and Kd from EEPROM
  prev_kp = (float)((uint8_t)EEPROM.read(KP_ADDRESS))/2.;
  prev_ki = (float)((uint8_t)EEPROM.read(KI_ADDRESS))/2.;
  prev_kd = (float)((uint8_t)EEPROM.read(KD_ADDRESS))/2.;
  device.setKp(prev_kp);
  device.setKi(prev_ki);
  device.setKd(prev_kd);
  
}

void loop() {
  // Get time
  unsigned long start = millis();

  // Validate PID params at the EEPROM
  kp = device.getKp();
  ki = device.getKi();
  kd = device.getKd();

  if (kp != prev_kp) {
    prev_kp = kp;
    EEPROM.write(KP_ADDRESS, (uint8_t)((int)(prev_kp*2)));
  }
  if (ki != prev_ki) {
    prev_ki = ki;
    EEPROM.write(KI_ADDRESS, (uint8_t)((int)(prev_ki*2)));
  }
  if (kd != prev_kd) {
    prev_kd = kd;
    EEPROM.write(KD_ADDRESS, (uint8_t)((int)(prev_kd*2)));
  }

  // Update control target from potentiometer
  device.updateTarget(round(analogRead(A5)/1023.*10*10)/10.);

  // Check input to update feedback from plant
  if (Serial.available()) {
    prev_input = cur_input;
    prev_output = Serial.parseFloat();
  }

  // Execute process from button
  device_res = device.process(single_plus|auto_repeat_plus, single_minus|auto_repeat_minus, single_set, hold_set, prev_output);

  // Turn on indicator LEDs
  if (device_res.super_state == 0) {
    digitalWrite(7, 1);
    if (device_res.operational_state == 1) {
      digitalWrite(8, 1);
    }
    else {
      digitalWrite(8, 0);
    }
  }
  else {
    digitalWrite(7, 0);
    if (device_res.set_state == 0) {
      digitalWrite(8, 1); digitalWrite(9, 0); digitalWrite(10, 0);
    }
    else if (device_res.set_state == 1) {
      digitalWrite(8, 0); digitalWrite(9, 1); digitalWrite(10, 0);
    }
    else {
      digitalWrite(8, 0); digitalWrite(9, 0); digitalWrite(10, 1);
    }
  }

  // Check whether operational mode is running
  running = device_res.super_state == OPERATIONAL && device_res.operational_state == RUNNING;
  if (running) {
    // Operational mode is running, control effort needs to be sent to the plant
    Serial.print(";");
    Serial.print(1);
    Serial.print(";");
    Serial.print(device_res.val);
    Serial.print(";");
  }
  else {
    // Operational mode is not running, nothing needs to be sent
    Serial.print(";");
    Serial.print(0);
    Serial.print(";");
  }
  
  // Reset every button commands
  single_plus = 0;
  single_minus = 0;
  single_set = 0;
  hold_set = 0;
  auto_repeat_plus = 0;
  auto_repeat_minus = 0;

  // Introduce 8ms delay
  while (millis()-start<8) {}

}

/**
 * @brief Button handler ISR, invoked every 100 us to detect button commands and tackle debouncing issues
 */
ISR(TIMER2_OVF_vect) {
  // Reset counter
  TCNT2 = 255 - 200;

  // Read button inputs
  plus_res = plus.process(digitalRead(2));
  minus_res = minus.process(digitalRead(3));
  set_res = set.process(digitalRead(4));

  // Update button commands
  single_plus = plus_res.single ? 1 : single_plus;
  single_minus = minus_res.single ? 1 : single_minus;
  single_set = set_res.single ? 1 : single_set;
  hold_set = set_res.hold ? 1 : hold_set;
  auto_repeat_plus = plus_res.auto_repeat ? 1 : auto_repeat_plus;
  auto_repeat_minus = minus_res.auto_repeat ? 1 : auto_repeat_minus;
}

/**
 * @brief Display handler ISR, invoked every 160 ms to update display on TM1637
 */
ISR(TIMER1_OVF_vect) {
  // Reset counter
  TCNT1 = 65525-2500;

  // Display values
  if (!running) {
    display.display(0, (int)(device_res.val/10)%10);
    display.display(1, (int)(device_res.val/1)%10);
    display.point(1);
    display.display(2, (int)(device_res.val*10)%10);
    display.display(3, (int)(device_res.val*100)%10);
  }
  else {
    display.display(0, (int)(prev_output/10)%10);
    display.display(1, (int)(prev_output/1)%10);
    display.point(1);
    display.display(2, (int)(prev_output*10)%10);
    display.display(3, (int)(prev_output*100)%10);
  }
}