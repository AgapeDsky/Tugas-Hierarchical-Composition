#include <Arduino.h>

// Plat params
#define tau 1.2
#define T 0.008

// Control values
float effort = 0, prev_effort = 0;
float output = 0, prev_output = 0;

/**
 * @brief Function to compute plant's response
 * 
 * @return float
 */
float compute(void);

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Check serial buffer
  if (Serial.available()) {
    bool operating = Serial.parseInt();

    // if operating is 1, then control should be computed
    if (operating) {
      prev_effort = effort;
      prev_output = output;

      effort = Serial.parseFloat();
      output = compute();

      Serial.println(output);
    }
    // else, nothing should be done
    else {
      effort = 0;
      output = 0;
      prev_effort = 0;
      prev_output = 0;
    }
    
  }
}

float compute() {
	float retVal = prev_effort/(2*tau+T)*T + effort/(2*tau+T)*T - prev_output*(T-2*tau)/(2*tau+T);
	return retVal;
}