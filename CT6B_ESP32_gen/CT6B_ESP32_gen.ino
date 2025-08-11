/*
  ESP32 RC PWM reader for 6 channels (FlySky CT6B receiver)
  Measures pulse width (HIGH time) in microseconds on each pin and prints to Serial.
  Pins used:
    CH1 -> GPIO34 (input-only)
    CH2 -> GPIO35 (input-only)
    CH3 -> GPIO4
    CH4 -> GPIO2
    CH5 -> GPIO15
    CH6 -> GPIO14
*/

#include <Arduino.h>

struct RcChannel {
  uint8_t pin;
  volatile uint32_t riseTime;     // micros() at rising edge
  volatile uint16_t pulseWidth;   // computed width in microseconds
  volatile bool newValue;         // flag that a new width was captured
};

// Define channels with your mapping
RcChannel channels[6] = {
  {34, 0, 1500, false}, // CH1
  {35, 0, 1500, false}, // CH2
  {4 , 0, 1500, false}, // CH3
  {2 , 0, 1500, false}, // CH4
  {15, 0, 1500, false}, // CH5
  {14, 0, 1500, false}  // CH6
};

// Forward declare ISRs
void IRAM_ATTR handleChange0();
void IRAM_ATTR handleChange1();
void IRAM_ATTR handleChange2();
void IRAM_ATTR handleChange3();
void IRAM_ATTR handleChange4();
void IRAM_ATTR handleChange5();

typedef void (*isr_t)();
isr_t isrList[6] = {
  handleChange0, handleChange1, handleChange2,
  handleChange3, handleChange4, handleChange5
};

void setupChannelPin(uint8_t idx) {
  pinMode(channels[idx].pin, INPUT);  // internal pullups usually not needed; receiver drives the line
  attachInterrupt(digitalPinToInterrupt(channels[idx].pin), isrList[idx], CHANGE);
}

// Common ISR body to reduce duplication
inline void IRAM_ATTR onEdge(RcChannel &ch) {
  int level = digitalRead(ch.pin);
  uint32_t now = micros();
  if (level) {
    // Rising edge
    ch.riseTime = now;
  } else {
    // Falling edge: compute width
    uint32_t width = now - ch.riseTime; // handles micros() wrap intrinsically for small intervals
    // Constrain to plausible RC range to reject noise
    if (width >= 800 && width <= 2500) {
      ch.pulseWidth = (uint16_t)width;
      ch.newValue = true;
    }
  }
}

// Generate ISRs
void IRAM_ATTR handleChange0() { onEdge(channels[0]); }
void IRAM_ATTR handleChange1() { onEdge(channels[1]); }
void IRAM_ATTR handleChange2() { onEdge(channels[2]); }
void IRAM_ATTR handleChange3() { onEdge(channels[3]); }
void IRAM_ATTR handleChange4() { onEdge(channels[4]); }
void IRAM_ATTR handleChange5() { onEdge(channels[5]); }

void setup() {
  Serial.begin(115200);
  // Small delay to ensure Serial ready
  delay(200);

  for (uint8_t i = 0; i < 6; i++) {
    setupChannelPin(i);
  }

  Serial.println("ESP32 RC PWM reader started");
  Serial.println("CH1..CH6 pulse widths (us)");
}

uint32_t lastPrint = 0;

void loop() {
  // Print at ~25Hz (40ms) to keep output readable
  uint32_t now = millis();
  if (now - lastPrint >= 40) {
    lastPrint = now;

    // Snapshot values atomically
    noInterrupts();
    uint16_t chVals[6];
    bool newFlags[6];
    for (uint8_t i = 0; i < 6; i++) {
      chVals[i] = channels[i].pulseWidth;
      newFlags[i] = channels[i].newValue;
      channels[i].newValue = false; // clear after reading
    }
    interrupts();

    // Print values; if no new frame recently, still show last measured
    Serial.print("CH1:");
    Serial.print(chVals[0]);
    Serial.print("  CH2:");
    Serial.print(chVals[1]);
    Serial.print("  CH3:");
    Serial.print(chVals[2]);
    Serial.print("  CH4:");
    Serial.print(chVals[3]);
    Serial.print("  CH5:");
    Serial.print(chVals[4]);
    Serial.print("  CH6:");
    Serial.println(chVals[5]);
  }

  // Optional: add a small delay to yield
  delay(1);
}
