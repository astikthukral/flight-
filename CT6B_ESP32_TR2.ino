#include <Arduino.h>
#include "driver/ledc.h"


// ---------------- Configuration ----------------
#define CALIB_PIN        0      // GPIO0: hold LOW at boot to enter calibration mode
#define CALIB_SWITCH_PIN 27     // GPIO27: toggle switch for calibration mode
#define SERIAL_BAUD      115200


// RC input pins (FlySky CT6B receiver)
struct RcChannel {
  uint8_t pin;
  volatile uint32_t riseTime;
  volatile uint16_t pulseWidth; // in microseconds
  volatile bool newValue;
};


RcChannel channels[6] = {
  {34, 0, 1500, false}, // CH1
  {35, 0, 1500, false}, // CH2
  {4 , 0, 1500, false}, // CH3 (throttle)
  {2 , 0, 1500, false}, // CH4
  {15, 0, 1500, false}, // CH5
  {14, 0, 1500, false}  // CH6
};


// ESC outputs: GPIOs for the six motors
struct EscOut {
  int pin;
  ledc_channel_t channel;
  ledc_timer_t timer;
  ledc_mode_t mode;
};


EscOut escOuts[6] = {
  {5 , LEDC_CHANNEL_0, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE},
  {18, LEDC_CHANNEL_1, LEDC_TIMER_1, LEDC_LOW_SPEED_MODE},
  {23, LEDC_CHANNEL_2, LEDC_TIMER_2, LEDC_LOW_SPEED_MODE},
  {13, LEDC_CHANNEL_3, LEDC_TIMER_3, LEDC_LOW_SPEED_MODE},
  {19, LEDC_CHANNEL_4, LEDC_TIMER_0, LEDC_HIGH_SPEED_MODE},
  {25, LEDC_CHANNEL_5, LEDC_TIMER_1, LEDC_HIGH_SPEED_MODE}
};


// PWM timing for ESCs
const uint32_t PWM_FREQ_HZ   = 50;   // 50Hz servo PWM
const uint8_t  PWM_RES_BITS  = 16;   // 16-bit duty
const uint32_t PWM_MAX_DUTY  = (1UL << PWM_RES_BITS) - 1;
const uint16_t MIN_US        = 1000; // 1.0ms
const uint16_t MAX_US        = 2000; // 2.0ms
const uint32_t PERIOD_US     = 1000000UL / PWM_FREQ_HZ; // 20000us


// ---------------- RC ISRs ----------------
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


inline void IRAM_ATTR onEdge(RcChannel &ch) {
  int level = digitalRead(ch.pin);
  uint32_t now = micros();
  if (level) {
    ch.riseTime = now;
  } else {
    uint32_t width = now - ch.riseTime;
    if (width >= 800 && width <= 2500) {
      ch.pulseWidth = (uint16_t)width;
      ch.newValue = true;
    }
  }
}


void IRAM_ATTR handleChange0() { onEdge(channels[0]); }
void IRAM_ATTR handleChange1() { onEdge(channels[1]); }
void IRAM_ATTR handleChange2() { onEdge(channels[2]); }
void IRAM_ATTR handleChange3() { onEdge(channels[3]); }
void IRAM_ATTR handleChange4() { onEdge(channels[4]); }
void IRAM_ATTR handleChange5() { onEdge(channels[5]); }


void setupChannelPin(uint8_t idx) {
  pinMode(channels[idx].pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(channels[idx].pin), isrList[idx], CHANGE);
}


// ---------------- LEDC helpers ----------------
uint32_t usToDuty(uint16_t us) {
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  return (uint32_t)((uint64_t)us * PWM_MAX_DUTY / PERIOD_US);
}


void setupPwmOutputs() {
  bool timerConfigured[2][4] = {}; // [modeIdx][timer]


  for (int i = 0; i < 6; i++) {
    int modeIdx = (escOuts[i].mode == LEDC_LOW_SPEED_MODE) ? 0 : 1;
    if (!timerConfigured[modeIdx][escOuts[i].timer]) {
      ledc_timer_config_t tcfg = {
        .speed_mode = escOuts[i].mode,
        .duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS,
        .timer_num = escOuts[i].timer,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
      };
      ledc_timer_config(&tcfg);
      timerConfigured[modeIdx][escOuts[i].timer] = true;
    }


    pinMode(escOuts[i].pin, OUTPUT);
    ledc_channel_config_t ccfg = {
      .gpio_num = escOuts[i].pin,
      .speed_mode = escOuts[i].mode,
      .channel = escOuts[i].channel,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = escOuts[i].timer,
      .duty = usToDuty(MIN_US),
      .hpoint = 0
    };
    ledc_channel_config(&ccfg);
  }
}


void writeEscPulseAll(uint16_t us) {
  uint32_t duty = usToDuty(us);
  for (int i = 0; i < 6; i++) {
    ledc_set_duty(escOuts[i].mode, escOuts[i].channel, duty);
    ledc_update_duty(escOuts[i].mode, escOuts[i].channel);
  }
}


// ---------------- Calibration control ----------------
bool calibrationRequested = false;


bool checkCalibrationJumper() {
  pinMode(CALIB_PIN, INPUT_PULLUP); // LOW when shorted to GND
  delay(5);
  return digitalRead(CALIB_PIN) == LOW;
}


bool checkToggleSwitchCalibration() {
  pinMode(CALIB_SWITCH_PIN, INPUT_PULLUP);
  delay(10);
  
  if (digitalRead(CALIB_SWITCH_PIN) == LOW) {
    Serial.println("Calibration switch is ON - Entering calibration mode!");
    return true;
  }
  
  Serial.println("Calibration switch is OFF - Normal flight mode");
  return false;
}


void runEscCalibration() {
  Serial.println("\n=== ESC CALIBRATION MODE ===");
  Serial.println("Remove props. Ensure ESC GND is common with ESP32 GND.");
  Serial.println("1) Connect LiPo now if not already connected.");
  Serial.println("2) ESCs will receive MAX throttle for 4s, then MIN for 6s.");


  // MAX phase
  Serial.println("Sending MAX (2000us) to all ESCs...");
  writeEscPulseAll(MAX_US);
  delay(4000);


  // MIN phase
  Serial.println("Sending MIN (1000us) to all ESCs...");
  writeEscPulseAll(MIN_US);
  delay(6000);


  Serial.println("Calibration sequence complete. Power-cycle ESCs if required by your model.");
  Serial.println("Exiting calibration mode, keeping outputs at MIN.");
}


// ---------------- Setup / Loop ----------------
uint32_t lastPrint = 0;


void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);


  // RC input setup
  for (uint8_t i = 0; i < 6; i++) setupChannelPin(i);


  // ESC outputs setup
  setupPwmOutputs();
  writeEscPulseAll(MIN_US); // safe at boot


  Serial.println("ESP32 Hexacopter (CH3 -> 6 ESC @50Hz) with Toggle Switch Calibration");


  // Check calibration methods: jumper or toggle switch
  bool jumper = checkCalibrationJumper();
  bool toggleSwitch = false;
  if (!jumper) {
    toggleSwitch = checkToggleSwitchCalibration();
  }
  calibrationRequested = jumper || toggleSwitch;


  if (calibrationRequested) {
    runEscCalibration();
  }
}


void loop() {
  // Snapshot RC channels
  noInterrupts();
  uint16_t chVals[6];
  for (uint8_t i = 0; i < 6; i++) {
    chVals[i] = channels[i].pulseWidth;
    channels[i].newValue = false;
  }
  interrupts();


  // Throttle from CH3
  uint16_t ch3 = chVals[2];
  if (ch3 < MIN_US) ch3 = MIN_US;
  if (ch3 > MAX_US) ch3 = MAX_US;


  // Output throttle to all ESCs
  writeEscPulseAll(ch3);


  // Debug print ~25Hz
  uint32_t now = millis();
  if (now - lastPrint >= 40) {
    lastPrint = now;
    Serial.printf("CH1:%u  CH2:%u  CH3:%u  CH4:%u  CH5:%u  CH6:%u  -> ESC(us): %u%s\n",
      chVals[0], chVals[1], chVals[2], chVals[3], chVals[4], chVals[5], ch3,
      calibrationRequested ? "  [Calibrated this boot]" : "");
  }


  delay(1);
}
