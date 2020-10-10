/* ============
 *  DEFINES
 * ============
 */

// Motor
#define PIN_ENA 9
#define PIN_IN1 8
#define PIN_IN2 7
#define PIN_ENGINE_INTERRUPT 0 
#define REDUCTION_COEFF (float)36.13
#define UPDATE_FREQ 5.f
#define PWM_TO_RPS (float)0.012

#include <Arduino.h>

/* ============
 *  VARIABLES
 * ============
 */

// Motor
uint8_t motorPower = 0;

volatile int tickCount = 0;
volatile unsigned long int timeStamp = 0;
volatile unsigned long int motorTimeStamp = 0;

float rvPerS = 0;

// PID - regulator vars
int pidSetPoint = 0;

int pidMin = 0;
int pidMax = 255;

int pidInput = 0;
int pidOutput = 0;

// PID - regulator coefs
float Kp = 1;
float Ki = 0;
float Kd = 0;
float dt_s = 0.2;

int pidPrevInput = 0;
float pidIntegral = 0.0;

float motorSpeed = 0;

/*
 * ============
 *  FUNCTIONS
 * ============
 */

float computePid(float input, float pidSetPoint) {
  float error = pidSetPoint - input;
  float d_input = pidPrevInput - input;

  pidPrevInput = input;
  pidIntegral += error * dt_s;

  pidOutput += Kp * error;
  pidOutput += Ki * pidIntegral;
  pidOutput += Kd * d_input * dt_s;

  pidOutput = abs(pidOutput);
  
  if (pidOutput < pidMin) {
    pidOutput = pidMin;
  }
   else if (pidOutput > pidMax) {
    pidOutput = pidMax;
   }


  return pidOutput / PWM_TO_RPS;
}

void setVelocity(float setspeed) {
  pidSetPoint = setspeed;
}

void activateInterrupt() {
  tickCount++;
}

// Print engine speed via Hall's sensor
void printEngineSpeed() { 
    rvPerS = tickCount / REDUCTION_COEFF * UPDATE_FREQ;
    tickCount = 0;
  }


/*
 * =======
 *  MAIN
 * =======
 */

void setup() {
    // Motors setup
    // Activate pins for the first engine
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENGINE_INTERRUPT, INPUT);

    attachInterrupt(PIN_ENGINE_INTERRUPT, activateInterrupt, RISING);

    // Turn off motors
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);

    Serial.begin(115200);
}

void loop() {

  if (millis() - motorTimeStamp >= 300) {
    setVelocity(1);
//    motorSpeed += 0.1;
//
//    if (motorSpeed > 3) {
//      motorSpeed = 0;
//    }
//    
    motorTimeStamp = millis();
  }

  
  if (millis() - timeStamp >= 200) {
    printEngineSpeed();
    unsigned char m = (unsigned char)computePid(rvPerS, pidSetPoint);
    analogWrite(PIN_ENA, m);
    //analogWrite(PIN_ENA, 255);
    Serial.println(rvPerS);
    Serial.println(pidOutput);
    Serial.println(m);
    timeStamp = millis();
  }
  
  
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
}
