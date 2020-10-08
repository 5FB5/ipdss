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
#define UPDATE_FREQ 10.f

/* ============
 *  VARIABLES
 * ============
 */

// Motor
uint8_t motorPower = 0;

volatile int tickCount = 0;
volatile unsigned long int timeStamp = 0;

float rvPerS = 0;

// PID - regulator vars
int pidSetPoint = 0;

int pidMin = 0;
int pidMax = 255;

int pidInput = 0;
int pidOutput = 0;

// PID - regulator coefs
float Kp = 1.0;
float Ki = 1.0;
float Kd = 1.0;
float dt_s = 0.1;

int pidPrevInput = 0;
float pidIntegral = 0.0;

/*
 * ============
 *  FUNCTIONS
 * ============
 */

float computePid() {
  float error = pidSetPoint - pidInput;
  float d_input = pidPrevInput - pidInput;

  pidPrevInput = 
}

void activateInterrupt() {
  tickCount++;
}

// Print engine speed via Hall's sensor
void printEngineSpeed() {
  if (millis() - timeStamp >= 100) {
    rvPerS = tickCount / REDUCTION_COEFF * UPDATE_FREQ;
    Serial.print(rvPerS);

    tickCount = 0;
    timeStamp = millis();
  }
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
  printEngineSpeed();
}
