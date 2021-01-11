//====================
//  ARDUINO
//====================
#define SERIAL_BUD_SPEED 115200 // check platformio.ini's monitor_speed and upload_speed 
                                // and change or set this value from/in file

//====================
//  MPU SETTINGS
//====================

// Uncomment define if you want to set your own offset for MPU's sensivity
// #define MPU_SET_OFFSET

#ifdef MPU_SET_OFFSET
    #define MPU_OFFSET_GYRO_X 220
    #define MPU_OFFSET_GYRO_Y 76
    #define MPU_OFFSET_GYRO_Z -85
    #define MPU_OFFSET_ACCEL_Z 1788
#endif

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
 
// What interrupt pin will receive data from MPU
#define MPU_INTERRUPT_PIN 18

// MPU's PID regulator settings
#define MPU_PID_KP 7
#define MPU_PID_KI 0 
#define MPU_PID_KD 0

// Range of degrees that MPU's PID will output, 
// for setting motor's speed based on MPU's inclination angle
#define MPU_PID_MIN -100
#define MPU_PID_MAX 100

//====================
//  MOTOR SETTINGS
//====================

// INTERRUPT PLATE PINS (Arduino Mega / Mega 2560)
//  INT0 - 2 PIN
//  INT1 - 3 PIN
//  INT3 - 20 PIN
//  INT4 - 19 PIN
//  INT5 - 18 PIN

// Interrupts that will accept encoder's data from motors
#define MOTOR1_INTERRUPT 0
#define MOTOR2_INTERRUPT 1

// Check your motor's specifications of reduction coefficient
#define MOTOR_REDUCTION_COEF (float)18.8f

// Time period of motor processing
#define MOTOR_PROCESS_PERIOD 10.f // milliseconds

// Motor's PID regulator coefficients
#define MOTOR_PID_KP 8
#define MOTOR_PID_KI 0
#define MOTOR_PID_KD 0
#define MOTOR_PID_MIN 0 
#define MOTOR_PID_MAX 255

// Motor 1
// Arduino's ENA/ENB pins will transmit calculated PWM value to motor driver's ENA/ENB pins
#define PIN_MOTOR_ENA 9
#define PIN_MOTOR_IN1 8
#define PIN_MOTOR_IN2 7

// Motor 2 
#define PIN_MOTOR2_ENB 12
#define PIN_MOTOR2_IN3 11
#define PIN_MOTOR2_IN4 10

#include "../include/I2Cdev.h"
#include "../include/CMotor.h"
#include "../include/CPid.h"
#include "../include/MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "../include/Wire.h"
#endif

// why the heck can't this be packed in class???
MPU6050 mpu;
CPid pidMpu;

CMotor motor1(MOTOR1_INTERRUPT, PIN_MOTOR_ENA, PIN_MOTOR_IN1, PIN_MOTOR_IN2);
CMotor motor2(MOTOR2_INTERRUPT, PIN_MOTOR2_ENB, PIN_MOTOR2_IN3, PIN_MOTOR2_IN4);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float mpuDeg;
float mpuYaw;
float mpuPitch;
float mpuRoll;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void initMpu() {
    pidMpu.pidSetCoefs(MPU_PID_KP, MPU_PID_KI, MPU_PID_KD);
    pidMpu.pidSetMinMax(MPU_PID_MIN, MPU_PID_MAX);
    pidMpu.computeMsToSeconds(10);

        // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(SERIAL_BUD_SPEED);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(MPU_INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    #ifdef MPU_SET_OFFSET
        mpu.setXGyroOffset(MPU_OFFSET_GYRO_X);
        mpu.setYGyroOffset(MPU_OFFSET_GYRO_Y);
        mpu.setZGyroOffset(MPU_OFFSET_GYRO_Z);
        mpu.setZAccelOffset(MPU_OFFSET_ACCEL_Z); // 1688 factory default for my test chip
    #endif

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void processMpu() {
        // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
	if(fifoCount < packetSize){
	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	}
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
	while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
	}

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            mpuYaw = ypr[0] * 180 / M_PI;
            mpuPitch = ypr[1] * 180 / M_PI;
            mpuRoll = ypr[2] * 180 / M_PI;

            // Serial.print("ypr\t");
            // Serial.print(mpuYaw);
            // Serial.print("\t");
            // Serial.print(mpuPitch);
            // Serial.print("\t");
            // Serial.println(mpuRoll);
        
        #endif


        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif

    }
}

void increaseMotor1EncoderTicks() {
    motor1.tickCount++;
}

void increaseMotor2EncoderTicks() {
    motor2.tickCount++;
}

void setup() {    
    initMpu();

    motor1.setup(MOTOR_REDUCTION_COEF, MOTOR_PROCESS_PERIOD, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_MIN, MOTOR_PID_MAX);
    motor2.setup(MOTOR_REDUCTION_COEF, MOTOR_PROCESS_PERIOD, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_MIN, MOTOR_PID_MAX);

    Serial.begin(SERIAL_BUD_SPEED);

    attachInterrupt(MOTOR1_INTERRUPT, increaseMotor1EncoderTicks, RISING);
    attachInterrupt(MOTOR2_INTERRUPT, increaseMotor2EncoderTicks, RISING);
}

void loop() {
    // If MPU works
    if (devStatus == 0) {
        processMpu();

        mpuDeg = (float)pidMpu.computePid(mpuPitch, 0);
        Serial.println(mpuDeg);
        

        // if (mpuDeg > 0) { // go to the left/right (check this)
        //     motor1.process(1, mpuDeg);
        //     motor2.process(1, mpuDeg);
        // }
    }
}