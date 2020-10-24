#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
#define SERIAL_BUD_SPEED 115200
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#include "CMpu.h"

void CMpu::dmpDataReady() {
    mpuInterrupt = true;
}

void CMpu::setMpu() {
        // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(SERIAL_BUD_SPEED);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu->initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu->dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu->setXGyroOffset(220);
    mpu->setYGyroOffset(76);
    mpu->setZGyroOffset(-85);
    mpu->setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu->setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); // TODO: place dmpDataReady in sketch
        mpuIntStatus = mpu->getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu->dmpGetFIFOPacketSize();
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

void CMpu::processMpu() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu->getFIFOCount();
        }  
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu->getIntStatus();

    // get current FIFO count
    fifoCount = mpu->getFIFOCount();
	if(fifoCount < packetSize){
	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	}
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu->resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
	while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
		mpu->getFIFOBytes(fifoBuffer, packetSize);
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
            mpu->dmpGetQuaternion(&q, fifoBuffer);
            mpu->dmpGetGravity(&gravity, &q);
            mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);

            mpuYaw = ypr[0] * 180 / M_PI;
            mpuPitch = ypr[1] * 180 / M_PI;
            mpuRoll = ypr[2] * 180 / M_PI;

            // Don't forget to choose axis
            if (mpuPitch > 0) {
              //anything...
            }
            // Don't forget to choose axis
             else if (mpuPitch < 0) {
              //anything...
             }
            
            Serial.print("ypr\t");
            Serial.print(mpuYaw);
            Serial.print("\t");
            Serial.print(mpuPitch);
            Serial.print("\t");
            Serial.println(mpuRoll);
        
        #endif


        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu->dmpGetQuaternion(&q, fifoBuffer);
            mpu->dmpGetAccel(&aa, fifoBuffer);
            mpu->dmpGetGravity(&gravity, &q);
            mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu->dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif

    }
}