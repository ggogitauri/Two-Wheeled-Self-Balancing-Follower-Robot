/********** Self-Balancing Robot **********/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <AFMotor.h>
#include <PID_v1.h>

// #define IR_RECEIVE_PIN 9
// #define USE_EXTENDED_NEC_PROTOCOL
// #include <TinyIRReceiver.hpp>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// --- MPU6050 ---
MPU6050 mpu;

#define INTERRUPT_PIN 2
#define LED_PIN 13

						  
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];


Quaternion q;
VectorFloat gravity;
float ypr[3];

			
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// --- Motors ---
 AF_DCMotor leftMotor(1);   // M1
AF_DCMotor rightMotor(4);  // M4

// --- PID ---
double pitchInput, pidOutput;
double setpoint = 0; //2.30


double Kp = 35;   //  40.0, 35.0
double Ki = 350;    //  500, 350
double Kd = 0.8;    //  1.0, 0.8

PID balancePID(&pitchInput, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Outer PID ---
double outerInput, outerOutput, outerSetpoint = 0;  // outerSetpoint is always 0 (we want zero imbalance)
double outerKp = 0.010;  // 0.010// keep very small  
double outerKi = 0.010;  // 0.010
double outerKd = 0.001;  // 0.001

PID outerPID(&outerInput, &outerOutput, &outerSetpoint, outerKp, outerKi, outerKd, DIRECT);


// --- IR ---
const double SETPOINT_STEP = 1.0;
const double SETPOINT_MIN  = -20.0;
const double SETPOINT_MAX  =  20.0;
double bankedSetpoint = setpoint;  // stores the banked value

/*
// --- Square Demo ---
bool demoRunning = false;
int demoStep = 0;                           // 0-3 = sides, 4-7 = turns
unsigned long demoStepStart = 0;

const double DEMO_FORWARD_OFFSET =  0.5;   // degrees - lean forward to drive
const double DEMO_TURN_OFFSET    =  0.0;   // no lean during turn
const unsigned long DEMO_DRIVE_TIME = 3000; // ms per side  
const unsigned long DEMO_TURN_TIME  = 600;  // ms per 90 degree turn 
*/

// --- Dynamic Setpoint Correction ---
double positiveCorrectionCount = 0;
double negativeCorrectionCount = 0;
unsigned long lastCorrectionCheck = 0;
const unsigned long CORRECTION_CHECK_INTERVAL = 400;  // check every 400ms
const double DYNAMIC_SETPOINT_RANGE = 3.0;  // max degrees it can drift from base



bool blinkState = false;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    delay(5000);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

//  Serial.begin(115200);

//    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(5958); //5627 
    mpu.setYAccelOffset(5866); //5893
    mpu.setZAccelOffset(9246); //9263
    mpu.setXGyroOffset(-33); // -30
    mpu.setYGyroOffset(-53); // -52
    mpu.setZGyroOffset(4);   //  1
							  
							  
							  




    if (devStatus == 0) {
													   
        //mpu.CalibrateAccel(6);
        //mpu.CalibrateGyro(6);
        //mpu.PrintActiveOffsets();

//        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

//        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

//        Serial.println(F("DMP ready!"));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
//     else {
// //        Serial.print(F("DMP Initialization failed (code "));
// //        Serial.print(devStatus);
// //        Serial.println(F(")"));
//     }

    // PID setup
    balancePID.SetMode(AUTOMATIC);
    balancePID.SetSampleTime(10);                  // compute every 10ms
    balancePID.SetOutputLimits(-200, 200);         // match motor speed range

    outerPID.SetMode(AUTOMATIC);
    outerPID.SetOutputLimits(-DYNAMIC_SETPOINT_RANGE, DYNAMIC_SETPOINT_RANGE); // max degrees it can shift setpoint
    

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

/*
    // IR setup
    initPCIInterruptForTinyReceiver();
*/
/*
    demoRunning = true;
    demoStep = 0;
    demoStepStart = millis();
    bankedSetpoint = setpoint;
*/
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                     MOTOR CONTROL                        ===
// ================================================================

void setMotors(int speed) { //(int)pidoutput
    
    speed = constrain(speed, -240, 240);
    int absSpeed = abs(speed);

    if (speed > 0) {
        leftMotor.run(FORWARD);
        rightMotor.run(BACKWARD);
    } else if (speed < 0) {
        leftMotor.run(BACKWARD);
        rightMotor.run(FORWARD);
    } else {
        leftMotor.run(RELEASE);
        rightMotor.run(RELEASE);
    }

    leftMotor.setSpeed(absSpeed);
    rightMotor.setSpeed(absSpeed);
    
}


// ================================================================
// ===                     IR HANDLING                          ===
// ================================================================
/*
void handleIR() {
    if (TinyReceiverDecode()) {
        if (TinyIRReceiverData.Flags != IRDATA_FLAGS_IS_REPEAT) {
            switch (TinyIRReceiverData.Command) {
                case 0x16:  // "up"
                    setpoint = constrain(setpoint + SETPOINT_STEP, SETPOINT_MIN, SETPOINT_MAX);
                    break;
                case 0x14:  // "down"
                    setpoint = constrain(setpoint - SETPOINT_STEP, SETPOINT_MIN, SETPOINT_MAX);
                    break;
                case 0x0:  // up 7 degrees
                    setpoint = constrain(setpoint + 2.0, SETPOINT_MIN, SETPOINT_MAX);
                    break;
                case 0x1:  // down 7 degrees
                    setpoint = constrain(setpoint - 2.0, SETPOINT_MIN, SETPOINT_MAX);
                    break;
                case 0xF:  // bank current setpoint
                    bankedSetpoint = setpoint;
                    break;
                case 0xB:  // restore banked setpoint
                    setpoint = bankedSetpoint;
                    break;
                default:
                    break;
            }
        }
    }
}
*/
/*
void handleDemo() {
    if (!demoRunning) return;

    unsigned long now = millis();
    bool isDriving = (demoStep % 2 == 0);  // even steps = drive, odd = turn

    unsigned long stepDuration = isDriving ? DEMO_DRIVE_TIME : DEMO_TURN_TIME;

    if (now - demoStepStart >= stepDuration) {
        demoStep++;
        demoStepStart = now;

        if (demoStep >= 8) {  // 4 sides + 4 turns complete
            demoRunning = false;
            demoStep = 0;
            setpoint = bankedSetpoint;  // restore original setpoint
            return;
        }
    }

    if (isDriving) {
        // lean forward by offsetting setpoint
        setpoint = bankedSetpoint + DEMO_FORWARD_OFFSET;
        // both motors same speed - balance loop handles actual motor output
    } else {
        // turn by giving each motor a different speed
        // balance loop still runs, we just bias the motors
        setpoint = bankedSetpoint;
        int turnBias = 60;  //  difference between left and right
        leftMotor.setSpeed(constrain((int)abs(pidOutput) + turnBias, 0, 240));
        rightMotor.setSpeed(constrain((int)abs(pidOutput) - turnBias, 0, 240));
    }
}
*/
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if (!dmpReady) return;

//    handleIR();
//    handleDemo();

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

        // Get pitch angle from DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        pitchInput = ypr[1] * 180 / M_PI;  // feed pitch into PID input

        // Kill motors if robot has fallen over
        if (abs(pitchInput) > 30) {
            setMotors(0);
            balancePID.SetMode(MANUAL);   // stop PID accumulating integral while fallen
            pidOutput = 0;
            balancePID.SetMode(AUTOMATIC);

            
            positiveCorrectionCount = 0;
            negativeCorrectionCount = 0;


            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
            return;
        }

        // Run PID
        balancePID.Compute();


        if (pidOutput > 0) positiveCorrectionCount += pidOutput*0.01;
        else if (pidOutput < 0) negativeCorrectionCount += abs(pidOutput*0.01);

				unsigned long now = millis();
        if (now - lastCorrectionCheck >= CORRECTION_CHECK_INTERVAL) {
            outerInput = positiveCorrectionCount - negativeCorrectionCount;  // imbalance is the error
            outerPID.Compute();
            setpoint = constrain(bankedSetpoint - outerOutput, SETPOINT_MIN, SETPOINT_MAX);

            
            positiveCorrectionCount *= 0.2;  // decay instead of reset
            negativeCorrectionCount *= 0.2;

            lastCorrectionCheck = now;
        }

        setMotors((int)pidOutput);

										 
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

			   
//         Serial.print("outerInput: ");  Serial.print(outerInput);
//         Serial.print("\touterOutput: ");  Serial.print(outerOutput);
//         Serial.print("\tPitch: ");  Serial.print(pitchInput);
// //        Serial.print("\tbankedSetpoint: ");  Serial.print(bankedSetpoint);
//        Serial.print("\tSetpoint: ");  Serial.print(setpoint);
//         Serial.print("\tPID: ");  Serial.println(pidOutput);

// plotter---------------------

// Serial.print("outerInput:");
// Serial.print(outerInput);
// Serial.print("\t");
// Serial.print("outerOutput:");
// Serial.print(outerOutput);
// Serial.print("\t");
// Serial.print("pitch:");
// Serial.print(pitchInput);
// Serial.print("\t");
// Serial.print("setpoint:");
// Serial.print(setpoint);
// Serial.print("\t");
// Serial.print("PID:");
// Serial.println(pidOutput);

    }
}