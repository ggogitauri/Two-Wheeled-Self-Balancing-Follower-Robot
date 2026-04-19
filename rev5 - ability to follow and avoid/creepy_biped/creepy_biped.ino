    /********** Self-Balancing Robot **********/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <AFMotor.h>
#include <PID_v1.h>
#include <Servo.h>

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
double setpoint = 0; //


double Kp = 35;   // 40.0, 35.0
double Ki = 350;    //  500, 350
double Kd = 0.8;    // 1.0, 0.8

PID balancePID(&pitchInput, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Outer PID ---
double outerInput, outerOutput, outerSetpoint = 0;  // outerSetpoint is always 0 (we want zero imbalance)
double outerKp = 0.010;  // 0.010// keep very small
double outerKi = 0.010;  // 0.010
double outerKd = 0.001;  // 0.001

PID outerPID(&outerInput, &outerOutput, &outerSetpoint, outerKp, outerKi, outerKd, DIRECT);

// --- Yaw PID ---
double yawInput, yawOutput, yawSetpoint = 0;
double yawKp = 1.5;
double yawKi = 0.0;
double yawKd = 0.03;

PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);

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

// --- Servo & Ultrasonic ---
const int tr = A2;  //white
const int ech = A3; //black
const float DETECTION_RANGE = 40.0;  // cm
const double SWEEP_STEP = 10;            // degrees per step
const int SWEEP_DELAY = 10;          // ms between steps - servo speed
const int SWEEP_MIN = 0;
const int SWEEP_MAX = 120;
unsigned long lastDistanceCheck = 0;
const unsigned long DISTANCE_CHECK_INTERVAL = 385;

double distance = 0;
Servo myservo;

double currentAngle = 0;
int sweepDirection = 1;      // 1 = increasing, -1 = decreasing
bool objectFound = false;
int leftEdge = -1;
int rightEdge = -1;

// --- ULTRASONIC READ ---
float readDistance() {
    digitalWrite(tr, LOW);
    delayMicroseconds(2);
    digitalWrite(tr, HIGH);
    delayMicroseconds(10);
    digitalWrite(tr, LOW);
    
    float duration = pulseIn(ech, HIGH, 5000); //5ms timeout for when this bitch bugs out

    if (duration == 0){
        return -1;
    }

    return (duration * 0.0343) / 2;
}

// --- SERVO MOVE ---
void moveServo(double angle) {
    angle = constrain(angle, SWEEP_MIN, SWEEP_MAX);
    myservo.write(angle);
    delay(SWEEP_DELAY);  // wait for servo to settle before reading
    currentAngle = angle;
}

// --- SWEEP ---
void sweep() {
    currentAngle += sweepDirection * SWEEP_STEP;
    
    // reverse direction at limits
    if (currentAngle >= SWEEP_MAX) { // 150
        currentAngle = SWEEP_MAX;
        sweepDirection = -1;          // left
    } else if (currentAngle <= SWEEP_MIN) { // 0
        currentAngle = SWEEP_MIN;
        sweepDirection = 1;           // right
    }
    
    moveServo(currentAngle);
}


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
    
    yawPID.SetMode(AUTOMATIC);
    yawPID.SetOutputLimits(-150, 150); 

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    // Servo & Ultrasonic setup
    pinMode(tr, OUTPUT);
    pinMode(ech, INPUT);
    myservo.attach(9);
    moveServo(0);  // start at 0 degrees

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
int absLeftSpeed = 0, absRightSpeed = 0;

void setMotors(int speed, int turnBias) { //  (int)pidOutput, (int)yawOutput
    
    int leftSpeed = constrain(speed + turnBias, -240, 240);
    int rightSpeed = constrain(speed - turnBias, -240, 240);

    absLeftSpeed = abs(leftSpeed);
    absRightSpeed = abs(rightSpeed);

    if (leftSpeed > 0) {
        leftMotor.run(FORWARD);
    } else if (leftSpeed < 0) {
        leftMotor.run(BACKWARD);
    } else {
        leftMotor.run(RELEASE);   
    }
    leftMotor.setSpeed(absLeftSpeed);
    

    if (rightSpeed > 0) {
        rightMotor.run(BACKWARD);
    } else if (rightSpeed < 0) {
        rightMotor.run(FORWARD);
    } else {
        rightMotor.run(RELEASE);
    }
    rightMotor.setSpeed(absRightSpeed);  
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
        int turnBias = 60;  // tune this - difference between left and right
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
        yawInput = ypr[0] * 180 / M_PI;  // Yaw angle in degrees

        // Kill motors if robot has fallen over
        if (abs(pitchInput) > 30) {
            setMotors(0, 0);
            balancePID.SetMode(MANUAL);   // stop PID accumulating integral while fallen
            pidOutput = 0;
            balancePID.SetMode(AUTOMATIC);
            yawSetpoint = yawInput;         // Reset yaw setpoint to current heading
            setpoint = bankedSetpoint;
            
            positiveCorrectionCount = 0;
            negativeCorrectionCount = 0;


            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
            return;
        }

        // Run PID
        balancePID.Compute();
        
         // Yaw setpoint to feed into YawPID based on Servo angle
        unsigned long noow = millis();
        if (noow - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
            
            distance = readDistance();
            //Serial.print("Angle: "); Serial.print(currentAngle);
            //Serial.print("\tDistance: "); Serial.println(distance);
            
            if (distance >= 0 && distance <= DETECTION_RANGE) { // object is within 25cm
                // object detected
                if (!objectFound) { // objectFound == false
                    objectFound = true;
                    yawSetpoint = (75 - currentAngle) * 0.5;
                }

            } 
            else {
              // no object - sweep
              objectFound = false;
              sweep(); //10ms cost
            }
            lastDistanceCheck = noow;
        }
  
        yawPID.Compute();
        


        if (pidOutput > 0) positiveCorrectionCount += pidOutput*0.01;
        else if (pidOutput < 0) negativeCorrectionCount += abs(pidOutput*0.01);

				unsigned long now = millis();
        if (now - lastCorrectionCheck >= CORRECTION_CHECK_INTERVAL) {
            outerInput = positiveCorrectionCount - negativeCorrectionCount;  // imbalance is the error
            outerPID.Compute();

            if (objectFound && distance > 20){
                setpoint = constrain(1.75 - outerOutput, SETPOINT_MIN, SETPOINT_MAX);
            }
            else if (objectFound && distance < 10){
                setpoint = constrain(-1.75 - outerOutput, SETPOINT_MIN, SETPOINT_MAX);
            }
            else {
                setpoint = constrain(bankedSetpoint - outerOutput, SETPOINT_MIN, SETPOINT_MAX);

            }
            
            
            positiveCorrectionCount *= 0.2;  // decay instead of reset
            negativeCorrectionCount *= 0.2;

            lastCorrectionCheck = now;
        }

        setMotors((int)pidOutput, (int)yawOutput);

										 
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);


// 		    Serial.print("yawInput: ");  Serial.print(yawInput);	   
//         // Serial.print("outerInput: ");  Serial.print(outerInput);
//         Serial.print("\tyawOutput: ");  Serial.print(yawOutput);
//         Serial.print("\tyawSetpoint: ");  Serial.print(yawSetpoint);
// //         Serial.print("\touterOutput: ");  Serial.print(outerOutput);
//         Serial.print("\tPitch: ");  Serial.print(pitchInput);
// //        Serial.print("\tbankedSetpoint: ");  Serial.print(bankedSetpoint);
//         Serial.print("\tSetpoint: ");  Serial.print(setpoint);
//         Serial.print("\tPID: ");  Serial.print(pidOutput);
//         Serial.print("\tLeft: ");  Serial.print(absLeftSpeed);
//         Serial.print("\tRight: ");  Serial.println(absRightSpeed);

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