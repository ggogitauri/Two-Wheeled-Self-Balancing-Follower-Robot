#include <Servo.h>


const int tr = A2;  //white
const int ech = A3; //black
const float DETECTION_RANGE = 15.0;  // cm
const double SWEEP_STEP = 10;            // degrees per step - tune this
const int SWEEP_DELAY = 10;          // ms between steps - tune this for servo speed
const int SWEEP_MIN = 0;
const int SWEEP_MAX = 120;
unsigned long lastDistanceCheck = 0;
const unsigned long DISTANCE_CHECK_INTERVAL = 400;

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


void setup() {

    pinMode(tr, OUTPUT);
    pinMode(ech, INPUT);
    // Serial.begin(9600);
    myservo.attach(9);
    moveServo(0);  // start at 0 degrees
}

void loop() {

        unsigned long noow = millis();
        if (noow - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) { //every 400ms
            
            distance = readDistance();
            //Serial.print("Angle: "); Serial.print(currentAngle);
            //Serial.print("\tDistance: "); Serial.println(distance);
            
            if (distance >= 0 && distance <= DETECTION_RANGE) { // object is within 10cm

            } 
            else {
              // no object - sweep
              objectFound = false;
              sweep(); //10ms cost
            }
            lastDistanceCheck = noow;
        }
    
    
}
