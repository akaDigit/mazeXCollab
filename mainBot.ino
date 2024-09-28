#include <Servo.h>
#define MAX_SPEED 255
#define BASE_SPEED 200  // prototype
#define TURN_SPEED 200
#define TURN_DURATION 1000   // milliseconds

// MOTORS
//Right motor
int enableRightMotor = 9, rightMotorPin1 = 6, rightMotorPin2 = 5;

//Left motor
int enableLeftMotor = 10, leftMotorPin1 = 8, leftMotorPin2 = 7;

//  MOVEMENT
int currentSpeed = BASE_SPEED;
int sv = 0, E = 0, E_ = 0;  // low Kp         sv = Kp * E + Kd * dE/dt    E_ = dE/dt
const int Kp = 5, Kd = 5;
bool turning = false;

// INFRA-RED
const int l = A0, cl = A4, c = A1, cr = A3, r = A5;
int lValue = 0, clValue = 0, cValue = 0, crValue = 0;
int rValue = 0;

// ULTRASOUND
const int trig = NULL, echo = NULL;
long duration;
bool obstaclePresent = false;

// SERVO
Servo s;
const int servoPin = NULL;
bool treasurePickedUp = false;


//  FUNCTION DECLARATIONS
void rotateMotor(int, int);\
void treasureCheck();

void setup() {

  //  MOTORS - pin setup
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  rotateMotor(0, 0);  // make sure they are stopped to start with


  // IRs - pin setup
  pinMode(l, INPUT);
  pinMode(cl, INPUT);
  pinMode(cr, INPUT);
  pinMode(r, INPUT);
  pinMode(c, INPUT);


  // US - pin setup
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);


  // Servo - pin setup
  s.attach(servoPin);

  Serial.begin(9600);
}


/*
  MAIN CODE
*/
void loop() {
  if(!treasurePickedUp)
    treasureCheck();
  turningDecisions();
  delay(500);
}

void turningDecisions()
{
  if(!turning)
  {
    if(lValue == LOW && clValue == LOW && cValue == HIGH && crValue == LOW && rValue == LOW)
    {
      if(currentSpeed < MAX_SPEED)
      rotateMotor(++currentSpeed, ++currentSpeed);
    }
    else if(clValue == HIGH)
    {
      //  ask pranay to do math if implementation of PID (without I) is required
      //  implement dE/dt by E2 - E1 / t2 - t1 for now -> t2 - t1 is loop delay (here 500 ms), math is required for E1, new Error which is
      //  calculated not read. only 3cm gap is read.

      for(int i = 0; i < 5; i ++)
      {
        if(currentSpeed < MAX_SPEED - 10)
        rotateMotor(currentSpeed, ++currentSpeed);
        else
        {
          rotateMotor(currentSpeed - 1, currentSpeed + 1);
        }
        delay(50);
      }
    }
    else if(crValue == HIGH)
    {
      for(int i = 0; i < 5; i ++)
      {
        if(currentSpeed < MAX_SPEED - 10)
        rotateMotor(++currentSpeed, currentSpeed);
        else
        {
          rotateMotor(currentSpeed + 1, currentSpeed - 1);
        }
        delay(50);
      }
    }
    else if(rValue == HIGH)
    {
      turning = true;
      rotateMotor(TURN_SPEED, 0);
      delay(TURN_DURATION);
      turning = false;
    }
    else if(lValue == HIGH)
    {
      turning = true;
      rotateMotor(TURN_SPEED, 0);
      delay(TURN_DURATION);
      turning = false;
    }
  }
}

/*
  INFRA - RED
*/
void readIR() {
  lValue = digitalRead(l);
  clValue = digitalRead(cl);
  cValue = digitalRead(c);
  crValue = digitalRead(cr);
  rValue = digitalRead(r);
}



/*
  MOVEMENT
*/
void turnLeft() {
}

void turnRight() {
}


void treasureCheck()
{
  
  if(lValue == HIGH && clValue == HIGH && cValue == HIGH && crValue == HIGH && rValue == HIGH && obstaclePresent)
  {
    Serial.println("Treasure Picked Up");
    treasurePickedUp = true;
    return;
  }
}


/* 
  Code that drives the motors according to required input.
*/
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {

  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed == 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }


  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed == 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
}