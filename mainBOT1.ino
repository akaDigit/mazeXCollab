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
const int l = A5, cl = A4, cr = A1, c = A3, r = A0;
int lValue = 0, clValue = 0, cValue = 0, crValue = 0;
int rValue = 0;

// ULTRASOUND
const int trig = 2, echo = 3;
long duration;
bool obstaclePresent = false;

// SERVO
Servo s;
const int servoPin = 11;
bool treasurePickedUp = false;


//  FUNCTION DECLARATIONS
void rotateMotor(int, int);
void treasureCheck();
void mazeAlgo();
void readIR();
void printIR();

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
  // if(!treasurePickedUp)
  //   treasureCheck();
  readIR();
  printIR();
  turningDecisions();
  mazeAlgo();
  // delay(500);
}

void printIR()
{
  // Serial.println(cValue);
  // Serial.println(clValue);
  // Serial.println(crValue);
  // Serial.println(lValue);
  // Serial.println(rValue);
  // Serial.println("\n");
}

void turningDecisions()
{
  // if(!turning)
  {
    if(lValue == LOW && clValue == LOW && cValue == HIGH && crValue == LOW && rValue == LOW)
    {
      if(currentSpeed < MAX_SPEED)
      {
      rotateMotor(currentSpeed, currentSpeed);
      currentSpeed = currentSpeed + 1;
      }
      else
      {
      rotateMotor(currentSpeed, currentSpeed);
      }

      //rotateMotor(255, 255);
    }

      //  ask pranay to do math if implementation of PID (without I) is required
      //  implement dE/dt by E2 - E1 / t2 - t1 for now -> t2 - t1 is loop delay (here 500 ms), math is required for E1, new Error which is
      //  calculated not read. only 3cm gap is read.

  // Serial.println("hello");
    else if(clValue == HIGH)
    {
      //for(int i = 0; i < 5; i ++)
      //{
        if(currentSpeed < MAX_SPEED - 10)
        {
        rotateMotor(currentSpeed, currentSpeed + 1);
        currentSpeed = currentSpeed + 1;
        }
        else
        {
          rotateMotor(currentSpeed - 1, currentSpeed + 1);
        }
        //delay(50);
      //}
    }
    else if(crValue == HIGH)
    {
      //for(int i = 0; i < 5; i ++)
      //{
        if(currentSpeed < MAX_SPEED - 10)
        {
        rotateMotor(++currentSpeed, currentSpeed);
        currentSpeed = currentSpeed + 1;
        }
        else
        {
          rotateMotor(currentSpeed + 1, currentSpeed - 1);
        }
        //delay(50);
      //}
    }
    // else if(rValue == HIGH)
    // {
    //   turning = true;
    //   rotateMotor(TURN_SPEED, 0);
    //   delay(TURN_DURATION);
    //   turning = false;
    // }
    // else if(lValue == HIGH)
    // {
    //   turning = true;
    //   rotateMotor(TURN_SPEED, 0);
    //   delay(TURN_DURATION);
    //   turning = false;
    // }
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
  turning = true;
  rotateMotor(TURN_SPEED, 0);
  delay(TURN_DURATION);
  turning = false;
  currentSpeed = BASE_SPEED;
}

void turnRight() {
  turning = true;
  rotateMotor(0, TURN_SPEED);
  delay(TURN_DURATION);
  turning = false;
  currentSpeed = BASE_SPEED;
}
void uTurn() {
  turning = true;
  rotateMotor(TURN_SPEED, -TURN_SPEED);
  delay(2*TURN_DURATION);
  turning = false;
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

  if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed == 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }


  if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed == 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  if(leftMotorSpeed != 0)
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));

  if(rightMotorSpeed != 0)
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
}



void mazeAlgo(){
  if (rValue == HIGH && turning == false)
  {
    turnRight();
  }
  else if(lValue == HIGH  && turning == false) 
  { 
    turnLeft();
  }
  else if(cValue == LOW && lValue == LOW && rValue == LOW && turning == false)
  {
    uTurn();
  }
}
