/*
 * Software to controll the motors of the UMI-RTX robot using multiple H-bidge IC's.
 * The software runs on an Arduino mega.
 * new version V0.6
 * 18/1/2018
 */
 
 /*
  * Changes in this ref:
  * - added basic movements.
  * - added motor stop at maximum.
  * - found a possible way to fix the yaw not properly following the elbow.
  * - added the option to set the yaw to follow the elbow.
  * - added serial communication to the vision host computer
  * 
  * Things that need to be added:
  * - zed encoder maximum needs to be changed.
  * - need to test everything.
  * - Think about restrictions of the robot. when starting from back a bit, then moving left then forwards. this causes problems. maybe the old software can calculate this.
  * - Exeption list with forbidden position. Motor might hit it's self if not implemented.
  * - 
  */

//Direction defines
#define RIGHT 2
#define LEFT  1
#define STOP  0

#define DEBUG
//#define TEST_POSITIONS
#define DEMO


#define SHOULDER //works
#define ZED// not tested
//#define WRIST_RH//works
//#define WRIST_LH//works
#define ELBOW// works
#define YAW //works
//#define WRIST
//toggles checking for power on.
//#define VOLTAGE

//way to test of the yaw following the elbow works.
#define FOLLOW_YAW

//this tests is the yaw follows the elbow properly.
#define TEST_YAW

//stops the motor when hitting the maximum encoder count.
#define MAXIMUM_LIMIT

//toggeling limits the movement the robot can make whem moving forward or backwards. 
//#define RESTRICK_MOVEMENT

//way to test if the serial connection works.
#define SERIAL_TEST
//pin to measure if power is present.
#define MEAS_VOLTAGE 14

//toggle to use basic or advanced movements. L-R basic is only yaw, advanced is whole robot.
#define BASIC_MOVEMENT

///////////////////
//maximum rage or motors: NUMBERS NEED TO BE CHECKED AND IMPLEMENTED!
#define SHOULDER_MAX_POS      2630 //correct   //5260
#define ELBOW_MAX_POS         2418//correct   // //2418
#define ZED_MAX_POS           1000   //2392//1196
#define WRIST_LH_MAX_POS      1220//2142 //1071
#define WRIST_RH_MAX_POS      1220//8442 // 44221
#define WRIST_MIN_POS        -1220
#define YAW_MAX_POS           1072//correct //2142
#define GRIPPER_MAX           1200
////////////////////////

//Motor pins need to change to real layout
#define SHOULDER_PLUS       22
#define SHOULDER_MIN        23
#define SHOULDER_ENABLE     4

#define ELBOW_PLUS          24
#define ELBOW_MIN           25
#define ELBOW_ENABLE        5

#define ZED_PLUS            26
#define ZED_MIN             27
#define ZED_ENABLE          6

#define WRIST_RH_PLUS       28
#define WRIST_RH_MIN        29
#define WRIST_RH_ENABLE     7

#define WRIST_LH_PLUS       30
#define WRIST_LH_MIN        31
#define WRIST_LH_ENABLE     8

#define YAW_PLUS            32
#define YAW_MIN             33
#define YAW_ENABLE          9

/////////////////////////////////////////////////////

//Encoder interrupt pins
#define SHOULDER_ENCODER          2
#define ELBOW_ENCODER             3
#define ZED_ENCODER               18
#define WRIST_RH_ENCODER          19
#define WRIST_LH_ENCODER          20
#define YAW_ENCODER               21
///////////////////////////////////////////////////////

//Encoder counters
static volatile int shoulderEncoder = 0;
static volatile int elbowEncoder = 0;
static volatile int zedEncoder = 0;
static volatile int wristRHEncoder = 0;
static volatile int wristLHEncoder = 0;
static volatile int yawEncoder = 0;
////////////////////////

// Target positions per axis
int shoulderTarget; //= 0;
int elbowTarget;// = 0;
int zedTarget;// = 0;
int wristRHTarget;// = 0;
int wristLHTarget;// = 0;
int yawTarget;// = 0;
///////////////////////////

//Move direction per motor
int elbowDirection = 0;
int shoulderDirection = 0;
int zedDirection = 0;
int wristRHDirection = 0;
int wristLHDirection = 0;
int yawDirection = 0;
////////////////////


//////////////

//init variables aka the initVar's
int initShoulder = 0;
int initZed = 0;
int initWristRH = 0;
int initWristLH = 0;
int initElbow = 0;
int initYaw = 0;
///////////////////////////////////////

//compensation position for the yaw
static int yawCounter = 0;
static volatile int yawEncoderCounter = 0;
boolean yawFollowElbow = false;  
boolean yawMiddle = false;
boolean movingForward = false;
boolean basicMovement = false;
boolean movingUp = false;
boolean movingDown = false;
boolean movingLeft = false;
boolean movingRight = false;

boolean wasStopped = false;



//position queues for all motors. used in demo mode
int shoulderPositionQueue[5] = {SHOULDER_MAX_POS/2, SHOULDER_MAX_POS/2, 0, 1300, SHOULDER_MAX_POS/2};
int elbowPositionQueue[5] = {ELBOW_MAX_POS/2, ELBOW_MAX_POS/2, ELBOW_MAX_POS/2, ELBOW_MAX_POS/2, ELBOW_MAX_POS/2};
int zedPositionQueue[5] = {ZED_MAX_POS/2, 400,ZED_MAX_POS/2, 400, ZED_MAX_POS/2};
int yawPositionQueue[5] = {YAW_MAX_POS/2, YAW_MAX_POS, YAW_MAX_POS/2, 0, YAW_MAX_POS/2};


//Motor controll functions

//the wrist has to be initialised in a diffrent way. the two motors, LH and RH, need to work together to find the right position.

//inits the provided motor by moving it till the encoders stop counting.
//demo's a few positions
void demoMode()
{
  for(int i = 0; i< 5; i++)
    {
        moveToPos(&zedTarget, zedPositionQueue[i], &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
        moveToPos(&shoulderTarget, shoulderPositionQueue[i], &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
        moveToPos(&elbowTarget, elbowPositionQueue[i], &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
        moveToPos(&yawTarget,yawPositionQueue[i], &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
        Serial.print("Position: ");
        Serial.println(i);

        delay(7000);
        Serial.print("ZED encoder count = ");
        Serial.println(getEncoderCount(ZED_PLUS));
        Serial.print("YAW encoder count = ");
        Serial.println(getEncoderCount(YAW_PLUS));
        Serial.print("Shoulder Encoder Count = ");
        Serial.println(getEncoderCount(SHOULDER_PLUS));
        Serial.print("Elbow encoder count = ");
        Serial.println(getEncoderCount(ELBOW_PLUS));
     }
}
//sets the motor so it moves to a given position
void moveToPos(int *target, int targetPosition, volatile int *encoder, int plus, int minus, int enable, int moveSpeed)
{
     basicMovement = false;
//  if(plus == WRIST_RH_PLUS)
//    *target = targetPosition * -1;
//  else
    #ifdef YAW_TEST
    if(plus == YAW_MOTOR_PLUS)
      *target = targetPostion + yawCounter;
    #else
    *target = targetPosition;
    #endif
  if(*target > *encoder)
    {
      //Serial.println("RIGHT");
       setDirection(plus, RIGHT);
       analogWrite(enable, moveSpeed);
       digitalWrite(plus,  HIGH);
       digitalWrite(minus, LOW);
    }
    else if(*target < *encoder)
    {
      //Serial.println("LEFT");
      setDirection(plus, LEFT);
      analogWrite(enable, moveSpeed);
      digitalWrite(plus,  LOW);
      digitalWrite(minus, HIGH);
    }
//    else
//      Serial.println("Position Error");
}
//maybe functions to set direction. This is ugly AF.

//moves motor for a given time
void moveMotor(int moveTime,int moveSpeed, int movementDirection, int plus, int minus, int enable)
{
  setDirection(plus, movementDirection);
  analogWrite(enable, moveSpeed);
  if(movementDirection == 2)
    {
      digitalWrite(plus, HIGH);
      digitalWrite(minus, LOW);
    }
    else if(movementDirection == 1)
    {
      digitalWrite(plus, LOW);
      digitalWrite(minus, HIGH);
      
    }
  delay(moveTime);
  digitalWrite(plus, LOW);
  digitalWrite(minus, LOW);
  analogWrite(enable, 0);
 }
//sets direction the motor is moving to appropriate value
void setDirection(int motorPlus,int moveDirection)
{
   switch(motorPlus)
  {
   case SHOULDER_PLUS:
    shoulderDirection = moveDirection;  
    break;
   case ELBOW_PLUS:
    elbowDirection = moveDirection;  
    break;
   case WRIST_RH_PLUS:
    wristRHDirection = moveDirection;  
    break;
   case WRIST_LH_PLUS:
    wristLHDirection = moveDirection;  
    break;
   case ZED_PLUS:
    zedDirection = moveDirection;  
    break;
   case YAW_PLUS:
    yawDirection = moveDirection;  
    break;
   default:
    Serial.println("error on direction");
    break;
   } 
}
//function to get the encoder count of a given motor. function only needs to be used if the value can not be called upon through a pointer.
int getEncoderCount(int motorPlus)
{
    switch(motorPlus)
        {
         case SHOULDER_PLUS:
          return shoulderEncoder;  
          break;
         case ELBOW_PLUS:
          return elbowEncoder;  
          break;
         case WRIST_RH_PLUS:
          return wristRHEncoder;  
          break;
         case WRIST_LH_PLUS:
          return wristLHEncoder;  
          break;
         case ZED_PLUS:
          return zedEncoder;  
          break;
         case YAW_PLUS:
          return yawEncoder;  
          break;
         default:
          Serial.println("encoder count problem");
          return -1;
          break;
         } 
}
//sets the selected encoder to a value.
void setEncoderCount(int motorPlus, int value)
{
    switch(motorPlus)
        {
         case SHOULDER_PLUS:
            shoulderEncoder = value;  
          break;
         case ELBOW_PLUS:
          elbowEncoder = value;  
          break;
         case WRIST_RH_PLUS:
          wristRHEncoder = value;  
          break;
         case WRIST_LH_PLUS:
          wristLHEncoder = value;  
          break;
         case ZED_PLUS:
          zedEncoder = value;  
          break;
         case YAW_PLUS:
          yawEncoder = value;  
          break;
         default:
          Serial.println("encoder set problem");
          break;
         } 
}

//sets the init value of the motor to true or false. This is done for the interrupt functions of the motor.
void setInit(int motorPlus, int stat)
{
    switch(motorPlus)
      {
       case SHOULDER_PLUS:
        shoulderEncoder = stat;  
        break;
       case ELBOW_PLUS:
        elbowEncoder = stat;  
        break;
       case WRIST_RH_PLUS:
        wristRHEncoder = stat;  
        break;
       case WRIST_LH_PLUS:
        wristLHEncoder = stat;  
        break;
       case ZED_PLUS:
        zedEncoder = stat;  
        break;
       case YAW_PLUS:
        yawEncoder = stat;  
        break;
       default:
        Serial.println("setup Error");
        break;
       } 
}
void testWrist()
{
  analogWrite(WRIST_LH_ENABLE, 255);
  digitalWrite(WRIST_LH_PLUS, LOW);
  digitalWrite(WRIST_LH_MIN, HIGH);
  analogWrite(WRIST_RH_ENABLE, 255);
  digitalWrite(WRIST_RH_PLUS, HIGH);
  digitalWrite(WRIST_RH_MIN, LOW);
  Serial.println("hond");
  delay(1500);
  exit(0);
//  analogWrite(WRIST_LH_ENABLE, 0);
//  digitalWrite(WRIST_LH_PLUS, LOW);
//  digitalWrite(WRIST_LH_MIN, LOW);
//  analogWrite(WRIST_RH_ENABLE, 0);
//  digitalWrite(WRIST_RH_PLUS, LOW);
//  digitalWrite(WRIST_RH_MIN, LOW);
  
  
}
void setup() {
  //setting pinmodes for motors
  for(int i = 4; i < 10; i++)
    pinMode(i, OUTPUT);
  //setting pinmodes for enables
  for(int i = 22; i < 34; i++)
     pinMode(i, OUTPUT);
  //setting pinmodes for encoder, pull-up for transistors in encoders
  pinMode(SHOULDER_ENCODER, INPUT_PULLUP);
  pinMode(ELBOW_ENCODER,    INPUT_PULLUP);
  pinMode(YAW_ENCODER,      INPUT_PULLUP);
  pinMode(ZED_ENCODER,      INPUT_PULLUP);
  pinMode(WRIST_RH_ENCODER, INPUT_PULLUP);
  pinMode(WRIST_LH_ENCODER, INPUT_PULLUP);
  //setting voltage measure pinmode
  pinMode(MEAS_VOLTAGE, INPUT);
  Serial.begin(9600);//might need to talk faster with host.
  
  //Attaching interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(SHOULDER_ENCODER), readShoulderEncoder, RISING);//CHANGE
  attachInterrupt(digitalPinToInterrupt(ELBOW_ENCODER),    readElbowEncoder,    RISING);//CHANGE
  attachInterrupt(digitalPinToInterrupt(WRIST_RH_ENCODER), readWristRHEncoder,  RISING);//CHANGE
  attachInterrupt(digitalPinToInterrupt(WRIST_LH_ENCODER), readWristLHEncoder,  RISING);//CHANGE
  #ifdef ZED
  attachInterrupt(digitalPinToInterrupt(ZED_ENCODER),      readZedEncoder,      RISING);//CHANGE
  #endif
  attachInterrupt(digitalPinToInterrupt(YAW_ENCODER),      readYawEncoder,      RISING);//CHANGE

  #ifdef VOLTAGE
  while(digitalRead(MEAS_VOLTAGE)!= HIGH)
  {
    Serial.println("please turn on the power");
    delay(3000);
  }
  #endif
  
  initAllMotors();
}

void loop() {
// put your main code here, to run repeatedly:
 // testWrist();
  #ifdef DEMO
//  moveToPos(&zedTarget, ZED_MAX_POS/2, &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
 //moveToPos(&shoulderTarget, 300, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
// //delay(100);
// moveToPos(&elbowTarget, ELBOW_MAX_POS/2, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
// //delay(100);
//  moveToPos(&yawTarget, (YAW_MAX_POS), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
//  delay(7000);
//   moveToPos(&shoulderTarget, SHOULDER_MAX_POS, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
// //delay(100);
// moveToPos(&elbowTarget, ELBOW_MAX_POS/2, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
// //delay(100);
//  moveToPos(&yawTarget, (YAW_MAX_POS/2), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
//  delay(7000);
//   moveToPos(&shoulderTarget, SHOULDER_MAX_POS/2, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
// //delay(100);
// moveToPos(&elbowTarget, ELBOW_MAX_POS/2, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
// //delay(100);
//  moveToPos(&yawTarget, (YAW_MAX_POS/2), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
//  delay(7000);
//demoMode();
  //exit(0);
 // moveMotor(1000,255, LEFT, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE);
 //Serial.println(shoulderEncoder);
  //Serial.println(shoulderTarget);
  //Serial.println(shoulderEncoder);
//  //moveToPos(&zedTarget, ZED_MAX_POS/2, &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
//   moveToPos(&wristLHTarget, 0, &wristLHEncoder, WRIST_LH_PLUS, WRIST_LH_MIN, WRIST_LH_ENABLE, 250);
//   moveToPos(&wristRHTarget, 0, &wristRHEncoder, WRIST_RH_PLUS, WRIST_RH_MIN, WRIST_RH_ENABLE, 250);
//delay(400);
//stopAllMotors();
 // delay(5000);
  #endif

  #ifdef SERIAL_TEST
  readData();
  #endif
  #ifdef TEST_POSITIONS
while(Serial.available()==0);
int y= Serial.parseInt();
Serial.println(y);
     moveToPos(&shoulderTarget, y, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 200); //200
while(Serial.available()==0);
int x= Serial.parseInt();
Serial.println(x);
     moveToPos(&elbowTarget, x, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE, 200); 
     while(Serial.available()==0);
int z= Serial.parseInt();
Serial.println(z);
     moveToPos(&yawTarget, z, &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 200); 
  #endif
}
