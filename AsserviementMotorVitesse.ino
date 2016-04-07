#include <digitalWriteFast.h>
//#include <Math.h>
#include <Canutil.h>
#include <SPI.h>
#include <MCP2510.h>
#include <Wire.h>

#define codeurPinA 3
#define codeurPinB 2 //brancher le codeur B sur le PIN 2
#define codeurPinC 11
#define codeurPinD 7

//  CAN DEFINE
#define REMOTE_NODE_ID 0x10
#define POWER_ANGLE_MESS_ID 0x1
#define OWN_NODE_ID 0x4

#define CAN_INTERRUPT_PIN 42
#define CAN_SS_PIN 52

volatile long tick_codeuseA = 0;    // Compteur de tick de la codeuse
volatile long tick_codeuseC = 0;

int etat = 0;
volatile float sensMotor1 = 1;
volatile float sensMotor2 = 1;
int etatA = 0;
int etatC = 0;

//moteur
int Motor_Right_PWM_P = 5;
int Motor_Right_PWM_N = 8;
int Motor_Left_PWM_P = 9;
int Motor_Left_PWM_N = 6;

long oneSecInMillis = 1000;

double nbOfTicksPerRotationRight = 3430;
double nbOfTicksPerRotationLeft = 3425;

double nbTicksPerSecondRight = 3850;
double nbTicksPerSecondLeft = 3650;

int diameterMM = 145;
double circonference = 3.14159*diameterMM;

double nbTicksRightPerMM = nbOfTicksPerRotationRight / circonference;
double nbTicksLeftPerMM = nbOfTicksPerRotationLeft / circonference;

double distanceBetweenWheels = 460; //en mm

double maxSpeed = circonference*nbTicksPerSecondRight/nbTicksPerSecondRight;
double speedMS = .1 ;
double realSpeedMMS = 0;
double correctionCoefRight = 1;
double correctionCoefLeft = 1;

double speedMMS= speedMS*1000; // in m/s

//Radius of circle given Alpha turn angle = (23*180)/(Alpha)
//  See documentation for equations explaination

double angleEntered=0.0;
double circRadius;  //Found in mm
double speedLeft;   //Speeds found in mm/s
double speedRight;  

double nbTicksRightPerSec;
double nbTicksLeftPerSec;

int maxPower = 100;

long checkFasterBy = 10;

long startTicksRight;
long startTicksLeft;

//volatile int percentPowerRight = (nbTicksRightPerSec/3500.0) *100;
//volatile int percentPowerLeft = (nbTicksLeftPerSec/3500.0) *100;
double percentPowerRight = 0;
double percentPowerLeft = 0;
volatile int powerReceived = 0;
volatile int angleReceived = 0;
volatile boolean newCommand = false;

double ki = 0.0123;
double ti = 0.009697;
double kp = 0.0171;

MCP2510* can_dev;
Canutil* canutil;

#define MODE_LINEAR 0
#define MODE_CIRCULAR 1
#define MODE_CONTROLLER 2

double curSpeed = 0.0;

void setup() {
  Serial.begin(9600); 

  pinMode(4,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(52,OUTPUT);
  pinMode(CAN_INTERRUPT_PIN, INPUT);
  can_dev = new MCP2510(CAN_SS_PIN); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
  canutil = new Canutil(*can_dev);
  uint8_t txstatus;

    attachInterrupt(2, compteurA, RISING);  // Interruption sur tick de la codeuse (interruption 0 = pin3 arduino nano)
  //attachInterrupt(3, compteurB, RISING);  // Interruption sur tick de la codeuse (interruption 1 = pin2 arduino nano)
    attachInterrupt(7, compteurC, RISING);
  //attachInterrupt(4, compteurD, RISING);

  pinMode(codeurPinA, INPUT);
  pinMode(codeurPinC,INPUT);

  Serial.println("setup");
  
  Motor_Setup();

  //  Serial.println(speedLeft);
  //  Serial.println(speedRight);

  can_dev->write(CANINTE, 0x01); //disables all interrupts but RX0IE (received message in RX buffer 0)
  can_dev->write(CANINTF, 0x00);  // Clears all interrupts flags

  canutil->setClkoutMode(0, 0); // disables CLKOUT
  canutil->setTxnrtsPinMode(0, 0, 0); // all TXnRTS pins as all-purpose digital input

  canutil->setOpMode(4); // sets configuration mode
  // IMPORTANT NOTE: configuration mode is the ONLY mode where bit timing registers (CNF1, CNF2, CNF3), acceptance
  // filters and acceptance masks can be modified
  Serial.println("waiting for op mode");
  //canutil->waitOpMode(4);  // waits configuration mode
  Serial.println("opmode received");

  can_dev->write(CNF1, 0x03); // SJW = 1, BRP = 3
  can_dev->write(CNF2, 0b10110001); //BLTMODE = 1, SAM = 0, PHSEG = 6, PRSEG = 1
  can_dev->write(CNF3, 0x05);  // WAKFIL = 0, PHSEG2 = 5

  // SETUP MASKS / FILTERS FOR CAN
  canutil->setRxOperatingMode(2,1, 0);  // standard ID messages only  and rollover
  //  canutil->setAcceptanceFilter(REMOTE_NODE_ID, POWER_ANGLE_MESS_ID, 1, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
  canutil->setAcceptanceFilter(0,0, 1, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
  canutil->setAcceptanceMask(0x000, 0x00000000, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, buffer# 0

  canutil->setOpMode(0); // sets normal mode
  can_dev->write(CANINTF, 0x00);  // Clears all interrupts flags
  Serial.println(digitalRead(42));

  //  pinMode(CAN_INTERRUPT_PIN,OUTPUT);
  SPI.usingInterrupt(CAN_INTERRUPT_PIN);
  //attachInterrupt(CAN_INTERRUPT_PIN, can_callback, FALLING);
  Serial.println(digitalRead(42));

  powerReceived = 0;
  angleReceived = 0;
}

void loop() {
  //Serial.println(digitalRead(CAN_INTERRUPT_PIN));
  int curPos[] = {0,0};
  double xmm = 0;
  double ymm = 250;
  double endAngle = 0;
  int samplingPeriodMillis = 10;
  newCommand = true;
  goTo(curPos, xmm, ymm, MODE_CONTROLLER, endAngle, speedMMS, samplingPeriodMillis);
  }

//----------------------------------------------------Compteurs--------------------------------------------------------------

// Moteur 1
void compteurA(){

  if (digitalRead(codeurPinA) == digitalRead(codeurPinB)){
    tick_codeuseA--;
    sensMotor1 = -1;
  }
  else {
    tick_codeuseA++;
    sensMotor1 = 1;
  }
  etatA = 1;
}

void compteurB(){
  // tick_codeuseB++;


  if (etatA = 0){
    sensMotor1 = sensMotor1 * (-1);
  }

  etatA = 0;
}

// Moteur 2
void compteurC(){

  if (digitalRead(codeurPinC) == digitalRead(codeurPinD)){
    tick_codeuseC--;
    sensMotor2 = -1;
  }
  else {
    tick_codeuseC++;
    sensMotor2 = 1;
  }
  etatC = 1;
}

void compteurD(){
  // tick_codeuseD++;

  if (etatC = 0){
    sensMotor2 = sensMotor2 * (-1);
  }

  etatC = 0;
}

//-----------------------------------------------Commande Moteur--------------------------------------------------------------------

void Motor_Setup()
{
  pinMode(Motor_Right_PWM_P, OUTPUT);
  pinMode(Motor_Right_PWM_N, OUTPUT);
  pinMode(Motor_Left_PWM_P, OUTPUT);
  pinMode(Motor_Left_PWM_N, OUTPUT);
  
  //Right_Motor(0);
  //Left_Motor(0);
}

void avance_recul(int pourcent_speed)
{
  Left_Motor(pourcent_speed);
  Right_Motor(-pourcent_speed);

}

void rotation(int pourcent_angle) //positif = Gauche , Negatif = Droit
{
  Left_Motor(pourcent_angle);
  Right_Motor(pourcent_angle);
}


void Left_Motor(int pourcent_speed) //Reglage de la vitesse du moteur gauche
  //Pour arréter le moteur : taper 0%
{
  if(pourcent_speed>=0)
  {//En avant
    analogWrite(Motor_Left_PWM_N, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Left_PWM_P, 0);  
  }
  else
  {//En arriére
    pourcent_speed = pourcent_speed*(-1); //on rend positif le pourcentage.

    analogWrite(Motor_Left_PWM_P, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Left_PWM_N, 0);  

  }
}

void Right_Motor(int pourcent_speed) //Reglage de la vitesse du moteur gauche
  //Pour arréter le moteur : taper 0%
{
  if(pourcent_speed>=0)
  {//En avant
    analogWrite(Motor_Right_PWM_N, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Right_PWM_P, 0);  
  }
  else
  {//En arriére
    pourcent_speed = pourcent_speed*(-1); //on rend positif le pourcentage.

    analogWrite(Motor_Right_PWM_P, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Right_PWM_N, 0);  
  }
}

//---------------------------------------MATH------------------------------------------------------------------------------

int convert_pourcent_to_digital(int pourcentage)  //Convertit un pourcentage positif en 0-255
{
  if(pourcentage>=0)
  {//then
    return(pourcentage*255/100);//convertion
  }
  else
  {//Si le pourcentage est négatif, on renvois zero.
    return(0);
  }    
}

void can_callback() {
  //  Serial.println(digitalRead(CAN_INTERRUPT_PIN));
  uint8_t canDataReceived[8];
  uint8_t recSize = canutil->whichRxDataLength(0); 
  uint16_t stdId = canutil->whichStdID(0);
  uint32_t extId = canutil->whichExtdID(0);


  for (uint8_t i = 0; i < recSize; i++) { // gets the bytes
    canDataReceived[i] = canutil->receivedDataValue(0, i);
  }

  powerReceived = (int8_t)canDataReceived[0];
  angleReceived = (int8_t)canDataReceived[1];
  newCommand = true;
  /*  Serial.print("Power received: ");
    Serial.println(powerReceived);
    Serial.print("Angle received: ");
    Serial.println(angleReceived);
    Serial.println(digitalRead(CAN_INTERRUPT_PIN));
   */
  can_dev->write(CANINTF, 0x00);  // Clears all interrupts flags
}

void goTo(int curPos[], double xmm, double ymm, int mode, double endAngle, double speedMMS, int samplingPeriodMillis){
  double distToMove = 0;
  switch(mode){
//    case 0:  //Mode = linear
//    {
//    
//      double angleDeplacement = 0;
//      
//      //Loop until at desired position
//      do{
//        //Calculate distance and pivot angle
//        distToMove = sqrt((curPos[0] - xmm)*(curPos[0] - xmm) + (curPos[1] - ymm)*(curPos[1] - ymm));
//        double pivotAngle = acos((curPos[0] - xmm)/distToMove);
//        angleDeplacement += pivotAngle;
//        
//        //Pivot angle to line up center with destination
////        pivot(pivotAngle);
//        
//        //Move at speedMMS
//        startTicksRight = tick_codeuseC;
//        startTicksLeft = tick_codeuseA;
//        
//        int wheelPercentPower = speedMMS/maxSpeed * 100;
//        Right_Motor(wheelPercentPower);
//        Left_Motor(wheelPercentPower);
//        
//        //Calculate current position after moving for sampling period
////        oldPos[0] = curPos[0];
//  //      oldPos[1] = curPos[1];
//        curPos[0] = curPos[0] + (tick_codeuseC - startTicksRight);
//        curPos[1] = curPos[1] + (tick_codeuseA - startTicksLeft);
//  
//        //Return if new command
////        if(newCommand){
//  //        return;
//    //    }
//        
//      }while(curPos[0] != xmm && curPos[1] != ymm);
//      
//      //Pivot to face end angle
//      //pivot(endAngle - angleDeplacement);
//      break;
//    }
//    case 1:  //Mode = circular
//     { 
//      //Loop until at desired position
//      do{
//        if(angleEntered > 0){
//          circRadius = (distanceBetweenWheels/2)*180/angleEntered;
//      
//          if(speedMMS > maxSpeed){
//            speedLeft = maxSpeed;
//          } else if (speedMMS < -maxSpeed){
//            speedLeft = -maxSpeed;
//          } else {
//            speedLeft = speedMMS;
//          }
//      
//          if(circRadius > distanceBetweenWheels){
//            speedRight = (speedLeft*(circRadius-distanceBetweenWheels))/(circRadius);
//          } else {
//            speedRight = 0;
//          }
//        } else if(angleEntered < 0){
//          circRadius = (distanceBetweenWheels/2)*180/(-1*angleEntered);
//      
//          if(speedMMS > maxSpeed){
//            speedRight = maxSpeed;
//          } else if (speedMMS < -maxSpeed){
//            speedRight = -maxSpeed;
//          } else {
//            speedRight = speedMMS;
//          }
//      
//          if(circRadius > distanceBetweenWheels){
//            speedLeft = (speedRight*(circRadius-distanceBetweenWheels))/(circRadius);
//          } else {
//            speedLeft = 0;
//          }
//        } else {
//          speedLeft = speedMMS;
//          speedRight = speedLeft;
//        }
//        //Calculate center curve
//        //Calculate outside and inside wheel curves
//        //Move center at speedMMS, scaling outside and inside wheels proportionally
//        //Calculate current position after moving for sampling period
//        //Return pos[x,y] if new command
//      }while(curPos[0] != xmm && curPos[1] != ymm);
//        
//      //Pivot to face end angle
////      pivot(endAngle - angleDeplacement);
//      break;
//     }
    case 2:  //Mode = Controller
      {
        double newSpeed = 0;
        double speedRight = 0;
        double speedLeft = 0;
        double angleEntered = 0;
        double ekRight = 0, ekLeft = 0, ekRightPrevious = 0, ekLeftPrevious = 0, previousPowerRight = 0, previousPowerLeft = 0, speedRightReal = 0, speedLeftReal = 0;
        Serial.print("speedRight,"); Serial.print("speedLeft,"); Serial.print("desired speed"); Serial.println(speedMMS);
        while(true){
      // Give expected power according to given speed

      newSpeed = maxSpeed * (powerReceived/maxPower);
      if (speedMMS != newSpeed){
        speedMMS = newSpeed;
      }
      
      if(angleEntered != angleReceived)
      {
        angleEntered = angleReceived;
        // Serial.println(angleEntered);
      }
      
      if(angleEntered > 0){
        circRadius = (distanceBetweenWheels/2)*180/angleEntered;
    
        if(speedMMS > maxSpeed){
          speedLeft = maxSpeed;
        } else if (speedMMS < -maxSpeed){
          speedLeft = -maxSpeed;
        } else {
          speedLeft = speedMMS;   //TODO: calculate actual speed having speedMMS as the center speed (may need to calc possible max speed before all this)
        }
    
        if(circRadius > distanceBetweenWheels){
          speedRight = (speedLeft*(circRadius-distanceBetweenWheels))/(circRadius);
        } else {
          speedRight = 0;
        }
      } else if(angleEntered < 0){
        circRadius = (distanceBetweenWheels/2)*180/(-1*angleEntered);
    
        if(speedMMS > maxSpeed){
          speedRight = maxSpeed;
        } else if (speedMMS < -maxSpeed){
          speedRight = -maxSpeed;
        } else {
          speedRight = speedMMS;
        }
    
        if(circRadius > distanceBetweenWheels){
          speedLeft = (speedRight*(circRadius-distanceBetweenWheels))/(circRadius);
        } else {
          speedLeft = 0;
        }
      } else {
        speedLeft = speedMMS;
        speedRight = speedLeft;
      }

      if(newCommand && speedMMS != newSpeed && angleEntered != angleReceived){
        percentPowerRight = (speedRight - 3.6693)/4.2285;
        percentPowerLeft = (speedLeft - 3.6693)/4.2285;
        newCommand = false;
      } else {
        previousPowerRight = percentPowerRight;
        previousPowerLeft = percentPowerLeft;
        percentPowerRight = previousPowerRight + (ki*(samplingPeriodMillis/1000.0)/ti)*(ekRight) + kp*(ekRight - ekRightPrevious);
        percentPowerLeft = previousPowerLeft + (ki*(samplingPeriodMillis/1000.0)/ti)*(ekLeft) + kp*(ekLeft - ekLeftPrevious);

//        Serial.print("ekRight "); Serial.println(ekRight);
//        Serial.print("ekLeft "); Serial.println(ekLeft);
//        Serial.print("powerRight "); Serial.println(percentPowerRight);
//        Serial.print("powerLeft "); Serial.println(percentPowerLeft);
      }
    
      if (percentPowerRight > maxPower){
        percentPowerRight = maxPower;
      }
      if (percentPowerLeft > maxPower){
        percentPowerLeft = maxPower;
      }
      if (percentPowerRight < -maxPower){
        percentPowerRight = -maxPower;
      }
      if (percentPowerLeft < -maxPower){
        percentPowerLeft = -maxPower;
      }
      
      //Serial.print(percentPowerRight);Serial.println("%");
      //Serial.print(percentPowerLeft);Serial.println("%");
      Right_Motor((int)percentPowerRight);
      Left_Motor((int)percentPowerLeft);
      
      // Record ticks over time
      long start = millis();
      startTicksRight = tick_codeuseC;
      startTicksLeft = tick_codeuseA;
      while(millis() - start < (samplingPeriodMillis)){
      }
      
      // Compare distance traveled over time
      // Change power based on difference of real speed vs given speed
      
      int rightTicks = tick_codeuseC - startTicksRight;
      int leftTicks = tick_codeuseA - startTicksLeft;


      //real wheel speed = (ticks/nbPerRotation) * circumferenceOfWheel/samplingPeriod 
      //ek = desired wheel speed - real wheel speed
      speedRightReal = ((rightTicks/nbOfTicksPerRotationRight) * circonference) / (samplingPeriodMillis/1000.0);
      speedLeftReal = ((leftTicks/nbOfTicksPerRotationLeft) * circonference) / (samplingPeriodMillis/1000.0);

      Serial.print(speedRightReal); Serial.print(","); Serial.println(speedLeftReal);

      ekRightPrevious = ekRight;
      ekLeftPrevious = ekLeft;
      
      ekRight = speedRight - speedRightReal;
      ekLeft = speedLeft - speedLeftReal;
        }
      break;
      }
      default:
      {
      break;
      }
  }
}

//Basic pivot function without asservissement
boolean pivot(double pivotAngle){
  double circRadius = distanceBetweenWheels*3.14159;
  
  double distWheels = (circRadius)*(pivotAngle/180.0);
  
  startTicksRight = tick_codeuseC;
  startTicksLeft = tick_codeuseA;
  
  int finalTicksRight = startTicksRight + distWheels*nbTicksRightPerMM;
  int finalTicksLeft = startTicksLeft + distWheels*nbTicksLeftPerMM;
  
  boolean rightStop = false, leftStop = false;
  Right_Motor(100);
  Left_Motor(100);
  while(!(rightStop && leftStop)){
    if (startTicksRight - tick_codeuseC < finalTicksRight){
      Right_Motor(0);
    }
    if(startTicksLeft - tick_codeuseA < finalTicksLeft){
      Left_Motor(0);
    }
  }
}
