#include <digitalWriteFast.h>
#include <Math.h>;
 
#define codeurPinA 3
#define codeurPinB 2 //brancher le codeur B sur le PIN 2
#define codeurPinC 4
#define codeurPinD 7
 
volatile long tick_codeuseA = 0;    // Compteur de tick de la codeuse
volatile long tick_codeuseC = 0;
 
int etat = 0;
volatile float sensMotor1 = 1;
volatile float sensMotor2 = 1;
int etatA = 0;
int etatC = 0;

//moteur
int Motor_Right_PWM_P = 9;
int Motor_Right_PWM_N = 6;
int Motor_Left_PWM_P = 5;
int Motor_Left_PWM_N = 8;
 
double nbOfTicksPerRotationRight = 3500;
double nbOfTicksPerRotationLeft = 3500;
int diameterMM = 150;
double circonference = 3.14159*diameterMM;
 
double nbTicksRightPerMM = nbOfTicksPerRotationRight / circonference;
double nbTicksLeftPerMM = nbOfTicksPerRotationLeft / circonference;
   
double distanceBetween2Wheels = 460; //en mm
 
double speedWheelMS = - 0.4 ;
  
double distanceEntered= speedWheelMS*1000; // in m/s

//int angleEntered=10;
 
//double inRadians = 3.14159*(90-angleEntered)/180.0;
 
//double w=distanceBetween2Wheels*cos(inRadians);
 
//double distanceLeft = distanceEntered - 2*w;
//double distanceRight = 2*distanceEntered - distanceLeft;
 
double speedLeft = distanceEntered;
double speedRight = distanceEntered;
 
double nbTicksRightPerSec = speedRight*nbTicksRightPerMM;
double nbTicksLeftPerSec = speedLeft*nbTicksLeftPerMM;
 
int maxPower = 100;
 
int percentPowerRight = (nbTicksRightPerSec/3500.0) *100;
int percentPowerLeft = (nbTicksLeftPerSec/3500.0) *100;
 
void setup() {
   Serial.begin(9600); 
   
   attachInterrupt(2, compteurA, RISING);  // Interruption sur tick de la codeuse (interruption 0 = pin3 arduino nano)
   //attachInterrupt(3, compteurB, RISING);  // Interruption sur tick de la codeuse (interruption 1 = pin2 arduino nano)
   attachInterrupt(7, compteurC, RISING);
   //attachInterrupt(4, compteurD, RISING);
 
   pinMode(codeurPinA, INPUT);
   pinMode(codeurPinC,INPUT);
 
   Motor_Setup();
 
   Serial.println(speedLeft);
   Serial.println(speedRight);
 
}
 
void loop() {
  // Give expected power according to given speed
 
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
  Serial.println(percentPowerRight);
  Serial.println(percentPowerLeft);
  Right_Motor(percentPowerRight);
  Left_Motor(percentPowerLeft);
  // Record ticks over time
  long start = millis();
  long startTicksRight = tick_codeuseA;
  long startTicksLeft = tick_codeuseC;
  while(millis() - start < 1000){
  }
  // Compare distance traveled over time
  // Change power based on difference of real speed vs given speed
  percentPowerRight *= 1.0+((nbTicksRightPerSec - (tick_codeuseA - startTicksRight))/nbTicksRightPerSec);
  percentPowerLeft *= 1.0+((nbTicksLeftPerSec - (tick_codeuseC - startTicksLeft))/nbTicksLeftPerSec);

  Serial.println(tick_codeuseA - startTicksRight);
  Serial.println(tick_codeuseC - startTicksLeft);
}
 
//----------------------------------------------------Compteurs--------------------------------------------------------------
 
// Moteur 1
void compteurA(){
 
   if (digitalRead(codeurPinA) == digitalRead(codeurPinB)){
      tick_codeuseA++;
      sensMotor1 = 1;
   }
   else {
      tick_codeuseA--;
      sensMotor1 = -1;
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
      tick_codeuseC++;
      sensMotor2 = 1;
    }
    else {
      tick_codeuseC--;
      sensMotor2 = -1;
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
    analogWrite(Motor_Left_PWM_P, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Left_PWM_N, 0);  
  }
  else
    {//En arriére
    pourcent_speed = pourcent_speed*(-1); //on rend positif le pourcentage.
    
    analogWrite(Motor_Left_PWM_N, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Left_PWM_P, 0);  
  
    }
}

void Right_Motor(int pourcent_speed) //Reglage de la vitesse du moteur gauche
//Pour arréter le moteur : taper 0%
{
  if(pourcent_speed>=0)
  {//En avant
    analogWrite(Motor_Right_PWM_P, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Right_PWM_N, 0);  
  }
  else
    {//En arriére
    pourcent_speed = pourcent_speed*(-1); //on rend positif le pourcentage.
    
    analogWrite(Motor_Right_PWM_N, convert_pourcent_to_digital(pourcent_speed));
    analogWrite(Motor_Right_PWM_P, 0);  
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

