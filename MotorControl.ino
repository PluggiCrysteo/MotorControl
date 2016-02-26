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

//	CAN DEFINE
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
int Motor_Right_PWM_P = 9;
int Motor_Right_PWM_N = 6;
int Motor_Left_PWM_P = 5;
int Motor_Left_PWM_N = 8;

long oneSecInMillis = 1000;

double nbOfTicksPerRotationRight = 3500;
double nbOfTicksPerRotationLeft = 3500;
int diameterMM = 150;
double circonference = 3.14159*diameterMM;

double nbTicksRightPerMM = nbOfTicksPerRotationRight / circonference;
double nbTicksLeftPerMM = nbOfTicksPerRotationLeft / circonference;

double distanceBetweenWheels = 460; //en mm

double maxSpeed = 470;
double speedMS = 0 ;

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

//volatile int percentPowerRight = (nbTicksRightPerSec/3500.0) *100;
//volatile int percentPowerLeft = (nbTicksLeftPerSec/3500.0) *100;
int percentPowerRight = 0;
int percentPowerLeft = 0;
volatile int powerReceived = 0;
volatile int angleReceived = 0;

MCP2510* can_dev;
Canutil* canutil;

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
	//	attachInterrupt(7, compteurC, RISING);
	//attachInterrupt(4, compteurD, RISING);

	pinMode(codeurPinA, INPUT);
	pinMode(codeurPinC,INPUT);

	Motor_Setup();

	//	Serial.println(speedLeft);
	//	Serial.println(speedRight);

	can_dev->write(CANINTE, 0x01); //disables all interrupts but RX0IE (received message in RX buffer 0)
	can_dev->write(CANINTF, 0x00);  // Clears all interrupts flags

	canutil->setClkoutMode(0, 0); // disables CLKOUT
	canutil->setTxnrtsPinMode(0, 0, 0); // all TXnRTS pins as all-purpose digital input

	canutil->setOpMode(4); // sets configuration mode
	// IMPORTANT NOTE: configuration mode is the ONLY mode where bit timing registers (CNF1, CNF2, CNF3), acceptance
	// filters and acceptance masks can be modified
	Serial.println("waiting for op mode");
	canutil->waitOpMode(4);  // waits configuration mode
	Serial.println("opmode received");

	can_dev->write(CNF1, 0x03); // SJW = 1, BRP = 3
	can_dev->write(CNF2, 0b10110001); //BLTMODE = 1, SAM = 0, PHSEG = 6, PRSEG = 1
	can_dev->write(CNF3, 0x05);  // WAKFIL = 0, PHSEG2 = 5

	// SETUP MASKS / FILTERS FOR CAN
	canutil->setRxOperatingMode(2,1, 0);  // standard ID messages only  and rollover
	//	canutil->setAcceptanceFilter(REMOTE_NODE_ID, POWER_ANGLE_MESS_ID, 1, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
	canutil->setAcceptanceFilter(0,0, 1, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
	canutil->setAcceptanceMask(0x000, 0x00000000, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, buffer# 0

	canutil->setOpMode(0); // sets normal mode
	can_dev->write(CANINTF, 0x00);  // Clears all interrupts flags
	Serial.println(digitalRead(42));

	//	pinMode(CAN_INTERRUPT_PIN,OUTPUT);
	SPI.usingInterrupt(CAN_INTERRUPT_PIN);
	attachInterrupt(CAN_INTERRUPT_PIN, can_callback, FALLING);
	Serial.println(digitalRead(42));

}

void loop() {
	//Serial.println(digitalRead(CAN_INTERRUPT_PIN));
	// Give expected pow/er according to given speed

	speedMMS = maxSpeed * powerReceived * 10;
//	Serial.println(speedMMS);
	angleEntered = angleReceived;
//	Serial.println(angleEntered);

	if(angleEntered > 0){
		circRadius = (distanceBetweenWheels/2)*180/angleEntered;

		if(speedMMS > maxSpeed){
			speedLeft = maxSpeed;
		} else if (speedMMS < -maxSpeed){
			speedLeft = -maxSpeed;
		} else {
			speedLeft = speedMMS;
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


	nbTicksRightPerSec = speedRight*nbTicksRightPerMM;
	nbTicksLeftPerSec = speedLeft*nbTicksLeftPerMM;
	Serial.println(nbTicksRightPerSec);
	Serial.println(nbTicksLeftPerSec);

	percentPowerRight = (nbTicksRightPerSec/3500.0) *100;
	percentPowerLeft = (nbTicksLeftPerSec/3500.0) *100;
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
	long checkFasterBy = 1;
	while(millis() - start < (oneSecInMillis/checkFasterBy)){
	}
	// Compare distance traveled over time
	// Change power based on difference of real speed vs given speed
	percentPowerRight *= 1.0+((nbTicksRightPerSec - (tick_codeuseA - startTicksRight))/(nbTicksRightPerSec/checkFasterBy));
	percentPowerLeft *= 1.0+((nbTicksLeftPerSec - (tick_codeuseC - startTicksLeft))/(nbTicksLeftPerSec/checkFasterBy));

	//Serial.println(tick_codeuseA - startTicksRight);
	//Serial.println(tick_codeuseC - startTicksLeft);
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

void can_callback() {
	//	Serial.println(digitalRead(CAN_INTERRUPT_PIN));
	uint8_t canDataReceived[8];
	uint8_t recSize = canutil->whichRxDataLength(0); 
	uint16_t stdId = canutil->whichStdID(0);
	uint32_t extId = canutil->whichExtdID(0);


	for (uint8_t i = 0; i < recSize; i++) { // gets the bytes
		canDataReceived[i] = canutil->receivedDataValue(0, i);
	}

	powerReceived = (int8_t)canDataReceived[0];
	angleReceived = (int8_t)canDataReceived[1];
	/*	Serial.print("Power received: ");
		Serial.println(powerReceived);
		Serial.print("Angle received: ");
		Serial.println(angleReceived);
		Serial.println(digitalRead(CAN_INTERRUPT_PIN));
	 */
	can_dev->write(CANINTF, 0x00);  // Clears all interrupts flags
}
