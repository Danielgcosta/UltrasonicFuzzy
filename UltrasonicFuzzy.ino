/*
Medusa 2.0	

Robô inteligente flutuante de acompanhamento ambiental
Environment monitoring smart floating robot

Pontifícia Universidade Católica do Rio de Janeiro
Departamento de Engenharia Elétrica

Name:		Medusa2
Created:	2015-2017
Author:		Daniel Guimarães Costa
Contact:	danielgc@ele.puc-rio.br

Advisors:
Marley Vellasco		marley@ele.puc-rio.br
Karla Figueiredo	karla@ele.puc-rio.br
*/

#include <String.h>
#include <SoftwareSerial.h> // For Oxygen Reduction and dissolvedOxygen sensors which simulate serial ports
using namespace std;

const int NUMBER_OF_READINGS = 3;

// PORT CONFIGURATION
#define sensor1Port		A1
#define sensor1Trigger	A2
#define sensor2Port		A3
#define sensor2Trigger	A4
#define sensor3Port		A5
#define sensor3Trigger	A6
#define thermalSensor1Port	A7
#define thermalSensor2Port	A8
#define phSensorPort		A9
#define orpInPort	A10	//Possible ports: 10 to 15; 50 to 53; A8 to A15
#define orpOutPort	A11
#define doInPort	A12	//Possible ports: 10 to 15; 50 to 53; A8 to A15
#define doOutPort	A13

SoftwareSerial doSerial(doInPort, doOutPort);
SoftwareSerial orpSerial(orpInPort, orpOutPort);

typedef struct uSensor { int dataPort; int triggerPort; double value; };
typedef struct tSensor { int dataPort; double value; };
typedef struct phSensor { int dataPort; double value; };
typedef struct doSensor { SoftwareSerial *serial; double value; };
typedef struct orpSensor { SoftwareSerial *serial; double value; };

uSensor ultrasonic1;
uSensor ultrasonic2;
uSensor ultrasonic3;
uSensor ultrasonicArray[3] = { ultrasonic1 , ultrasonic2, ultrasonic3 };
tSensor thermal1;
tSensor thermal2;
doSensor dissolvedOxygen;
orpSensor oxygenReduction;
phSensor ph;

// Class variables
// Sensor readings
float _leftSensorReading = 0;
float _frontSensorReading = 0;
float _rightSensorReading = 0;
float _distanceReading = 0;
float _angleReading = 0;

// Membership values
// Left Sensor
float _veryNearLeftSensorMembership = 0;
float _nearLeftSensorMembership = 0;
float _farLeftSensorMembership = 0;
float _veryFarLeftSensorMembership = 0;

// Front Sensor
float _veryNearFrontSensorMembership = 0;
float _nearFrontSensorMembership = 0;
float _farFrontSensorMembership = 0;
float _veryFarFrontSensorMembership = 0;

// Right Sensor
float _veryNearRightSensorMembership = 0;
float _nearRightSensorMembership = 0;
float _farRightSensorMembership = 0;
float _veryFarRightSensorMembership = 0;

// Distance to Goal
float _veryNearGoalMembership = 0;
float _nearGoalMembership = 0;
float _farGoalMembership = 0;
float _veryFarGoalMembership = 0;

// Angle between Robot and Goal
float _veryNegativeAngleMembership = 0;
float _negativeAngleMembership = 0;
float _zeroAngleMembership = 0;
float _positiveAngleMembership = 0;
float _veryPositiveAngleMembership = 0;

// Left Motor Power
float _veryNegativeLeftPowerMembership = 0;
float _negativeLeftPowerMembership = 0;
float _zeroLeftPowerMembership = 0;
float _positiveLeftPowerMembership = 0;
float _veryPositiveLeftPowerMembership = 0;

// Right Motor Power
float _veryNegativeRightPowerMembership = 0;
float _negativeRightPowerMembership = 0;
float _zeroRightPowerMembership = 0;
float _positiveRightPowerMembership = 0;
float _veryPositiveRightPowerMembership = 0;

// Motor activation 
float _leftMotorActivation = 0;
float _rightMotorActivation = 0;

void setup()
{
	Serial.begin(9600);
	Serial.println("Port open");

	ultrasonicArray[0] = { sensor1Port, sensor1Trigger, 0 };
	ultrasonicArray[1] = { sensor2Port, sensor2Trigger, 0 };
	ultrasonicArray[2] = { sensor3Port, sensor3Trigger, 0 };
	thermal1 = { thermalSensor1Port, 0 };
	thermal2 = { thermalSensor2Port, 0 };
	doSerial.begin(9600);
	orpSerial.begin(9600);
	dissolvedOxygen = { &doSerial, 0 };
	oxygenReduction = { &orpSerial, 0 };
	ph = { phSensorPort, 0 };

	Serial.println("Sensors initialized");
	Serial.println("Beginning loop");
	delay(1000);
}

void loop()
{
	/*
	Serial.println("Rodando.");
	double readings[3][NUMBER_OF_READINGS];
	double sum[3] = { 0,0,0 };
	for (unsigned int reading = 0; reading < NUMBER_OF_READINGS; reading++) {
		for (unsigned int sensor = 0; sensor < 3; sensor++) {
			readings[sensor][reading] = evaluateSensor(ultrasonicArray[sensor]);
			sum[sensor] += readings[sensor][reading];
		}
	}
	for (unsigned int sensor = 0; sensor < 3; sensor++) {
		ultrasonicArray[sensor].value = sum[sensor] / NUMBER_OF_READINGS;
	}

	// Mostrar valor dos sensores na porta Serial
	for (unsigned int sensor = 0; sensor < 3; sensor++) {
		Serial.print("Valor do sensor ");
		Serial.print(sensor);
		Serial.print(" = ");
		Serial.print(ultrasonicArray[sensor].value);
		Serial.print("   ");
	}
	delay(5000);
	*/
	
/*
	// Sensor reading (substituir pela leitura do sensor)
	_leftSensorReading = 10;
	_frontSensorReading = 10;
	_rightSensorReading = 10;
	_distanceReading = 1;
	_angleReading = 0;
	
	// Fuzzification
	Fuzzify();

	// Rule application
	processRules();

	// Defuzzification
	LeftMotorDefuzzification();
	RightMotorDefuzzification();
	
	printOutput();
	delay(2000);*/

	SimulateWalk();	

	/*thermal1.value = evaluateSensor(thermal1);
	Serial.println((double)thermal1.value);
	delay(1000);*/

	//multipleUltrasonicReadings();

	//double value = evaluateSensor(ultrasonicArray[0]);
}

// Ultrassonic sensor Acquisition
double evaluateSensor(struct uSensor sensor){
	// Trigger
	// Serial.println("Triggering");
	digitalWrite(sensor.triggerPort, LOW);
	delayMicroseconds(20);
	digitalWrite(sensor.triggerPort, HIGH);
	delayMicroseconds(20);
	digitalWrite(sensor.triggerPort, LOW);

	// Reading
	// Serial.println("Reading...");
	unsigned long pulse = pulseIn(sensor.dataPort, HIGH);
	sensor.value = pulse / 58.;
	return sensor.value;
}

// Thermal sensor Acquisition
double evaluateSensor(struct tSensor sensor) {
	// Calibrate sensor
	// 1st wire ----[GND]
	// 2nd wire ----[DATA] 
	//			\---[4,7K]----[5V]

	// analog pin reads 130 in 100°C and 694 in 0°C
	// multiplication is due map only using integers
	sensor.value = map(analogRead(sensor.dataPort), 694, 119, 0, 10000)/100.;

	//Serial.print("Temperature = ");
	//Serial.print(sensor.value);
	//Serial.println("C");
	return sensor.value;
}

// Dissolved Oxygen sensor Acquisition
double evaluateSensor(struct doSensor sensor){
	String sensorString;
	// If a Stringacter has been received
	if (sensor.serial->available() > 0){
		// Gets the received String
		String inString = (String)sensor.serial->read();
		// Composes the String
		sensorString += inString;
		// Reading ends with a CR
		if (inString == "\r") {
			sensor.value = sensorString.toDouble();
		}
	}
	return sensor.value;
}

// Oxygen Reduction Potential sensor Acquisition
double evaluateSensor(struct orpSensor sensor) {
	String sensorString;
	// If a Stringacter has been received
	if (sensor.serial->available() > 0){
		// Gets the received String
		String inString = (String)sensor.serial->read();
		// Composes the String
		sensorString += inString;
		// Reading ends with a CR
		if (inString == "\r") {
			sensor.value = sensorString.toDouble();
		}		
	}
	return sensor.value;
}

// pH sensor Acquisition
double evaluateSensor(struct phSensor sensor) {
	double value = 0;

	// Nernst equation
	// E = E0 + 2.3 RT*ln(alphaH+)/F

	// R = Ideal Gas Constant
	const double r = 8.314472; // J/(mol K)
	// T = Temperature in Kelvin
	double t = 273.15 + (thermal1.value + thermal2.value) / 2;
	// F = Faraday constant
	const double f = 96485.33289;

	// Nernst factor
	// kT = 2.3*R*T/F

	// Hence we must calibrate E0 with the neutral solution
	const float E0 = 1; //dummy value for now, overwrite with calibration value
						//_value = (E0-E)*F/(2.303*R*T)

	// Substituir por Serial // !!
	sensor.value = (E0 - analogRead(sensor.dataPort) ) / (2.3*r*t / f);
	return sensor.value;
}

void multipleUltrasonicReadings() {
	double readings[3][NUMBER_OF_READINGS];
	double sum[3] = { 0,0,0 };
	for (unsigned int reading = 0; reading < NUMBER_OF_READINGS; reading++) {
		for (unsigned int sensor = 0; sensor < 3; sensor++) {
			readings[sensor][reading] = evaluateSensor(ultrasonicArray[sensor]);
			sum[sensor] += readings[sensor][reading];
		}
	}
	for (unsigned int sensor = 0; sensor < 3; sensor++) {
		ultrasonicArray[sensor].value = sum[sensor] / NUMBER_OF_READINGS;
	}
	// Print in serial port
	for (unsigned int sensor = 0; sensor < 3; sensor++) {
		Serial.print("Valor do sensor ");
		Serial.print(sensor);
		Serial.print(" = ");
		Serial.print(ultrasonicArray[sensor].value);
		Serial.print("   ");
	}
}


/* ULTRASONIC FUZZY LOGIC */

// Fuzzification

// Ultrassonic sensors
// Support and core values 
const float VERY_NEAR_SENSOR_SET_CORE = 0.5;
const float VERY_NEAR_SENSOR_SET_SUPPORT = 1;

const float NEAR_SENSOR_SET_SUPPORT_1 = 0.5;
const float NEAR_SENSOR_SET_CORE_1 = 1;
const float NEAR_SENSOR_SET_CORE_2 = 2;
const float NEAR_SENSOR_SET_SUPPORT_2 = 2.5;

const float FAR_SENSOR_SET_SUPPORT_1 = 2;
const float FAR_SENSOR_SET_CORE_1 = 2.5;
const float FAR_SENSOR_SET_CORE_2 = 3;
const float FAR_SENSOR_SET_SUPPORT_2 = 3.5;

const float VERY_FAR_SENSOR_SET_SUPPORT = 3;
const float VERY_FAR_SENSOR_SET_CORE = 3.5;

// Membership calculation
float VeryNearSensorSet(float value) {
	float membership = 0;
	if (value <= VERY_NEAR_SENSOR_SET_CORE) {
		membership = 1;
	}
	else if (value < VERY_NEAR_SENSOR_SET_SUPPORT) {
		membership = (value - VERY_NEAR_SENSOR_SET_SUPPORT)
			/ (VERY_NEAR_SENSOR_SET_CORE - VERY_NEAR_SENSOR_SET_SUPPORT);
	}
	return membership;
}

float NearSensorSet(float value) {
	float membership = 0;
	if (value > NEAR_SENSOR_SET_SUPPORT_1 && value < NEAR_SENSOR_SET_CORE_1) {
		membership = (value - NEAR_SENSOR_SET_SUPPORT_1)
			/ (NEAR_SENSOR_SET_CORE_1 - NEAR_SENSOR_SET_SUPPORT_1);
	}
	else if (value >= NEAR_SENSOR_SET_CORE_1 && value <= NEAR_SENSOR_SET_CORE_2) {
		membership = 1;
	}
	else if (value > NEAR_SENSOR_SET_CORE_2 && value < NEAR_SENSOR_SET_SUPPORT_2) {
		membership = (value - NEAR_SENSOR_SET_SUPPORT_2)
			/ (NEAR_SENSOR_SET_CORE_2 - NEAR_SENSOR_SET_SUPPORT_2);
	}
	return membership;
}

float FarSensorSet(float value) {
	float membership = 0;
	if (value > FAR_SENSOR_SET_SUPPORT_1 && value < FAR_SENSOR_SET_CORE_1) {
		membership = (value - FAR_SENSOR_SET_SUPPORT_1)
			/ (FAR_SENSOR_SET_CORE_1 - FAR_SENSOR_SET_SUPPORT_1);
	}
	else if (value >= FAR_SENSOR_SET_CORE_1 && value <= FAR_SENSOR_SET_CORE_2) {
		membership = 1;
	}
	else if (value > FAR_SENSOR_SET_CORE_2 && value < FAR_SENSOR_SET_SUPPORT_2) {
		membership = (value - FAR_SENSOR_SET_SUPPORT_2)
			/ (FAR_SENSOR_SET_CORE_2 - FAR_SENSOR_SET_SUPPORT_2);
	}
	return membership;
}

float VeryFarSensorSet(float value) {
	float membership = 0;
	if (value >= VERY_FAR_SENSOR_SET_CORE){
		membership = 1;
	}
	else if (value > VERY_FAR_SENSOR_SET_SUPPORT) {
		membership = (value - VERY_FAR_SENSOR_SET_SUPPORT)
			/ (VERY_FAR_SENSOR_SET_CORE - VERY_FAR_SENSOR_SET_SUPPORT);
	}
	return membership;
}

// Test
void SensorFuzzyTest() {
	float m1 = 0;
	float m2 = 0;
	float m3 = 0;
	float m4 = 0;
	float value = 0;
	Serial.println("Fuzzy output: ");
	for (int i = 0; i < 50; i++) {
		value = i / 10.;
		m1 = VeryNearSensorSet(value);
		m2 = NearSensorSet(value);
		m3 = FarSensorSet(value);
		m4 = VeryFarSensorSet(value);
		Serial.print("MP=");
		Serial.print(m1);
		Serial.print("; P=");
		Serial.print(m2);
		Serial.print("; L=");
		Serial.print(m3);
		Serial.print("; ML=");
		Serial.println(m4);
	}
	delay(1000);
}

// Distance to goal
// Support and core values 
const float VERY_NEAR_GOAL_CORE = 0.15;
const float VERY_NEAR_GOAL_SUPPORT = 0.45;

const float NEAR_GOAL_SUPPORT_1 = 0.15;
const float NEAR_GOAL_CORE = 0.45;
const float NEAR_GOAL_SUPPORT_2 = 0.75;

const float FAR_GOAL_SUPPORT_1 = 0.45;
const float FAR_GOAL_CORE = 0.75;
const float FAR_GOAL_SUPPORT_2 = 1;

const float VERY_FAR_GOAL_SUPPORT = .75;
const float VERY_FAR_GOAL_CORE = 1;

// Membership calculation
float VeryNearGoalSet(float value) {
	float membership = 0;
	if (value <= VERY_NEAR_GOAL_CORE) {
		membership = 1;
	}
	else if (value < VERY_NEAR_GOAL_SUPPORT) {
		membership = (value - VERY_NEAR_GOAL_SUPPORT)
			/ (VERY_NEAR_GOAL_CORE - VERY_NEAR_GOAL_SUPPORT);
	}
	return membership;
}

float NearGoalSet(float value) {
	float membership = 0;
	if (value > NEAR_GOAL_SUPPORT_1 && value < NEAR_GOAL_CORE) {
		membership = (value - NEAR_GOAL_SUPPORT_1)
			/ (NEAR_GOAL_CORE - NEAR_GOAL_SUPPORT_1);
	}
	else if (value == NEAR_GOAL_CORE) {
		membership = 1;
	}
	else if (value > NEAR_GOAL_CORE && value < NEAR_GOAL_SUPPORT_2) {
		membership = (value - NEAR_GOAL_SUPPORT_2)
			/ (NEAR_GOAL_CORE - NEAR_GOAL_SUPPORT_2);
	}
	return membership;
}

float FarGoalSet(float value) {
	float membership = 0;
	if (value > FAR_GOAL_SUPPORT_1 && value < FAR_GOAL_CORE) {
		membership = (value - FAR_GOAL_SUPPORT_1)
			/ (FAR_GOAL_CORE - FAR_GOAL_SUPPORT_1);
	}
	else if (value == FAR_GOAL_CORE) {
		membership = 1;
	}
	else if (value > FAR_GOAL_CORE && value < FAR_GOAL_SUPPORT_2) {
		membership = (value - FAR_GOAL_SUPPORT_2)
			/ (FAR_GOAL_CORE - FAR_GOAL_SUPPORT_2);
	}
	return membership;
}

float VeryFarGoalSet(float value) {
	float membership = 0;
	if (value >= VERY_FAR_GOAL_CORE) {
		membership = 1;
	}
	else if (value > VERY_FAR_GOAL_SUPPORT) {
		membership = (value - VERY_FAR_GOAL_SUPPORT)
			/ (VERY_FAR_GOAL_CORE - VERY_FAR_GOAL_SUPPORT);
	}
	return membership;
}

// Test
void GoalFuzzyTest() {
	float m1 = 0;
	float m2 = 0;
	float m3 = 0;
	float m4 = 0;
	float value = 0;
	Serial.println("Fuzzy output: ");
	for (int i = 0; i < 50; i++) {
		value = i / 50.;
		m1 = VeryNearGoalSet(value);
		m2 = NearGoalSet(value);
		m3 = FarGoalSet(value);
		m4 = VeryFarGoalSet(value);
		Serial.print("MP=");
		Serial.print(m1);
		Serial.print("; P=");
		Serial.print(m2);
		Serial.print("; L=");
		Serial.print(m3);
		Serial.print("; ML=");
		Serial.println(m4);
	}
	delay(1000);
}

// Angle to objective (in degrees)
// Follows right-hand rule (counter clockwise):
// Robot pointing left is positive and pointing right is negative
// Support and core values 
const float VERY_NEGATIVE_ANGLE_SUPPORT_1 = -180;
const float VERY_NEGATIVE_ANGLE_CORE = -125;
const float VERY_NEGATIVE_ANGLE_SUPPORT_2 = -72.5;

const float NEGATIVE_ANGLE_SUPPORT_1 = -125;
const float NEGATIVE_ANGLE_CORE = -72.5;
const float NEGATIVE_ANGLE_SUPPORT_2 = -20;

const float ZERO_ANGLE_SUPPORT_1 = -45;
const float ZERO_ANGLE_CORE = 0;
const float ZERO_ANGLE_SUPPORT_2 = 45;

const float POSITIVE_ANGLE_SUPPORT_1 = 20;
const float POSITIVE_ANGLE_CORE = 72.5;
const float POSITIVE_ANGLE_SUPPORT_2 = 125;

const float VERY_POSITIVE_ANGLE_SUPPORT_1 = 72.5;
const float VERY_POSITIVE_ANGLE_CORE = 125;
const float VERY_POSITIVE_ANGLE_SUPPORT_2 = 180;

// Membership calculation
float VeryNegativeAngleSet(float value) {
	float membership = 0;
	if (value > VERY_NEGATIVE_ANGLE_SUPPORT_1 && value < VERY_NEGATIVE_ANGLE_CORE) {
		membership = (value - VERY_NEGATIVE_ANGLE_SUPPORT_1)
			/ (VERY_NEGATIVE_ANGLE_CORE - VERY_NEGATIVE_ANGLE_SUPPORT_1);
	}
	else if (value == VERY_NEGATIVE_ANGLE_CORE) {
		membership = 1;
	}
	else if (value > VERY_NEGATIVE_ANGLE_CORE && value < VERY_NEGATIVE_ANGLE_SUPPORT_2) {
		membership = (value - VERY_NEGATIVE_ANGLE_SUPPORT_2)
			/ (VERY_NEGATIVE_ANGLE_CORE - VERY_NEGATIVE_ANGLE_SUPPORT_2);
	}
	return membership;
}

float NegativeAngleSet(float value) {
	float membership = 0;
	if (value > NEGATIVE_ANGLE_SUPPORT_1 && value < NEGATIVE_ANGLE_CORE) {
		membership = (value - NEGATIVE_ANGLE_SUPPORT_1)
			/ (NEGATIVE_ANGLE_CORE - NEGATIVE_ANGLE_SUPPORT_1);
	}
	else if (value == NEGATIVE_ANGLE_CORE) {
		membership = 1;
	}
	else if (value > NEGATIVE_ANGLE_CORE && value < NEGATIVE_ANGLE_SUPPORT_2) {
		membership = (value - NEGATIVE_ANGLE_SUPPORT_2)
			/ (NEGATIVE_ANGLE_CORE - NEGATIVE_ANGLE_SUPPORT_2);
	}
	return membership;
}

float ZeroAngleSet(float value) {
	float membership = 0;
	if (value > ZERO_ANGLE_SUPPORT_1 && value < ZERO_ANGLE_CORE) {
		membership = (value - ZERO_ANGLE_SUPPORT_1)
			/ (ZERO_ANGLE_CORE - ZERO_ANGLE_SUPPORT_1);
	}
	else if (value == ZERO_ANGLE_CORE) {
		membership = 1;
	}
	else if (value > ZERO_ANGLE_CORE && value < ZERO_ANGLE_SUPPORT_2) {
		membership = (value - ZERO_ANGLE_SUPPORT_2)
			/ (ZERO_ANGLE_CORE - ZERO_ANGLE_SUPPORT_2);
	}
	return membership;
}

float PositiveAngleSet(float value) {
	float membership = 0;
	if (value > POSITIVE_ANGLE_SUPPORT_1 && value < POSITIVE_ANGLE_CORE) {
		membership = (value - POSITIVE_ANGLE_SUPPORT_1)
			/ (POSITIVE_ANGLE_CORE - POSITIVE_ANGLE_SUPPORT_1);
	}
	else if (value == POSITIVE_ANGLE_CORE) {
		membership = 1;
	}
	else if (value > POSITIVE_ANGLE_CORE && value < POSITIVE_ANGLE_SUPPORT_2) {
		membership = (value - POSITIVE_ANGLE_SUPPORT_2)
			/ (POSITIVE_ANGLE_CORE - POSITIVE_ANGLE_SUPPORT_2);
	}
	return membership;
}

float VeryPositiveAngleSet(float value) {
	float membership = 0;
	if (value > VERY_POSITIVE_ANGLE_SUPPORT_1 && value < VERY_POSITIVE_ANGLE_CORE) {
		membership = (value - VERY_POSITIVE_ANGLE_SUPPORT_1)
			/ (VERY_POSITIVE_ANGLE_CORE - VERY_POSITIVE_ANGLE_SUPPORT_1);
	}
	else if (value == VERY_POSITIVE_ANGLE_CORE) {
		membership = 1;
	}
	else if (value > VERY_POSITIVE_ANGLE_CORE && value < VERY_POSITIVE_ANGLE_SUPPORT_2) {
		membership = (value - VERY_POSITIVE_ANGLE_SUPPORT_2)
			/ (VERY_POSITIVE_ANGLE_CORE - VERY_POSITIVE_ANGLE_SUPPORT_2);
	}
	return membership;
}

// Test
void AngleFuzzyTest() {
	float m1 = 0;
	float m2 = 0;
	float m3 = 0;
	float m4 = 0;
	float m5 = 0;
	float value = 0;
	Serial.println("Fuzzy output: ");
	for (int i = 0; i < 63; i++) {
		value = (i - 32) / 10.;
		m1 = VeryNegativeAngleSet(value);
		m2 = NegativeAngleSet(value);
		m3 = ZeroAngleSet(value);
		m4 = PositiveAngleSet(value);
		m5 = VeryPositiveAngleSet(value);
		Serial.print("MN=");
		Serial.print(m1);
		Serial.print("; N=");
		Serial.print(m2);
		Serial.print("; Z=");
		Serial.print(m3);
		Serial.print("; P=");
		Serial.print(m4);
		Serial.print("; MP=");
		Serial.println(m5);
	}
	delay(1000);
}

// Motor Power
// Support and core values 
const float VERY_NEGATIVE_POWER_SUPPORT_1 = -1;
//const float VERY_NEGATIVE_POWER_CORE = -0.8;
const float VERY_NEGATIVE_POWER_CORE = -1; // Because of deffuz method
const float VERY_NEGATIVE_POWER_SUPPORT_2 = -0.4;

const float NEGATIVE_POWER_SUPPORT_1 = -0.6;
const float NEGATIVE_POWER_CORE = -0.4;
const float NEGATIVE_POWER_SUPPORT_2 = -0.1;

const float ZERO_POWER_SUPPORT_1 = -0.2;
const float ZERO_POWER_CORE = 0;
const float ZERO_POWER_SUPPORT_2 = 0.2;

const float POSITIVE_POWER_SUPPORT_1 = 0.1;
const float POSITIVE_POWER_CORE = 0.4;
const float POSITIVE_POWER_SUPPORT_2 = 0.6;

const float VERY_POSITIVE_POWER_SUPPORT_1 = 0.4;
//const float VERY_POSITIVE_POWER_CORE = 0.8;
const float VERY_POSITIVE_POWER_CORE = 1; // Because of deffuz method
const float VERY_POSITIVE_POWER_SUPPORT_2 = 1;

// Membership calculation
float VeryNegativePowerSet(float value) {
	float membership = 0;
	if (value > VERY_NEGATIVE_POWER_SUPPORT_1 && value < VERY_NEGATIVE_POWER_CORE) {
		membership = (value - VERY_NEGATIVE_POWER_SUPPORT_1)
			/ (VERY_NEGATIVE_POWER_CORE - VERY_NEGATIVE_POWER_SUPPORT_1);
	}
	else if (value == VERY_NEGATIVE_POWER_CORE) {
		membership = 1;
	}
	else if (value > VERY_NEGATIVE_POWER_CORE && value < VERY_NEGATIVE_POWER_SUPPORT_2) {
		membership = (value - VERY_NEGATIVE_POWER_SUPPORT_2)
			/ (VERY_NEGATIVE_POWER_CORE - VERY_NEGATIVE_POWER_SUPPORT_2);
	}
	return membership;
}

float NegativePowerSet(float value) {
	float membership = 0;
	if (value > NEGATIVE_POWER_SUPPORT_1 && value < NEGATIVE_POWER_CORE) {
		membership = (value - NEGATIVE_POWER_SUPPORT_1)
			/ (NEGATIVE_POWER_CORE - NEGATIVE_POWER_SUPPORT_1);
	}
	else if (value == NEGATIVE_POWER_CORE) {
		membership = 1;
	}
	else if (value > NEGATIVE_POWER_CORE && value < NEGATIVE_POWER_SUPPORT_2) {
		membership = (value - NEGATIVE_POWER_SUPPORT_2)
			/ (NEGATIVE_POWER_CORE - NEGATIVE_POWER_SUPPORT_2);
	}
	return membership;
}

float ZeroPowerSet(float value) {
	float membership = 0;
	if (value > ZERO_POWER_SUPPORT_1 && value < ZERO_POWER_CORE) {
		membership = (value - ZERO_POWER_SUPPORT_1)
			/ (ZERO_POWER_CORE - ZERO_POWER_SUPPORT_1);
	}
	else if (value == ZERO_POWER_CORE) {
		membership = 1;
	}
	else if (value > ZERO_POWER_CORE && value < ZERO_POWER_SUPPORT_2) {
		membership = (value - ZERO_POWER_SUPPORT_2)
			/ (ZERO_POWER_CORE - ZERO_POWER_SUPPORT_2);
	}
	return membership;
}

float PositivePowerSet(float value) {
	float membership = 0;
	if (value > POSITIVE_POWER_SUPPORT_1 && value < POSITIVE_POWER_CORE) {
		membership = (value - POSITIVE_POWER_SUPPORT_1)
			/ (POSITIVE_POWER_CORE - POSITIVE_POWER_SUPPORT_1);
	}
	else if (value == POSITIVE_POWER_CORE) {
		membership = 1;
	}
	else if (value > POSITIVE_POWER_CORE && value < POSITIVE_POWER_SUPPORT_2) {
		membership = (value - POSITIVE_POWER_SUPPORT_2)
			/ (POSITIVE_POWER_CORE - POSITIVE_POWER_SUPPORT_2);
	}
	return membership;
}

float VeryPositivePowerSet(float value) {
	float membership = 0;
	if (value > VERY_POSITIVE_POWER_SUPPORT_1 && value < VERY_POSITIVE_POWER_CORE) {
		membership = (value - VERY_POSITIVE_POWER_SUPPORT_1)
			/ (VERY_POSITIVE_POWER_CORE - VERY_POSITIVE_POWER_SUPPORT_1);
	}
	else if (value == VERY_POSITIVE_POWER_CORE) {
		membership = 1;
	}
	else if (value > VERY_POSITIVE_POWER_CORE && value < VERY_POSITIVE_POWER_SUPPORT_2) {
		membership = (value - VERY_POSITIVE_POWER_SUPPORT_2)
			/ (VERY_POSITIVE_POWER_CORE - VERY_POSITIVE_POWER_SUPPORT_2);
	}
	return membership;
}

// Test
void PowerFuzzyTest() {
	float m1 = 0;
	float m2 = 0;
	float m3 = 0;
	float m4 = 0;
	float m5 = 0;
	float value = 0;
	Serial.println("Fuzzy output: ");
	for (int i = 0; i < 50; i++) {
		value = (i - 25) / 25.;
		m1 = VeryNegativePowerSet(value);
		m2 = NegativePowerSet(value);
		m3 = ZeroPowerSet(value);
		m4 = PositivePowerSet(value);
		m5 = VeryPositivePowerSet(value);
		Serial.print("MN=");
		Serial.print(m1);
		Serial.print("; N=");
		Serial.print(m2);
		Serial.print("; Z=");
		Serial.print(m3);
		Serial.print("; P=");
		Serial.print(m4);
		Serial.print("; MP=");
		Serial.println(m5);
	}
	delay(1000);
}

void Fuzzify() {
	// Left Sensor
	_veryNearLeftSensorMembership = VeryNearSensorSet(_leftSensorReading);
	_nearLeftSensorMembership = NearSensorSet(_leftSensorReading);
	_farLeftSensorMembership = FarSensorSet(_leftSensorReading);
	_veryFarLeftSensorMembership = VeryFarSensorSet(_leftSensorReading);

	// Front Sensor
	_veryNearFrontSensorMembership = VeryNearSensorSet(_frontSensorReading);
	_nearFrontSensorMembership = NearSensorSet(_frontSensorReading);
	_farFrontSensorMembership = FarSensorSet(_frontSensorReading);
	_veryFarFrontSensorMembership = VeryFarSensorSet(_frontSensorReading);

	// Right Sensor
	_veryNearRightSensorMembership = VeryNearSensorSet(_rightSensorReading);
	_nearRightSensorMembership = NearSensorSet(_rightSensorReading);
	_farRightSensorMembership = FarSensorSet(_rightSensorReading);
	_veryFarRightSensorMembership = VeryFarSensorSet(_rightSensorReading);

	// Distance to Goal
	_veryNearGoalMembership = VeryNearGoalSet(_distanceReading);
	_nearGoalMembership = NearGoalSet(_distanceReading);
	_farGoalMembership = FarGoalSet(_distanceReading);
	_veryFarGoalMembership = VeryFarGoalSet(_distanceReading);

	// Angle between Robot and Goal
	_veryNegativeAngleMembership = VeryNegativeAngleSet(_angleReading);
	_negativeAngleMembership = NegativeAngleSet(_angleReading);
	_zeroAngleMembership = ZeroAngleSet(_angleReading);
	_positiveAngleMembership = PositiveAngleSet(_angleReading);
	_veryPositiveAngleMembership = VeryPositiveAngleSet(_angleReading);
}

// FUZZY RULES

float LeftMotorVeryPositive() {
	float maximum = 0., leftMotor[477];
	leftMotor[0] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[1] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[2] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[3] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[4] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[5] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[6] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[7] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[8] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[9] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[10] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[11] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[12] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[13] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[14] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[15] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[16] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[17] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[18] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[19] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[20] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[21] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[22] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[23] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[24] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[25] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[26] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[27] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[28] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[29] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[30] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[31] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[32] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[33] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[34] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[35] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[36] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[37] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[38] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[39] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[40] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[41] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[42] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[43] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[44] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[45] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[46] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[47] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[48] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[49] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[50] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[51] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[52] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[53] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[54] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[55] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[56] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[57] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[58] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[59] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[60] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[61] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[62] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[63] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[64] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[65] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[66] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[67] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[68] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[69] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[70] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[71] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[72] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[73] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[74] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[75] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[76] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[77] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[78] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[79] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[80] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[81] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[82] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[83] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[84] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[85] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[86] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[87] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[88] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[89] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[90] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[91] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[92] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[93] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[94] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[95] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[96] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[97] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[98] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[99] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[100] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[101] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[102] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[103] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[104] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[105] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[106] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[107] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[108] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[109] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[110] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[111] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[112] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[113] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[114] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[115] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[116] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[117] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[118] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[119] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[120] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[121] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[122] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[123] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[124] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[125] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[126] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[127] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[128] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[129] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[130] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[131] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[132] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[133] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[134] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[135] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[136] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[137] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[138] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[139] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[140] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[141] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[142] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[143] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[144] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[145] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[146] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[147] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[148] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[149] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[150] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[151] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[152] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[153] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[154] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[155] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[156] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[157] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[158] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[159] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[160] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[161] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[162] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[163] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[164] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[165] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[166] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[167] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[168] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[169] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[170] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[171] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[172] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[173] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[174] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[175] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[176] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[177] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[178] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[179] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[180] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[181] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[182] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[183] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[184] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[185] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[186] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[187] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[188] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[189] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[190] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[191] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[192] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[193] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[194] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[195] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[196] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[197] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[198] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[199] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[200] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[201] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[202] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[203] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[204] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[205] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[206] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[207] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[208] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[209] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[210] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[211] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[212] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[213] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[214] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[215] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[216] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[217] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[218] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[219] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[220] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[221] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[222] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[223] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[224] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[225] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[226] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[227] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[228] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[229] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[230] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[231] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[232] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[233] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[234] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[235] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[236] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[237] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[238] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[239] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[240] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[241] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[242] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[243] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[244] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[245] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[246] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[247] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[248] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[249] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[250] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[251] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[252] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[253] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[254] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[255] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[256] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[257] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[258] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[259] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[260] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[261] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[262] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[263] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[264] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[265] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[266] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[267] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[268] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[269] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[270] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[271] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[272] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[273] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[274] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[275] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[276] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[277] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[278] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[279] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[280] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[281] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[282] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[283] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[284] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[285] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[286] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[287] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[288] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[289] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[290] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[291] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[292] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[293] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[294] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[295] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[296] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[297] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[298] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[299] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[300] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[301] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[302] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[303] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[304] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[305] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[306] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[307] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[308] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[309] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[310] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[311] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[312] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[313] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[314] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[315] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[316] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[317] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[318] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[319] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[320] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[321] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[322] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[323] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[324] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[325] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[326] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[327] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[328] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[329] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[330] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[331] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[332] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[333] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[334] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[335] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[336] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[337] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[338] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[339] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[340] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[341] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[342] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[343] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[344] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[345] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[346] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[347] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[348] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[349] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[350] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[351] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[352] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[353] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[354] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[355] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[356] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[357] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[358] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[359] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[360] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[361] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[362] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[363] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[364] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[365] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[366] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[367] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[368] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[369] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[370] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[371] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[372] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[373] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[374] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[375] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[376] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[377] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[378] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[379] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[380] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[381] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[382] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[383] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[384] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[385] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[386] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[387] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[388] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[389] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[390] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[391] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[392] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[393] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[394] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[395] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[396] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[397] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[398] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[399] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[400] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[401] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[402] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[403] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[404] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[405] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[406] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[407] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[408] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[409] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[410] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[411] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[412] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[413] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[414] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[415] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[416] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[417] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[418] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[419] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[420] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[421] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[422] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[423] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[424] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[425] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[426] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[427] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[428] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[429] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[430] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[431] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[432] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[433] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[434] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[435] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[436] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[437] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[438] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[439] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[440] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[441] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[442] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[443] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[444] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[445] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[446] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[447] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[448] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[449] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[450] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[451] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[452] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[453] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[454] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[455] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[456] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[457] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[458] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[459] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[460] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[461] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[462] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[463] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[464] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[465] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[466] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[467] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[468] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[469] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[470] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[471] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[472] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[473] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[474] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[475] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[476] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	for (unsigned int i = 0; i < 477; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorPositive() {
	float maximum = 0., leftMotor[184];
	leftMotor[0] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[1] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[2] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[3] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[4] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[5] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[6] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[7] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[8] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[9] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[10] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[11] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[12] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[13] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[14] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[15] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[16] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[17] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[18] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[19] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[20] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[21] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[22] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[23] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[24] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[25] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[26] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[27] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[28] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[29] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[30] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[31] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[32] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[33] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[34] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[35] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[36] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[37] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[38] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[39] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[40] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[41] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[42] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[43] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[44] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[45] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[46] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[47] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[48] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[49] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[50] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[51] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[52] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[53] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[54] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[55] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[56] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[57] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[58] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[59] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[60] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[61] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[62] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[63] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[64] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[65] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[66] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[67] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[68] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[69] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[70] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[71] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[72] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[73] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[74] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[75] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[76] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[77] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[78] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[79] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[80] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[81] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[82] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[83] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[84] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[85] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[86] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[87] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[88] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[89] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[90] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[91] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[92] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[93] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[94] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[95] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[96] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[97] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[98] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[99] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[100] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[101] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[102] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[103] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[104] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[105] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[106] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[107] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[108] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[109] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[110] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[111] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[112] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[113] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[114] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[115] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[116] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[117] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[118] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[119] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[120] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[121] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[122] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[123] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[124] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[125] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[126] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[127] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[128] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[129] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[130] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[131] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[132] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[133] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[134] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[135] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[136] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[137] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[138] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[139] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[140] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[141] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[142] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[143] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[144] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[145] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[146] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[147] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[148] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[149] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[150] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[151] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[152] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[153] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[154] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[155] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[156] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[157] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[158] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[159] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[160] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[161] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[162] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[163] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[164] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[165] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[166] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[167] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[168] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[169] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[170] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[171] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[172] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[173] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[174] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[175] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[176] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[177] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[178] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[179] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[180] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[181] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[182] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[183] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	for (unsigned int i = 0; i < 184; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorZero() {
	float maximum = 0., leftMotor[335];
	leftMotor[0] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[1] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[2] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[3] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[4] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[5] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[6] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[7] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[8] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[9] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[10] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[11] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[12] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[13] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[14] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[15] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[16] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[17] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[18] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[19] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[20] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[21] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[22] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[23] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[24] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[25] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[26] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[27] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[28] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[29] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[30] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[31] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[32] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[33] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[34] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[35] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[36] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[37] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[38] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[39] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[40] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[41] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[42] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[43] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[44] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[45] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[46] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[47] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[48] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[49] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[50] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[51] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[52] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[53] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[54] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[55] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[56] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[57] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[58] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[59] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[60] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[61] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[62] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[63] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[64] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[65] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[66] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[67] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[68] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[69] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[70] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[71] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[72] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[73] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[74] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[75] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[76] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[77] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[78] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
	leftMotor[79] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[80] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[81] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[82] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[83] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[84] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[85] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[86] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[87] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[88] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[89] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[90] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[91] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[92] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[93] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[94] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[95] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[96] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[97] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[98] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[99] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[100] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[101] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[102] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[103] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[104] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[105] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[106] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[107] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[108] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[109] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[110] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[111] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[112] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[113] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[114] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[115] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[116] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[117] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[118] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[119] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[120] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[121] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[122] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[123] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[124] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[125] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[126] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[127] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[128] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[129] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[130] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[131] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[132] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[133] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[134] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[135] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[136] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[137] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[138] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[139] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[140] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[141] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[142] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[143] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[144] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[145] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[146] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[147] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[148] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[149] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[150] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[151] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[152] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[153] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[154] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[155] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[156] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[157] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[158] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[159] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[160] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[161] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[162] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[163] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[164] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[165] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[166] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[167] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[168] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[169] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[170] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[171] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[172] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[173] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[174] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[175] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[176] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[177] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[178] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[179] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[180] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[181] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[182] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[183] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[184] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[185] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[186] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[187] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[188] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[189] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[190] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[191] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[192] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[193] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[194] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[195] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[196] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[197] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[198] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[199] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[200] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[201] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[202] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[203] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[204] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[205] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[206] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
	leftMotor[207] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[208] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[209] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[210] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[211] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[212] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[213] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[214] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[215] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[216] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[217] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[218] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[219] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[220] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[221] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[222] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[223] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[224] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[225] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[226] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[227] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[228] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[229] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[230] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[231] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[232] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[233] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[234] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[235] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[236] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[237] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[238] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[239] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[240] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[241] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[242] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[243] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[244] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[245] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[246] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[247] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[248] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[249] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[250] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[251] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[252] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[253] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[254] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[255] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[256] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[257] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[258] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[259] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[260] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[261] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[262] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[263] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[264] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[265] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[266] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[267] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[268] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[269] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[270] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[271] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[272] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[273] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[274] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[275] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[276] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[277] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[278] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[279] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[280] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[281] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[282] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[283] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[284] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[285] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[286] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[287] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[288] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[289] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[290] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[291] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[292] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[293] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[294] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[295] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[296] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[297] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[298] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[299] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[300] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[301] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[302] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[303] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[304] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[305] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[306] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[307] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[308] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[309] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[310] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[311] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[312] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[313] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[314] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[315] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[316] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[317] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[318] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[319] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[320] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[321] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[322] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[323] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[324] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[325] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[326] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[327] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[328] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[329] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[330] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[331] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[332] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[333] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	leftMotor[334] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
	for (unsigned int i = 0; i < 334; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorNegative() {
	float maximum = 0., leftMotor[34];
	leftMotor[0] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[1] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[2] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[3] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[4] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[5] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[6] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[7] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[8] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[9] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[10] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[11] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[12] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[13] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[14] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[15] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[16] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[17] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[18] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[19] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[20] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[21] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[22] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[23] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[24] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[25] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[26] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[27] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[28] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[29] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[30] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[31] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[32] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[33] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	for (unsigned int i = 0; i < 34; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorVeryNegative() {
	float maximum = 0., leftMotor[250];
	leftMotor[0] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[1] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[2] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[3] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[4] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[5] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[6] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[7] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[8] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[9] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[10] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[11] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[12] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[13] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[14] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[15] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[16] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[17] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[18] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[19] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[20] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[21] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[22] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[23] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[24] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[25] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[26] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[27] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[28] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[29] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[30] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[31] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[32] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[33] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[34] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[35] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[36] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[37] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[38] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[39] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[40] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[41] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[42] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[43] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[44] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[45] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[46] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[47] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
	leftMotor[48] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
	leftMotor[49] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
	leftMotor[50] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[51] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[52] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[53] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[54] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[55] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[56] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[57] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[58] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[59] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[60] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[61] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[62] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[63] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[64] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[65] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[66] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[67] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[68] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[69] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[70] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[71] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[72] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[73] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[74] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[75] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[76] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[77] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[78] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[79] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[80] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[81] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[82] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[83] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[84] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[85] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[86] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[87] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[88] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[89] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[90] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[91] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[92] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[93] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[94] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[95] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[96] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[97] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
	leftMotor[98] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[99] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
	leftMotor[100] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[101] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[102] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[103] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[104] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[105] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[106] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[107] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[108] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[109] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[110] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[111] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[112] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[113] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[114] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[115] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[116] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[117] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[118] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[119] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[120] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[121] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[122] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[123] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[124] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[125] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[126] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[127] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[128] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[129] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[130] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[131] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[132] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[133] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[134] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[135] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[136] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[137] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[138] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[139] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[140] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[141] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[142] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[143] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[144] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[145] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[146] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[147] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
	leftMotor[148] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
	leftMotor[149] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
	leftMotor[150] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[151] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[152] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[153] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[154] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[155] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[156] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[157] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[158] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[159] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[160] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[161] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[162] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[163] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[164] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[165] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[166] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[167] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[168] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[169] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[170] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[171] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[172] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[173] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[174] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[175] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[176] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[177] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[178] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[179] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[180] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[181] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[182] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[183] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[184] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[185] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[186] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[187] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[188] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[189] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[190] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[191] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[192] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[193] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[194] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[195] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[196] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[197] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
	leftMotor[198] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[199] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
	leftMotor[200] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[201] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[202] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[203] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[204] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[205] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[206] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[207] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[208] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[209] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[210] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[211] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[212] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[213] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[214] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[215] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[216] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[217] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[218] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[219] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[220] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[221] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[222] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[223] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[224] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[225] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[226] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[227] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[228] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[229] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[230] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[231] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[232] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[233] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[234] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[235] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[236] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[237] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[238] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[239] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[240] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[241] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[242] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[243] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[244] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[245] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[246] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	leftMotor[247] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
	leftMotor[248] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
	leftMotor[249] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
	for (unsigned int i = 0; i < 250; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float RightMotorVeryPositive() {
float maximum = 0., rightMotor[607];
rightMotor[0] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[1] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[2] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[3] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[4] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[5] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[6] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[7] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[8] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[9] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[10] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[11] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[12] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[13] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[14] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[15] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[16] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[17] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[18] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[19] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[20] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[21] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[22] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[23] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[24] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[25] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[26] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[27] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[28] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[29] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[30] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[31] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[32] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[33] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[34] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[35] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[36] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[37] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[38] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[39] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[40] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[41] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[42] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[43] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[44] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[45] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[46] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[47] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[48] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[49] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[50] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[51] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[52] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[53] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[54] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[55] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[56] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[57] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[58] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[59] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[60] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[61] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[62] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[63] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[64] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[65] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[66] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[67] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[68] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[69] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[70] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[71] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[72] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[73] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[74] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[75] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[76] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[77] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[78] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[79] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[80] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[81] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[82] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[83] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[84] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[85] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[86] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[87] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[88] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[89] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[90] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[91] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[92] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[93] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[94] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[95] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[96] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[97] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[98] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[99] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[100] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[101] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[102] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[103] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[104] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[105] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[106] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[107] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[108] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[109] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[110] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[111] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[112] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[113] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[114] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[115] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[116] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[117] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[118] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[119] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[120] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[121] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[122] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[123] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[124] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[125] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[126] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[127] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[128] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[129] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[130] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[131] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[132] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[133] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[134] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[135] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[136] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[137] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[138] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[139] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[140] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[141] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[142] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[143] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[144] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[145] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[146] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[147] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[148] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[149] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[150] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[151] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[152] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[153] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[154] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[155] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[156] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[157] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[158] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[159] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[160] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[161] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[162] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[163] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[164] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[165] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[166] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[167] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[168] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[169] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[170] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[171] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[172] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[173] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[174] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[175] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[176] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[177] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[178] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[179] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[180] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[181] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[182] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[183] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[184] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[185] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[186] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[187] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[188] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[189] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[190] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[191] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[192] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[193] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[194] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[195] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[196] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[197] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[198] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[199] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[200] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[201] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[202] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[203] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[204] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[205] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[206] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[207] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[208] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[209] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[210] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[211] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[212] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[213] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[214] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[215] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[216] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[217] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[218] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[219] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[220] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[221] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[222] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[223] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[224] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[225] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[226] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[227] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[228] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[229] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[230] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[231] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[232] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[233] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[234] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[235] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[236] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[237] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[238] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[239] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[240] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[241] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[242] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[243] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[244] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[245] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[246] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[247] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[248] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[249] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[250] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[251] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[252] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[253] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[254] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[255] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[256] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[257] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[258] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[259] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[260] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[261] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[262] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[263] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[264] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[265] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[266] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[267] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[268] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[269] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[270] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[271] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[272] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[273] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[274] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[275] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[276] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[277] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[278] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[279] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[280] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[281] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[282] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[283] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[284] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[285] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[286] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[287] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[288] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[289] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[290] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[291] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[292] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[293] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[294] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[295] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[296] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[297] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[298] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[299] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[300] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[301] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[302] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[303] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[304] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[305] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[306] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[307] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[308] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[309] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[310] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[311] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[312] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[313] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[314] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[315] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[316] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[317] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[318] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[319] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[320] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[321] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[322] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[323] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[324] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[325] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[326] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[327] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[328] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[329] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[330] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[331] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[332] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[333] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[334] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[335] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[336] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[337] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[338] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[339] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[340] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[341] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[342] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[343] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[344] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[345] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[346] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[347] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[348] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[349] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[350] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[351] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[352] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[353] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[354] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[355] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[356] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[357] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[358] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[359] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[360] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[361] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[362] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[363] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[364] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[365] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[366] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[367] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[368] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[369] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[370] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[371] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[372] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[373] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[374] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[375] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[376] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[377] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[378] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[379] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[380] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[381] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[382] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[383] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[384] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[385] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[386] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[387] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[388] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[389] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[390] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[391] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[392] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[393] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[394] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[395] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[396] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[397] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[398] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[399] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[400] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[401] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[402] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[403] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[404] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[405] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[406] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[407] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[408] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[409] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[410] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[411] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[412] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[413] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[414] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[415] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[416] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[417] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[418] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[419] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[420] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[421] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[422] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[423] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[424] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[425] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[426] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[427] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[428] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[429] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[430] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[431] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[432] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[433] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[434] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[435] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[436] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[437] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[438] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[439] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[440] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[441] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[442] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[443] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[444] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[445] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[446] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[447] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[448] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[449] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[450] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[451] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[452] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[453] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[454] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[455] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[456] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[457] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[458] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[459] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[460] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[461] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[462] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[463] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[464] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[465] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[466] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[467] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[468] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[469] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[470] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[471] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[472] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[473] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[474] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[475] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[476] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[477] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[478] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[479] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[480] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[481] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[482] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[483] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[484] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[485] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[486] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[487] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[488] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[489] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[490] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[491] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[492] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[493] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[494] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[495] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[496] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[497] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[498] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[499] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[500] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[501] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[502] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[503] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[504] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[505] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[506] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[507] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[508] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[509] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[510] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[511] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[512] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[513] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[514] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[515] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[516] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[517] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[518] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[519] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[520] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[521] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[522] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[523] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[524] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[525] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[526] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[527] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[528] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[529] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[530] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[531] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[532] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[533] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[534] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[535] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[536] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[537] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[538] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[539] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[540] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[541] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[542] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[543] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[544] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[545] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[546] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[547] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[548] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[549] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[550] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[551] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[552] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[553] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[554] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[555] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[556] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[557] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[558] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[559] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[560] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[561] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[562] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[563] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[564] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[565] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[566] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[567] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[568] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[569] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[570] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[571] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[572] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[573] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[574] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[575] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[576] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[577] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[578] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[579] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[580] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[581] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[582] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[583] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[584] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[585] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[586] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[587] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[588] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[589] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[590] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[591] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[592] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[593] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[594] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[595] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[596] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[597] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[598] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[599] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[600] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[601] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[602] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[603] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[604] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[605] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[606] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
for (unsigned int i = 0; i < 606; i++) {
maximum = max(maximum, rightMotor[i]);
}
return maximum;
}

float RightMotorPositive() {
float maximum = 0., rightMotor[169];
rightMotor[0] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[1] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[2] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[3] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[4] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[5] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[6] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[7] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[8] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[9] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[10] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[11] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[12] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[13] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[14] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[15] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[16] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[17] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[18] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[19] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[20] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[21] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[22] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[23] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[24] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[25] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[26] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[27] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[28] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[29] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[30] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[31] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[32] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[33] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[34] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[35] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[36] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[37] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[38] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[39] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[40] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[41] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[42] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[43] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[44] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[45] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[46] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[47] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[48] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[49] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[50] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[51] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[52] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[53] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[54] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[55] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[56] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[57] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[58] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[59] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[60] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[61] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[62] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[63] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[64] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[65] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[66] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[67] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[68] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[69] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[70] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[71] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[72] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[73] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[74] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[75] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[76] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[77] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[78] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[79] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[80] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[81] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[82] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[83] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[84] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[85] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[86] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[87] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[88] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[89] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[90] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[91] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[92] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[93] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[94] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[95] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[96] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[97] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[98] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[99] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[100] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[101] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[102] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[103] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[104] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[105] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[106] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[107] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[108] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[109] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[110] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[111] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[112] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[113] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[114] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[115] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[116] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[117] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[118] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[119] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[120] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[121] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[122] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[123] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[124] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[125] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[126] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[127] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[128] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[129] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[130] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[131] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[132] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[133] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[134] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[135] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[136] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[137] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[138] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[139] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[140] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[141] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[142] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[143] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[144] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[145] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[146] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[147] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[148] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[149] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[150] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[151] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[152] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[153] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[154] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[155] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[156] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[157] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[158] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[159] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[160] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[161] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[162] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[163] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[164] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[165] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[166] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[167] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[168] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
for (unsigned int i = 0; i < 168; i++) {
maximum = max(maximum, rightMotor[i]);
}
return maximum;
}

float RightMotorZero() {
float maximum = 0., rightMotor[320];
rightMotor[0] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[1] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[2] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[3] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[4] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[5] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[6] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[7] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[8] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[9] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[10] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[11] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[12] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[13] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[14] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[15] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[16] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[17] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[18] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[19] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[20] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[21] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[22] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[23] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[24] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[25] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[26] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[27] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[28] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[29] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[30] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[31] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[32] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[33] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[34] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[35] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[36] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[37] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[38] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[39] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[40] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[41] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[42] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[43] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[44] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[45] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[46] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[47] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[48] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[49] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[50] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[51] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[52] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[53] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[54] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[55] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[56] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[57] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[58] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[59] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[60] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[61] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[62] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[63] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryNearGoalMembership))));
rightMotor[64] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[65] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[66] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[67] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[68] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[69] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[70] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[71] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[72] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[73] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[74] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[75] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[76] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[77] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[78] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[79] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[80] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[81] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[82] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[83] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[84] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[85] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[86] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[87] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[88] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[89] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[90] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[91] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[92] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[93] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[94] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[95] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[96] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[97] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[98] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[99] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[100] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[101] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[102] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[103] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[104] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[105] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[106] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[107] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[108] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[109] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[110] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[111] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[112] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[113] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[114] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[115] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[116] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[117] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[118] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[119] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[120] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[121] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[122] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[123] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[124] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[125] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[126] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[127] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryNearGoalMembership))));
rightMotor[128] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[129] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[130] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[131] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[132] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[133] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[134] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[135] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[136] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[137] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[138] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[139] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[140] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[141] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[142] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[143] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[144] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[145] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[146] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[147] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[148] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[149] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[150] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[151] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[152] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[153] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[154] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[155] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[156] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[157] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[158] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[159] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[160] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[161] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[162] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[163] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[164] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[165] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[166] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[167] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[168] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[169] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[170] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[171] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[172] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[173] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[174] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[175] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[176] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[177] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[178] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[179] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[180] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[181] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[182] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[183] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[184] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[185] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[186] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[187] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[188] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[189] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[190] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[191] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryNearGoalMembership))));
rightMotor[192] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[193] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[194] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[195] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[196] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[197] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[198] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[199] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[200] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[201] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[202] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[203] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[204] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[205] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[206] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[207] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[208] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[209] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[210] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[211] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[212] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[213] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[214] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[215] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[216] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[217] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[218] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[219] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[220] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[221] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[222] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[223] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[224] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[225] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[226] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[227] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[228] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[229] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[230] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[231] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[232] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[233] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[234] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[235] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[236] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[237] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[238] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[239] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[240] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[241] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[242] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[243] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[244] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[245] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[246] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[247] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[248] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[249] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[250] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[251] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[252] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[253] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[254] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[255] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryNearGoalMembership))));
rightMotor[256] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[257] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[258] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[259] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[260] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[261] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[262] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[263] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[264] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[265] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[266] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[267] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[268] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[269] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[270] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[271] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[272] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[273] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[274] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[275] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[276] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[277] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[278] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[279] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[280] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[281] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[282] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[283] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[284] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[285] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[286] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[287] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[288] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[289] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[290] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[291] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[292] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[293] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[294] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[295] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[296] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[297] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[298] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[299] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[300] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[301] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[302] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[303] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[304] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[305] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[306] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[307] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[308] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[309] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[310] = min(_nearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[311] = min(_veryNearLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[312] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[313] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[314] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[315] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[316] = min(_nearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryNearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[317] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[318] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
rightMotor[319] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryNearGoalMembership))));
for (unsigned int i = 0; i < 319; i++) {
maximum = max(maximum, rightMotor[i]);
}
return maximum;
}

float RightMotorNegative() {
float maximum = 0., rightMotor[19];
rightMotor[0] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[1] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[2] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[3] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[3] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[4] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[4] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[5] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[5] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[6] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[6] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[7] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[7] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[8] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[8] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[9] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[9] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[10] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[10] = min(_veryFarLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[11] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[11] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[12] = min(_farLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[12] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[13] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[13] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[14] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[14] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[15] = min(_farLeftSensorMembership, min(_farFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[15] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[16] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[16] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[17] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[17] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[18] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
for (unsigned int i = 0; i < 18; i++) {
maximum = max(maximum, rightMotor[i]);
}
return maximum;
}

float RightMotorVeryNegative() {
float maximum = 0., rightMotor[150];
rightMotor[0] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[1] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[2] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[3] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[4] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[5] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[6] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[7] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[8] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[9] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[10] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[11] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[12] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[13] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[14] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[15] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[16] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[17] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[18] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[19] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[20] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[21] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[22] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[23] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[24] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[25] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[26] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[27] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _nearGoalMembership))));
rightMotor[28] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership))));
rightMotor[29] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_zeroAngleMembership, _farGoalMembership))));
rightMotor[30] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[31] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[32] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[33] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[34] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[35] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[36] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[37] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[38] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[39] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[40] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[41] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[42] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[43] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[44] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[45] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[46] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[47] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[48] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[49] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[50] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[51] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[52] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[53] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[54] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[55] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[56] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[57] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _nearGoalMembership))));
rightMotor[58] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _veryFarGoalMembership))));
rightMotor[59] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_positiveAngleMembership, _farGoalMembership))));
rightMotor[60] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[61] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[62] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[63] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[64] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[65] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[66] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[67] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[68] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[69] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[70] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[71] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[72] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[73] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[74] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[75] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[76] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[77] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[78] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[79] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[80] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[81] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[82] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[83] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[84] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[85] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[86] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[87] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _nearGoalMembership))));
rightMotor[88] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _veryFarGoalMembership))));
rightMotor[89] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryPositiveAngleMembership, _farGoalMembership))));
rightMotor[90] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[91] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[92] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[93] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[94] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[95] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[96] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[97] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[98] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[99] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[100] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[101] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[102] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[103] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[104] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[105] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[106] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[107] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[108] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[109] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[110] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[111] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[112] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[113] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[114] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[115] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[116] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[117] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _nearGoalMembership))));
rightMotor[118] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _veryFarGoalMembership))));
rightMotor[119] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_negativeAngleMembership, _farGoalMembership))));
rightMotor[120] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[121] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[122] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[123] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[124] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[125] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[126] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[127] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[128] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[129] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[130] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[131] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[132] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[133] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[134] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[135] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[136] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[137] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[138] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[139] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[140] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[141] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[142] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[143] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[144] = min(_veryNearLeftSensorMembership, min(_farFrontSensorMembership, min(_nearRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[145] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[146] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
rightMotor[147] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _nearGoalMembership))));
rightMotor[148] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _veryFarGoalMembership))));
rightMotor[149] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, min(_farRightSensorMembership, min(_veryNegativeAngleMembership, _farGoalMembership))));
for (unsigned int i = 0; i < 149; i++) {
maximum = max(maximum, rightMotor[i]);
}
return maximum;
}

void processRules() {
	// Left Motor Power
	_veryNegativeLeftPowerMembership = LeftMotorVeryNegative();
	_negativeLeftPowerMembership = LeftMotorNegative();
	_zeroLeftPowerMembership = LeftMotorZero();
	_positiveLeftPowerMembership = LeftMotorPositive();
	_veryPositiveLeftPowerMembership = LeftMotorVeryPositive();

	// Right Motor Power
	_veryNegativeRightPowerMembership = RightMotorVeryNegative();
	_negativeRightPowerMembership = RightMotorNegative();
	_zeroRightPowerMembership = RightMotorZero();
	_positiveRightPowerMembership = RightMotorPositive();
	_veryPositiveRightPowerMembership = RightMotorVeryPositive();
}

// Defuzzification 
float LeftMotorDefuzzification() {
	// Center of Maximum
	_leftMotorActivation = (VERY_NEGATIVE_POWER_CORE * _veryNegativeLeftPowerMembership +
		NEGATIVE_POWER_CORE * _negativeLeftPowerMembership +
		ZERO_POWER_CORE * _zeroLeftPowerMembership +
		POSITIVE_POWER_CORE * _positiveLeftPowerMembership +
		VERY_POSITIVE_POWER_CORE * _veryPositiveLeftPowerMembership) /
		(_veryNegativeLeftPowerMembership + _negativeLeftPowerMembership +
			_zeroLeftPowerMembership + _positiveLeftPowerMembership +
			_veryPositiveLeftPowerMembership);
	return _leftMotorActivation;
}

float RightMotorDefuzzification() {
	// Center of Maximum
	_rightMotorActivation = (VERY_NEGATIVE_POWER_CORE * _veryNegativeRightPowerMembership +
		NEGATIVE_POWER_CORE * _negativeRightPowerMembership +
		ZERO_POWER_CORE * _zeroRightPowerMembership +
		POSITIVE_POWER_CORE * _positiveRightPowerMembership +
		VERY_POSITIVE_POWER_CORE * _veryPositiveRightPowerMembership) /
		(_veryNegativeRightPowerMembership + _negativeRightPowerMembership +
			_zeroRightPowerMembership + _positiveRightPowerMembership +
			_veryPositiveRightPowerMembership);
	return _rightMotorActivation;
}

// Prints organized membership information
void printData() {
	//DEBUG PRINT
	// Sensor Readings
	Serial.println("SENSOR READING:");
	Serial.print("LEFT  ");
	Serial.println(_leftSensorReading);
	Serial.print("FRONT ");
	Serial.println(_frontSensorReading);
	Serial.print("RIGHT ");
	Serial.println(_rightSensorReading);
	Serial.print("DISTANCE TO GOAL ");
	Serial.println(_distanceReading);
	Serial.print("ANGLE TO GOAL    ");
	Serial.println(_angleReading);

	// Left Sensor
	Serial.println("LEFT SENSOR");
	Serial.print("VERY NEAR ");
	Serial.println(_veryNearLeftSensorMembership);
	Serial.print("NEAR      ");
	Serial.println(_nearLeftSensorMembership);
	Serial.print("FAR       ");
	Serial.println(_farLeftSensorMembership);
	Serial.print("VERY FAR  ");
	Serial.println(_veryFarLeftSensorMembership);

	// Front Sensor
	Serial.println("FRONT SENSOR");
	Serial.print("VERY NEAR ");
	Serial.println(_veryNearFrontSensorMembership);
	Serial.print("NEAR      ");
	Serial.println(_nearFrontSensorMembership);
	Serial.print("FAR       ");
	Serial.println(_farFrontSensorMembership);
	Serial.print("VERY FAR  ");
	Serial.println(_veryFarFrontSensorMembership);

	// Right Sensor
	Serial.println("RIGHT SENSOR");
	Serial.print("VERY NEAR ");
	Serial.println(_veryNearRightSensorMembership);
	Serial.print("NEAR      ");
	Serial.println(_nearRightSensorMembership);
	Serial.print("FAR       ");
	Serial.println(_farRightSensorMembership);
	Serial.print("VERY FAR  ");
	Serial.println(_veryFarRightSensorMembership);

	// Distance to Goal
	Serial.println("DISTANCE TO GOAL");
	Serial.print("VERY NEAR ");
	Serial.println(_veryNearGoalMembership);
	Serial.print("NEAR      ");
	Serial.println(_nearGoalMembership);
	Serial.print("FAR       ");
	Serial.println(_farGoalMembership);
	Serial.print("VERY FAR  ");
	Serial.println(_veryFarGoalMembership);

	// Angle between Robot and Goal
	Serial.println("ANGLE TO GOAL");
	Serial.print("VERY NEGATIVE ");
	Serial.println(_veryNegativeAngleMembership);
	Serial.print("NEGATIVE      ");
	Serial.println(_negativeAngleMembership);
	Serial.print("ZERO          ");
	Serial.println(_zeroAngleMembership);
	Serial.print("POSITIVE      ");
	Serial.println(_positiveAngleMembership);
	Serial.print("VERY POSITIVE ");
	Serial.println(_veryPositiveAngleMembership);

	// Left Motor Power
	Serial.println("LEFT MOTOR POWER");
	Serial.print("VERY NEGATIVE ");
	Serial.println(_veryNegativeLeftPowerMembership);
	Serial.print("NEGATIVE      ");
	Serial.println(_negativeLeftPowerMembership);
	Serial.print("ZERO          ");
	Serial.println(_zeroLeftPowerMembership);
	Serial.print("POSITIVE      ");
	Serial.println(_positiveLeftPowerMembership);
	Serial.print("VERY POSITIVE ");
	Serial.println(_veryPositiveLeftPowerMembership);

	// Right Motor Power
	Serial.println("RIGHT MOTOR POWER");
	Serial.print("VERY NEGATIVE ");
	Serial.println(_veryNegativeRightPowerMembership);
	Serial.print("NEGATIVE      ");
	Serial.println(_negativeRightPowerMembership);
	Serial.print("ZERO          ");
	Serial.println(_zeroRightPowerMembership);
	Serial.print("POSITIVE      ");
	Serial.println(_positiveRightPowerMembership);
	Serial.print("VERY POSITIVE ");
	Serial.println(_veryPositiveRightPowerMembership);

	Serial.println();
	Serial.print("LEFT=");
	Serial.print(_leftMotorActivation);
	Serial.print(" RIGHT=");
	Serial.println(_rightMotorActivation);
	Serial.println();
}

void printOutput() {
	Serial.print("LEFT MOTOR = ");
	Serial.print(_leftMotorActivation*100);
	Serial.print("%\t");
	Serial.print(" RIGHT MOTOR = ");
	Serial.print(_rightMotorActivation*100);
	Serial.println("%");
}

// Input mode for testing 
void setSimInput(float leftSensor, float frontSensor, float rightSensor, float distanceToGoal, float angleToGoal) {
	_leftSensorReading = leftSensor;
	_frontSensorReading = frontSensor;
	_rightSensorReading = rightSensor;
	_distanceReading = distanceToGoal;
	_angleReading = angleToGoal;
}

// Fuzzy process run example
void runFuzzyProcess() {
	Fuzzify();
	processRules();
	LeftMotorDefuzzification();
	RightMotorDefuzzification();
}

// Testing some positions to know how the fuzzy is working
void SimulateWalk() {
	float barrierPosition = 10;

	float leftSensor = 10;
	float frontSensor = 10;
	float rightSensor = 10;
	float angleToGoal = -3.1415972/10;

	float positionX = 0;
	float positionY = 0;
	float distanceToGoal = sqrt(positionX*positionX + (15 - positionY)*(15 - positionY));

	float speed = 0.1;
	int counter = 0;
	for (unsigned int i = 0; i < 1000; i++) {
		setSimInput(leftSensor, frontSensor, rightSensor, distanceToGoal, angleToGoal);
		runFuzzyProcess();

		// angle and position correction
		angleToGoal += (_rightMotorActivation - _leftMotorActivation)*speed;
		positionX += -sin(angleToGoal)*(_rightMotorActivation + _leftMotorActivation)*speed / 2;
		positionY += cos(angleToGoal)*(_rightMotorActivation + _leftMotorActivation)*speed / 2;
		distanceToGoal = sqrtf(positionX*positionX + (15 - positionY)*(15 - positionY));

		// sensor correction
		if (cos(angleToGoal + 3.1415 / 8) <= 0) {
			leftSensor = 1000;
		}
		else {
			leftSensor = (barrierPosition - positionY) / cos(angleToGoal + 3.1415 / 8);
		}
		if ( cos(angleToGoal) <= 0){
			frontSensor = 1000;
		}
		else {
			frontSensor = (barrierPosition - positionY) / cos(angleToGoal);
		}
		if (cos(angleToGoal - 3.1415 / 8) <= 0){
			rightSensor = 1000;
		}
		else {
			rightSensor = (barrierPosition - positionY) / cos(angleToGoal - 3.1415 / 8);
		}

		// Print results on serial port		
		Serial.print("Iteration ");
		Serial.println(counter);

		Serial.print("Left ");
		Serial.print(leftSensor);
		Serial.print(" / Front ");
		Serial.print(frontSensor);
		Serial.print(" / Right ");
		Serial.println(rightSensor);

		Serial.print("Angle ");
		Serial.print(angleToGoal);
		Serial.print(" / PosX ");
		Serial.print(positionX);
		Serial.print(" / PosY ");
		Serial.println(positionY);

		Serial.print("LEFT MOTOR = ");
		Serial.print(_leftMotorActivation * 100);
		Serial.print("%\t");
		Serial.print(" RIGHT MOTOR = ");
		Serial.print(_rightMotorActivation * 100);
		Serial.println("%");
		Serial.println();

		if (counter == 200) {
			barrierPosition = 1000;
		}
		counter++;
		if (counter > 200) {
			delay(2000);
		}
		delay(1000);
	}
}
