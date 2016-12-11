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
using namespace std;

const int NUMBER_OF_READINGS = 3;

typedef struct uSensor { int dataPort; int triggerPort; double value; };
typedef struct tSensor { int dataPort; double value; };
typedef struct doSensor { int dataPort; double value; };
typedef struct orpSensor { int dataPort; double value; };
typedef struct phSensor { int dataPort; double value; };

// PORT CONFIGURATION
#define sensor1Port		1
#define sensor1Trigger	2
int sensor2Port		= 3;
int sensor2Trigger	= 4;
int sensor3Port		= 5;
int sensor3Trigger	= 6;
int thermalAnalogPort = A0;


uSensor ultrasonic1;
uSensor ultrasonic2;
uSensor ultrasonic3;
uSensor ultrasonicArray[3] = { ultrasonic1 , ultrasonic2, ultrasonic3 };
tSensor thermal = { thermalAnalogPort, 0 };

// For Oxygen Reduction and dissolvedOxygen sensors
#include <SoftwareSerial.h>
#define oxygenReductionReceive 10	//Possible ports: 10 to 15; 50 to 53; A8 to A15
#define oxygenReductionTransmit 11
#define dissolvedOxygenReceive 12	//Possible ports: 10 to 15; 50 to 53; A8 to A15
#define dissolvedOxygenTransmit 13
SoftwareSerial oxygenReductionSerial(oxygenReductionReceive, oxygenReductionTransmit);
SoftwareSerial dissolvedOxygenSerial(dissolvedOxygenReceive, dissolvedOxygenTransmit);

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

	//SimulateWalk();

	

	delay(1000);
}

double evaluateSensor(struct uSensor sensor)
{
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
	return pulse / 58.;
}

double evaluateSensor(struct tSensor sensor) {
	// Calibrate sensor
	// 1st wire ----[GND]
	// 2nd wire ----[DATA] 
	//			\---[4,7K]----[5V]

	// analog pin reads 130 in 100°C and 694 in 0°C
	// multiplication is due map only using integers
	sensor.value = map(analogRead(sensor.dataPort), 694, 134, 0, 1000)/10.;

	//Serial.print("Temperature = ");
	//Serial.print(evaluateSensor(thermal));
	//Serial.println("C");
	return sensor.value;
}


double evaluateSensor(struct doSensor sensor){
	String sensorString;
	double value = 0;
	// If a Stringacter has been received
	if (dissolvedOxygenSerial.available() > 0){
		// Gets the received String
		String inString = (String)dissolvedOxygenSerial.read();
		// Composes the String
		sensorString += inString;
		// Reading ends with a CR
		if (inString == "\r") {
			float value = sensorString.toFloat();
			sensor.value = value;
		}
	}
	return value;
}


double evaluateSensor(struct orpSensor sensor) {
	String sensorString;
	double value = 0;
	// If a Stringacter has been received
	if (oxygenReductionSerial.available() > 0){
		// Gets the received String
		String inString = (String)oxygenReductionSerial.read();
		// Composes the String
		sensorString += inString;
		// Reading ends with a CR
		if (inString == "\r") {
			value = sensorString.toFloat();
			sensor.value = value;
		}		
	}
	return value;
}


double evaluateSensor(struct phSensor sensor) {
	// E = E0 + RT*ln(alphaH+)/F = E0 - 2.303*R*T*pH/F
	// R = Ideal Gas Constant
	// T = Temperature in Kelvin
	// F = Faraday constant

	// Hence we must calibrate E0 with the neutral solution
	const float E0 = 1; //dummy value for now, overwrite with calibration value
						//_value = (E0-E)*F/(2.303*R*T)

	//_value = 2;
}


/* FUZZY LOGIC */

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
	float maximum = 0., leftMotor[27];
	// IF( (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (ANGLE IS ZERO) AND (GOAL IS NEAR) )
	// IF( (FRONT IS VERY FAR) AND (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (FRONT IS FAR) AND (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (FRONT IS VERY FAR) AND (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (FRONT IS FAR) AND (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (FRONT IS NEAR) AND (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (FRONT IS NEAR) AND (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS FAR) AND (ANGLE IS VERY POSITIVE) )
	// IF( (LEFT  IS NEAR) AND (ANGLE IS VERY POSITIVE) )
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS VERY NEAR) )
	// IF( (LEFT  IS NEAR) AND (FRONT IS VERY NEAR) )
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS NEAR) )
	// IF( (LEFT  IS NEAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS VERY FAR) AND (ANGLE IS POSITIVE) )
	// IF( (RIGHT IS FAR) AND (ANGLE IS POSITIVE) )
	// IF( (LEFT  IS VERY FAR) AND (ANGLE IS POSITIVE) )
	// IF( (LEFT  IS FAR) AND (ANGLE IS POSITIVE) )
	// IF( (LEFT  IS VERY FAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS FAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS NEAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS FAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS VERY FAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS VERY FAR) AND (ANGLE IS VERY POSITIVE) )
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	//leftMotor[0] = min(_zeroAngleMembership, _veryFarGoalMembership);
	//leftMotor[1] = min(_zeroAngleMembership, _farGoalMembership);
	//leftMotor[2] = min(_zeroAngleMembership, _nearGoalMembership);
	leftMotor[3] = min(_veryFarFrontSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership));
	leftMotor[4] = min(_farFrontSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership));
	leftMotor[5] = min(_veryFarFrontSensorMembership, min(_zeroAngleMembership, _farGoalMembership));
	leftMotor[6] = min(_farFrontSensorMembership, min(_zeroAngleMembership, _farGoalMembership));
	leftMotor[7] = 0;
	//leftMotor[7] = min(_nearFrontSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership));
	leftMotor[8] = min(_nearFrontSensorMembership, min(_zeroAngleMembership, _farGoalMembership));
	leftMotor[9] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[10] = min(_farLeftSensorMembership, _veryPositiveAngleMembership);
	leftMotor[11] = min(_nearLeftSensorMembership, _veryPositiveAngleMembership);
	leftMotor[12] = min(_veryNearLeftSensorMembership, _veryNearFrontSensorMembership);
	leftMotor[13] = min(_nearLeftSensorMembership, _veryNearFrontSensorMembership);
	leftMotor[14] = min(_veryNearLeftSensorMembership, _nearFrontSensorMembership);
	leftMotor[15] = 0;
	//leftMotor[15] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[16] = min(_veryFarRightSensorMembership, _positiveAngleMembership);
	leftMotor[17] = min(_farRightSensorMembership, _positiveAngleMembership);
	leftMotor[18] = min(_veryFarLeftSensorMembership, _positiveAngleMembership);
	leftMotor[19] = min(_farLeftSensorMembership, _positiveAngleMembership);
	leftMotor[20] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[21] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[22] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[23] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[24] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[25] = min(_veryFarLeftSensorMembership, _veryPositiveAngleMembership);
	leftMotor[26] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	leftMotor[27] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, _veryNegativeAngleMembership)));
	for (unsigned int i = 3; i < 28; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorPositive() {
	float maximum = 0., leftMotor[8];
	// IF( (FRONT IS NEAR) AND (RIGHT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS VERY FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (RIGHT IS FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (LEFT  IS VERY FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (LEFT  IS FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS VERY FAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS FAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS NEAR) AND (FRONT IS NEAR) )
	leftMotor[0] = min(_nearFrontSensorMembership, min(_nearRightSensorMembership, _zeroAngleMembership));
	leftMotor[1] = min(_veryFarRightSensorMembership, _negativeAngleMembership);
	leftMotor[2] = min(_farRightSensorMembership, _negativeAngleMembership);
	leftMotor[3] = min(_veryFarLeftSensorMembership, _negativeAngleMembership);
	leftMotor[4] = min(_farLeftSensorMembership, _negativeAngleMembership);
	leftMotor[5] = min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, _zeroAngleMembership));
	leftMotor[6] = min(_nearFrontSensorMembership, min(_farRightSensorMembership, _zeroAngleMembership));
	leftMotor[7] = min(_nearLeftSensorMembership, _nearFrontSensorMembership);
	for (unsigned int i = 0; i < 8; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorZero() {
	// IF( (GOAL IS VERY NEAR) )
	return _veryNearGoalMembership;
}

float LeftMotorNegative() {
	float maximum = 0., leftMotor[6];
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS FAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS VERY FAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS VERY FAR) AND (ANGLE IS VERY NEGATIVE) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS NEAR) )
	leftMotor[0] = min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, _zeroAngleMembership));
	leftMotor[1] = min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, _zeroAngleMembership));
	leftMotor[2] = min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, _zeroAngleMembership));
	leftMotor[3] = min(_veryFarRightSensorMembership, _veryNegativeAngleMembership);
	leftMotor[4] = min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, _zeroAngleMembership));
	leftMotor[5] = min(_nearFrontSensorMembership, _nearRightSensorMembership);
	for (unsigned int i = 0; i < 6; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorVeryNegative() {
	float maximum = 0., leftMotor[6];
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS FAR) AND (ANGLE IS VERY NEGATIVE) )
	// IF( (RIGHT IS NEAR) AND (ANGLE IS VERY NEGATIVE) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS VERY NEAR) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS VERY NEAR) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS NEAR) )
	leftMotor[0] = min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, _zeroAngleMembership));
	leftMotor[1] = min(_farRightSensorMembership, _veryNegativeAngleMembership);
	leftMotor[2] = min(_nearRightSensorMembership, _veryNegativeAngleMembership);
	leftMotor[3] = min(_veryNearFrontSensorMembership, _veryNearRightSensorMembership);
	leftMotor[4] = min(_nearFrontSensorMembership, _veryNearRightSensorMembership);
	leftMotor[5] = min(_veryNearFrontSensorMembership, _nearRightSensorMembership);
	leftMotor[6] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, _veryNegativeAngleMembership)));
	for (unsigned int i = 0; i < 7; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float RightMotorVeryPositive() {
	float maximum = 0., rightMotor[27];
	// IF( (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (ANGLE IS ZERO) AND (GOAL IS NEAR) )
	// IF( (FRONT IS VERY FAR) AND (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (FRONT IS FAR) AND (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (FRONT IS VERY FAR) AND (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (FRONT IS FAR) AND (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (FRONT IS NEAR) AND (ANGLE IS ZERO) AND (GOAL IS VERY FAR) )
	// IF( (FRONT IS NEAR) AND (ANGLE IS ZERO) AND (GOAL IS FAR) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS VERY FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (RIGHT IS FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (LEFT  IS VERY FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (LEFT  IS FAR) AND (ANGLE IS NEGATIVE) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS VERY FAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS FAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS FAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS VERY FAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS VERY FAR) AND (ANGLE IS VERY NEGATIVE) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS FAR) AND (ANGLE IS VERY NEGATIVE) )
	// IF( (RIGHT IS NEAR) AND (ANGLE IS VERY NEGATIVE) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS VERY NEAR) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS VERY NEAR) )
	// IF( (FRONT IS VERY NEAR) AND (RIGHT IS NEAR) )
	//rightMotor[0] = min(_zeroAngleMembership, _veryFarGoalMembership);
	//rightMotor[1] = min(_zeroAngleMembership, _farGoalMembership);
	//rightMotor[2] = min(_zeroAngleMembership, _nearGoalMembership);
	rightMotor[3] = min(_veryFarFrontSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership));
	rightMotor[4] = min(_farFrontSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership));
	rightMotor[5] = min(_veryFarFrontSensorMembership, min(_zeroAngleMembership, _farGoalMembership));
	rightMotor[6] = min(_farFrontSensorMembership, min(_zeroAngleMembership, _farGoalMembership));
	rightMotor[7] = 0;
	//rightMotor[7] = min(_nearFrontSensorMembership, min(_zeroAngleMembership, _veryFarGoalMembership));
	rightMotor[8] = min(_nearFrontSensorMembership, min(_zeroAngleMembership, _farGoalMembership));
	rightMotor[9] = 0;
	//rightMotor[9] = min(_nearFrontSensorMembership, min(_nearRightSensorMembership, _zeroAngleMembership));
	rightMotor[10] = min(_veryFarRightSensorMembership, _negativeAngleMembership);
	rightMotor[11] = min(_farRightSensorMembership, _negativeAngleMembership);
	rightMotor[12] = min(_veryFarLeftSensorMembership, _negativeAngleMembership);
	rightMotor[13] = min(_farLeftSensorMembership, _negativeAngleMembership);
	rightMotor[14] = min(_nearFrontSensorMembership, min(_veryFarRightSensorMembership, _zeroAngleMembership));
	rightMotor[15] = min(_nearFrontSensorMembership, min(_farRightSensorMembership, _zeroAngleMembership));
	rightMotor[16] = min(_veryNearFrontSensorMembership, min(_nearRightSensorMembership, _zeroAngleMembership));
	rightMotor[17] = min(_veryNearFrontSensorMembership, min(_farRightSensorMembership, _zeroAngleMembership));
	rightMotor[18] = min(_veryNearFrontSensorMembership, min(_veryFarRightSensorMembership, _zeroAngleMembership));
	rightMotor[19] = min(_veryFarRightSensorMembership, _veryNegativeAngleMembership);
	rightMotor[20] = min(_nearFrontSensorMembership, min(_veryNearRightSensorMembership, _zeroAngleMembership));
	rightMotor[21] = min(_veryNearFrontSensorMembership, min(_veryNearRightSensorMembership, _zeroAngleMembership));
	rightMotor[22] = min(_farRightSensorMembership, _veryNegativeAngleMembership);
	rightMotor[23] = min(_nearRightSensorMembership, _veryNegativeAngleMembership);
	rightMotor[24] = min(_veryNearFrontSensorMembership, _veryNearRightSensorMembership);
	rightMotor[25] = min(_nearFrontSensorMembership, _veryNearRightSensorMembership);
	rightMotor[26] = min(_veryNearFrontSensorMembership, _nearRightSensorMembership);
	rightMotor[27] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, _veryNegativeAngleMembership)));
	for (unsigned int i = 3; i < 28; i++) {
		maximum = max(maximum, rightMotor[i]);
	}
	return maximum;
}

float RightMotorPositive() {
	float maximum = 0., rightMotor[8];
	// IF( (LEFT  IS NEAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (RIGHT IS VERY FAR) AND (ANGLE IS POSITIVE) )
	// IF( (RIGHT IS FAR) AND (ANGLE IS POSITIVE) )
	// IF( (LEFT  IS VERY FAR) AND (ANGLE IS POSITIVE) )
	// IF( (LEFT  IS FAR) AND (ANGLE IS POSITIVE) )
	// IF( (LEFT  IS VERY FAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS FAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (FRONT IS NEAR) AND (RIGHT IS NEAR) )
	rightMotor[0] = min(_nearLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[1] = min(_veryFarRightSensorMembership, _positiveAngleMembership);
	rightMotor[2] = min(_farRightSensorMembership, _positiveAngleMembership);
	rightMotor[3] = min(_veryFarLeftSensorMembership, _positiveAngleMembership);
	rightMotor[4] = min(_farLeftSensorMembership, _positiveAngleMembership);
	rightMotor[5] = min(_veryFarLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[6] = min(_farLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[7] = min(_nearFrontSensorMembership, _nearRightSensorMembership);
	for (unsigned int i = 0; i < 8; i++) {
		maximum = max(maximum, rightMotor[i]);
	}
	return maximum;
}

float RightMotorZero() {
	// IF( (GOAL IS VERY NEAR) )
	return _veryNearGoalMembership;
}

float RightMotorNegative() {
	float maximum = 0., rightMotor[6];
	// IF( (LEFT  IS NEAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS FAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS VERY FAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS VERY FAR) AND (ANGLE IS VERY POSITIVE) )
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS NEAR) AND (FRONT IS NEAR) )
	rightMotor[0] = min(_nearLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[1] = min(_farLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[2] = min(_veryFarLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[3] = min(_veryFarLeftSensorMembership, _veryPositiveAngleMembership);
	rightMotor[4] = min(_veryNearLeftSensorMembership, min(_nearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[5] = min(_nearLeftSensorMembership, _nearFrontSensorMembership);
	for (unsigned int i = 0; i < 6; i++) {
		maximum = max(maximum, rightMotor[i]);
	}
	return maximum;
}

float RightMotorVeryNegative() {
	float maximum = 0., rightMotor[6];
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS VERY NEAR) AND (ANGLE IS ZERO) )
	// IF( (LEFT  IS FAR) AND (ANGLE IS VERY POSITIVE) )
	// IF( (LEFT  IS NEAR) AND (ANGLE IS VERY POSITIVE) )
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS VERY NEAR) )
	// IF( (LEFT  IS NEAR) AND (FRONT IS VERY NEAR) )
	// IF( (LEFT  IS VERY NEAR) AND (FRONT IS NEAR) )
	rightMotor[0] = min(_veryNearLeftSensorMembership, min(_veryNearFrontSensorMembership, _zeroAngleMembership));
	rightMotor[1] = min(_farLeftSensorMembership, _veryPositiveAngleMembership);
	rightMotor[2] = min(_nearLeftSensorMembership, _veryPositiveAngleMembership);
	rightMotor[3] = min(_veryNearLeftSensorMembership, _veryNearFrontSensorMembership);
	rightMotor[4] = min(_nearLeftSensorMembership, _veryNearFrontSensorMembership);
	rightMotor[5] = min(_veryNearLeftSensorMembership, _nearFrontSensorMembership);
	rightMotor[6] = min(_veryFarLeftSensorMembership, min(_veryFarFrontSensorMembership, min(_veryFarRightSensorMembership, _veryPositiveAngleMembership)));
	for (unsigned int i = 0; i < 7; i++) {
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
void setInput(float leftSensor, float frontSensor, float rightSensor, float distanceToGoal, float angleToGoal) {
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
		setInput(leftSensor, frontSensor, rightSensor, distanceToGoal, angleToGoal);
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
