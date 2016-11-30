const int NUMBER_OF_READINGS = 3;

typedef struct uSensor { int dataPort; int triggerPort; double value; };
int sensor1Port		= 1;
int sensor1Trigger	= 2;
int sensor2Port		= 3;
int sensor2Trigger	= 4;
int sensor3Port		= 5;
int sensor3Trigger	= 6;
uSensor ultrasonic1;
uSensor ultrasonic2;
uSensor ultrasonic3;
uSensor ultrasonicArray[3] = { ultrasonic1 , ultrasonic2, ultrasonic3 };

// Class variables
// Left Sensor
float _veryNearLeftSensorMembership;
float _nearLeftSensorMembership;
float _farLeftSensorMembership;
float _veryFarLeftSensorMembership;

// Front Sensor
float _veryNearFrontSensorMembership;
float _nearFrontSensorMembership;
float _farFrontSensorMembership;
float _veryFarFrontSensorMembership;

// Right Sensor
float _veryNearRightSensorMembership;
float _nearRightSensorMembership;
float _farRightSensorMembership;
float _veryFarRightSensorMembership;

// Distance to Goal
float _veryNearGoalMembership;
float _nearGoalMembership;
float _farGoalMembership;
float _veryFarGoalMembership;

// Angle between Robot and Goal
float _veryNegativeAngleMembership;
float _negativeAngleMembership;
float _zeroAngleMembership;
float _positiveAngleMembership;
float _veryPositiveAngleMembership;

// Left Motor Power
float _veryNegativeLeftPowerMembership;
float _negativeLeftPowerMembership;
float _zeroLeftPowerMembership;
float _positiveLeftPowerMembership;
float _veryPositiveLeftPowerMembership;

// Right Motor Power
float _veryNegativeRightPowerMembership;
float _negativeRightPowerMembership;
float _zeroRightPowerMembership;
float _positiveRightPowerMembership;
float _veryPositiveRightPowerMembership;

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
	

	// Sensor reading
	float leftSensorValue = 0; // Substituir pela entrada do sensor
	float frontSensorValue = 0; // Substituir pela entrada do sensor
	float rightSensorValue = 0; // Substituir pela entrada do sensor
	float distanceToGoal = 0; // Substituir pela entrada do GPS
	float angleToGoal = 0; // Substituir pela entrada do GPS


	// Fuzzification
	// Left Sensor
	float _veryNearLeftSensorMembership = VeryNearSensorSet(leftSensorValue);
	float _nearLeftSensorMembership = NearSensorSet(leftSensorValue);
	float _farLeftSensorMembership = FarSensorSet(leftSensorValue);
	float _veryFarLeftSensorMembership = VeryFarSensorSet(leftSensorValue);

	// Front Sensor
	float _veryNearFrontSensorMembership = VeryNearSensorSet(frontSensorValue);
	float _nearFrontSensorMembership = NearSensorSet(frontSensorValue);
	float _farFrontSensorMembership = FarSensorSet(frontSensorValue);
	float _veryFarFrontSensorMembership = VeryFarSensorSet(frontSensorValue);

	// Right Sensor
	float _veryNearRightSensorMembership = VeryNearSensorSet(rightSensorValue);
	float _nearRightSensorMembership = NearSensorSet(rightSensorValue);
	float _farRightSensorMembership = FarSensorSet(rightSensorValue);
	float _veryFarRightSensorMembership = VeryFarSensorSet(rightSensorValue);

	// Distance to Goal
	float _veryNearGoalMembership = VeryNearGoalSet(distanceToGoal);
	float _nearGoalMembership = NearGoalSet(distanceToGoal);
	float _farGoalMembership = FarGoalSet(distanceToGoal);
	float _veryFarGoalMembership = VeryFarGoalSet(distanceToGoal);

	// Angle between Robot and Goal
	float _veryNegativeAngleMembership = VeryNegativeAngleSet(angleToGoal);
	float _negativeAngleMembership = NegativeAngleSet(angleToGoal);
	float _zeroAngleMembership = ZeroAngleSet(angleToGoal);
	float _positiveAngleMembership = PositiveAngleSet(angleToGoal);
	float _veryPositiveAngleMembership = VeryPositiveAngleSet(angleToGoal);

	// Rule application
	// Left Motor Power
	float _veryNegativeLeftPowerMembership = LeftMotorVeryNegative();
	float _negativeLeftPowerMembership = LeftMotorNegative();
	float _zeroLeftPowerMembership = LeftMotorZero();
	float _positiveLeftPowerMembership = LeftMotorPositive();
	float _veryPositiveLeftPowerMembership = LeftMotorVeryPositive();

	// Right Motor Power
	float _veryNegativeRightPowerMembership = RightMotorVeryNegative();
	float _negativeRightPowerMembership = RightMotorNegative();
	float _zeroRightPowerMembership = RightMotorZero();
	float _positiveRightPowerMembership = RightMotorPositive();
	float _veryPositiveRightPowerMembership = RightMotorVeryPositive();


	// Defuzzification
	float leftPowerOutput = LeftMotorDefuzzification();
	float rightPowerOutput = RightMotorDefuzzification();
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

// Angle to objective
// Support and core values 
const float VERY_NEGATIVE_ANGLE_SUPPORT_1 = -3.14;
const float VERY_NEGATIVE_ANGLE_CORE = -2.4;
const float VERY_NEGATIVE_ANGLE_SUPPORT_2 = -1.5;

const float NEGATIVE_ANGLE_SUPPORT_1 = -2.4;
const float NEGATIVE_ANGLE_CORE = -1.5;
const float NEGATIVE_ANGLE_SUPPORT_2 = -0.4;

const float ZERO_ANGLE_SUPPORT_1 = -0.9;
const float ZERO_ANGLE_CORE = 0;
const float ZERO_ANGLE_SUPPORT_2 = 0.9;

const float POSITIVE_ANGLE_SUPPORT_1 = 0.4;
const float POSITIVE_ANGLE_CORE = 1.5;
const float POSITIVE_ANGLE_SUPPORT_2 = 2.4;

const float VERY_POSITIVE_ANGLE_SUPPORT_1 = 1.5;
const float VERY_POSITIVE_ANGLE_CORE = 2.4;
const float VERY_POSITIVE_ANGLE_SUPPORT_2 = 3.14;

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
const float VERY_NEGATIVE_POWER_CORE = -0.8;
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
const float VERY_POSITIVE_POWER_CORE = 0.8;
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

// FUZZY RULES
float LeftMotorVeryNegative() {
	float maximum = 0., leftMotor[4];
	// (Left very near) and (Front very near)
	// (Left Near)		and (Front very near)
	// (Left very near) and (Front near)
	// (Left very far)	and (Angle very positive)
	leftMotor[0] = min(_veryNearLeftSensorMembership, _veryNearFrontSensorMembership);
	leftMotor[1] = min(_nearLeftSensorMembership, _veryNearFrontSensorMembership);
	leftMotor[2] = min(_veryNearLeftSensorMembership, _nearLeftSensorMembership);
	leftMotor[3] = min(_veryFarLeftSensorMembership, _veryPositiveAngleMembership);
	for (unsigned int i = 0; i < 4; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorNegative() {
	float maximum = 0., leftMotor[4];
	// (Left near)	   and (Front near)
	// (Left very far) and (Front very near) and (Angle zero)
	// (Left far)	   and (Front very near) and (Angle zero)
	// (Left far)	   and (Angle very positive)
	leftMotor[0] = min(_nearLeftSensorMembership, _nearFrontSensorMembership);
	leftMotor[1] = min(_veryFarLeftSensorMembership, _veryNearFrontSensorMembership, _zeroAngleMembership);
	leftMotor[2] = min(_farLeftSensorMembership, _veryNearFrontSensorMembership, _zeroAngleMembership);
	leftMotor[3] = min(_farLeftSensorMembership, _veryPositiveAngleMembership);
	for (unsigned int i = 0; i < 4; i++) {
		maximum = max(maximum, leftMotor[i]);
	}	
	return maximum;
}

float LeftMotorZero() {
	// (Very near Goal)
	return _veryNearGoalMembership;
}

float LeftMotorPositive() {
	float maximum = 0., leftMotor[5];
	// (Front near) and (zero angle) and (Left very far)
	// (Front near) and (zero angle) and (Left far)
	// (Angle positive) and (Left very far)
	// (Angle positive) and (Left far)
	// (Front near) and (Left near)
	leftMotor[0] = min(_nearFrontSensorMembership, _zeroAngleMembership, _veryFarLeftSensorMembership);
	leftMotor[1] = min(_nearFrontSensorMembership, _zeroAngleMembership, _farLeftSensorMembership);
	leftMotor[2] = min(_veryFarLeftSensorMembership, _positiveAngleMembership);
	leftMotor[3] = min(_farLeftSensorMembership, _positiveAngleMembership);
	leftMotor[4] = min(_nearFrontSensorMembership, _nearLeftSensorMembership);
	for (unsigned int i = 0; i < 5; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float LeftMotorVeryPositive() {
	float maximum = 0., leftMotor[15];
	// (Front very far) and (Angle zero) and (Goal very far)
	// (Front far)		and (Angle zero) and (Goal very far)
	// (Front very far) and (Angle zero) and (goal far)
	// (Front far)		and (Angle zero) and (goal far)
	// (Front near)		and (Angle zero) and (Left near)
	// (Front near)		and (Angle zero) and (Left very near)
	// (Front very near)and (Angle zero) and (Left near)
	// (Front very near)and (Angle zero) and (Left very near)
	// (Angle negative) and (Right very far)
	// (Angle negative) and (Right far)
	// (Angle very negative) and (Right very far)
	// (Angle very negative) and (Right far)
	// (Front very near) and (Left very near)
	// (Front very near) and (Left near)
	// (Front near)		 and (Left very near)
	leftMotor[0] = min(_veryFarFrontSensorMembership, _zeroAngleMembership, _veryFarGoalMembership);
	leftMotor[1] = min(_farFrontSensorMembership, _zeroAngleMembership, _veryFarGoalMembership);
	leftMotor[2] = min(_veryFarFrontSensorMembership, _zeroAngleMembership, _farGoalMembership);
	leftMotor[3] = min(_farFrontSensorMembership, _zeroAngleMembership, _farGoalMembership);
	leftMotor[4] = min(_nearFrontSensorMembership, _zeroAngleMembership, _nearLeftSensorMembership);
	leftMotor[5] = min(_nearFrontSensorMembership, _zeroAngleMembership, _veryNearLeftSensorMembership);
	leftMotor[6] = min(_veryNearFrontSensorMembership, _zeroAngleMembership, _nearLeftSensorMembership);
	leftMotor[7] = min(_veryNearFrontSensorMembership, _zeroAngleMembership, _veryNearLeftSensorMembership);
	leftMotor[8] = min(_negativeAngleMembership, _veryFarRightSensorMembership);
	leftMotor[9] = min(_negativeAngleMembership, _farRightSensorMembership);
	leftMotor[10] = min(_veryNegativeAngleMembership, _veryFarRightSensorMembership);
	leftMotor[11] = min(_veryNegativeAngleMembership, _farRightSensorMembership);
	leftMotor[12] = min(_veryNearFrontSensorMembership,_veryNearLeftSensorMembership);
	leftMotor[13] = min(_veryNearFrontSensorMembership,_nearLeftSensorMembership);
	leftMotor[14] = min(_nearFrontSensorMembership, _veryNearLeftSensorMembership);
	for (unsigned int i = 0; i < 15; i++) {
		maximum = max(maximum, leftMotor[i]);
	}
	return maximum;
}

float RightMotorVeryNegative() {
	float maximum = 0., rightMotor[5];
	// (Right very near) and (Front very near)
	// (Right Near)		 and (Front very near)
	// (Right very near) and (Front near)
	// (Left very near)  and (Front very near) and (Angle zero)
	// (Right very far)  and (Angle very negative)
	rightMotor[0] = min(_veryNearRightSensorMembership, _veryNearRightSensorMembership);
	rightMotor[1] = min(_nearRightSensorMembership, _veryNearRightSensorMembership);
	rightMotor[2] = min(_veryNearRightSensorMembership, _nearRightSensorMembership);
	rightMotor[3] = min(_veryNearLeftSensorMembership, +_veryNearFrontSensorMembership, _zeroAngleMembership);
	rightMotor[4] = min(_veryFarRightSensorMembership, _veryNegativeAngleMembership);
	for (unsigned int i = 0; i < 5; i++) {
		maximum = max(maximum, rightMotor[i]);
	}
	return maximum;
}

float RightMotorNegative() {
	float maximum = 0., rightMotor[4];
	// (Left near) and (Front near)
	// (Left near) and (Front very near) and (Angle zero)
	// (Left very near) and (Front near) and (Angle zero)
	// (Right far) and (Angle very negative)
	rightMotor[0] = min(_nearLeftSensorMembership, _nearFrontSensorMembership);
	rightMotor[1] = min(_nearLeftSensorMembership, _veryNearFrontSensorMembership, _zeroAngleMembership);
	rightMotor[2] = min(_veryNearLeftSensorMembership, _nearFrontSensorMembership, _zeroAngleMembership);
	rightMotor[3] = min(_farRightSensorMembership, _veryNegativeAngleMembership);
	for (unsigned int i = 0; i < 4; i++) {
		maximum = max(maximum, rightMotor[i]);
	}
	return maximum;
}

float RightMotorZero() {
	// (Very near Goal)
	return _veryNearGoalMembership;
}

float RightMotorPositive() {
	float maximum = 0., rightMotor[4];
	// (Front near) and (Angle zero) and (Left near)
	// (Right very far) and (Angle negative)
	// (Right far) and (Angle negative)
	// (Right near) and (Front near)
	rightMotor[0] = min(_nearFrontSensorMembership, _zeroAngleMembership, _nearLeftSensorMembership);
	rightMotor[1] = min(_veryFarRightSensorMembership, _negativeAngleMembership);
	rightMotor[2] = min(_farRightSensorMembership, _negativeAngleMembership);
	rightMotor[3] = min(_nearRightSensorMembership, _nearFrontSensorMembership);
	for (unsigned int i = 0; i < 4; i++) {
		maximum = max(maximum, rightMotor[i]);
	}
	return maximum;
}

float RightMotorVeryPositive() {
	float maximum = 0., rightMotor[15];
	// (Front very far) and (Angle zero) and (Goal very far)
	// (Front far)		and (Angle zero) and (Goal very far)
	// (Front very far) and (Angle zero) and (Goal far)
	// (Front far)		and (Angle zero) and (Goal far)
	// (Front near)		and (Angle zero) and (Goal very far)
	// (Front near)		and (Angle zero) and (Goal far)
	// (Front very near) and (Angle zero) and (Left very far)
	// (Front very near) and (Angle zero) and (Left far)
	// (Left very far) and (Angle positive)
	// (Left very far) and (Angle very positive)
	// (Left far)	   and (Angle positive)
	// (Left far)	   and (Angle very positive)
	// (Front very near) and (Right very near)
	// (Front very near) and (Right near)
	// (Front near)		 and (Right very near)
	rightMotor[0] = min(_veryFarFrontSensorMembership, _zeroAngleMembership, _veryFarGoalMembership);
	rightMotor[1] = min(_farFrontSensorMembership, _zeroAngleMembership, _veryFarGoalMembership);
	rightMotor[2] = min(_veryFarFrontSensorMembership, _zeroAngleMembership, _farGoalMembership);
	rightMotor[3] = min(_farFrontSensorMembership, _zeroAngleMembership, _farGoalMembership);
	rightMotor[4] = min(_nearFrontSensorMembership, _zeroAngleMembership, _veryFarGoalMembership);
	rightMotor[5] = min(_nearFrontSensorMembership, _zeroAngleMembership, _farGoalMembership);
	rightMotor[6] = min(_veryNearFrontSensorMembership, _zeroAngleMembership, _veryFarLeftSensorMembership);
	rightMotor[7] = min(_veryNearFrontSensorMembership, _zeroAngleMembership, _farLeftSensorMembership);
	rightMotor[8] = min(_veryFarLeftSensorMembership, _positiveAngleMembership);
	rightMotor[9] = min(_veryFarLeftSensorMembership, _veryPositiveAngleMembership);
	rightMotor[10] = min(_farLeftSensorMembership, _positiveAngleMembership);
	rightMotor[11] = min(_farLeftSensorMembership, _veryPositiveAngleMembership);
	rightMotor[12] = min(_veryNearFrontSensorMembership, _veryNearRightSensorMembership);
	rightMotor[13] = min(_veryNearFrontSensorMembership, _nearRightSensorMembership);
	rightMotor[14] = min(_nearFrontSensorMembership, _veryNearRightSensorMembership);
	for (unsigned int i = 0; i < 15; i++) {
		maximum = max(maximum, rightMotor[i]);
	}
	return maximum;
}

// Defuzzification 
float LeftMotorDefuzzification() {
	float leftMotorPower[5] = { 0,0,0,0,0 }, power = 0;
	if (_veryNegativeLeftPowerMembership > 0) {
		leftMotorPower[0] = VeryNegativePowerSet(_veryNegativeLeftPowerMembership);
	}
	if (_negativeLeftPowerMembership > 0) {
		leftMotorPower[1] = NegativePowerSet(_negativeLeftPowerMembership);
	}
	if (_zeroLeftPowerMembership) {
		leftMotorPower[2] = ZeroPowerSet(_zeroLeftPowerMembership);
	}
	if (_positiveLeftPowerMembership) {
		leftMotorPower[3] = PositivePowerSet(_positiveLeftPowerMembership);
	}
	if (_veryPositiveLeftPowerMembership) {
		leftMotorPower[4] = VeryPositivePowerSet(_veryPositiveLeftPowerMembership);
	}
	power = (leftMotorPower[0] * _veryNegativeLeftPowerMembership +
		leftMotorPower[1] * _negativeLeftPowerMembership +
		leftMotorPower[2] * _zeroLeftPowerMembership +
		leftMotorPower[3] * _positiveLeftPowerMembership +
		leftMotorPower[4] * _veryPositiveLeftPowerMembership) /
		(_veryNegativeLeftPowerMembership + _negativeLeftPowerMembership +
			_zeroLeftPowerMembership + _positiveLeftPowerMembership +
			_veryPositiveLeftPowerMembership);
	return power;
}

float RightMotorDefuzzification() {
	float RightMotorPower[5] = { 0,0,0,0,0 }, power = 0;
	if (_veryNegativeRightPowerMembership > 0) {
		RightMotorPower[0] = VeryNegativePowerSet(_veryNegativeRightPowerMembership);
	}
	if (_negativeRightPowerMembership > 0) {
		RightMotorPower[1] = NegativePowerSet(_negativeRightPowerMembership);
	}
	if (_zeroRightPowerMembership) {
		RightMotorPower[2] = ZeroPowerSet(_zeroRightPowerMembership);
	}
	if (_positiveRightPowerMembership) {
		RightMotorPower[3] = PositivePowerSet(_positiveRightPowerMembership);
	}
	if (_veryPositiveRightPowerMembership) {
		RightMotorPower[4] = VeryPositivePowerSet(_veryPositiveRightPowerMembership);
	}
	power = (RightMotorPower[0] * _veryNegativeRightPowerMembership +
		RightMotorPower[1] * _negativeRightPowerMembership +
		RightMotorPower[2] * _zeroRightPowerMembership +
		RightMotorPower[3] * _positiveRightPowerMembership +
		RightMotorPower[4] * _veryPositiveRightPowerMembership) /
		(_veryNegativeRightPowerMembership + _negativeRightPowerMembership +
			_zeroRightPowerMembership + _positiveRightPowerMembership +
			_veryPositiveRightPowerMembership);
	return power;
}