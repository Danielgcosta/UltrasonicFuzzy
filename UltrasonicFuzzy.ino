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

void setup()
{
	Serial.begin(9600);
	Serial.println("Porta aberta");
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
	
	PowerFuzzyTest();
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

/* FUZZY LOGIC SET INFORMATION */

// Ultrassonic Sensors
// Support and core values 
const float VERY_CLOSE_SENSOR_SET_CORE = 0.5;
const float VERY_CLOSE_SENSOR_SET_SUPPORT = 1;

const float CLOSE_SENSOR_SET_SUPPORT_1 = 0.5;
const float CLOSE_SENSOR_SET_CORE_1 = 1;
const float CLOSE_SENSOR_SET_CORE_2 = 2;
const float CLOSE_SENSOR_SET_SUPPORT_2 = 2.5;

const float FAR_SENSOR_SET_SUPPORT_1 = 2;
const float FAR_SENSOR_SET_CORE_1 = 2.5;
const float FAR_SENSOR_SET_CORE_2 = 3;
const float FAR_SENSOR_SET_SUPPORT_2 = 3.5;

const float VERY_FAR_SENSOR_SET_SUPPORT = 3;
const float VERY_FAR_SENSOR_SET_CORE = 3.5;

// Membership calculation
float VeryCloseSensorSet(float value) {
	float membership = 0;
	if (value <= VERY_CLOSE_SENSOR_SET_CORE) {
		membership = 1;
	}
	else if (value < VERY_CLOSE_SENSOR_SET_SUPPORT) {
		membership = (value - VERY_CLOSE_SENSOR_SET_SUPPORT)
			/ (VERY_CLOSE_SENSOR_SET_CORE - VERY_CLOSE_SENSOR_SET_SUPPORT);
	}
	return membership;
}

float CloseSensorSet(float value) {
	float membership = 0;
	if (value > CLOSE_SENSOR_SET_SUPPORT_1 && value < CLOSE_SENSOR_SET_CORE_1) {
		membership = (value - CLOSE_SENSOR_SET_SUPPORT_1)
			/ (CLOSE_SENSOR_SET_CORE_1 - CLOSE_SENSOR_SET_SUPPORT_1);
	}
	else if (value >= CLOSE_SENSOR_SET_CORE_1 && value <= CLOSE_SENSOR_SET_CORE_2) {
		membership = 1;
	}
	else if (value > CLOSE_SENSOR_SET_CORE_2 && value < CLOSE_SENSOR_SET_SUPPORT_2) {
		membership = (value - CLOSE_SENSOR_SET_SUPPORT_2)
			/ (CLOSE_SENSOR_SET_CORE_2 - CLOSE_SENSOR_SET_SUPPORT_2);
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

void SensorFuzzyTest() {
	float m1 = 0;
	float m2 = 0;
	float m3 = 0;
	float m4 = 0;
	float value = 0;
	Serial.println("Fuzzy output: ");
	for (int i = 0; i < 50; i++) {
		value = i / 10.;
		m1 = VeryCloseSensorSet(value);
		m2 = CloseSensorSet(value);
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
const float VERY_CLOSE_GOAL_CORE = 0.15;
const float VERY_CLOSE_GOAL_SUPPORT = 0.45;

const float CLOSE_GOAL_SUPPORT_1 = 0.15;
const float CLOSE_GOAL_CORE = 0.45;
const float CLOSE_GOAL_SUPPORT_2 = 0.75;

const float FAR_GOAL_SUPPORT_1 = 0.45;
const float FAR_GOAL_CORE = 0.75;
const float FAR_GOAL_SUPPORT_2 = 1;

const float VERY_FAR_GOAL_SUPPORT = .75;
const float VERY_FAR_GOAL_CORE = 1;

// Membership calculation
float VeryCloseGoalSet(float value) {
	float membership = 0;
	if (value <= VERY_CLOSE_GOAL_CORE) {
		membership = 1;
	}
	else if (value < VERY_CLOSE_GOAL_SUPPORT) {
		membership = (value - VERY_CLOSE_GOAL_SUPPORT)
			/ (VERY_CLOSE_GOAL_CORE - VERY_CLOSE_GOAL_SUPPORT);
	}
	return membership;
}

float CloseGoalSet(float value) {
	float membership = 0;
	if (value > CLOSE_GOAL_SUPPORT_1 && value < CLOSE_GOAL_CORE) {
		membership = (value - CLOSE_GOAL_SUPPORT_1)
			/ (CLOSE_GOAL_CORE - CLOSE_GOAL_SUPPORT_1);
	}
	else if (value == CLOSE_GOAL_CORE) {
		membership = 1;
	}
	else if (value > CLOSE_GOAL_CORE && value < CLOSE_GOAL_SUPPORT_2) {
		membership = (value - CLOSE_GOAL_SUPPORT_2)
			/ (CLOSE_GOAL_CORE - CLOSE_GOAL_SUPPORT_2);
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

void GoalFuzzyTest() {
	float m1 = 0;
	float m2 = 0;
	float m3 = 0;
	float m4 = 0;
	float value = 0;
	Serial.println("Fuzzy output: ");
	for (int i = 0; i < 50; i++) {
		value = i / 50.;
		m1 = VeryCloseGoalSet(value);
		m2 = CloseGoalSet(value);
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