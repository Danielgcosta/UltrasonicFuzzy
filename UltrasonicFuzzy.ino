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

// FUZZY LOGIC SET INFORMATION

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

// Membership calculations
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
	if (value > VERY_FAR_SENSOR_SET_SUPPORT) {
		membership = (value - VERY_FAR_SENSOR_SET_SUPPORT)
			/ (VERY_FAR_SENSOR_SET_CORE - VERY_FAR_SENSOR_SET_SUPPORT);
	}
	else if (value >= VERY_FAR_SENSOR_SET_CORE) {
		membership = 1;
	}
	return membership;
}