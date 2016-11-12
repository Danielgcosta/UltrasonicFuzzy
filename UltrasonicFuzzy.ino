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
