#define ECHO_PIN_1 23
#define TRIG_PIN_1 22
#define ECHO_PIN_2 25
#define TRIG_PIN_2 24
#define ECHO_PIN_3 27
#define TRIG_PIN_3 26
#define ECHO_PIN_4 29
#define TRIG_PIN_4 28
#define ECHO_PIN_5 31
#define TRIG_PIN_5 30

unsigned long previousMillis = 0;
const long interval = 100; // Intervalo de tiempo en milisegundos

void setup() {
    Serial.begin(9600);

    pinMode(TRIG_PIN_1, OUTPUT);
    pinMode(ECHO_PIN_1, INPUT);
    pinMode(TRIG_PIN_2, OUTPUT);
    pinMode(ECHO_PIN_2, INPUT);
    pinMode(TRIG_PIN_3, OUTPUT);
    pinMode(ECHO_PIN_3, INPUT);
    pinMode(TRIG_PIN_4, OUTPUT);
    pinMode(ECHO_PIN_4, INPUT);
    pinMode(TRIG_PIN_5, OUTPUT);
    pinMode(ECHO_PIN_5, INPUT);
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        readUltrasonicSensors();
    }
}

void readUltrasonicSensors() {
    Serial.println("Distancias de los sensores ultrasónicos:");

    Serial.print("Sensor 1: ");
    Serial.print(getDistance(TRIG_PIN_1, ECHO_PIN_1));
    Serial.println(" cm");

    Serial.print("Sensor 2: ");
    Serial.print(getDistance(TRIG_PIN_2, ECHO_PIN_2));
    Serial.println(" cm");

    Serial.print("Sensor 3: ");
    Serial.print(getDistance(TRIG_PIN_3, ECHO_PIN_3));
    Serial.println(" cm");

    Serial.print("Sensor 4: ");
    Serial.print(getDistance(TRIG_PIN_4, ECHO_PIN_4));
    Serial.println(" cm");

    Serial.print("Sensor 5: ");
    Serial.print(getDistance(TRIG_PIN_5, ECHO_PIN_5));
    Serial.println(" cm");
}

long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2; // Velocidad del sonido en cm/ms

    return distance;
}