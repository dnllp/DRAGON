// Definición de los pines del Arduino conectados al VNH2SP30
const int inAPin = 10;  // INA pin
const int inBPin = 9;  // INB pin
const int pwMPin = 8;  // PWM pin

unsigned long startTime = 0;
const long runTime = 10000;  // Tiempo de ejecución en milisegundos (10 segundos)
bool motorStarted = false;

void setup() {
  pinMode(inAPin, OUTPUT);
  pinMode(inBPin, OUTPUT);
  pinMode(pwMPin, OUTPUT);
}

void loop() {
  if (!motorStarted) {
    // Iniciar el motor hacia adelante
    digitalWrite(inAPin, LOW);
    digitalWrite(inBPin, HIGH);
    analogWrite(pwMPin, 255);  // Velocidad máxima

    startTime = millis();  // Guardar el tiempo de inicio
    motorStarted = true;
  }
  else if (millis() - startTime >= runTime) {
    // Detener el motor después de 10 segundos
    digitalWrite(inAPin, LOW);
    digitalWrite(inBPin, LOW);
    analogWrite(pwMPin, 0);  // Detener el motor

    // No es necesario actualizar startTime o motorStarted ya que el motor solo se activa una vez
  }
}