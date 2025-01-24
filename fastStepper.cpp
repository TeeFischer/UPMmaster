#include "fastStepper.h"  // Die Header-Datei einbinden

// Initialisierung der Variablen (aus der Header-Datei)
volatile long currentPosition = 0;      // Speichert die aktuelle Position des Motors
volatile long targetPosition = 0;       // Speichert die Zielposition des Motors
volatile bool direction = 0;
volatile float speed = 5;                 // in mm/s
unsigned int pulseInterval = 1000000 / (speed * spmm);  // Intervall für den Timer
bool timerPaused = false;               // Gibt an, ob der Timer pausiert ist
bool safetyBoundsOff = false;           // Schutz für Grenzwerte der Bewegung


void pauseTimer() {
  TCCR1B &= ~(1 << CS11);  // Stoppe den Timer, indem du den Prescaler entfernst
  timerPaused = true;      // Markiere den Timer als pausiert
}

void startTimer() {
  TCCR1B |= (1 << CS11);  // Setze den Prescaler, um den Timer wieder zu starten
  timerPaused = false;    // Markiere den Timer als nicht pausiert
}

void updateTimerFrequency() {
  noInterrupts();  // Deaktiviere Interrupts, um den Timer sicher zu ändern
  OCR1A = (pulseInterval * (F_CPU / 1000000) / 8) - 1;  // Berechne den neuen Vergleichswert
  interrupts();  // Reaktiviere Interrupts
}

void setupStepper(){
  // Timer1 konfigurieren
  noInterrupts();           // Unterbrechungen während der Timer-Konfiguration deaktivieren
  TCCR1A = 0;               // Timer1 Steuerregister A
  TCCR1B = 0;               // Timer1 Steuerregister B
  TCNT1 = 0;                // Setze den Timerzähler zurück

  // Timer1 auf einen Intervall von 'pulseInterval' Mikrosekunden konfigurieren
  OCR1A = (pulseInterval * (F_CPU / 1000000) / 8) - 1;  // Berechne die Vergleichszahl für 1 kHz Pulsrate
  TCCR1B |= (1 << WGM12);    // CTC-Modus (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11);     // Prescaler auf 8 (F_CPU / 8)
  TIMSK1 |= (1 << OCIE1A);   // Erlaube Timer1 Compare Interrupt

  pauseTimer();

  interrupts();  // Unterbrechungen wieder aktivieren
}

// Interrupt-Service-Routine für Timer1
ISR(TIMER1_COMPA_vect) {
  if(currentPosition == targetPosition){
    pauseTimer();
  }else{
    // Schicke einen Impuls an den Step-Pin
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);

    // Verfolge die Motorposition
    if (direction) {
      currentPosition++;
    }else {
      currentPosition--;
    }
  }
}

void loosePosition(){
  safetyBoundsOff = true;
}


bool isMotorRunning(){
  if(targetPosition != currentPosition){
    return true;
  }
  return false;
}

void setSpeed(float _speed){
  speed = _speed;
  pulseInterval = 1000000/(speed*spmm);
  updateTimerFrequency();
  Serial.print("Speed is set to:");
  Serial.println(speed);
  Serial.print("Debug: PulseInterval is set to:");
  Serial.println(pulseInterval);
}

// targets a position and starts the motor to go there
void moveToPosition(float pos_mm){
  bool move = true;

  if (safetyBoundsOff){
    Serial.println("Debug: Ignoring safety bounds");
  }
  else if(pos_mm < 0){
    Serial.println("Moving to 0 Position");
    pos_mm = 0;
  }else if (pos_mm > maxDistance) {
    Serial.println("Moving to MAX Position");
    pos_mm = maxDistance;
  }
  else{
    Serial.print("Debug: Moving to your Position: ");
    Serial.println(pos_mm);
  }

  targetPosition = pos_mm * spmm;
  
  if(targetPosition == currentPosition){
    move = false;
  }else if (targetPosition > currentPosition) {
    direction = 1;
  } else {
    direction = 0;
  }

  if(move){
    setDirection(direction);
    startTimer();
  }
}

void step_relative(float move_distance_mm){
  long position = currentPosition/(float)spmm;
  if (safetyBoundsOff){
    Serial.println("Debug: Ignoring safety bounds");
    position = position + move_distance_mm;
  }
  else if(position + move_distance_mm < 0){
    Serial.println("Moving to 0 Position");
    position = 0;
  }else if (position + move_distance_mm > maxDistance) {
    Serial.println("Moving to MAX Position");
    position = maxDistance;
  }
  else{
    position = position + move_distance_mm;
    Serial.print("Debug: Moving to your Position: ");
    Serial.println(position);
  }
  moveToPosition(position);
}

void setDirection(bool _dir){
#if invert
  bool setDirection = !_dir;
#else 
  bool setDirection = _dir;
#endif
  digitalWrite(dirPin, setDirection);
}

void stopMotor(){
  pauseTimer();
}

// This function sets the current position manually, and deactivates the safetyBounds
// so the user can move the axis to any position
void setPosition(long new_position_mm){
  currentPosition = new_position_mm*spmm;
  safetyBoundsOff = true;
}

void setHomePosition(long new_position_mm){
  currentPosition = new_position_mm*spmm;
  safetyBoundsOff = false;
}

// this returns the latest targeted position, not the actual position
float getTargetPosition(){
  return targetPosition/(float)spmm;
}

// this returns the actual position, reported by the stepper library
float getPosition(){
  return currentPosition/(float)spmm;
}

bool endstopHit(){
#if endstopNormallyOpen
  // for Normally Open endstops this returns TRUE if the signal is HIGH
  return digitalRead(endstopPin);
#else
  // for Normally Closed endstops this returns TRUE if the signal is LOW
  return !digitalRead(endstopPin);
#endif
}