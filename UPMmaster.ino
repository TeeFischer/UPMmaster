// this declaration of the function is necessary because it contains optional arguments
void moveTillForce(String directionInput, float offsetForce = 100, float backUpDistance = 0);
void blockingMove(float _pos);

#include <SPI.h>
#include <Controllino.h>
#include "waage.h"
#include "fastStepper.h"

#define debugRunCommands false
#define debugComms false
#define eStopPin CONTROLLINO_I18
bool eStop = false;
bool isHoming = false;

#define touchdownForce 100   // (in gramms) force that has to be exceeded to count a touchdown
#define pressForce   50000   // (in gramms) target force for the press test
#define pressDeformation 10  // (in mm) target deformation for the press test
#define testSpeed         1  // (in mm/s) slow speed for accurate tests

unsigned long time = 0;
bool setupComplete = false;

bool autoWaage = true;  //defines wether the scale is read in the loop or just on demand

// Interrupt Service Routine (ISR)
void endstopISR() {
  // this statements checks wether the machine is homing
  // during homing it is normal to hit the endstop, otherwise cut the motor off
  if(!isHoming){
    stopMotor();
    Serial.println("Error: Endstop Hit, Motor stopped");
  }
  else{
    Serial.println("Info: Endstop Hit");
  }
}

void setup() {
  pinMode(endstopPin, INPUT);

  // Interrupt für den Button-Pin einrichten
  // attachInterrupt(pin, ISR, mode)
  // mode: FALLING = wenn der Schalter von HIGH auf LOW geht
  attachInterrupt(digitalPinToInterrupt(endstopPin), endstopISR, RISING);

  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");

  //calibrate the scale on start-up
  for(uint8_t i=1; i<4; i++){
    Serial.print("Debug: Scale Startup Try ");
    Serial.println(i);

    if( waage_setup()){
      setupComplete = true;
      break;
    }
  }

  //calibrate();

  setupStepper();
  // set motor settings (currently very high)
  setSpeed(maxSpeed);       

  if(!setupComplete){
    Serial.println("Error: Scale not initiated!");
  }
  Serial.println("Load_cell: grams Position: mm");
}

void loop() {

  //read the eStop Input Pin and write the result to eStop variable
  if(digitalRead(eStopPin) == LOW){
    eStop = true;
  } else{
    eStop = false;
  }

  // if eStop is pressed, stop the motor
  if(eStop){
    stopMotor();
    Serial.println("Error: E-STOP pressed! Motor halted");
  }
  // if the estop is not pressed continue the current routine
  else{
#if debugRunCommands
    Serial.print(getTargetPosition());
    Serial.println("Run in main loop()");
#endif
  
    // get (smoothed) value from the scale (if available)
    if (waage_hasUpdate() && autoWaage) {
      float measurement = LoadCell.getData();
      getScaleAndPosition();
    }
  }

  // receive command from serial terminal (if available)
  if (Serial.available() > 0) {
    char inByte = Serial.read();

#if debugComms
    Serial.print("Pong: ");
    Serial.println(inByte);
#endif

    if      (inByte == 't') tareScale(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
    else if (inByte == 'a') setAutoWaage(); // change the state of the automatic read out of the scale
    else if (inByte == 'h') home(); //homing
    else if (inByte == 'm') moveRelativeMenu(); //move a relative distance
    else if (inByte == 'f') moveAbsoluteMenu(); //move fast to a specified position
    else if (inByte == 'p') pressTestPosition(); //carry out a compression test
    else if (inByte == 'k') pressTestForce(); //carry out a compression test
    else if (inByte == 'z') setMotorSpeed(); //read the set speed from the stepper
  }
}  // end of (main) loop()

float getScaleAndPosition(){
  float measurement = LoadCell.getData();

      // print out force and position values
      Serial.print("Load_cell:");
      Serial.print(measurement);
      Serial.print(" Position:");
      // print(float,4) returns 4 decimal characters 
      Serial.println(getPosition(),4);
}

float getScaleMeasurement(){
  float measurement = LoadCell.getData();
  Serial.print("Load_cell:");
  Serial.println(measurement);
  return measurement;
}

void tareScale(){
  LoadCell.tare();
  Serial.println("Tare complete");
  
}

void setMotorSpeed(){
  float input = 0;
  Serial.println("Enter speed (in steps/second):");
  
  while(true){
    if (Serial.available() > 0) {
      input = Serial.parseFloat();
    }
    if(input != 0){
      break;
    }
  }

  if(input == 0){
    Serial.println("Error: No movement was entered!");
  }
  // Gib die Eingabe aus
  Serial.print("You entered: ");
  Serial.println(input);

  setSpeed(float(input));
}

// This function carrys out a compression test until a specified force
void pressTestForce(){
  float input = 0;
  Serial.print("Compression Test (Force: ");
  Serial.print(pressForce);
  Serial.println(" gramms)\nHow many Press Cycles:");
  
  while(true){
    if (Serial.available() > 0) {
      input = Serial.parseFloat();
    }
    if(input != 0){
      break;
    }
  }

  if(input == 0){
    Serial.println("Error: No cycles were entered!");
  }
  else{
    input = floor(input);
    // Gib die Eingabe aus
    Serial.print("You entered: ");
    Serial.println(input);
  }

  Serial.println("Starting Compression Test...");
  tareScale();

  Serial.println("Scale tared. Moving till touchdown...");
  moveTillForce("down", touchdownForce);
  float specimenHeight = getPosition();

  // back up from specimen
  Serial.print("Debug: BackupPoint: ");
  Serial.print(specimenHeight - 1);
  Serial.print(", CurrentPoint: ");
  Serial.println(getPosition());
  blockingMove(specimenHeight - 1);

  // do the number of requested press cycles
  for (int n = 0; n < input; n++){
    Serial.print("Press cycle: ");
    Serial.println(n);

    // TODO change the 1000 value to the desired press force
    moveTillForce("down", pressForce, 0);
    Serial.print("Debug: BackupPoint: ");
    Serial.println(specimenHeight - 1);
    blockingMove(specimenHeight - 1);
  }
}  // end of pressTestForce()

// This function carrys out a compression test until a specified deformation
void pressTestPosition(){
  float input = 0;
  Serial.print("Compression Test (Deformation: ");
  Serial.print(pressDeformation);
  Serial.println(" mm)\nHow many Press Cycles:");
  
  while(true){
    if (Serial.available() > 0) {
      input = Serial.parseFloat();
    }
    if(input != 0){
      break;
    }
  }

  if(input == 0){
    Serial.println("Error: No cycles were entered!");
  }
  else{
    input = floor(input);
    // Gib die Eingabe aus
    Serial.print("You entered: ");
    Serial.println(input);
  }

  Serial.println("Starting Compression Test...");
  tareScale();

  Serial.println("Scale tared. Moving till touchdown...");
  moveTillForce("down", touchdownForce);
  float specimenHeight = getPosition();
  blockingMove(specimenHeight - 2);
  setSpeed(testSpeed);
  moveTillForce("down", touchdownForce);
  specimenHeight = getPosition();
  setSpeed(maxSpeed);

  // back up from specimen
  Serial.print("Debug: BackupPoint: ");
  Serial.print(specimenHeight - 1);
  Serial.print(", CurrentPoint: ");
  Serial.println(getPosition());
  blockingMove(specimenHeight - 1);

  // do the number of requested press cycles
  for (int n = 0; n < input; n++){
    Serial.print("Press cycle: ");
    Serial.println(n);

    if (n % 100 == 0){
      setSpeed(testSpeed);
      Serial.println("Slow Cycle");
    }
    
    blockingMove(specimenHeight + pressDeformation);
    Serial.print("Debug: BackupPoint: ");
    Serial.println(specimenHeight - 1);
    blockingMove(specimenHeight - 1);

    if (n % 100 == 0){
      setSpeed(maxSpeed);
    }
  }
}  // end of pressTestForce()

// change the state of the automatic read out of the scale
void setAutoWaage(){
  autoWaage = !autoWaage;
  if(autoWaage){
    Serial.println("Info: Scale printout active!");
  }
  else{
    Serial.println("Info: Scale printout deactivated!");
  }
}

void moveRelativeMenu(){
  float input = 0;
  Serial.println("Enter relative distance (in mm):");
  
  while(true){
    if (Serial.available() > 0) {
      input = Serial.parseFloat();
    }
    if(input != 0){
      break;
    }
  }

  if(input == 0){
    Serial.println("Error: No movement was entered!");
  }
  else{
    // Gib die Eingabe aus
    Serial.print("You entered: ");
    Serial.println(input);

    step_relative(input);
  }
  
}

void moveTillForce(String directionInput, float offsetForce = 100, float backUpDistance = 0){
  // check the direction input
  int8_t direction = 1;
  if (directionInput == "down"){
    direction = 1;
  }else if (directionInput == "up") {
    direction = -1;
  }else{
    Serial.println("Error: Wrong Direction Input");
  }

  // this idle loop waits for the next scale value
  while (!waage_hasUpdate()) {}

  // measure the current force, to substract it later
  float startForce = LoadCell.getData();
  Serial.print("Debug: startForce= ");
  Serial.println(startForce);

  float measurement = LoadCell.getData();
  float upperLimit = startForce + offsetForce;
  float lowerLimit = startForce - offsetForce;

  // move until force increases (or decreases) (or eStop is hit)
  moveToPosition(maxDistance * direction);

  while((measurement < upperLimit && measurement > lowerLimit) && digitalRead(eStopPin) == 1){
    if (waage_hasUpdate()) {
      measurement = LoadCell.getData();
      getScaleAndPosition();
    }
#if debugRunCommands
    Serial.println("Run in moveTillForce()");
#endif
  }
  stopMotor();
  Serial.println("Debug: Force reached!");

  if (!(backUpDistance == 0)){
    float forcePoint = getPosition();

    moveToPosition(forcePoint - backUpDistance);
  }

}  //end of moveTillForce()

void moveAbsoluteMenu(){
  float input = 0;
  Serial.println("Enter absolute point (in mm):");
  
  // this loop waits for user input
  while(true){
    if (Serial.available() > 0) {
      input = Serial.parseFloat();
    }
    if(input != 0){
      break;
    }
  }

  // 0 can occur if user enters a String that cannot be parsed into a number
  // therefore "0" is not a valid input yet
  if(input == 0){
    Serial.println("Info: No movement was entered!");
  }
  else{

    // Gib die Eingabe aus
    Serial.print("You entered: ");
    Serial.println(input);

    moveToPosition(input);
  }
}  // end of moveFast()

void home(){
  isHoming = true;

  Serial.println("Homing...");

  setSpeed(maxSpeed);
  moveToPosition(-maxDistance);

  // step backwards until endstop is hit
  while(digitalRead(endstopPin) == 0 && digitalRead(eStopPin) == 1){
    if (waage_hasUpdate()) {
      getScaleAndPosition();
    }
#if debugRunCommands
    Serial.println("Run 1 in home()");
#endif
  }
  stopMotor();
  // First Hit, zero the position
  setPosition(0);

  // step out of the sensor range
  blockingMove(2);

  // step backwards until endstop is hit
  moveToPosition(-1);

  while(digitalRead(endstopPin) == 0 && digitalRead(eStopPin) == 1){
    if (waage_hasUpdate()) {
      getScaleAndPosition();
    }
#if debugRunCommands
    Serial.println("Run 3 in home()");
#endif
  }
  stopMotor();
  // Second Hit, zero the position
  setPosition(0);

  // step out of the sensor range
  blockingMove(0.2);

  // set the home position
  setHomePosition(0);
  isHoming = false;
  Serial.println("System Homed!");
}  // end of home()

void blockingMove(float _pos){
  moveToPosition(_pos);

  while(isMotorRunning() && digitalRead(eStopPin) == 1){
    if (waage_hasUpdate()) {
      getScaleAndPosition();
    }
#if debugRunCommands
    Serial.println("Run in blockingMove()");
#endif
  }
  
  stopMotor();
}

//TODO:
// druckversuch schnell ohne read out
// druckversuch langsam mit readout
