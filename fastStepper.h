#ifndef fastStepper_h
#define fastStepper_h

/**
 * @file fastStepper.h
 * @brief Header-Datei für die timer basierte Steuerung von Schrittmotoren für schnelle Step-Pulse
 *
 * Diese Bibliothek stellt Funktionen zur Verfügung, die die Ansteuerung von Schrittmotoren auf 
 * Mikrocontroller-Plattformen wie Arduino ermöglichen. Der Fokus liegt auf einer schnellen 
 * und effizienten Steuerung, um die maximale Frequenz and Step-Pulsen zu erreichen.
 * Dafür wird der Timer1 des Arduino genutzt, weswegen auch nur ein Motor mit dieser Bibliothek 
 * angesteuert werden kann.
 *
 * Funktionen:
 * - Steuerung der Schrittmotorposition und -geschwindigkeit
 * - Kompatibilität mit gängigen STEP/DIR Interfaces
 *
 * @author Tilman Fischer
 * @version 1.0
 * @date 14.01.2024
 * 
 */

#include "Arduino.h"
#include <Controllino.h>

// Pins:
#define stepPin CONTROLLINO_D0 //mcu > STEP pin
#define dirPin CONTROLLINO_D1  //mcu > DIR pin
#define endstopPin CONTROLLINO_IN0

// Parameter:
#define endstopNormallyOpen true
#define invert false                  // change to true or false if motor turns the wrong way
#define spmm 6400                     // 6400 (microsteps pro umdrehung)* 5 (getriebe)/ 5 (spindel) = 6400
// #define spmm 12800                   // value for finer microstepping
#define maxSpeed 5                    // maximum allowed Speed (in mm/s), if set too high the pulseInterval gets too short
#define maxDistance 150               // maximum distance (in mm) that is reasonable to move on the machine (e.g. maximum travel)
#define maxSteps maxDistance*spmm

// variables to store the current motor values
extern volatile long currentPosition;    // Speichert die aktuelle Position des Motors
extern volatile long targetPosition;     // Speichert die Zielposition des Motors
extern volatile bool direction;          // saves the currently set motor direction
extern volatile float speed;        // in mm/

// timer1 interrupt intervall (in microseconds)
extern unsigned int pulseInterval;  // in microseconds; 1000 microseconds = 1 ms -> 1kHz pulse rate
extern bool timerPaused;            // indicates wether the timer is paused

extern bool safetyBoundsOff;         // defines wether the motor can turn below 0

// functions (for extern use):
void setupStepper();
void loosePosition();
bool isMotorRunning();
void setSpeed(float _speed);
float getSpeed();
void moveToPosition(float pos_mm);
void step_relative(float move_distance_mm);
void stopMotor();
void setPosition(long new_position_mm);
void setHomePosition(long new_position_mm);
float getTargetPosition();
float getPosition();
bool endstopHit();

// functions (primarily for internal use):
void pauseTimer();              // used by stopMotor()
void startTimer();              // used by various move commands to start the motor
void updateTimerFrequency();    // used by setSpeed()
void setDirection(bool _dir);   // used by various move commands

#endif