//This is just a test code for the feeder. Look at the homing step mentioned on line 78 for doing precise homing

// Include the DRV8834 library
#include <DRV8834.h>

// Define the number of steps per revolution for the stepper motors
#define MOTOR_STEPS 200

// Define the RPM (Rotations Per Minute) for the A-axis and E-axis stepper motors
#define A_RPM 200
#define E_RPM 200

// Define the microstepping value (16 for DRV8834)
#define MICROSTEP 16

// Define the pin number for the endstop switch
#define ENDSTOP 2

// Create instances of the DRV8834 stepper motor drivers for A-axis and E-axis
DRV8834 E(MOTOR_STEPS, A1, A0, 38);
DRV8834 A(MOTOR_STEPS, A7, A6, A2);

void setup() {
  // Begin serial communication
  Serial.begin(9600);

  // Set the ENDSTOP pin as INPUT_PULLUP
  pinMode(ENDSTOP, INPUT_PULLUP);

  // Initialize the E-axis and A-axis stepper motors
  E.begin(E_RPM);
  A.begin(A_RPM);

  // Set the RPM for E-axis and A-axis stepper motors
  E.setRPM(E_RPM);
  A.setRPM(A_RPM);

  // Set the enable pin active state to LOW
  E.setEnableActiveState(LOW);
  A.setEnableActiveState(LOW);

  // Enable the E-axis and A-axis stepper motors
  E.enable();
  A.enable();

  // Set the microstepping value for E-axis and A-axis stepper motors
  E.setMicrostep(MICROSTEP);
  A.setMicrostep(MICROSTEP);
}

void loop() {
  // Delay for 2 seconds
  delay(2000);

  // Move A-axis until endstop is reached
  while (digitalRead(ENDSTOP)) {
    A.move(1);
    delayMicroseconds(300);
  }

  // Count the steps until endstop is reached
  int count = 0;
  while (!digitalRead(ENDSTOP)) {
    A.move(1);
    delayMicroseconds(300);
    count++;
  }

  //IMPORTANT!!!!!
  //TAKE THE CENTER POINT OF BETWEEN WHEN THE ENDSTOP IS TRIGERRED AND WHEN IT IS RELEASED
  A.move(-1 * (count / 2));

  // Set RPM for A-axis and delay
  A.setRPM(E_RPM);
  delay(500);

  // Move E-axis forward and backward
  E.move(MICROSTEP * (MOTOR_STEPS)*4);
  E.move(-MICROSTEP * (MOTOR_STEPS)*4);

  // Delay and move A-axis
  delay(500);
  A.move(MICROSTEP * (MOTOR_STEPS / 4));  //Select Tool 2
  delay(500);

  // Move E-axis forward and backward
  E.move(MICROSTEP * (MOTOR_STEPS)*4);
  E.move(-MICROSTEP * (MOTOR_STEPS)*4);

  // Delay and move A-axis
  delay(500);
  A.move(MICROSTEP * (MOTOR_STEPS / 4));  //Select Tool 3
  delay(500);

  // Move E-axis forward and backward
  E.move(-MICROSTEP * (MOTOR_STEPS)*4);
  E.move(MICROSTEP * (MOTOR_STEPS)*4);


  delay(500);
  A.move(MICROSTEP * (MOTOR_STEPS / 4));  //Select Tool 4
  delay(500);

  // Move E-axis forward and backward
  E.move(-MICROSTEP * (MOTOR_STEPS)*4);
  E.move(MICROSTEP * (MOTOR_STEPS)*4);
}
