#include "mmu_config.h"
#include "mmu_hardware_config.h"
#include "Settings.h"
#include <FastLED.h>
#include <Regexp.h>
#include <TMCStepper.h>
#include <EEPROM.h>

#define NUM_LEDS 2
CRGB leds[NUM_LEDS];

Settings mmu_setting;

int error = false;

bool isHomed;
int unloadStatus = -1;
int selectedTool = -1;
int loadedFilamentToolNumber = -1;
int APosition = 0;
float rps = 0.75;
int32_t A_Home_velocity = (rps * MOTOR_STEPS_PER_REV * A_MICROSTEP) / 0.715;

TMC2209Stepper driverA(&TMC_SERIAL, R_SENSE, A_DRIVER_ADDRESS);
TMC2209Stepper driverE(&TMC_SERIAL, R_SENSE, E_DRIVER_ADDRESS);

bool stalled_A = false;
bool stalled_E = false;
bool parked = false;

void stallInterruptA() {  // flag set for motor A when motor stalls
  stalled_A = true;
}

void stallInterruptE() {  // flag set for motor E when motor stalls
  stalled_E = true;
}

void fatalError() {
  leds[0] = CRGB::Red;
  FastLED.show();
  while (true) {
    //hang
  }
}

void InitStepperA() {
  pinMode(A_EN_PIN, OUTPUT);
  pinMode(A_STEP_PIN, OUTPUT);
  pinMode(A_DIR_PIN, OUTPUT);
  digitalWrite(A_EN_PIN, LOW);

  pinMode(A_STALL_PIN, INPUT);

  driverA.begin();
  driverA.reset();
  if (driverA.test_connection() != 0) {
    Serial.println("error: InitStepperA()");
    fatalError();
  }

  // driverA.irun(20);//temp
  // Sets the slow decay time (off time) [1... 15]. This setting also limits
  // the maximum chopper frequency. For operation with StealthChop,
  // this parameter is not used, but it is required to enable the motor.
  // In case of operation with StealthChop only, any setting is OK.
  driverA.toff(4);

  // VACTUAL allows moving the motor by UART control.
  // It gives the motor velocity in +-(2^23)-1 [μsteps / t]
  // 0: Normal operation. Driver reacts to STEP input.
  // /=0: Motor moves with the velocity given by VACTUAL.
  // Step pulses can be monitored via INDEX output.
  // The motor direction is controlled by the sign of VACTUAL.
  //driverA.VACTUAL(speed);

  // Comparator blank time. This time needs to safely cover the switching
  // event and the duration of the ringing on the sense resistor. For most
  // applications, a setting of 16 or 24 is good. For highly capacitive
  // loads, a setting of 32 or 40 will be required.
  driverA.blank_time(24);

  driverA.semin(5);
  driverA.semax(2);

  // Sets the number of StallGuard2 readings above the upper threshold necessary
  // for each current decrement of the motor current.
  driverA.sedn(0b01);

  driverA.rms_current(A_CURRENT);  // mA
  driverA.microsteps(A_MICROSTEP);

  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  driverA.TCOOLTHRS(0xFFFFF);  // 20bit max
  //driverA.TCOOLTHRS(0x3FF);  // 10bit max

  // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
  // motor specific characteristics and controls sensitivity. A higher value gives a higher
  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
  // indicate a stall. The double of this value is compared to SG_RESULT.
  // The stall output becomes active if SG_RESULT fall below this value.
  //driverA.SGTHRS(255);//A_STALL_VALUE);

  //driverA.en_spreadCycle(true);
  //driverA.pwm_autoscale(1);
  //driverA.intpol(true);
  //driverA.PWM_AUTO();  // Enable extremely quiet stepping

  // CoolStep lower threshold [0... 15].
  // If SG_RESULT goes below this threshold, CoolStep increases the current to both coils.
  // 0: disable CoolStep
  driverA.semin(3);  //5
}

void InitStepperE() {
  pinMode(E_EN_PIN, OUTPUT);
  pinMode(E_STEP_PIN, OUTPUT);
  pinMode(E_DIR_PIN, OUTPUT);
  digitalWrite(E_EN_PIN, LOW);

  driverE.begin();
  driverE.reset();

  if (driverE.test_connection() != 0) {
    Serial.println("error: InitStepperE()");
    fatalError();
  }

  driverE.toff(4);

  driverE.blank_time(24);

  driverE.semin(5);
  driverE.semax(2);

  driverE.sedn(0b01);

  driverE.rms_current(A_CURRENT);  // mA
  driverE.microsteps(E_MICROSTEP);

  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  driverA.TCOOLTHRS(0xFFFFF);  // 20bit max
  //driverE.TCOOLTHRS(0x3FF);  // 10bit max
  driverA.semin(5);
}
//extractCodeInt(input, "(T)(%d+)")
int extractCodeInt(const String &input, const String &pattern) {
  // Create a MatchState object
  MatchState ms;

  // Use Target() to set the input string
  ms.Target(const_cast<char *>(input.c_str()));

  // Use Match() to find the pattern in the input string
  ms.Match(pattern.c_str());

  // Check if the pattern is found
  if (ms.GetResult() == REGEXP_MATCHED) {
    // Extract the matched value and convert it to an integer
    char codeString[16];           // Adjust the size based on your expected maximum number of digits
    ms.GetCapture(codeString, 1);  // Assuming the pattern has a capture group for the digits
    return atoi(codeString);
  }

  // Return a default value or handle the case when the pattern is not found
  return -1;  // or any other default value
}

//extractCodeFloat("M24 E1.3","(E)(-?%d+%.?%d*)")
double extractCodeDouble(String input, String pattern) {
  // Create a MatchState object
  MatchState ms;

  // Use Target() to set the input string
  ms.Target(const_cast<char *>(input.c_str()));

  // Use Match() to find the pattern in the input string
  ms.Match(pattern.c_str());

  // Check if the pattern is found
  if (ms.GetResult() == REGEXP_MATCHED) {
    // Extract the matched value and convert it to an integer
    char codeString[16];           // Adjust the size based on your expected maximum number of digits
    ms.GetCapture(codeString, 1);  // Assuming the pattern has a capture group for the digits
    return atof(codeString);
  }

  // Return a default value or handle the case when the pattern is not found
  return -1;  // or any other default value
}

long encoderCounter;
void EncoderStepCount() {
  encoderCounter++;
}

void setBusy() {
  digitalWrite(BUSY_STATUS_PIN, LOW);
  leds[0] = CRGB::Blue;
  FastLED.show();
}

void resetBusy() {
  digitalWrite(BUSY_STATUS_PIN, HIGH);
  Serial.println("OK");
  delay(1);
  Serial.println("OK");
  Serial.flush();
  leds[0] = CRGB::Green;
  FastLED.show();
}

void SetIdelState() {
  driverE.rms_current(E_CURRENT_POWER_DOWN);  // mA
  driverA.rms_current(A_CURRENT_POWER_DOWN);  // mA
}

void setup() {
  Serial.begin(115200);
  TMC_SERIAL.begin(115200);
  delay(2000);
  mmu_setting.loadFromEEPROM();
  FastLED.addLeds<WS2812, RGB_LED_PIN, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB::Green;
  FastLED.show();
  while (!TMC_SERIAL) {}

  pinMode(FILAMENT_SENSOR_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(FILAMENT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(BUSY_STATUS_PIN, OUTPUT);  //pull up present on pin

  attachInterrupt(digitalPinToInterrupt(FILAMENT_ENCODER_PIN), EncoderStepCount, FALLING);

  isHomed = false;
  InitStepperA();
  InitStepperE();
}

void home() {
  Serial.println("Homing Start");
  if (driverA.test_connection() != 0) {
    Serial.println("error: home()");
    return;
  }

  digitalWrite(A_DIR_PIN, HIGH);

  int steps = 10 * A_MICROSTEP;
  while (steps > 0) {
    digitalWrite(A_STEP_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(A_STEP_PIN, LOW);
    delayMicroseconds(250);
    steps--;
  }

  driverA.rms_current(A_HOMING_CURRENT);
  int count = 0;
  int result = 0;
  driverA.shaft(false);
  driverA.VACTUAL(A_Home_velocity);
  driverA.SGTHRS(mmu_setting.aStallValue);
  delay(10);
  int oneRPMTime = ceil(1000.0 * 1000.0 / rps);  //us/roatation
  attachInterrupt(digitalPinToInterrupt(A_STALL_PIN), stallInterruptA, CHANGE);
  stalled_A = false;
  while (oneRPMTime > 0) {
    delayMicroseconds(40);
    oneRPMTime -= 40;
    if (stalled_A) {
      Serial.println("stallInterruptA");
      break;
    }
  }
  driverA.VACTUAL(0);
  if (oneRPMTime <= 0) {
    Serial.println("error: home()");
    //HOMING ERROR
    error = true;
    isHomed = false;
    APosition = -1;
  } else {
    isHomed = true;
    APosition = 0;
  }
  selectedTool = -1;
  driverA.SGTHRS(255);
  driverA.rms_current(A_CURRENT);

  detachInterrupt(digitalPinToInterrupt(A_STALL_PIN));
  parked = false;
  Serial.println("Homing End");
}


float MoveE(float extrude_mm, bool slow = false) {

  if (parked) {
    unParkToolHead();
  }
  float pending = 0.0;
  int stepsToExtrude;

  switch (selectedTool) {
    case 0:
      pending = (MOTOR_STEPS_PER_REV * E_MICROSTEP * extrude_mm) / mmu_setting.eRotationDistanceT0;
      break;
    case 1:
      pending = (MOTOR_STEPS_PER_REV * E_MICROSTEP * extrude_mm) / mmu_setting.eRotationDistanceT1;
      break;
    case 2:
      pending = (MOTOR_STEPS_PER_REV * E_MICROSTEP * extrude_mm) / mmu_setting.eRotationDistanceT2;
      break;
    case 3:
      pending = (MOTOR_STEPS_PER_REV * E_MICROSTEP * extrude_mm) / mmu_setting.eRotationDistanceT3;
      break;
  }

  stepsToExtrude = (int)pending;

  pending = pending - stepsToExtrude;

  digitalWrite(E_EN_PIN, LOW);
  if (selectedTool == 0 || selectedTool == 1) {
    extrude_mm = -1 * stepsToExtrude;
  }

  if (extrude_mm > 0) {
    digitalWrite(E_DIR_PIN, LOW);
  } else {
    digitalWrite(E_DIR_PIN, HIGH);
  }
  int steps = abs(stepsToExtrude);
  int slowStartDelay = 50;
  while (steps > 0) {
    digitalWrite(E_STEP_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(E_STEP_PIN, LOW);
    delayMicroseconds(100);
    if (slowStartDelay > 0) {
      delayMicroseconds(slowStartDelay);
      slowStartDelay -= 5;
    }
    if (slow)
      delayMicroseconds(50);
    steps--;
  }
  return pending;
}

int GetLoadedLilament() {
  if (!digitalRead(FILAMENT_SENSOR_SWITCH_PIN)) {
    return -1;  //No Filament loaded
  }
  for (int i = 0; i < NumberOfTools; i++) {
    ToolSelect(i);
    encoderCounter = 0;
    MoveE(-mmu_setting.filamentEncoderDetectionLength * 3.5 * EXTRUDER_DIRECTION, true);
    delay(5);
    MoveE(mmu_setting.filamentEncoderDetectionLength * 3.5 * EXTRUDER_DIRECTION, true);
    delay(5);
    if (encoderCounter != 0) {
      MoveE(-mmu_setting.filamentEncoderDetectionLength * 3.5 * EXTRUDER_DIRECTION, true);
      delay(5);
      MoveE(mmu_setting.filamentEncoderDetectionLength * 3.5 * EXTRUDER_DIRECTION, true);
      delay(5);
    }
    Serial.print("Tool: ");
    Serial.print(i);
    Serial.print("Encoder Count: ");
    Serial.println(encoderCounter);
    if (encoderCounter >= 5) {
      Serial.print("Active Tool Found: ");
      Serial.println(i);
      loadedFilamentToolNumber = i;
      return i;
    }
  }
  Serial.println("No Active Tool Found");
  loadedFilamentToolNumber = -2;
  return -1;
}

void ToolChange(int toolNumber) {
  UnloadFilament();
  ToolSelect(toolNumber);
}

void ToolSelect(int toolNumber) {
  if (error) {
    return;
  }
  if (!isHomed) {
    home();
    if (!isHomed)
      return;
  }

  Serial.print("Selecting Tool: ");
  Serial.println(toolNumber);
  if (parked) {
    unParkToolHead();
  }
  switch (toolNumber) {
    case 0:
      MoveA(APosition);
      selectedTool = 0;
      break;
    case 1:
      MoveA(APosition - (150 * A_MICROSTEP));
      selectedTool = 1;
      break;
    case 2:
      MoveA(APosition - (100 * A_MICROSTEP));
      selectedTool = 2;
      break;
    case 3:
      MoveA(APosition - (50 * A_MICROSTEP));
      selectedTool = 3;
      break;
  }
}

void parkToolHead() {
  if (!isHomed) {
    home();
    if (!isHomed)
      return;
  }
  if (parked) {
    return;
  }
  if (selectedTool < 0) {
    return;
  }
  Serial.print("Parking Tool: ");
  Serial.println(selectedTool);
  if (selectedTool == 1 || selectedTool == 2) {
    MoveA(100 * A_MICROSTEP); 
  } else {
    MoveA(-100 * A_MICROSTEP);
  }
  parked = true;
}

void unParkToolHead() {
  if (!isHomed) {
    home();
    if (!isHomed)
      return;
  }
  if (!parked) {
    return;
  }
  if (selectedTool < 0) {
    return;
  }
  Serial.print("unParking Tool: ");
  Serial.println(selectedTool);
  if (selectedTool == 1 || selectedTool == 2) {
    MoveA(-100 * A_MICROSTEP);  //clk
  } else {
    MoveA(100 * A_MICROSTEP);  //clk 25
  }
  parked = false;
}

void MoveA(int position) {
  Serial.print("APosition Full Step: ");
  Serial.println(position / A_MICROSTEP);
  if (position > 0) {
    digitalWrite(A_DIR_PIN, LOW);
  } else {
    digitalWrite(A_DIR_PIN, HIGH);
  }
  int steps = abs(position);
  int slowStartDelay = 100;
  while (steps > 0) {
    digitalWrite(A_STEP_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(A_STEP_PIN, LOW);
    delayMicroseconds(140);
    if (slowStartDelay > 0) {
      delayMicroseconds(slowStartDelay);
      slowStartDelay -= 5;
    }
    steps--;
  }
  APosition = APosition - position;
}

void UnloadTillSwitch() {
  float unloadDistance = mmu_setting.distanceFromSwitchToHead + 200;
  while (digitalRead(FILAMENT_SENSOR_SWITCH_PIN) && unloadDistance > 1.0) {
    if (unloadDistance > 15) {
      unloadDistance += MoveE(-15 * EXTRUDER_DIRECTION);
      unloadDistance -= 15;
    } else {
      unloadDistance += MoveE(-1 * EXTRUDER_DIRECTION);
      unloadDistance--;
    }
  }
  if (unloadDistance == 0) {
    Serial.println("error: UnloadTillSwitch()");
    error = true;
  }
}

void CalibrateESteps(int tool, float rev = 30) {
  ToolChange(tool);
  LoadTillSwitch();
  if (error)
    return;
  encoderCounter = 0;
  int stepsPerRev = MOTOR_STEPS_PER_REV * E_MICROSTEP;
  digitalWrite(E_EN_PIN, LOW);
  driverE.rms_current(E_CURRENT);  // mA
  bool dir;
  if (tool == 0 || tool == 1) {
    dir = false;
  } else {
    dir = true;
  }

  if (dir) {
    digitalWrite(E_DIR_PIN, LOW);
  } else {
    digitalWrite(E_DIR_PIN, HIGH);
  }
  for (int i = 0; i < rev; i++) {
    for (int j = 0; j < stepsPerRev; j++) {
      ExtrudeStep();
    }
  }
  float totalDistance = (float)encoderCounter * FILAMENT_ENCODER_DETECTION_LENGTH;
  float mmPerRev = totalDistance / rev;
  float accuracy = (FILAMENT_ENCODER_DETECTION_LENGTH / totalDistance) * 100;
  Serial.print("Tool: ");
  Serial.print(tool);
  Serial.print(" mmPerRev: ");
  Serial.print(mmPerRev);
  Serial.print(" Accuracy: ");
  Serial.print(accuracy);
  Serial.println("%");
}

void ExtrudeStep() {
  digitalWrite(E_STEP_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(E_STEP_PIN, LOW);
  delayMicroseconds(140);
}

bool UnloadFilament() {
  if (error) {
    return 0;
  }
  if (!isHomed) {
    home();
  }
  int loadedFilament = GetLoadedLilament();
  //Unload Filament
  if (loadedFilament >= 0) {
    UnloadTillSwitch();
    MoveE(-mmu_setting.distanceFromSwitchToSafeZone * EXTRUDER_DIRECTION);
    unloadStatus = 1;
    return 1;
  }
  Serial.println("UnloadFilament: No Filament loaded");
  return 0;
}

bool LoadTillSwitch() {
  if (error) {
    return false;
  }
  int MaxDistance = mmu_setting.distanceFromSwitchToSafeZone + 80;
  while (!digitalRead(FILAMENT_SENSOR_SWITCH_PIN) && MaxDistance > 0) {
    MaxDistance += MoveE(2 * EXTRUDER_DIRECTION);
    MaxDistance -= 2;
  }
  if (MaxDistance == 0) {
    //error
    error = true;
    Serial.print("error: LoadTillSwitch()");
    return false;
  }
  return true;
}

void LoadFilament() {
  if (error) {
    return;
  }
  if (LoadTillSwitch()) {
    encoderCounter = 0;
    int expectedEncoderCount = (mmu_setting.distanceFromSwitchToHead / FILAMENT_ENCODER_DETECTION_LENGTH);

    MoveE(mmu_setting.distanceFromSwitchToHead * EXTRUDER_DIRECTION);
    int oldEncoderVal = encoderCounter;
    while (encoderCounter <= expectedEncoderCount) {
      MoveE(FILAMENT_ENCODER_DETECTION_LENGTH + FILAMENT_ENCODER_DETECTION_LENGTH / 2);
      if (oldEncoderVal < encoderCounter)  //Moving
      {
        oldEncoderVal = encoderCounter;
      } else {
        error = true;
        Serial.println("error: LoadFilament() movement not detected");
        break;
      }
    }

    if (encoderCounter < expectedEncoderCount) {
      error = true;
      Serial.print("error: LoadFilament() Expected :");
      Serial.print(expectedEncoderCount);
      Serial.print(" Actual :");
      Serial.println(encoderCounter);
    }
    unloadStatus = 0;
  }
}

void loop() {
  if (Serial.available() > 0) {
    error = false;
    String input = Serial.readString();
    char startCommandCode = input.charAt(0);
    setBusy();
    driverE.rms_current(E_CURRENT);  // mA
    driverA.rms_current(A_CURRENT);  // mA
    delay(100);
    Serial.println(input);

    if (startCommandCode == 'G') {
      //G-Code
      int gCode = extractCodeInt(input, "(G)(%d+)");
      if (gCode == 28) {
        //G28 Home
        home();
      } else if (gCode == 1 || gCode == 0) {
        String st = "(E)(-?%d+%.?%d*)";
        double extrude = extractCodeDouble(input, st);
        ToolSelect(selectedTool);
        MoveE(extrude * EXTRUDER_DIRECTION);
      } else if (gCode == 900) {
        parkToolHead();
      } else if (gCode == 901) {
        unParkToolHead();
      }
    } else if (startCommandCode == 'T') {
      //T-Code
      //Tool Select G-Code T0,T1,T2,T3...
      double tCode = extractCodeInt(input, "(T)(-?%d+%.?%d*)");
      int code = (int)tCode;
      UnloadFilament();
      ToolSelect(abs((int)tCode));
      LoadFilament();

      parkToolHead();
    } else if (startCommandCode == 'S') {
      //S-Code custom to switch tool without extrusion
      //Tool Select G-Code S0,S1,S2,S3...
      double tCode = extractCodeInt(input, "(S)(-?%d+%.?%d*)");
      int code = (int)tCode;
      ToolSelect(abs((int)tCode));
    } else if (startCommandCode == 'O') {
      //override settings
      int oCode = extractCodeInt(input, "(O)(%d+)");
      double value = extractCodeInt(input, "(V)(-?%d+%.?%d*)");
      if (oCode == 1) {
        mmu_setting.distanceFromSwitchToHead = value;
      } else if (oCode == 2) {
        mmu_setting.distanceFromSwitchToSafeZone = value;
      } else if (oCode == 3) {
        mmu_setting.filamentEncoderDetectionLength = value;
      } else if (oCode == 4) {
        mmu_setting.eRotationDistanceT0 = value;
      } else if (oCode == 5) {
        mmu_setting.eRotationDistanceT1 = value;
      } else if (oCode == 6) {
        mmu_setting.eRotationDistanceT2 = value;
      } else if (oCode == 7) {
        mmu_setting.eRotationDistanceT3 = value;
      } else if (oCode == 8) {
        mmu_setting.aStallValue = value;
        driverA.SGTHRS((uint8_t)value);
      }
      mmu_setting.saveToEEPROM();
    } else if (startCommandCode == 'M') {
      //M-Code
      int mCode = extractCodeInt(input, "(M)(%d+)");
      if (mCode == 600) {
        if (UnloadFilament()) {
          parkToolHead();
        }
      }
    } else if (startCommandCode == 'C') {
      //C-Code
      Serial.println("Calibrate");
      int tool = extractCodeInt(input, "(C)(%d+)");
      double rev = extractCodeInt(input, "(V)(-?%d+%.?%d*)");
      CalibrateESteps(tool, rev);
    }

    if (error) {
      Serial.println("MMU ERROR !!!!!");
      Serial.flush();
      delay(30 * 1000);  // trigger timeout on klipper
    }
    resetBusy();
    SetIdelState();
  }
}