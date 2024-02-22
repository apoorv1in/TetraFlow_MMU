#define FILAMENT_SENSOR_SWITCH_PIN 3 //Y-limit Switch connector
#define FILAMENT_ENCODER_PIN 16 //E0-limit swith connector SKR Pico
#define BUSY_STATUS_PIN 20 //25 //Z-limit swith connector SKR Pico
#define RGB_LED_PIN 24 

#define EXTRUDER_DIRECTION 1.0 //reverse extruder direstion by setting 1 or -1 E200 is extrude 200 and E-200 is retract 200

#define R_SENSE 0.11f //for TMC2209 present on SKR-PICO

#define TMC_SERIAL Serial2 //Serial port to communicate with TMC2209 Serial2 for SKR-PICO

#define MOTOR_STEPS_PER_REV 200.0

//=========== X-Stepper - port of SKR - PICO
#define A_EN_PIN 12            // Enable
#define A_DIR_PIN 10           // Direction
#define A_STEP_PIN 11          // Step
#define A_DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2

#define A_CURRENT 800
#define A_CURRENT_POWER_DOWN 450
#define A_STALL_PIN 4          // Connected to DIAG pin on the TMC2209 X-Diag pin jumper connected
#define A_HOMING_CURRENT 600
#define A_STALL_VALUE 60  // [0..255]
#define A_MICROSTEP 32

//=========== E0-Stepper - port of SKR - PICO
#define E_EN_PIN 15    // Enable
#define E_DIR_PIN 13   // Direction
#define E_STEP_PIN 14  // Step
#define E_DRIVER_ADDRESS 0b11 

#define E_CURRENT 800
#define E_CURRENT_POWER_DOWN 450

//2, 4, 8, 16, 32, 64, 128, or 256
#define E_MICROSTEP 32.0