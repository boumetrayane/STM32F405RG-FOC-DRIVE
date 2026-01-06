#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>

// STM32F405RG Arduino Pin Mapping
//PC13 → PC13 (LED)
//PA0  → PA0  (ADC1_IN0 - Phase A current)
//PA1  → PA1  (ADC1_IN1 - Phase B current)  
//PA4  → PA4  (ADC1_IN4 - Phase C current)
//PA5  → PA5  (ADC1_IN5 - Vbus sense)
//PA8  → PA8  (TIM1_CH1 - PWM Phase A)
//PA9  → PA9  (TIM1_CH2 - PWM Phase B)
//PA10 → PA10 (TIM1_CH3 - PWM Phase C)
//PB12 → PB12 (Driver Enable)
//PB6  → PB6  (I2C1_SCL - AS5600)
//PB7  → PB7  (I2C1_SDA - AS5600)
//PA2  → PA2  (USART2_TX - Future ESP32)
//PA3  → PA3  (USART2_RX - Future ESP32)

const int ledPin = PC13; // On-board LED pin for STM32F4
const int currcomp_A = PA0;  // Phase A current comparator input
const int currcomp_B = PA1;  // Phase B current comparator input
const int currcomp_C = PA4;  // Phase C current comparator input
const int VBUS_sense = PA5;    // Vbus voltage sense input
const int pwmA = PA8;      // PWM output for Phase A
const int pwmB = PA9;      // PWM output for Phase B
const int pwmC = PA10;     // PWM output for Phase C
const int drive_en = PB12; // Driver enable pin
const int I2C_SCL = PB6;   // I2C SCL pin for AS5600
const int I2C_SDA = PB7;   // I2C SDA pin for AS5600
const int USART2_TX = PA2; // USART2 TX pin for future ESP32 connection
const int USART2_RX = PA3; // USART2 RX pin for future ESP32 connection

const int motor_pole_pairs = 10; // Number of pole pairs for the motor
const int max_motor_current = 12; // Maximum motor current in Amperes
const float motor_phase_resistance = 0.5; // Phase resistance in Ohms
const float motor_phase_inductance = 0.0005; // Phase inductance in Henrys
const int voltage_limit = 12;
const int motor_kv_rating = 220; // Motor KV rating in RPM/V

const float current_sense_gain = 0.066; // Current sense amplifier gain
const float SHUNT_RESISTOR = 0;    // No shunt (using ACS712)
const float AMP_GAIN = 20;   // Op-amp gain (if used)
const float VREF = 1.65;   // ACS712 zero-current voltage (Vcc/2)

#define CURRENT_DIVIDER_USED   false   // Set to 'true' if you add divider later
// ACS712 specifications
#define ACS712_VCC             5.0f    // ACS712 supply voltage
#define ACS712_SENSITIVITY     0.066f  // 66mV/A for ACS712-30A at 5V
// Voltage divider for CURRENT SENSORS (R1=10kΩ, R2=15kΩ)
/*
ACS712 ─── 10kΩ ───┬─── STM32 ADC
                   │
                  15kΩ
                   │
                  GND
*/
// Ratio = R2 / (R1 + R2) = 15k / 25k = 0.6
#define CURRENT_DIVIDER_RATIO  0.6f    
// Calculate actual values seen by STM32 ADC for CURRENT sensing:
#if CURRENT_DIVIDER_USED
  // With voltage divider (for >12A capability)
  #define CURRENT_VREF  (ACS712_VCC / 2.0f * CURRENT_DIVIDER_RATIO)        // 2.5V × 0.6 = 1.5V
  #define CURRENT_SENSE_GAIN (ACS712_SENSITIVITY * CURRENT_DIVIDER_RATIO)  // 66mV × 0.6 = 39.6mV/A
  #define MAX_SAFE_CURRENT   25.0f     // Safe up to 25A with divider
#else
  // Direct connection (for ≤12A operation only)
  #define CURRENT_VREF  (ACS712_VCC / 2.0f)    // 2.5V at zero current
  #define CURRENT_SENSE_GAIN ACS712_SENSITIVITY  // 66mV/A
  #define MAX_SAFE_CURRENT   12.0f     // ⚠️ Limited to 12A without divider
#endif

const float VBUS_divider_ratio = 11; // Voltage divider ratio for Vbus sensing
const float battery_voltage = 11.4;
const float battery_current = 2.5;
const float voltage_max_limit = 12;
const float current_max_limit = 3;

const float I2C_FREQUENCY = 400000; // I2C frequency for AS5600
const float pwm_frequency = 20000; // PWM frequency in Hz
const int pwm_resolution = 12; // PWM resolution in bits

BLDCMotor motor = BLDCMotor(motor_pole_pairs);
BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, drive_en);
//SensorAS5600 sensor = SensorAS5600(0x36, I2C_FREQUENCY);
//CurrentSenseACS712 current_sense = CurrentSenseACS712(currcomp_A, currcomp_B, currcomp_C, V_sense, current_sense_gain, VREF, voltage_divider_ratio);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C); // Using predefined config in MagneticSensorI2C.h for AS5600

InlineCurrentSense current_sense = InlineCurrentSense(
  current_sense_gain, VREF, currcomp_A, currcomp_B, currcomp_C);

//CONTROL PARAMETERS

float target_velocity = 0.0; // target velocity in rad/s
float target_current = 0.0; // target current in Amperes
float target_torque = 0.0; // target torque in Amperes
float target_position = 0.0; // target position in radians
float target_angle = 0.0; // target angle in radians

// PID Controllers (auto-tuned by SimpleFOC, but can be adjusted)
// Velocity PID
float velocity_P = 0.5; // maximum velocity in rad/s
float velocity_I = 10; // velocity integral gain
float velocity_D = 0.0001; // velocity derivative gain
float velocity_ramp = 1000; // Acceleration limit in rad/s^2
float velocity_lowpass = 0.02; // velocity low-pass filter constant

// Angle PID 
float angle_P = 20; // angle proportional gain
float angle_I = 0; // angle integral gain
float angle_D = 0.1; // angle derivative gain
float angle_lowpass = 0.01; // angle low-pass filter constant

// Current PID (q-axis)
float current_q_P = 3; // current proportional gain
float current_q_I = 300; // current integral gain
float current_q_lowpass = 0.01; // current low-pass filter constant

// Current PID (d-axis - field weakening)
float current_d_P = 3; // current proportional gain
float current_d_I = 300; // current integral gain  
float current_d_lowpass = 0.01; // current low-pass filter constant  

//SERIAL MONITORING
unsigned long last_print = 0;
const unsigned long print_interval = 100; // Print every 100ms

// Voltage monitoring
float VBUS_voltage = 0;

// Forward declarations
void process_serial_commands();
void printMotorStatus();
void printDetailedInfo();
void performSafetyChecks();
void testVbusDivider();

void setup() {
pinMode(ledPin, OUTPUT);
digitalWrite(ledPin, HIGH);
pinMode(drive_en, OUTPUT);
Serial.begin(115200);
while (!Serial && millis()< 300);
Serial.println("STM32F405RG foc startup sequence");
Serial.println("please wait...");

// SAFETY CHECK: Verify ADC inputs are safe
Serial.println("[...] Checking current sensor voltages...");  
// Read raw ADC values (motor should be OFF - no current)
  int adc_a = analogRead(currcomp_A);
  int adc_b = analogRead(currcomp_B);
  int adc_c = analogRead(currcomp_C);
  
  // Convert to voltage (STM32 ADC: 12-bit, 0-4095 = 0-3.3V)
  float v_a = (adc_a / 4095.0f) * 3.3f;
  float v_b = (adc_b / 4095.0f) * 3.3f;
  float v_c = (adc_c / 4095.0f) * 3.3f;
  
  Serial.print("    Phase A: ");
  Serial.print(v_a, 3);
  Serial.print(" V (ADC: ");
  Serial.print(adc_a);
  Serial.println(")");
  
  Serial.print("    Phase B: ");
  Serial.print(v_b, 3);
  Serial.print(" V (ADC: ");
  Serial.print(adc_b);
  Serial.println(")");
  
  Serial.print("    Phase C: ");
  Serial.print(v_c, 3);
  Serial.print(" V (ADC: ");
  Serial.print(adc_c);
  Serial.println(")");
  
  // CRITICAL SAFETY CHECK
  #if !CURRENT_DIVIDER_USED
    // Direct connection mode - warn user about limits
    Serial.println("");
    Serial.println("╔════════════════════════════════════════════════╗");
    Serial.println("║ ⚠️  OPERATING WITHOUT VOLTAGE DIVIDER          ║");
    Serial.println("╠════════════════════════════════════════════════╣");
    Serial.println("║ Configuration: Direct 5V ACS712 → STM32 ADC   ║");
    Serial.println("║                                                ║");
    Serial.println("║ CURRENT LIMIT: ≤12A (ADC safe up to ~3.29V)   ║");
    Serial.println("║                                                ║");
    Serial.println("║ ⚠️ DO NOT EXCEED 12A BATTERY CURRENT! ⚠️       ║");
    Serial.println("║                                                ║");
    Serial.println("║ If you need >12A in the future:               ║");
    Serial.println("║ 1. Add voltage divider (10kΩ + 15kΩ)          ║");
    Serial.println("║ 2. Change: CURRENT_DIVIDER_USED = true        ║");
    Serial.println("║ 3. Re-upload code                             ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println("");
    
    // Check if any voltage is dangerously high (shouldn't happen at zero current)
    if (v_a > 3.25f || v_b > 3.25f || v_c > 3.25f) {
      Serial.println("[!] CRITICAL: Sensor voltage too high at zero current!");
      Serial.println("    This indicates a wiring problem or 5V supply is >5.3V");
      Serial.println("    STOPPING FOR SAFETY");
      
      // Blink LED rapidly to indicate error
      while(true) {
        digitalWrite(ledPin, !digitalRead(ledPin));
        delay(200);
      }
    }
  #else
    // Voltage divider mode - higher current capability
    Serial.println("");
    Serial.println("╔════════════════════════════════════════════════╗");
    Serial.println("║ ✓ VOLTAGE DIVIDER MODE ACTIVE                 ║");
    Serial.println("╠════════════════════════════════════════════════╣");
    Serial.println("║ Configuration: 5V ACS712 → Divider → STM32    ║");
    Serial.println("║ Current limit: Up to 25A safely                ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println("");
  #endif
  
  // Check if readings are reasonable (should be near CURRENT_VREF)
  float expected_vref = CURRENT_VREF;
  float tolerance = 0.3f; // ±300mV tolerance
  
  if (abs(v_a - expected_vref) > tolerance || 
      abs(v_b - expected_vref) > tolerance || 
      abs(v_c - expected_vref) > tolerance) {
    Serial.println("[!] WARNING: Current sensor offset unusual!");
    Serial.print("    Expected ~");
    Serial.print(expected_vref, 2);
    Serial.println(" V at zero current");
    Serial.println("    Check: sensor wiring, power supply stability");
    Serial.println("    This is OK if motor is already spinning/loaded");
  } else {
    Serial.println("[✓] Current sensor voltages within expected range");
  }

Wire.setSDA(I2C_SDA);
Wire.setSCL(I2C_SCL); 
//Wire.begin(I2C_SDA, I2C_SCL);
Wire.setClock(I2C_FREQUENCY);//better for AS5600 
Wire.begin();           // 100kHz (standard mode) is default
sensor.init(&Wire); 
/* Library: SimpleFOC → MagneticSensorI2C.cpp
Class: MagneticSensorI2C
What it does:
Sends I2C command to AS5600 address 0x36
Reads configuration registers to verify sensor is responding
Configures AS5600 settings:
  12-bit resolution (4096 positions per revolution)
  Continuous reading mode
  Sets up angle output format
*/

Serial.println("serial i2c initialisation ....");
Serial.println("AS5600 encoder initialised");
motor.linkSensor(&sensor);
/* Library: SimpleFOC → BLDCMotor.cpp
Function Signature: void BLDCMotor::linkSensor(Sensor* _sensor)
simplified code:
    // Inside BLDCMotor.cpp:
    void BLDCMotor::linkSensor(Sensor* _sensor) {
        sensor = _sensor;  // Store pointer to sensor
    }

    // Later in FOC loop:
    float BLDCMotor::shaftAngle() {
        return sensor->getAngle();  // Reads AS5600 via I2C
    }
*/

Serial.println("sensor linked to motor");

driver.voltage_power_supply = battery_voltage;
driver.voltage_limit = voltage_limit;
driver.pwm_frequency = pwm_frequency;
/*  Library: SimpleFOC → BLDCDriver3PWM.cpp
What each one does:
voltage_power_supply = BATTERY_VOLTAGE
  Tells driver the actual DC bus voltage
  Used for voltage-to-duty-cycle conversion
  Formula: duty_cycle = requested_voltage / voltage_power_supply
  Example: Request 6V on 12V supply → 50% duty cycle

voltage_limit = VOLTAGE_LIMIT
  Software safety limit for maximum voltage
  Protects motor from over-voltage
  Typically set 10-20% below supply voltage
  Prevents voltage spikes from exceeding supply

pwm_frequency = PWM_FREQUENCY
  Sets PWM switching frequency (20kHz)
  Higher frequency = smoother torque, more switching losses
  Lower frequency 
*/

driver.init();
/*  Library: SimpleFOC → BLDCDriver3PWM.cpp
What it does:

Configures TIM1 (Advanced Timer):
  Center-aligned PWM mode (for SVPWM)
  Prescaler calculation for desired frequency
  Auto-reload register (ARR) for period

Sets up GPIO pins (PA8, PA9, PA10):
  Alternate function mode (AF1 = TIM1)
  Push-pull output
  High speed mode

Initializes PWM channels:
  Compare registers (CCR1, CCR2, CCR3)
  Output enable
  Polarity configuration

Configures enable pin (PB12):
  GPIO output mode
  Sets LOW initially (drivers disabled)
*/

Serial.println("driver initialised"); 
Serial.print("PWM Frequency: ");
Serial.print(PWM_FREQUENCY);
Serial.println(" Hz");
motor.linkDriver(&driver);
/*Library: SimpleFOC → BLDCMotor.cpp
Function: void BLDCMotor::linkDriver(BLDCDriver* _driver)
What it does:

Creates pointer from motor to driver
Motor can now call driver.setPwm(Ua, Ub, Uc) to set voltages
Enables FOC algorithm to output control signals

somplified code:
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
    driver = _driver;
}

// Later in FOC loop:
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle) {
    // SVPWM calculation (Space Vector PWM)
    float Ua, Ub, Uc;
    _sincos(angle, &sine, &cosine);
    // ... Clarke/Park inverse transforms ...
    driver->setPwm(Ua, Ub, Uc);  // Update PWM duty cycles
}
*/

Serial.println("driver linked to motor");

current_sense.linkDriver(&driver);
/* Library: `SimpleFOC` → `InlineCurrentSense.cpp`

What it does:
- Current sensing needs synchronization with PWM signals
- Links driver to trigger ADC sampling at optimal times
- Samples during PWM "valley" (low-side FET ON) for accuracy

Timing synchronization:
PWM Cycle:
    ____        ____
___|    |______|    |___  ← High-side FET
       |______|            ← Low-side FET ON (sample here!)
          ↑
     ADC trigger

Why synchronization matters:
  Sampling during switching = noisy readings (EMI spikes)
  Sampling during low-side ON = clean shunt voltage measurement
  Driver provides timer trigger signal for ADC
*/

Serial.println("current sense linked to driver");
current_sense.init();
/* Library: SimpleFOC → InlineCurrentSense.cpp
What it does:
  Configures ADC1 channels:
    PA0 → ADC1_IN0 (Phase A)
    PA1 → ADC1_IN1 (Phase B)
    PA4 → ADC1_IN4 (Phase C)

  Sets ADC parameters:
    12-bit resolution (0-4095)
    Sampling time: 15 cycles (fast)
    Scan mode: Sequential channel reading

  Configures DMA (Direct Memory Access):
    Automatically transfers ADC readings to memory
    No CPU interruption needed
    Triple-buffer for 3 channels

  Sets up TIM1 trigger:
    ADC conversion starts on PWM timer event
    Ensures sampling at valley of PWM

  Automatic every PWM cycle:
*/

current_sense.skip_align = false;  //calibration
/* Library: SimpleFOC → InlineCurrentSense.h
Variable: bool skip_align
What it does:
  false = Perform current sensor offset calibration
  true = Skip calibration (use previous values)
*/

Serial.println("current sense configured and initialised");
motor.linkCurrentSense(&current_sense);
/* Library: SimpleFOC → BLDCMotor.cpp
What it does:
  Motor can now read real-time phase currents
  Enables FOC current control (torque mode)
  Allows Clarke/Park transforms on current measurements

Simplified code:
// In motor.loopFOC():
PhaseCurrent_s currents = current_sense->getPhaseCurrents();
// Returns: Ia, Ib, Ic

// Clarke Transform:
float I_alpha = currents.a;
float I_beta = (currents.a + 2*currents.b) / sqrt(3);

// Park Transform:
float I_d = I_alpha * cos(angle) + I_beta * sin(angle);
float I_q = -I_alpha * sin(angle) + I_beta * cos(angle);

// PID control:
voltage_q = PID_current_q.compute(target_Iq - I_q);
voltage_d = PID_current_d.compute(target_Id - I_d);

Data Flow Summary:
    Here's how all these components work together:
    ```
    ┌─────────────┐
    │   AS5600    │ ←── I2C (Wire.h)
    │  (Encoder)  │
    └──────┬──────┘
          │ sensor.getAngle()
          ↓
    ┌─────────────────────────────────────┐
    │         BLDCMotor Object            │
    │  • Reads angle from sensor          │
    │  • Reads currents from current_sense│
    │  • Commands voltages to driver      │
    │  • Performs FOC algorithm           │
    └─────────┬───────────────────┬───────┘
              │                   │
              │ setPwm()          │ getPhaseCurrents()
              ↓                   ↓
      ┌─────────────┐     ┌──────────────┐
      │BLDCDriver3PWM│     │InlineCurrent │
      │  (TIM1 PWM)  │     │Sense (ADC1)  │
      └──────┬───────┘     └──────┬───────┘
              │                    │
              ↓                    ↓
        [ MOSFETs ]          [ ACS712 ]
              │                    │
              └────→ [ MOTOR ] ←───┘
*/

Serial.println("current sense linked to motor");

//Motor Control Mode
// Choose one: TorqueControlType::voltage, foc_current, dc_current
motor.torque_controller = TorqueControlType::foc_current;
/*  Library: SimpleFOC → BLDCMotor.h
    Enum Definition: common/base_classes/FOCMotor.h

    other options: 
      enum TorqueControlType {
        voltage,      // Open-loop: Just apply voltage (no current feedback)
        dc_current,   // Closed-loop: Use DC current measurement only
        foc_current   // Closed-loop: Full FOC with real-time current sensing ← YOU USE THIS
      };
    Why this matters:
      Voltage mode: Fast but inaccurate (no feedback)
      FOC current mode: Precise torque control (uses Clarke/Park) ← Best for robotics

What it does:
  Sets HOW torque is generated at the lowest level
  foc_current means: Use measured phase currents in FOC algorithm

  simplified code:
      // Inside SimpleFOC BLDCMotor.cpp:
    if (torque_controller == TorqueControlType::foc_current) {
        // Read phase currents from ADC
        PhaseCurrent_s currents = current_sense->getPhaseCurrents();
        
        // Clarke transform: abc → αβ
        current.alpha = currents.a;
        current.beta = _1_SQRT3 * (currents.b + currents.c);
        
        // Park transform: αβ → dq
        current.q = -current.alpha * sin(angle) + current.beta * cos(angle);
        current.d = current.alpha * cos(angle) + current.beta * sin(angle);
        
        // PID control on actual currents
        voltage.q = PID_current_q(target_Iq - current.q);
        voltage.d = PID_current_d(target_Id - current.d);
    }
*/

motor.controller = MotionControlType::torque;
/*  Library: SimpleFOC → BLDCMotor.h
    Enum Definition: common/base_classes/FOCMotor.h

  What it does:
    Sets WHAT you're controlling at the high level
    Creates the control cascade hierarchy

  other options:
      enum MotionControlType {
        torque,          // Direct torque (Iq current) control ← YOU START HERE
        velocity,        // Speed control (uses torque internally)
        angle,           // Position control (uses velocity→torque cascade)
        velocity_openloop, // Open-loop speed (no encoder)
        angle_openloop   // Open-loop position (no encoder)
      }

      Control Cascade:
        TORQUE mode:
          User command → Iq target → Motor torque
          (Single level - direct control)

        VELOCITY mode:
          User command → PID_velocity → Iq target → Motor torque
          (Two levels - velocity PID controls torque)

        ANGLE mode:
          User command → P_angle → velocity target → PID_velocity → Iq target → Motor
          (Three levels - cascaded control)
  
  simplified code:
    // Inside SimpleFOC - move() function:
    switch(controller) {
        case MotionControlType::torque:
            // Direct: target is already Iq current
            current_sp = target;
            break;
            
        case MotionControlType::velocity:
            // Calculate velocity error
            vel_error = target - shaft_velocity;
            // PID outputs torque command
            current_sp = PID_velocity(vel_error);
            break;
            
        case MotionControlType::angle:
            // First: position → velocity
            vel_target = P_angle(target - shaft_angle);
            // Then: velocity → torque
            current_sp = PID_velocity(vel_target - shaft_velocity);
            break;
    }
*/

Serial.println("motor control mode set to torque control using FOC current");
// to choose a diffrent motion control type uncomment 2 of the 4 lines below:
// motor.controller = MotionControlType::velocity; // For velocity control
// Serial.println("motor control mode set to velocity control");
// motor.controller = MotionControlType::angle;    // For position control
// Serial.println("motor control mode set to position control");

//motor limits for safety
motor.voltage_limit = voltage_limit;
/*  Library: SimpleFOC → BLDCMotor.h
    Member Variable: float voltage_limit
  What it does:
    Software clamp on maximum voltage applied to motor phases
    Prevents over-voltage that could damage motor/driver

  simplified code:
    // Inside setPhaseVoltage():
    float Uq_clamped = constrain(Uq, -voltage_limit, voltage_limit);
    float Ud_clamped = constrain(Ud, -voltage_limit, voltage_limit);

    // Also affects SVPWM:
    float modulation_index = sqrt(Uq² + Ud²) / voltage_limit;
    if (modulation_index > 1.0) {
        // Scale back to prevent over-modulation
        Uq *= 1.0 / modulation_index;
        Ud *= 1.0 / modulation_index;
    }
  
  Why 11V instead of 12V?
    Safety margin for voltage spikes
    Prevents PWM saturation (100% duty cycle)
    Leaves room for back-EMF at high speeds
*/

motor.current_limit = max_motor_current;
/*  Library: SimpleFOC → BLDCMotor.h
    Member Variable: float current_limit
  What it does:
    Maximum allowed current (Iq + Id magnitude)
    Protects motor, MOSFETs, and battery

  simplified code:
    // Inside FOC algorithm:
    float current_magnitude = sqrt(current.q² + current.d²);

    if (current_magnitude > current_limit) {
        // Scale down both components proportionally
        float scale = current_limit / current_magnitude;
        current.q *= scale;
        current.d *= scale;
    }
      
  Physical meaning:
    Iq = torque-producing current (tangential to rotor)
    Id = flux-producing current (radial to rotor)
    Total = √(Iq² + Id²) must be ≤ current_limit
*/

motor.velocity_limit = (motor_kv_rating * battery_voltage * 2.0f * M_PI) / 60.0f; // rad/s
/*  Library: `SimpleFOC` → `BLDCMotor.h`  
    Member Variable: `float velocity_limit`

  What it does:
  - Maximum angular velocity in radians/second
  - Used in velocity and angle control modes

  Conversion reference:
    100 rad/s = 955 RPM
    Formula: RPM = (rad/s × 60) / (2π)
            RPM = 100 × 9.549 = 955 RPM 

  simplified code:
    // In velocity control mode:
    float vel_error = target - shaft_velocity;

    // Clamp target velocity
    if (abs(target) > velocity_limit) {
        target = copysign(velocity_limit, target);
    }

    // Also used in angle mode to limit speed of position changes
    float vel_target = P_angle(angle_error);
    vel_target = constrain(vel_target, -velocity_limit, velocity_limit);
*/

Serial.println("motor limits configured");

//configure Velocity PID
motor.PID_velocity.P = velocity_P;
motor.PID_velocity.I = velocity_I;
motor.PID_velocity.D = velocity_D;
motor.PID_velocity.output_ramp = velocity_ramp;
/*Library: SimpleFOC → common/pid.h and pid.cpp
  Class: PIDController
  somplified code:
    class PIDController {
      public:
          float P;              // Proportional gain
          float I;              // Integral gain
          float D;              // Derivative gain
          float output_ramp;    // Max output change per second
          float limit;          // Output saturation limit
          
      private:
          float integral;       // Accumulated error
          float prev_error;     // Last error (for derivative)
          float prev_output;    // Last output (for ramping)
      };

  PID controle code:
    float PIDController::operator()(float error) {
    // Proportional term
      float P_term = P * error;
      
      // Integral term (accumulated error over time)
      integral += I * error * deltaT;
      integral = constrain(integral, -limit, limit);  // Anti-windup
      
      // Derivative term (rate of error change)
      float D_term = D * (error - prev_error) / deltaT;
      prev_error = error;
      
      // Sum all terms
      float output = P_term + integral + D_term;
      
      // Apply output ramping (slew rate limiting)
      float max_change = output_ramp * deltaT;
      output = constrain(output, 
                        prev_output - max_change,
                        prev_output + max_change);
      prev_output = output;
      
      // Saturate to limits
      return constrain(output, -limit, limit);
  }
  

  Parameter meanings:
    P (Proportional) = 0.5
      Effect: Immediate response to error
      Higher P: Faster response, more oscillation
      Lower P: Slower response, more stable
      Example: If velocity error = 10 rad/s, P_term = 0.5 × 10 = 5A torque

    I (Integral) = 10.0
      Effect: Eliminates steady-state error
      Higher I: Faster convergence, risk of overshoot
      Lower I: Slower convergence, stable
      Anti-windup: Prevents integral from growing unbounded

    D (Derivative) = 0.0001
      Effect: Damping (reduces overshoot)
      Higher D: More damping, sensitive to noise
      Lower D: Less damping, smooth
      Why so small?: Motor velocity is noisy, strong D amplifies noise

    output_ramp = 1000.0
      Effect: Limits torque change rate (A/s)
      Purpose: Prevents jerky motion, protects mechanics
      Example: Can increase torque by max 1000A per second
*/

motor.LPF_velocity.Tf = velocity_lowpass;
/*  Library: SimpleFOC → common/lowpass_filter.h
    Class: LowPassFilter
  What it does:
    Smooths noisy velocity measurements from encoder
    Tf = filter time constant (seconds)

  internal code :  
    class LowPassFilter {
    private:
        float y_prev;  // Previous filtered value
        float Tf;      // Time constant
        
    public:
        float operator()(float x) {
            // First-order IIR filter
            float alpha = Tf / (Tf + deltaT);
            float y = alpha * y_prev + (1 - alpha) * x;
            y_prev = y;
            return y;
        }
    };

  Time constant meaning:
    Tf = 0.02 → Cutoff frequency ≈ 8 Hz
    Removes high-frequency noise while keeping response quick
    Smaller Tf: Less filtering, faster response, noisier
    Larger Tf: More filtering, slower response, smoother

  Frequency response:
    Cutoff freq (Hz) = 1 / (2π × Tf)
                    = 1 / (2π × 0.02)
                    ≈ 8 Hz
*/

Serial.println("velocity PID configured");

//configure Angle PID
motor.P_angle.P = angle_P;
motor.P_angle.I = angle_I;
motor.P_angle.D = angle_D;
motor.LPF_angle.Tf = angle_lowpass;
/*Library: SimpleFOC → common/pid.h  
  Class: PIDController

  This is the **outer loop** in cascaded control:
    Position error → P_angle → Velocity target → PID_velocity → Torque

  Parameter choices explained:
    P = 20.0 (High)
      Position control needs strong proportional action
      Acts like a spring constant
      Higher P = stiffer position hold (robotic actuator behavior)
      Example: 0.1 rad error → 20 × 0.1 = 2 rad/s velocity command

    I = 0.0 (Off)
      Usually not needed for position control
      Velocity loop already has integral action
      Adding I here can cause double-integration instability

    D = 0.1 (Damping)
      Prevents overshoot when reaching target
      Acts like mechanical damping
      Smooths approach to position

    LPF = 0.01 (Faster than velocity)
      Position signal is already smooth (integrated from velocity)
      Less filtering needed
*/

Serial.println("angle PID configured");

//configure Current PID (q-axis)
motor.PID_current_q.P = current_q_P;
motor.PID_current_q.I = current_q_I;
motor.LPF_current_q.Tf = current_q_lowpass;
Serial.println("current q-axis PID configured");

//configure Current PID (d-axis)
motor.PID_current_d.P = current_d_P;
motor.PID_current_d.I = current_d_I;
motor.LPF_current_d.Tf = current_d_lowpass;
/*Library: SimpleFOC → common/pid.h
  What they control:
    Q-axis Current (Iq):
      Physical meaning: Torque-producing current
      Analogy: Like armature current in DC motor
      Target: Set by higher-level controllers (velocity/angle) or directly (torque mode)

    D-axis Current (Id):
      Physical meaning: Flux-producing current
      Normal operation: Target Id = 0 (maximum efficiency)
      Field weakening: Target Id < 0 at high speeds (advanced)

    Why such high I gain (300)?:
      Current loops are the innermost, fastest loops:
        Loop hierarchy (slowest → fastest):
        Position: ~10-100 Hz
          ↓
        Velocity: ~100-1000 Hz
          ↓
        Current: ~1000-10000 Hz ← Must be VERY fast!

  internal code:
    // Inside motor.loopFOC() - called at high frequency:
    // Read currents
    PhaseCurrent_s i_abc = current_sense->getPhaseCurrents();

    // Clarke + Park transforms
    float i_alpha = i_abc.a;
    float i_beta = _1_SQRT3 * (i_abc.b + i_abc.c);
    float i_q = -i_alpha * sin(angle) + i_beta * cos(angle);
    float i_d = i_alpha * cos(angle) + i_beta * sin(angle);

    // Filter measured currents
    i_q_filtered = LPF_current_q(i_q);
    i_d_filtered = LPF_current_d(i_d);

    // PID control
    voltage.q = PID_current_q(target_Iq - i_q_filtered);
    voltage.d = PID_current_d(target_Id - i_d_filtered);

    // Inverse Park + Clarke + SVPWM
    setPhaseVoltage(voltage.q, voltage.d, angle);
*/

Serial.println("current d-axis PID configured");

Serial.println("[...] Starting FOC initialization...");
Serial.println("      (Motor will move during calibration)");
motor.init();
/*Library: SimpleFOC → BLDCMotor.cpp
  Function: void BLDCMotor::init()

  internal code:
    void BLDCMotor::init() {
    // 1. Initialize driver (PWM setup)
    driver->init();
    driver->enable();
    
    // 2. Initialize sensor (encoder communication)
    sensor->init();
    
    // 3. Initialize current sensing
    if (current_sense) {
        current_sense->init();
        current_sense->linkDriver(driver);
    }
    
    // 4. Set initial phase resistance (for voltage calculations)
    if (phase_resistance == NOT_SET) {
        phase_resistance = 0.0f;  // Will be estimated
    }
    
    // 5. Initialize control loops
    PID_velocity.reset();
    PID_current_q.reset();
    PID_current_d.reset();
    P_angle.reset();
    
    // 6. Set initial electrical angle to zero
    electrical_angle = 0;
    shaft_angle = 0;
    
    // 7. Enable driver output
    driver->enable();
    
    Serial.println("Motor initialized - ready for calibration");
  }


Does NOT move motor yet - that happens in `initFOC()` next!
  
  COMPLETE DATA FLOW SUMMARY
    Here's the full system data flow with all the code we just analyzed:
    ┌─────────────────────────────────────────────────────────────────────┐
    │                        USER COMMAND INPUT                           │
    │  Serial: "T2.5" or "V10" or "A3.14"                                 │
    └──────────────────────────────┬──────────────────────────────────────┘
                                  ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                    MOTION CONTROL MODE SELECT                        │
    │  motor.controller = {torque, velocity, angle}                        │
    └──────────────────────────────┬───────────────────────────────────────┘
                                  ↓
            ┌─────────────────────┴──────────────────────┐
            │                                            │
        [TORQUE MODE]                              [VELOCITY MODE]
            │                                            │
    Direct: target → Iq                         target → PID_velocity
            │                                            ↓
            │                                     Velocity error
            │                                            ↓
            │                                    LPF_velocity (Tf=0.02)
            │                                            ↓
            │                                    PID (P=0.5, I=10, D=0.0001)
            │                                            ↓
            │                                    output_ramp (1000 A/s)
            │                                            ↓
            └─────────────────────┬──────────────────────┘
                                  │
                            [ANGLE MODE]
                                  │
                      target → P_angle (P=20, D=0.1)
                                  │
                            velocity_target
                                  │
                            PID_velocity
                                  │
                                  ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                      TORQUE COMMAND (Iq_target)                      │
    │  • Clamped to current_limit (10A)                                    │
    │  • Id_target usually = 0 (max efficiency)                            │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                       CURRENT SENSING FEEDBACK                       │
    │  1. ADC reads PA0, PA1, PA4 (triggered by TIM1)                      │
    │  2. DMA transfers to buffer                                          │
    │  3. Convert: (adc_val - CURRENT_VREF) / CURRENT_SENSE_GAIN           │
    │  4. Get: Ia, Ib, Ic (phase currents)                                 │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                      CLARKE TRANSFORM (abc → αβ)                     │
    │  I_alpha = Ia                                                        │
    │  I_beta = (Ia + 2*Ib) / √3                                           │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                    ENCODER ANGLE READING                             │
    │  1. I2C read from AS5600 (address 0x36, reg 0x0C)                    │
    │  2. Get raw angle (0-4095)                                           │
    │  3. Convert to radians: angle = raw * 2π / 4096                      │
    │  4. Calculate electrical angle: θ_elec = angle * pole_pairs          │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                      PARK TRANSFORM (αβ → dq)                        │
    │  I_d = I_alpha * cos(θ) + I_beta * sin(θ)    [Flux component]        │
    │  I_q = -I_alpha * sin(θ) + I_beta * cos(θ)   [Torque component]      │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                    LOW-PASS FILTER CURRENTS                          │
    │  I_q_filt = LPF_current_q(I_q)    [Tf = 0.01]                        │
    │  I_d_filt = LPF_current_d(I_d)    [Tf = 0.01]                        │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                      CURRENT PID CONTROLLERS                         │
    │  Q-axis: V_q = PID_current_q(Iq_target - I_q_filt)                   │
    │          P=3.0, I=300.0  → voltage command                           │
    │                                                                      │
    │  D-axis: V_d = PID_current_d(0 - I_d_filt)                           │
    │          P=3.0, I=300.0  → voltage command                           │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                    VOLTAGE LIMIT ENFORCEMENT                         │
    │  V_magnitude = √(V_q² + V_d²)                                        │
    │  if V_magnitude > voltage_limit (11V):                               │
    │      scale = voltage_limit / V_magnitude                             │
    │      V_q *= scale                                                    │
    │      V_d *= scale                                                    │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                   INVERSE PARK TRANSFORM (dq → αβ)                   │
    │  V_alpha = V_d * cos(θ) - V_q * sin(θ)                               │
    │  V_beta = V_d * sin(θ) + V_q * cos(θ)                                │ 
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                 INVERSE CLARKE TRANSFORM (αβ → abc)                  │
    │  V_a = V_alpha                                                       │
    │  V_b = -V_alpha/2 + (√3/2) * V_beta                                  │
    │  V_c = -V_alpha/2 - (√3/2) * V_beta                                  │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                    SVPWM (Space Vector PWM)                          │
    │  1. Find min/max: U_min = min(V_a, V_b, V_c)                         │
    │                    U_max = max(V_a, V_b, V_c)                        │
    │  2. Center: offset = (U_max + U_min) / 2                             │
    │  3. Shift: V_a -= offset, V_b -= offset, V_c -= offset               │
    │  4. Scale to duty: duty_a = (V_a/Vdc + 1) / 2                        │ 
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                    STM32 TIMER REGISTER UPDATE                       │
    │  TIM1->CCR1 = duty_a * 4095    [PA8 - Phase A PWM]                   │
    │  TIM1->CCR2 = duty_b * 4095    [PA9 - Phase B PWM]                   │
    │  TIM1->CCR3 = duty_c * 4095    [PA10 - Phase C PWM]                  │
    │  Frequency: 20kHz (50µs period)                                      │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                      MOSFET GATE DRIVERS (IR2104S)                   │
    │  High-side: Bootstrap capacitor charges during low-side ON           │
    │  Low-side: Direct drive from logic level                             │
    │  Deadtime: Prevents shoot-through (~1µs)                             │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                      POWER MOSFETS (IRFS4115PBF)                     │
    │  Switch 12V battery at 20kHz                                         │
    │  Create effective phase voltages through PWM averaging               │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                         BLDC MOTOR PHASES                            │
    │  3-phase AC currents (120° apart)                                    │
    │  Rotate magnetic field → Rotor follows                               │
    │  Torque ∝ Iq current                                                 │
    └──────────────────────────────┬───────────────────────────────────────┘
                                   ↓
    ┌──────────────────────────────────────────────────────────────────────┐
    │                      MECHANICAL OUTPUT                               │
    │  Shaft rotation → Encoder reads → Back to top of loop!               │
    │  Loop frequency: 1-10 kHz (loopFOC() call rate)                      │
    └──────────────────────────────────────────────────────────────────────┘

    TIMING BREAKDOWN:
      ├─ loopFOC():     Every 100-1000µs  (1-10 kHz)
      ├─ move():        Every 1-10ms      (100Hz-1kHz)
      ├─ Serial check:  Every loop        (~10kHz)
      └─ Safety check:  Every 50ms        (20 Hz)

    CONTROL HIERARCHY:
      Level 3 (Slowest):  Position → Velocity target
      Level 2 (Medium):   Velocity → Current target  
      Level 1 (Fastest):  Current → Voltage output
      Level 0 (Hardware): Voltage → PWM → MOSFETs
*/

// Current sensing calibration happens internally in motor.init() since skip_align is set to false
// Align Encoder and Motor
/* //uncomment (/*) if current sense calibration does not happen internaly
// Calibrate Current Sensing
Serial.println("[...] Calibrating current sensing...");
current_sense.calibrateOffsets(); 
*/
Serial.println("[...] Aligning encoder (motor will move)...");
motor.initFOC();
  
Serial.println("[✓] FOC Initialization Complete!");
Serial.println("================================");
Serial.println("Motor ready for operation");
Serial.println("Commands:");
Serial.println("  T<value> - Set torque (A)");
Serial.println("  V<value> - Set velocity (rad/s)");
Serial.println("  A<value> - Set angle (rad)");
Serial.println("  M<mode>  - Change mode (0=torque, 1=velocity, 2=angle)");
Serial.println("  S        - Stop motor");
Serial.println("================================\n");
digitalWrite(ledPin, LOW);
}

void loop() {
motor.loopFOC(); // This MUST be called as fast as possible (>1kHz recommended)
// FOC Algorithm Execution (Clarke/Park transforms, SVPWM)

motor.move(target_torque);   // This can be called at a lower rate (100Hz - 1kHz)
// Motion Control Loop (PID controllers)
int adc_value = analogRead(VBUS_sense);
VBUS_voltage = (adc_value / 4095.0) * 3.3 * VBUS_divider_ratio; //previously 0 now calculates Vin
process_serial_commands(); // Check for user input via Serial
if (millis() - last_print >= print_interval) {
    last_print = millis();
    printMotorStatus();
    Serial.print("Angle: ");
    Serial.print(motor.shaft_angle);
    Serial.print(" rad, Velocity: ");
    Serial.print(motor.shaft_velocity);
    Serial.print(" rad/s, Torque: ");
    Serial.print(target_torque);
    Serial.print(" A, Vbus: ");
    Serial.print(VBUS_voltage);
    Serial.println(" V");
  }
performSafetyChecks(); // Monitor voltage/current for safety
}

void process_serial_commands() {
  // Example: Read from Serial and process commands
  if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      // Add your command processing logic here
      if (command == "start") {
          // Handle start command
      } else if (command == "stop") {
          // Handle stop command
      }
  }
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'T': // Torque command
      case 't':
        target_torque = Serial.parseFloat();
        motor.controller = MotionControlType::torque;
        Serial.print("→ Torque mode: ");
        Serial.print(target_torque);
        Serial.println(" A");
        break;
        
      case 'V': // Velocity command
      case 'v':
        target_velocity = Serial.parseFloat();
        motor.controller = MotionControlType::velocity;
        motor.target = target_velocity;
        Serial.print("→ Velocity mode: ");
        Serial.print(target_velocity);
        Serial.println(" rad/s");
        break;
        
      case 'A': // Angle command
      case 'a':
        target_angle = Serial.parseFloat();
        motor.controller = MotionControlType::angle;
        motor.target = target_angle;
        Serial.print("→ Angle mode: ");
        Serial.print(target_angle);
        Serial.println(" rad");
        break;
        
      case 'M': // Mode change
      case 'm': {
        int mode = Serial.parseInt();
        if (mode == 0) {
          motor.controller = MotionControlType::torque;
          Serial.println("→ Mode: Torque control");
        } else if (mode == 1) {
          motor.controller = MotionControlType::velocity;
          Serial.println("→ Mode: Velocity control");
        } else if (mode == 2) {
          motor.controller = MotionControlType::angle;
          Serial.println("→ Mode: Position control");
        }
        break;
      }
        
      case 'S': // Stop
      case 's':
        target_torque = 0;
        target_velocity = 0;
        motor.target = 0;
        Serial.println("→ Motor STOPPED");
        break;
        
      case 'I': // Info
      case 'i':
        printDetailedInfo();
        break;
        
      default:
        // Ignore unknown commands
        break;
    }
    
    // Clear remaining serial buffer
    while (Serial.available()) Serial.read();
  }
}

// Print motor status to serial (diagnostics)

void printMotorStatus() {
  Serial.print("Angle: ");
  Serial.print(motor.shaft_angle, 2);
  Serial.print(" rad | Vel: ");
  Serial.print(motor.shaft_velocity, 2);
  Serial.print(" rad/s | Iq: ");
  Serial.print(motor.current.q, 2);
  Serial.print(" A | Id: ");
  Serial.print(motor.current.d, 2);
  Serial.print(" A | Vbus: ");
  Serial.print(VBUS_voltage, 2);
  Serial.println(" V");
}

/**
 * Print detailed system information
 */
void printDetailedInfo() {
  Serial.println("\n========== MOTOR STATUS ==========");
  Serial.print("Electrical Angle: ");
  Serial.print(motor.electrical_angle, 3);
  Serial.println(" rad");
  
  Serial.print("Shaft Angle: ");
  Serial.print(motor.shaft_angle, 3);
  Serial.println(" rad");
  
  Serial.print("Shaft Velocity: ");
  Serial.print(motor.shaft_velocity, 2);
  Serial.println(" rad/s");
  
  Serial.print("Target: ");
  Serial.println(motor.target, 3);
  
  Serial.println("\n--- Currents (FOC) ---");
  Serial.print("Iq (torque): ");
  Serial.print(motor.current.q, 3);
  Serial.println(" A");
  Serial.print("Id (flux): ");
  Serial.print(motor.current.d, 3);
  Serial.println(" A");
  
  Serial.println("\n--- Phase Currents ---");
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  Serial.print("Ia: ");
  Serial.print(currents.a, 3);
  Serial.println(" A");
  Serial.print("Ib: ");
  Serial.print(currents.b, 3);
  Serial.println(" A");
  Serial.print("Ic: ");
  Serial.print(currents.c, 3);
  Serial.println(" A");
  
  Serial.println("\n--- Voltages ---");
  Serial.print("Vq: ");
  Serial.print(motor.voltage.q, 2);
  Serial.println(" V");
  Serial.print("Vd: ");
  Serial.print(motor.voltage.d, 2);
  Serial.println(" V");
  Serial.print("Vbus: ");
  Serial.print(VBUS_voltage, 2);
  Serial.println(" V");
  
  Serial.println("==================================\n");
}

// Safety checks (overvoltage, overcurrent, etc.)

void performSafetyChecks() {
  static unsigned long last_check = 0;
  
  // Check every 50ms
  if (millis() - last_check < 50) return;
  last_check = millis();
  
  bool error = false;
  
  // Overvoltage protection
  if (VBUS_voltage > battery_voltage * 1.2f) {
    Serial.println("[!] OVERVOLTAGE DETECTED! Stopping motor.");
    error = true;
  }
  
  // Undervoltage protection  
  if (VBUS_voltage < battery_voltage * 0.7f) {
    Serial.println("[!] UNDERVOLTAGE DETECTED! Stopping motor.");
    error = true;
  }
  
  // Overcurrent protection (read actual currents)
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  float i_total = sqrt(currents.a * currents.a + currents.b * currents.b + currents.c * currents.c);
  
  // Check against software limit
  if (i_total > current_max_limit * 1.2f) {
    Serial.println("[!] SOFTWARE OVERCURRENT! Stopping motor.");
    error = true;
  }
  
  // Additional check: Verify we're not exceeding hardware safe limit
  #if !CURRENT_DIVIDER_USED
    if (i_total > MAX_SAFE_CURRENT) {
      Serial.println("[!] CRITICAL: Current exceeds ADC safe limit (>12A)!");
      Serial.println("    Add voltage divider or reduce current limit!");
      error = true;
    }
  #endif
  
  // If error detected, stop motor
  if (error) {
    motor.disable();
    target_torque = 0;
    target_velocity = 0;
    digitalWrite(ledPin, HIGH); // Turn on LED to indicate error
    delay(5000); // Wait 5 seconds before re-enabling
    motor.enable();
    digitalWrite(ledPin, LOW);
  }
}

void testVbusDivider() {
  // Read battery voltage
  int adc_val = analogRead(VBUS_sense);
  float adc_voltage = (adc_val / 4095.0) * 3.3;
  float calculated_vbus = adc_voltage * VBUS_divider_ratio;
  
  Serial.println("=== Vbus Divider Test ===");
  Serial.print("ADC reads: ");
  Serial.print(adc_voltage, 3);
  Serial.println(" V");
  
  Serial.print("Calculated battery: ");
  Serial.print(calculated_vbus, 2);
  Serial.println(" V");
  
  // Measure with multimeter and compare
  Serial.println("Measure battery with multimeter and verify!");
  
  // If calculated value is wrong, adjust VBUS_DIVIDER_RATIO
  if (calculated_vbus < 10.0f || calculated_vbus > 14.0f) {
    Serial.println("⚠️ Voltage reading seems wrong!");
    Serial.println("   1. Measure battery with multimeter (actual voltage)");
    Serial.println("   2. Note ADC voltage from above");
    Serial.println("   3. New ratio = actual / ADC_voltage");
    Serial.println("   4. Update VBUS_DIVIDER_RATIO in code");
  }
}

/*
// 9.1 ESP32 Communication via UART (Future)
void setupESP32Communication() {
  Serial2.setRx(PA3);
  Serial2.setTx(PA2);
  Serial2.begin(115200);
}

// 9.2 Field Weakening for High Speed
void enableFieldWeakening() {
  motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  motor.current_limit = CURRENT_LIMIT;
  // SimpleFOC will automatically apply field weakening
}

// 9.3 Sensor Fusion (if adding second sensor)
void setupSensorFusion() {
  // Combine encoder with Hall sensors or back-EMF
  motor.linkSensor(&sensor);
  // motor.sensor_direction = Direction::CCW; // If direction is reversed
}
*/
