/* 26-11-22 v3.05
 * For the Nerf Comunity, open progect
 * 
 * Brushless | Solenoid Pusher | Dual Trigger | I2C OLED 128 x 64 display | Rotary Encoder | Serial Configuration
 * 
 * Origina Code from:
 *------------------------------------------------------------------------ 
 * (c) 2019,2020 Michael Ireland / Ireland Software
 * 
 * Creative Commons: Attribution, Non-Commercial
 * Free for Non-Commercial use
 * -----------------------------------------------------------------------
 * 
 * v1.0 - Initial release based on Swordfish firing logic
 * v2.0 - Updated for v4 Firing logic.
 * v2.1 - Increased configuration options to include:
 *        * Select Fire Switch reassignment
 *        * Safe, Ramp, ?Binary, and Zerg modes
 *        * User-defined battery calibration set-points
 *        * Configurable motor delay and bias
 * v3.00 - Richard porting to Nano pcb       
 * v3.03 - Low Battery Alarm, D13, needs mosfet BST72 to drive external LED. D13 is also on board led         
 *       - Version number in serial menu
 *       - Battery Offset adjustment via serial menu #BO-pxx$ and writes to EEprom
 *       - Bug Fix burst mode setting not working #BS-03# 
 *       - Bug Fix range testing in serial interface
 * v3.04 - Firing LED, D2, needs mosfet BST72 to drive external LED.      
 * v3.05 - Rename and mode to become Thors Hammer, History and versions keept for my referance
 *        
 * 
 * Notes:
 *   It would be prudent to use a HC-05 bluetooth module. The back EMF from these high current devises can kill a USB port and laptop.
 *   The Bluetooth HC05 goes on the TX/D1 & RX/D0 pins and will allow a blue tooth laptop to connect, via a serial terminal( Arduino IDE).
 *   This porovides isolated testing environment (your laptop does not make magic smoke).
 *   You will need to use a USB cable for sketch downloads. Just power down HC05 when a USB connection is required.
 *   
 *   
 * How it works:-   
 *   After program download,connect to serial port, via 'Serial Monitor' at 9600bps and send ? to display configuration commands. 
 *   
 *  Command list:

    ? - will display current settings and blaster parameters.

    #QD$ Queary Device, returns CB-OK
    #QV$ Queary Battery voltage

    #FP-XXX$ - Fulll Trigger Power setting where XXX = 030 to 100, Default = 50
    #BS-XX$ - Burst size where XX = 01 - 99, Default = 2
    #BR-XX$ - Burst Rate of Fire where XX = desired DPS (99 = max). Pusher may not achieve this in reality, Default = 0
    #FR-XX$ - Full Auto Rate of Fire where XX = desired DPS (99 = max). Pusher may not achieve this in reality, Default = 0
    #PH-XXX$ - Solenoid full battery pulse time where XXX = 001 - 500 milliseconds, Default = 35
    #PL-XXX$ - Solenoid low battery pulse time where XXX = 001 - 500 milliseconds, Default = 45
    #SR-XXX$ - Solenoid retract time time where XXX = 001 - 500 milliseconds, Default = 45
    #MA-XXXX$ - Motor ramp up time where XXXX = 0000 - 5000 milliseconds, Default = 0
    #MD-XXXX$ - Motor ramp down time where XXXX = 0000 - 5000 milliseconds (n.b. setting this too low will cause flywheel melting), Default = 4000
    #UD-XXXX$ - Motor on dwell time where XXXX = 0000 - 5000 milliseconds (this allows more time to stabalise the motors before firing), Default = 0
    #DD-XXXX$ - Motor off dwell time where XXXX = 0000 - 5000 milliseconds (this holds the motors running after a shot for snappy follow-ups), Default = 200
    #S1-X$ - Select Fire Position 1, Default = 0
    #S2-x$ - Select Fire Position 2, Default = 1
    #S3-x$ - Select Fire Position 3, Default = 2
    #RD-xx$ - Ramp DPS 02-09, Default = 6
    #RT-XX$ - Ramp Threshold DPS 02-09, Default = 6
    #SH-XXX$ - Solenoid High Voltage xxx =070 - 180 and xxx is measured voltage * 10 - use #QV$, Default = 168
    #SL-XXX$ - Solenoid Low Voltage xxx = 070 - 180 and xxx is measured voltage * 10 - use #QV$, Default = 130
    #ML-XXX$ - Motor Spin-up Lag xxx = 000 - 999 milliseconds, Default = 220
    #MB-XXX$ - Motor Compensation Bias xxx = 000 - 999 milliseconds, Default = 75
    #BO-pxx$ - Battery Offset (p= + or -)xx = -2.0 - +2.0 volts, entered as #BO--20$  to  #BO-+20$, Default = -02

 *  
 *================================================================
 *  Richards Notes, from porting to Arduino Nano 17-4-22
 *=============================================================== 
 * 
 * NOTE: It is not the best idea to do soleniode firing with laptop connected, damage can ocure.
 *       Recommend 5v psu (buck) onto Arduino with diode protection and a capacitor. This capacitor goes on the +5v out, not Vin
 *       Unplug laptop and connect soleniod to test, else leave soleniod disconnected, code will serialprint bang at end of firing cycle, comment out later
 * 
 * 
 * BATTERY VOLTAGE
 *     A7 analog input is used to monitor battery voltage via a resistor devider, 47K/10K (preferd values). Published and avaliable Ardunio device is 30K/7K (try get these values).
 *   
 *     Added new calibration line for Ardreno device with 30K/7K5 resistors (modual), search 'ProcessBatteryMonitor', comment out the line your not using.
 *   
 *     I made my own resistor devider, the values are 'preferd resistor' values = easy to get. 47K/10K
 *     
 *   Battery Calibration
 *     Check caliration by attaching a fully charged battery, and comparing the blaster voltage (Command #QV$) to a voltmeter.
 *     then a discharged one, a 9V battery and get the blalaster low reading and a voltemeter reading
 *      
 *     work out the differance between blaster and voltmeter for each set of readings, they should be the same or very similar. this is your Battery Offset
 *   
 *     #define BATTERY_OFFSET is where this Offset value goes. This can be changed via the serial menu interface
 *   
 *     BATTERY LOW ALARM, D13 on board LED, SYSTEM_MODE=1 Lowbattery, system will stop firing. D13 led will com on.
 *     This OP can be used to drive an audible AC Buzzer, via a transistor driver. You can bye a smalll PCB asmebed
 *     coment out appropriate code for buzzer or led (default buzzer).
 *   
 *     Driving LEDS/buzzer directly off these OP's can result in dead Nano, Use Ardunio AC buzzer pcb, has transistor driver on board. Connect 5v to your PSU
 *   
 *     If there is a low voltage signal on this A7 input, battery monitoring will set SYSTEM_MODE_LOWBAT=1, disabling the system, D13 LED/Buzzer will come on.
 *     Search SYSTEM_MODE_LOWBAT
 *     NOTHING works in SYSTEM MODE = 1 (LOWBAT) including changing values via serial menu
 *   
 *   3S/4S
 *     Max and Min values for a 3S and 4S battery are different and acomidated by constants in the code, and by changing 4S to 3S at appropriate lines you can change battery type 
 *     The code is set for 3S. (4S commented out). 4S is recommended
 * 
 * SOLENOID TUNING - Done after Battery Calibration
 *    In order to get a good solenoid tune, you need to have 3 things: Push-out time while LiPO is fully charged, push-out time while LiPO is depleted, and retract time. 
 *    It is advisable to disconnect the ESC signal wire so that you don't have the motors draining power, and affecting your tune.
 *
 *    Focus on the return rate first. Set PH and PL to 150 milliseconds (#PH-150$ and #PL-150$). Set Auto ROF to 99 (#FR-99$).
 *    Set select fire switch to auto. Start at 100ms (#SR-100$) to see if the solenoid is fully retracting between shots. 
 *    You might be able to use your finger to feel it. Increase or reduce this number until you have a consistent and reliable retraction.
 *    n.b. solenoids will retract slower with a fully loaded magazine, so once you have found the number, add a bit extra to it (like 10ms) 
 *   
 *    Then focus on the fully charged pulse time. Fully charge your LiPO. Start reducing both PH and PL (keep the numbers the same) until it's quick, but not so quick as it's short-stroking.
 *    Use your finger up the magwell to determine how hard the solenoid is hitting it. Keep reducing the time until it starts to feel weaker, then back off. 
 *    Add a bit extra to it for safety margin. Write down this value.
 *
 *    For the low battery discharge time, bring the LiPO to storage charge. Then do the same thing as above.
 *
 *    Once you have both the PH and PL value, key them in and test the solenoid over a range of battery values. Reducing the timing will make it faster, at the expense of reliability.
 *
 * BLUE TOOTH HC-05 device intergration
 *    The HC-05 Bluetooth device, use BT 2.0 this does not work with iPhones. iPhone uses BLE 4.0, your iPhone will not see this device
 *    The device works on 3.3 volts. So the TX line from the Arduino needs to go from TTL level to 3.3v. 
 *    A 1K/2K2 voltage divider is used. Arduino TX(D1) to the 1K, join of 1k/2k2 to HC-05 RX pin, other end of 2k2 to GND 
 *    Ardunio RX(D0) to HC-05 TX Pin
 *    NOTE the TX/RX pins are also used by the USB connection, you can not have the HC-05 powered while using the USB via Arduino IDE, to get a Bluetooth connection
 *    You need to provide an independant 5v supply to the Arduino, and unplug the USB cable, for this to work.
 *    Dispite verious attempts I have been unable to talk to the HC-05 in AT mode. This would only be nessicary if using more than one HC-05, ie more than one project. 
 *    You then need to chnage there name in AT mode else, you will need to remove the BT device in you windows BT manager and refind it, to connect to diff project.
 *    
 *    To program HC-05 in AT mode Documents would indicate... Unplug HC-05, connect USB and load empty sketch to Nano board. 
 *    Power up HC-05 while holding mode button down, after powering Nano.
 *    Then open IDE serial window at 38400bps. Send AT, should get back OK. See HC-05 documentation for AT commands
 *    
 *    The serial.begin line in this code needs to be set to 9600 for the HC-05 to work in data slave mode, default 9600bps, allowing the Blaster serial interface to work
 *    Procedure...
 *    Connect HC-05, wires do not connect power wire to it
 *    connect an indipendent 5v supply ie your 4S ot equvilent, via 5v regulator to +5v not Vin
 *    Connect USB and program, download this firmware, confirm operation by sending a ? to get the menu display back, at 9600bps
 *    Disconnect USB, and power up HC-05
 *    Via laptop find your BT settings normaly task bar bottom right
 *    find the HC-05 blue tooth and pair it. Pairing code is 1234. HC-05 led will flash twice every 2 sec
 *    Go back to Arduino IDE, use tools tab to find the com port of the BT HC-05 and select it 
 *    open serial window, and send ?, should have the menu up on your screen, good to go....
 *    HC-05 LEDS
 *    - Blink once in 2 sec: Modual in AT command mode
 *    - Repeted Blinking: Waiting for connection
 *    - Blink twice in 1 sec: Connection susesfull, in data mode
 *     
 * - serialprint Bang!! Enabled so can have solioide disconected while testing. Disable print line later, Search Bang!!
 * - Changing sel fire via serial interface does not take affect until the physical select fire switch is moved.
 * - The Firing LED use the tone function. this function interfaresc with PWM on pin 3 and 11. So can not be used with these pins
 * 
 * TO DO:-
 * 
 * Use existing fire select interupt to read encoder 
 * Get Oled working
 * 
 * 
 * 
 * 
 */

#include <EEPROM.h>

#define VERSION 3.05

//-------------------------------------------------------------------------------------------
// BIOS Def
//-------------------------------------------------------------------------------------------
#define PIN_ESC 10           //      OP D10 Flywheels ESC stage 1 signal
#define PIN_ESC_STAGE2 9     //      OP D9 Flywheels ESC stage 2 signal

#define PIN_PUSHER_RUN 5     //      OP D5 to Solenoide mosfet

//#define PIN_PUSHER_STOP 15   //      OP D15/A1 This is to do with a Narfduino H Bridge, NOT USED ON NANO part of H Bridge on Narfduino

#define PIN_LOWBAT 13        //      OP D13 Existing on board LED. Used as Low Battery indicator, Can connect AC buzzer, via a transistor, module avaliable
#define PIN_FIRING 2         //      OP D2 Firing LED, just for fun, drive via mosfet BST72

#define PIN_REV_SW_FRONT 3   //      IP D3 Front SW on Trigger asemnbly, NO & COM 
#define PIN_TRIGGER 16       //      IP D16/A2/PC2 NOTE: Use NC and COM on switch. 

#define PIN_ENCODER_SW 8     //      IP D8
#define PIN_ENCODER_DT 7     //      IP D7
#define PIN_ENCODER_CLK 6    //      OP D6

#define PIN_SELECT_FIRE_A 12 //      IP D12 Selectfire binaary IP A. Use On/Off/On switch. This is interupt driven, change to encoder
#define PIN_SELECT_FIRE_B 14 //      IP A0/D14 Selectfire binary IP B.  Nano HW PC0

#define PIN_SDA 4                //       IP A4/D18 I2C interface to OLED display
#define PIN_SCL 5                //       OP A5/D19 I2C Interface to OLED display

#define PIN_BATT_MON A7      //      IP A7/ADC7 Analog battery reading, Resistor devideder 47K/10K 

//-------------------------------------------------------------------------------------------------------------------------

// Pin macros
/*
 * Read pin state
 * 
 * Valid for:
 * PD6/D6 - Trigger - Orange/Black            FIRE_T
 * PD7/D7 - Mag Switch - Blue/Blue            MAG_S
 * PD3/INT1/D3 - Pusher Rear - Green/Green    PUSH_R
 * PD2/INT0/D2 - Pusher Front - Yellow/Yellow PUSH_F
 * PC3/D17 - Display Interrupt                DISP_INT
 */
#define GET_FIRE_T ((PINC & 0b00000100) ? HIGH : LOW )
#define GET_SEL_FIRE_A ((PINB & 0b00010000) ? HIGH : LOW )
#define GET_SEL_FIRE_B ((PINC & 0b00000001) ? HIGH : LOW )  // Nano HW

/*
 * Set Pin State
 * 
 * Valid for:
 * PC1 P-BRAKE
 * PD5 P-RUN
 * PB1/OC1A F-RUN
 */
#define SET_P_BRAKE_FULL_ON (PORTC |= 0b00000010)
#define SET_P_BRAKE_FULL_OFF (PORTC &= 0b11111101)
#define SET_P_RUN_FULL_ON (PORTD |= 0b00100000)
#define SET_P_RUN_FULL_OFF (PORTD &= 0b11011111)
/*
 * Get Fet State
 */
#define GET_BRIDGE_ON_STATE ((PORTD & 0b00100000))
#define GET_BRIDGE_BRAKE_STATE ((PORTC & 0b00000010))


// Motor Controls
#define MOTOR_SPINUP_LAG 220 // How long we give the motors before we know that have spun up.
#define MOTOR_COMPENSATION_BIAS 75
#define MOTOR_MAX_SPEED 2000
int MotorSpinupLag = MOTOR_SPINUP_LAG;
int MotorCompensationBias = MOTOR_COMPENSATION_BIAS;
int MaxMotorSpeed = MOTOR_MAX_SPEED;
int DecelerateTime = 4000;
int AccelerateTime = 0;
long MotorRampUpPerMS = 0;
long MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
byte SetMaxSpeed = 100; // in percent.
unsigned long TimeLastMotorSpeedChanged = 0; 
#define COMMAND_REV_NONE 0
#define COMMAND_REV_FULL 1
byte CommandRev = COMMAND_REV_NONE;
byte PrevCommandRev = COMMAND_REV_NONE;
byte MotorSpeedFull = 80;
int MotorStartDwellTime = 0; // Use this to hold the rev
int MotorStopDwellTime = 0; // Use this to hold the rev


// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
#define RepollInterval 250 // Just check the buttons ever xxx just in case we missed an interrupt.
// For ISR use
volatile bool TriggerChanged = false;
volatile bool SelFireAChanged = false;
volatile bool SelFireBChanged = false;
#define BTN_LOW 0
#define BTN_HIGH 1
#define BTN_ROSE 2
#define BTN_FELL 3
byte TriggerButtonState = BTN_HIGH;
byte SelFireAButtonState = BTN_HIGH;
byte SelFireBButtonState = BTN_HIGH;
volatile byte LastPINB = 0; // Preload with PINB Prior to events happening
volatile byte LastPINC = 0; // Preload with PINC Prior to events happening


// Pusher Controls
byte TargetDPS = 0; // Used to calculate the next adjustment into
byte TargetDPSAuto = 0; // Used to store the Auto adjustment
byte TargetDPSBurst = 0; // Used to store the Burst adjustment
volatile bool RequestStop = false;
bool PusherRunning = false;
// Pusher 4S
#define PULSE_ON_TIME_HI_4S 35   //50
#define PULSE_ON_TIME_LO_4S 45   //90
#define PULSE_RETRACT_TIME_4S 25   //45
int PulseOnTime;
int PulseOnTimeHigh;
int PulseOnTimeLow;
int PulseRetractTime;
#define SOLENOID_CYCLE_IDLE 0
#define SOLENOID_CYCLE_PULSE 1
#define SOLENOID_CYCLE_RETRACT 2
#define SOLENOID_CYCLE_COOLDOWN 3
byte CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
unsigned long LastSolenoidCycleStarted = 0;
int SolenoidHighVoltage = 1;
int SolenoidLowVoltage = 1;


// Firing Controls
int DartsToFire = 0;
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_BURST 1
#define FIRE_MODE_AUTO 2
#define FIRE_MODE_AUTO_LASTSHOT 3
#define FIRE_MODE_BINARY 4
#define FIRE_MODE_RAMPED 5
#define FIRE_MODE_ZERG 6
#define FIRE_MODE_SAFE 7
#define FIRE_MODE_IDLE 8
byte CurrentFireMode = FIRE_MODE_IDLE;
byte ProcessingFireMode = FIRE_MODE_SINGLE;
bool RunFiringSequence = false;
bool StartNewFiringSequence = false;
bool ShutdownFiringSequence = false;
byte BurstSize = 2;
bool ExecuteFiring = false; // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0; // Calculated to lower ROF
long ShotsToFire = 0; // Number of shots in the queue
unsigned long LastShot = 0; // When the last shot took place.
bool FiringLastShot = false; // We have queued up our last shot. Indicates pusher decelleration
unsigned long LastTriggerPull = 0; // Used to store the last trigger pull for the Ramping mode
unsigned int RampFrequency = 333; // The number of milliseconds since the last trigger pull to activate / sustain ramping
byte CurrentRampCount = 0; // 
byte RampThreshold = 5;
bool IsRamping = false;
byte RampDPS = 6; // Used mostly for a placeholder...


// User interface
#define BTN_SINGLE 0
#define BTN_BURST 1
#define BTN_AUTO 2
#define BTN_BINARY 3
#define BTN_RAMP 4
#define BTN_ZERG 5
#define BTN_SAFE 6
#define BTN_NOTHING 7
#define BTN_IDX_SF1 0
#define BTN_IDX_SF2 1
#define BTN_IDX_SF3 2
byte ButtonActions[3] = {BTN_SINGLE, BTN_BURST, BTN_AUTO};


// System
#define SYSTEM_MODE_NORMAL 0
#define SYSTEM_MODE_LOWBATT 1
byte SystemMode = SYSTEM_MODE_NORMAL;


// Battery Controls
#define BATTERY_3S_MIN 9.6
#define BATTERY_3S_MAX 13.0
#define BATTERY_4S_MIN 13.2 // 13.2
#define BATTERY_4S_MAX 16.8
#define BATTERY_OFFSET -0.2                // Offset Adjustment for Battery calibration hard code  <====================
float BatteryOffset = BATTERY_OFFSET;      // asign to a floating point veriable so can be adjusted in serial menu
float BatteryCurrentVoltage = 99.0;
float BatteryMaxVoltage = BATTERY_3S_MAX;
float BatteryMinVoltage = BATTERY_3S_MIN;
bool BatteryFlat = false;
volatile bool ADCRead = false;
volatile unsigned long ADCValue = 0;
int BatteryPercent = 100;


// Profile Storage... I really should transact out of these.. But it allows for more flexability in a remote control scenario
byte CurrentProfile = 0;
struct ProfileDef
{
  byte MotorSpeedFull = 50;
  byte TargetDPSBurst = 0;
  byte TargetDPSAuto = 0;
  byte BurstSize = 2;
  int AccelerateTime = 0;
  int DecelerateTime = 4000;
  int MotorStartDwellTime = 0;
  int MotorStopDwellTime = 200;
  int PulseOnTimeHigh = 35;
  int PulseOnTimeLow = 45;
  int PulseRetractTime = 45;
  byte BtnSF1 = BTN_SINGLE;
  byte BtnSF2 = BTN_BURST;
  byte BtnSF3 = BTN_AUTO;  
  byte RampDPS = 6;
  byte RampThreshold = 6;
  int SolenoidHighVoltage = (int)(BATTERY_4S_MAX*10.0);
  int SolenoidLowVoltage = (int)(BATTERY_4S_MIN*10.0);
};
ProfileDef Profiles[1];


// EEPROM addresses
#define ADDR_MSF 0
#define ADDR_ROF_BURST 1
#define ADDR_ROF_AUTO 2
#define ADDR_BURST 3
#define ADDR_ACCEL 4
#define ADDR_DECEL 6
#define ADDR_STARTD 8
#define ADDR_STOPD 10
#define ADDR_PULSE_HIGH 12
#define ADDR_PULSE_LOW 14
#define ADDR_PULSE_RETRACT 16
#define ADDR_BTN_SF1 18
#define ADDR_BTN_SF2 19
#define ADDR_BTN_SF3 20
#define ADDR_RAMP_DPS 21
#define ADDR_RAMP_THRESHOLD 22
#define ADDR_RAMP_SOL_HI_VOLT 23
#define ADDR_RAMP_SOL_LO_VOLT 25
#define ADDR_PROA_BASE 0
#define ADDR_SECTORL 27
// Start general settings at address 60
#define ADDR_MOTOR_SPINUP_LAG 60
#define ADDR_MOTOR_COMPENSATION_BIAS 62
#define ADDR_BATTERY_OFFSET 64              //rn


// Serial Comms
#define SERIAL_INPUT_BUFFER_MAX 25
char SerialInputBuffer[SERIAL_INPUT_BUFFER_MAX];


//-------------------------------------------------- SETUP -----------------------------------------------------------------
void setup() {
  Serial.begin(9600); // This needs to be at 9600 if using the HC-05 Bluetooth device else can be 57600. Close terminal mode before progam download.
  Serial.println( F("Booting") );

  unsigned long BootStart = millis();

  //Use D13 built in LED to indicate low battery. a LED or Buzzer can be connected via mosfet
  //RN
  pinMode(PIN_LOWBAT, OUTPUT);
  digitalWrite(PIN_LOWBAT, LOW);    // turn the LOW Battery LED/Buzzer off by making the voltage LOW
  pinMode(PIN_FIRING, OUTPUT);
  noTone(PIN_FIRING);               // turn firing led off
  
  // PCINT Inputs
  // PB - PCINT Vector 0
  pinMode(PIN_SELECT_FIRE_A, INPUT_PULLUP);
  PCMSK0 |= _BV(PCINT4);
  pinMode(PIN_SELECT_FIRE_B, INPUT_PULLUP);
  //PCMSK0 |= _BV(PCINT5); // this is for Narduino HW
  PCMSK0 |= _BV(PCINT8); // Nano HW
  
  // PC - PCINT Vector 1
  pinMode(PIN_TRIGGER, INPUT_PULLUP);
  PCMSK1 |= _BV(PCINT10);

  LastPINB = PINB; // Ensure that we pre-load the LastPINC register to capture changes from NOW...
  LastPINC = PINC; // Ensure that we pre-load the LastPINC register to capture changes from NOW...
  PCICR |=  _BV(PCIE0) |_BV(PCIE1); // Activates control register for PCINT vector 1 & 2

  // Setup Motor Outputs
  Serial.println( F("Configuring PWM Ports") );
  pinMode( PIN_ESC, OUTPUT );
  PORTD &= 0b11011111;
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1B1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;
  UpdatePWM( 1000 );

  // Setup Pusher Outputs
  Serial.println( F("Configuring Pusher FET") );
  pinMode( PIN_PUSHER_RUN, OUTPUT );
  SET_P_RUN_FULL_OFF;
//  pinMode( PIN_PUSHER_STOP, OUTPUT );
  SET_P_BRAKE_FULL_OFF;    

  // Calculate the motor ramp rates
  CalculateRampRates(); 

  // Set the system mode to normal
  SystemMode = SYSTEM_MODE_NORMAL;

  // Setup the ADC
  ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX1) | (1 << MUX2); // Use AVCC as the aref; Set MUX to ADC7
  ADCSRA = 0;
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // Turn on ADC, Interrupt enable, Prescalar to 128
  ADCSRB = 0;
  // SFIOR has ADTS0-2 already set to 0 for free-running mode.
  // Start ADC running
  ADCSRA |= (1 << ADSC);
  
  // Pre-charge the battery indicator
  for( byte c = 0; c < 8; c++ )
  {
    ProcessBatteryMonitor(); // Check battery voltage occasionally
    delay( 550 );
  }

  // Wait for the sync - 2 seconds, but run the button debouncer to capture a trigger down state for EEPROM reset
  while( millis() - BootStart < 5000 )
  {
    ProcessDebouncing();
    delay( 10 );
  }

  // Load and apply configuration
  LoadEEPROM();
  UpdateProfile();

  // Now wait until the trigger is high
  Serial.println( F("Waiting for trigger safety") );
  while( (TriggerButtonState == BTN_HIGH) || (TriggerButtonState == BTN_ROSE) )
  {
    ProcessDebouncing();
    delay(10);
  }
  delay(10);
 
  Serial.println( F("Booted") );  
}

//------------------------------------- LOAD EEPROM Subroutine---------------------------------------------
void LoadEEPROM()
{
  // Just dump the EEPROM into SRAM and worry about it later... Coz I'm a lazy fuck.
  byte* ProfileBaseAddress = &Profiles[0].MotorSpeedFull;
  for( byte c = 0; c < ADDR_SECTORL; c++ )
  {
    *(ProfileBaseAddress + c) = EEPROM.read( c + ADDR_PROA_BASE );
  }

  EEPROM.get( ADDR_MOTOR_SPINUP_LAG, MotorSpinupLag );
  EEPROM.get( ADDR_MOTOR_COMPENSATION_BIAS, MotorCompensationBias );
  EEPROM.get( ADDR_BATTERY_OFFSET, BatteryOffset );                                  //rn

  // Check for validity of data
  bool CorruptData = false;
  #define BTN_MAX 7
  if( (Profiles[0].MotorSpeedFull < 30) || (Profiles[0].MotorSpeedFull > 100) ) CorruptData = true;
  if( (Profiles[0].TargetDPSBurst < 0) || (Profiles[0].TargetDPSBurst > 99) ) CorruptData = true;
  if( (Profiles[0].TargetDPSAuto < 0) || (Profiles[0].TargetDPSAuto > 99) ) CorruptData = true;
  if( (Profiles[0].BurstSize < 2) || (Profiles[0].BurstSize > 99) ) CorruptData = true;
  if( (Profiles[0].AccelerateTime < 0) || (Profiles[0].AccelerateTime > 5000) ) CorruptData = true;
  if( (Profiles[0].DecelerateTime < 0) || (Profiles[0].DecelerateTime > 5000) ) CorruptData = true;
  if( (Profiles[0].MotorStartDwellTime < 0) || (Profiles[0].MotorStartDwellTime > 5000) ) CorruptData = true;
  if( (Profiles[0].MotorStopDwellTime < 0) || (Profiles[0].MotorStopDwellTime > 5000) ) CorruptData = true;
  if( (Profiles[0].BtnSF1 < 0) || (Profiles[0].BtnSF1 > BTN_MAX) ) CorruptData = true;
  if( (Profiles[0].BtnSF2 < 0) || (Profiles[0].BtnSF2 > BTN_MAX) ) CorruptData = true;
  if( (Profiles[0].BtnSF3 < 0) || (Profiles[0].BtnSF3 > BTN_MAX) ) CorruptData = true;
  if( (Profiles[0].RampThreshold < 2) || (Profiles[0].RampThreshold >= 10) ) CorruptData = true;
  if( (Profiles[0].RampDPS < 2) || (Profiles[0].RampDPS >= 10) ) CorruptData = true;
  if( (Profiles[0].SolenoidHighVoltage < 70) || (Profiles[0].SolenoidHighVoltage >= 180) ) CorruptData = true;
  if( (Profiles[0].SolenoidLowVoltage < 70) || (Profiles[0].SolenoidLowVoltage >= 180) ) CorruptData = true;  

  if( (MotorSpinupLag < 1) || (MotorSpinupLag > 999) ) CorruptData = true;
  if( (MotorCompensationBias < 0) || (MotorCompensationBias > 100) ) CorruptData = true;
  if( (BatteryOffset < -20) || (BatteryOffset > 20) ) CorruptData = true;                   //rn
  
  // Data is not valid, or the trigger was held on power-on for a reset
  if( (TriggerButtonState == BTN_HIGH) || CorruptData )
  {
    Serial.println( F("Something wrong with EEPROM or held trigger while booting") );

    // Create a fresh profile to use as a template
    ProfileDef TemplateProfile;
    
    // Just dump it over the old ones. If I was less lazy, i'd use memcpy :p
    byte* ProfileBaseAddress = &Profiles[0].MotorSpeedFull;
    byte* TemplateBaseAddress = &TemplateProfile.MotorSpeedFull;
    for( byte c = 0; c < ADDR_SECTORL; c++ )
    {
      *(ProfileBaseAddress + c) = *(TemplateBaseAddress + c);
    }
    ProfileBaseAddress = &Profiles[1].MotorSpeedFull;
    
    // And flash it back to EEPROM
    ProfileBaseAddress = &Profiles[0].MotorSpeedFull;
    for( byte c = 0; c < ADDR_SECTORL; c++ )
    {
      EEPROM.write( c + ADDR_PROA_BASE, *(ProfileBaseAddress + c) );
    }
    
    MotorSpinupLag = MOTOR_SPINUP_LAG;
    EEPROM.put( ADDR_MOTOR_SPINUP_LAG, MotorSpinupLag );
    
    MotorCompensationBias = MOTOR_COMPENSATION_BIAS;
    EEPROM.put( ADDR_MOTOR_COMPENSATION_BIAS, MotorCompensationBias );
      
    BatteryOffset = BATTERY_OFFSET;                                     //rn
    EEPROM.put (ADDR_BATTERY_OFFSET, BatteryOffset );                   //rn
  }
  
}

//---------------------------------------------- Calculate Ramp Rates Subroutine -----------------------------------------------
/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 */
void CalculateRampRates()
{
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if( DecelerateTime == 0 )
  {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }  
}

//-------------------------------------- Updates the PWM Timers Subrutine -------------------------------------------------
void UpdatePWM( int NewSpeed )
{
  NewSpeed = (NewSpeed * 2) + 2; // Adjust for the prescalar
  OCR1B = NewSpeed;
}

//----------------------------------------- ISR Sections --------------------------------------------------------
// PCINT - PB
ISR( PCINT0_vect ) // PB
{
  // SelFireA is PB4
  if( (PINB & 0b00010000) != (LastPINB & 0b00010000) )
  {
    SelFireAChanged = true;
  }
  // SelFireB is PC0 was PB5 on Narfduino
  //if( (PINB & 0b00100000) != (LastPINB & 0b00100000) ) // Narfdunio HW test
  if( (PINC & 0b00000001) != (LastPINC & 0b00000001) ) //Nano HW test
  {
    SelFireBChanged = true;
  }

  LastPINB = PINB;
  LastPINC = PINC; //rn added this line in porting to Nano PCB
}

// PCINT - PC
ISR( PCINT1_vect ) // PC
{
  // Trigger is PC2
  if( (PINC & 0b00000100) != (LastPINC & 0b00000100) )
  {
    TriggerChanged = true;
  }
  LastPINC = PINC;
}

// ADC ISR
ISR( ADC_vect )
{
  ADCRead = true;
  ADCValue = ADCL | (ADCH << 8);
}


//------------------------------------ Process the debouncing of the directly connected MCU inputs subroutine ---------------------------
void ProcessDebouncing()
{
  // Single call to millis() for better performance
  unsigned long CurrentMillis = millis();
  
  /*
   * Trigger
   */
  static bool RunTriggerTimer = false;
  static unsigned long LastTriggerPress = 0;
  static byte TriggerDebounceState = 0;
  // Set up a repoll interval, just in case the interrupt is missed.
  if( CurrentMillis - LastTriggerPress > RepollInterval ) TriggerChanged = true; 
  // Move from edge to steady state
  if( TriggerButtonState == BTN_ROSE ) TriggerButtonState = BTN_HIGH;
  if( TriggerButtonState == BTN_FELL ) TriggerButtonState = BTN_LOW;  
  if( TriggerChanged )
  {
   TriggerChanged = false; 
   if( !RunTriggerTimer )
   {
    LastTriggerPress = CurrentMillis;

    TriggerDebounceState = 0b01010101; // We need to give the button time to settle. This will track. 
    RunTriggerTimer = true;    
   }   
  }
  if( RunTriggerTimer && ( CurrentMillis - LastTriggerPress > DebounceWindow  ) )
  {
    TriggerDebounceState = (TriggerDebounceState << 1) | ((PIND >> 6) & 0b00000001); // Shift the register pin to the left, Shift the pin result to the right most position, and tack it onto the debounce state. Ensure that only the last position can be a 1.

    if( (TriggerDebounceState == 0) || (TriggerDebounceState == 255) ) // All 0's or all 1's. This means we have settled.
    {
      RunTriggerTimer = false;
    
      if( GET_FIRE_T )
      { 
        if( TriggerButtonState != BTN_HIGH ) TriggerButtonState = BTN_ROSE;
      }
      else
      { 
        if( TriggerButtonState != BTN_LOW ) TriggerButtonState = BTN_FELL;
      }    
    }
  }

  /*
   * SelFireA
   */
  static bool RunSelFireATimer = false;
  static unsigned long LastSelFireAPress = 0;
  static byte SelFireADebounceState = 0;
  // Set up a repoll interval, just in case the interrupt is missed.
  if( CurrentMillis - LastSelFireAPress > RepollInterval ) SelFireAChanged = true; 
  // Move from edge to steady state
  if( SelFireAButtonState == BTN_ROSE ) SelFireAButtonState = BTN_HIGH;
  if( SelFireAButtonState == BTN_FELL ) SelFireAButtonState = BTN_LOW;  
  if( SelFireAChanged )
  {
   SelFireAChanged = false; 
   if( !RunSelFireATimer )
   {
    LastSelFireAPress = CurrentMillis;

    SelFireADebounceState = 0b01010101; // We need to give the button time to settle. This will track. 
    RunSelFireATimer = true;    
   }   
  }
  if( RunSelFireATimer && ( CurrentMillis - LastSelFireAPress > DebounceWindow  ) )
  {
    SelFireADebounceState = (SelFireADebounceState << 1) | ((PINB >> 4) & 0b00000001); // Shift the register pin to the left, Shift the pin result to the right most position, and tack it onto the debounce state. Ensure that only the last position can be a 1.

    if( (SelFireADebounceState == 0) || (SelFireADebounceState == 255) ) // All 0's or all 1's. This means we have settled.
    {
      RunSelFireATimer = false;
    
      if( GET_SEL_FIRE_A )
      { 
        if( SelFireAButtonState != BTN_HIGH ) SelFireAButtonState = BTN_ROSE;
      }
      else
      { 
        if( SelFireAButtonState != BTN_LOW ) SelFireAButtonState = BTN_FELL;
      }    
    }
  }

  /*
   * SelFireB
   */
  static bool RunSelFireBTimer = false;
  static unsigned long LastSelFireBPress = 0;
  static byte SelFireBDebounceState = 0;
  // Set up a repoll interval, just in case the interrupt is missed.
  if( CurrentMillis - LastSelFireBPress > RepollInterval ) SelFireBChanged = true; 
  // Move from edge to steady state
  if( SelFireBButtonState == BTN_ROSE ) SelFireBButtonState = BTN_HIGH;
  if( SelFireBButtonState == BTN_FELL ) SelFireBButtonState = BTN_LOW;  
  if( SelFireBChanged )
  {
   SelFireBChanged = false; 
   if( !RunSelFireBTimer )
   {
    LastSelFireBPress = CurrentMillis;

    SelFireBDebounceState = 0b01010101; // We need to give the button time to settle. This will track. 
    RunSelFireBTimer = true;    
   }   
  }
  if( RunSelFireBTimer && ( CurrentMillis - LastSelFireBPress > DebounceWindow  ) )
  {
    SelFireBDebounceState = (SelFireBDebounceState << 1) | ((PINB >> 5) & 0b00000001); // Shift the register pin to the left, Shift the pin result to the right most position, and tack it onto the debounce state. Ensure that only the last position can be a 1.

    if( (SelFireBDebounceState == 0) || (SelFireBDebounceState == 255) ) // All 0's or all 1's. This means we have settled.
    {
      RunSelFireBTimer = false;
    
      if( GET_SEL_FIRE_B )
      { 
        if( SelFireBButtonState != BTN_HIGH ) SelFireBButtonState = BTN_ROSE;
      }
      else
      { 
        if( SelFireBButtonState != BTN_LOW ) SelFireBButtonState = BTN_FELL;
      }    
    }
  }
}

//-------------------------------------------------------------------------------------------------------------
// Main loop. Run the blaster here.
//-------------------------------------------------------------------------------------------------------------
void loop() {
  ProcessDebouncing(); // Process the pin input, and handle any debouncing
  ProcessButtons(); // Where interperation is necessary, handle it here
  ProcessBatteryMonitor(); // Check battery voltage occasionally
  ProcessSystemMode(); // Find out what the system should be doing

  // Process Serial input
  if( ProcessSerialInput() )
  {
    ProcessSerialCommand();
  }

  // Detected a change to the command. Reset the last speed change timer.
  if( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }
  ProcessSpeedControl(); // Process speed control to resolve a % target
  ProcessMotorSpeed(); // Calculate the new motor speed in terms of a PWM signal
  ProcessMainMotors();  // Send the speed to the ESC

  ProcessFiringControl(); // Handle the cycle control here

  ProcessPusherControl(); // Handle the pusher management here

}

//------------------------------- Process the main system state subroutine ---------------------------------
void ProcessSystemMode()
{
  if( BatteryFlat )
  {
    SystemMode = SYSTEM_MODE_LOWBATT;
//RN
      digitalWrite(PIN_LOWBAT, HIGH);   // turn the LED on (HIGH is the voltage level)
//        tone(PIN_LOWBAT, 1000);            //turn buzzer on
  }
  else 
  {
    SystemMode = SYSTEM_MODE_NORMAL;
    digitalWrite(PIN_LOWBAT, LOW);    // turn the LED off by making the voltage LOW
//      noTone(PIN_LOWBAT);                //Turn buzzer off
  }
}

//------------------------------------ Handle the high-level firing logic subroutine ---------------------------
void ProcessFiringControl()
{

  #define FS_STAGE_IDLE 0
  #define FS_STAGE_REV 1
  #define FS_STAGE_FIRING 2
  #define FS_STAGE_DECEL 3
  static byte FiringSequenceStage = FS_STAGE_IDLE;
  static byte LastSystemMode = 99;
  static bool PreviousRunFiringSequence = true;
  static unsigned long RevStart = 0;
  static unsigned long DecelStart = 0; 
  static unsigned long MotorCompensation = 0;

  int StartLag = MotorSpinupLag + AccelerateTime; // Calculate the lag for the motor start


  if( SystemMode != SYSTEM_MODE_NORMAL )
  {
    if( StartNewFiringSequence )
      StartNewFiringSequence = false; // Abort a new cycle
      CommandRev = COMMAND_REV_NONE;
    if( RunFiringSequence )
    {
      switch( FiringSequenceStage )
      {
        case FS_STAGE_REV:
          // Bypass the firing sequence and start decelerating
          FiringSequenceStage = FS_STAGE_DECEL;
          DecelStart = 0;
          break;
        case FS_STAGE_FIRING:
          // Set one dart into the queue if there is more than one
          if( DartsToFire > 1 ) DartsToFire = 1;
          break;
        case FS_STAGE_DECEL:
          // Already decelerating
          break;
        default:
          break; // Idle
      }
    }
  }

  if( StartNewFiringSequence )
  {
    StartNewFiringSequence = false;
    if( (SystemMode != SYSTEM_MODE_NORMAL) )
    {
      return; // Don't start a new firing sequence if out of the right mode.
    }
    if( RunFiringSequence )
    {
      switch( FiringSequenceStage )
      {
        case FS_STAGE_REV:
          // Carry on
          break;
        case FS_STAGE_FIRING:
          // Add more darts to the queue
          switch( CurrentFireMode )
          {
            case FIRE_MODE_SINGLE: // Add one to the queue
              DartsToFire ++;
              break;
            case FIRE_MODE_BURST:
              DartsToFire = BurstSize;
              break;
            case FIRE_MODE_AUTO:
              DartsToFire = 99;
              break;
            case FIRE_MODE_BINARY:
              DartsToFire ++;
              break;
            case FIRE_MODE_RAMPED: case FIRE_MODE_ZERG:
              if( IsRamping )
                DartsToFire = 99;
              else
                DartsToFire ++;
              break;
            default: // Idle and Safe Mode
              break;
          }
          break;
        case FS_STAGE_DECEL:
          // Start again from scratch, but factor in the current motor speed          
          if( CurrentFireMode != FIRE_MODE_SAFE )
          {
            RunFiringSequence = true;
            FiringSequenceStage = FS_STAGE_REV;
            RevStart = 0;
            ProcessingFireMode = CurrentFireMode;
          }
          break;
        default:
          // We shouldn't be here, but we were probably idle.
          break;
      }
    }
    else
    {
      if( CurrentFireMode != FIRE_MODE_SAFE )
      {
        RunFiringSequence = true;
        FiringSequenceStage = FS_STAGE_REV;
        RevStart = 0;
        ProcessingFireMode = CurrentFireMode;
      }
    }
  }

  if( ShutdownFiringSequence )
  {
    // Only need for auto, reset dart counter to 1 and let it fly out naturally
    ShutdownFiringSequence = false;
    if( CurrentFireMode == FIRE_MODE_AUTO )
    {
      DartsToFire = 1;
    }
    if( (CurrentFireMode == FIRE_MODE_BINARY) && (CommandRev == COMMAND_REV_FULL) ) // Will fire another shot if the motors are still running. 
    {
      RunFiringSequence = true;
      FiringSequenceStage = FS_STAGE_REV;
      RevStart = 0;
      ProcessingFireMode = CurrentFireMode;
    }
    if( (CurrentFireMode == FIRE_MODE_ZERG) && !IsRamping && (CommandRev == COMMAND_REV_FULL) ) // Will fire another shot if the motors are still running. 
    {
      RunFiringSequence = true;
      FiringSequenceStage = FS_STAGE_REV;
      RevStart = 0;
      ProcessingFireMode = CurrentFireMode;
    }
  }
  // Catch the de-ramp
  if( ((CurrentFireMode == FIRE_MODE_RAMPED) || (CurrentFireMode == FIRE_MODE_ZERG)) && DartsToFire > 1 && !IsRamping )
  {
    DartsToFire = 1;
  }
  
  if( RunFiringSequence )
  {
    if( FiringSequenceStage == FS_STAGE_REV )
    {
      if( RevStart == 0 )
      {
        // Init
        switch( CurrentFireMode )
        {
          case FIRE_MODE_SINGLE: case FIRE_MODE_BINARY: // Add one to the queue
            TargetDPS = 99;
            break;
          case FIRE_MODE_BURST:
            TargetDPS = TargetDPSBurst;
            break;
          case FIRE_MODE_AUTO:
            TargetDPS = TargetDPSAuto;
            break;
          case FIRE_MODE_RAMPED: case FIRE_MODE_ZERG:
            if( IsRamping )
              TargetDPS = TargetDPSAuto;
            else
              TargetDPS = 0;
            break;
          default: // Idle and Safe Mode
            break;
        }        
        RequestStop = false;
        if( CommandRev == COMMAND_REV_NONE ) // Motor is cold or cooling down
        {
          CommandRev = COMMAND_REV_FULL;

          // Compensate for a motor warm start
          MotorCompensation = 0;
          unsigned int TargetMotorSpeed = map( MotorSpeedFull, 0, 100, MinMotorSpeed, MaxMotorSpeed );        
          MotorCompensation = (map( CurrentMotorSpeed, MinMotorSpeed, TargetMotorSpeed, 0, MotorSpinupLag ) * (MotorCompensationBias / 100.0));
          RevStart = millis();
        }
        else // Motor is already hot
        {
          RevStart = 0;
          MotorCompensation = 0; // Reset this value
          FiringSequenceStage = FS_STAGE_FIRING;
          // *** RUN PUSHER HERE
          LastSolenoidCycleStarted = millis();
          //PulseOnTime = map( BatteryPercent, 1, 100, PulseOnTimeLow, PulseOnTimeHigh );
          PulseOnTime = map( (int)(BatteryCurrentVoltage*10), SolenoidHighVoltage, SolenoidLowVoltage, PulseOnTimeLow, PulseOnTimeHigh );
          if( PulseOnTime < 0 ) 
            PulseOnTime = 0;
          ExecuteFiring = true;     
        }
                
        switch( CurrentFireMode )
        {
          case FIRE_MODE_SINGLE: // Add one to the queue
            DartsToFire = 1;
            break;
          case FIRE_MODE_BURST:
            DartsToFire = BurstSize;
            break;
          case FIRE_MODE_AUTO:
            DartsToFire = 99;
            break;
          case FIRE_MODE_BINARY:
            DartsToFire = 1;
            break;
          case FIRE_MODE_RAMPED: case FIRE_MODE_ZERG:
            if( IsRamping )
              DartsToFire = 99;
            else
              DartsToFire = 1;
            break;
          default: // Idle and Safe Mode
            break;
        }
      }
      else if( millis() - RevStart > max(0, StartLag + MotorStartDwellTime - MotorCompensation) ) // We have waited long enough for the motor to ramp
      {
        RevStart = 0;
        MotorCompensation = 0; // Reset this value
        FiringSequenceStage = FS_STAGE_FIRING;
        // *** RUN PUSHER HERE
        LastSolenoidCycleStarted = millis();
        PulseOnTime = map( (int)(BatteryCurrentVoltage*10), SolenoidHighVoltage, SolenoidLowVoltage, PulseOnTimeLow, PulseOnTimeHigh );
          if( PulseOnTime < 0 ) 
            PulseOnTime = 0;
        ExecuteFiring = true;
      }
    }
    else if( FiringSequenceStage == FS_STAGE_FIRING )
    {
      if( RequestStop ) // Want to stop - move from firing state to stop state
      {
        FiringSequenceStage = FS_STAGE_DECEL;
        DecelStart = 0;
      }   
      if( CommandRev == COMMAND_REV_NONE ) // Somehow the motors aren't spinning
      {
        RequestStop = true;
        FiringSequenceStage = FS_STAGE_DECEL;
        DecelStart = 0;                
      }
    }
    else if( FiringSequenceStage == FS_STAGE_DECEL )
    {
      if( DecelStart == 0 ) // Initiate shutdown
      {
        DecelStart = millis();
      }
      else if( ((millis() - DecelStart > MotorStopDwellTime) || (millis() - DecelStart > 5000)) && !(CommandRev == COMMAND_REV_NONE) )
      { 
        CommandRev = COMMAND_REV_NONE;
      }
      else if( CurrentMotorSpeed == MinMotorSpeed ) // Shutdown complete
      {
        FiringSequenceStage = FS_STAGE_IDLE;
        RunFiringSequence = false;
        DecelStart = 0;
        ProcessingFireMode = FIRE_MODE_IDLE;
      }
    }
  }
  else 
  {
    CommandRev = COMMAND_REV_NONE;
  }

}

//------------------------------------- Process Motor Speed Subroutine ------------------------------------------------------------------
// Calculate the desired motor speed based on where we are, and where we need to be. This takes into account the ramp rates of the motor.
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if( CurrentMotorSpeed < TargetMotorSpeed )
  {
    long SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.    
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..  

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if( CurrentMotorSpeed > TargetMotorSpeed )
  {
    long SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed - 10 <= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}

//----------------------------------- Process Speed Control Subroutine -----------------------------------------------------------
// We need to set the Target Motor Speed here, as a percentage. Sense check it to ensure it's not too slow, or too fast.
void ProcessSpeedControl()
{ 
  static byte LastSetMaxSpeed = 100;

  if( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;
}


//-------------------------------- Update the motors with the new speed subroutine ------------------------------------
void ProcessMainMotors()
{
  static int PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    if( CurrentMotorSpeed > MaxMotorSpeed )
      UpdatePWM( MaxMotorSpeed );
    else
      UpdatePWM( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

//------------------------------------------- Process Buttons Subroutine ---------------------------------------------------
void ProcessButtons()
{
  // Button Controls
  if( TriggerButtonState == BTN_ROSE )
  {
    StartNewFiringSequence = true;
    if( millis() - LastTriggerPull < RampFrequency )
    {
      if( CurrentRampCount >= RampThreshold )
      {
        IsRamping = true;
      }
      else
      {
        CurrentRampCount ++;
        IsRamping = false;
      }
    }
    else
    {
      IsRamping = false;
      CurrentRampCount = 0;
    }
    LastTriggerPull = millis(); // Record the last trigger pull for the Ramped calculations
  }
  else if( TriggerButtonState == BTN_FELL )
  {
    if( RunFiringSequence )
    {
      ShutdownFiringSequence = true;
    }
  }

  // Handle the de-ramp now
  if( ((CurrentRampCount > 0) || IsRamping) && (millis() - LastTriggerPull >= RampFrequency) )
  {
    CurrentRampCount = 0;
    IsRamping = false;
  }

  static byte LastSFRead = 4;
  byte SFRead = 3;
  // Determine the current select fire mode
  if( SelFireAButtonState == BTN_LOW && SelFireBButtonState == BTN_HIGH  )
    SFRead = 2;
  else if( SelFireAButtonState == BTN_HIGH && SelFireBButtonState == BTN_HIGH )
    SFRead = 1;
  else if( SelFireAButtonState == BTN_HIGH && SelFireBButtonState == BTN_LOW )
    SFRead = 0;

  if( SFRead != LastSFRead )
  {
    LastSFRead = SFRead;
    switch( SFRead )
    {
      case 0:
        PerformButtonAction( ButtonActions[BTN_IDX_SF1] );
        break;
      case 1:
        PerformButtonAction( ButtonActions[BTN_IDX_SF2] );
        break;
      case 2:
        PerformButtonAction( ButtonActions[BTN_IDX_SF3] );
        break;
      default:
        break;
    }
  }
}

// ------------------------------ Process Pusher Control Subroutine ---------------------------------------
void ProcessPusherControl()
{

  if( !ExecuteFiring ) // Just skip if there is no firing to execute
  {
    return;
  }

  // Calculate duty cycle whenever the target changes.
  static byte PrevTargetDPS = 0;
  static bool ResetPulseTimer = true;
  if( PrevTargetDPS != TargetDPS )
  {
    PrevTargetDPS = TargetDPS;
    if( TargetDPS == 99 ) // Full rate
    {
      TimeBetweenShots = 0;
    }
    else
    {
      int PulseOverhead = PulseOnTime + PulseRetractTime;
      int TotalPulseOverhead = PulseOverhead * TargetDPS;
      int FreeMS = 1000 - TotalPulseOverhead;
      if( FreeMS <= 0 )
      {
        TimeBetweenShots = 0; // Pusher won't achieve this rate
      }
      else
      {
        TimeBetweenShots = FreeMS / TargetDPS;
      }
    }
  }

  // We actually have nothing to do
  if( ProcessingFireMode == FIRE_MODE_IDLE )
  {
    return; // Solenoid is idling.
  }

  // We are apparently supposed to fire 0 darts... Typically for end-of-firing scenarios
  if( (DartsToFire == 0) && (ProcessingFireMode != FIRE_MODE_IDLE) )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    SET_P_RUN_FULL_OFF;
    ExecuteFiring = false;
    RequestStop = true;
    return;    
  }

  // Last check to ensure the motors are running before we send a dart into them
  if( (CommandRev == COMMAND_REV_NONE) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    SET_P_RUN_FULL_OFF;
    ExecuteFiring = false;
    RequestStop = true;
    return;        
  }

  // Pulse solenoid on high
  if( (millis() - LastSolenoidCycleStarted) < PulseOnTime )
  {
    //Serial.println( "Pulse" );
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_PULSE )
    {
      if( (SystemMode != SYSTEM_MODE_NORMAL) ) // Don't fire unless the system m ode is normal
      {
        DartsToFire = 0;
        return;
      }
    }
    if( ResetPulseTimer )
    {
      ResetPulseTimer = false;
      LastSolenoidCycleStarted = millis();
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_PULSE;
    SET_P_RUN_FULL_ON;
    return;
  }

  // Release solenoid for retraction
  if( (millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime) )
  {
    //Serial.println( "Retract" );
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_RETRACT;
    SET_P_RUN_FULL_OFF;
    return;      
  }  

  // Wait for the Global Cool Down... i.e. ROF adjustment
  if((millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime + TimeBetweenShots))
  {
    //Serial.println( "Cool" );
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_COOLDOWN;
    SET_P_RUN_FULL_OFF;
    return;      
  }

  // We have completed a single solenoid cycle. Return to idle, ready for the next shot.
  CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
  DartsToFire -= 1;
  LastShot = millis();
  LastSolenoidCycleStarted = millis();
  ResetPulseTimer = true;
  Serial.println( "Bang!" );
  //RN
  tone(PIN_FIRING, 1000, 100);            //turn firing led on, 1000hz 100ms
}

//----------------------------------------- Update Profile Subroutine ----------------------------------------------
void UpdateProfile()
{
  MotorSpeedFull = Profiles[CurrentProfile].MotorSpeedFull;
  TargetDPSBurst = Profiles[CurrentProfile].TargetDPSBurst;
  TargetDPSAuto = Profiles[CurrentProfile].TargetDPSAuto;
  BurstSize = Profiles[CurrentProfile].BurstSize;
  AccelerateTime = Profiles[CurrentProfile].AccelerateTime;
  DecelerateTime = Profiles[CurrentProfile].DecelerateTime;
  MotorStartDwellTime = Profiles[CurrentProfile].MotorStartDwellTime;
  MotorStopDwellTime = Profiles[CurrentProfile].MotorStopDwellTime;
  PulseOnTimeHigh = Profiles[CurrentProfile].PulseOnTimeHigh;
  PulseOnTimeLow = Profiles[CurrentProfile].PulseOnTimeLow;
  PulseRetractTime = Profiles[CurrentProfile].PulseRetractTime; 
  ButtonActions[BTN_IDX_SF1] = Profiles[CurrentProfile].BtnSF1;
  ButtonActions[BTN_IDX_SF2] = Profiles[CurrentProfile].BtnSF2;
  ButtonActions[BTN_IDX_SF3] = Profiles[CurrentProfile].BtnSF3;
  RampDPS = Profiles[CurrentProfile].RampDPS;
  RampThreshold = Profiles[CurrentProfile].RampThreshold;  
  SolenoidHighVoltage = Profiles[CurrentProfile].SolenoidHighVoltage;
  SolenoidLowVoltage = Profiles[CurrentProfile].SolenoidLowVoltage;
  CalculateRampRates(); // Recalculate the ramp rates
  CalculateRampFrequency(); // Recalculate the ramp frequency
}

//-------------------------------- Keep tabs on the Battery Subrotine------------------------------------
void ProcessBatteryMonitor()
{

  #define BM_STATUS_IDLE 0
  #define BM_STATUS_READING 1
  #define BM_STATUS_READY 2

  static byte CurrentStatus = 0;
  static unsigned long LastCheck = 0;

  // Every 500ms, start a background ADC read
  if( CurrentStatus == BM_STATUS_IDLE )
  {
    if( millis() - LastCheck < 250 )
    {
      return;
    }
    
    LastCheck = millis();
    CurrentStatus = BM_STATUS_READING;
    ADCSRA |= (1 << ADSC); // Run the ADC
    return;
  }

  // When the ADC has finished it's conversion, proceed to the processing step
  if( CurrentStatus == BM_STATUS_READING )
  {
    if( !ADCRead )
    {
      return; // Nothing to see here
    }
    ADCRead = false;
    CurrentStatus = BM_STATUS_READY;
  }


  if( CurrentStatus != BM_STATUS_READY )
    return;
  
  #define NUM_SAMPLES 6
  static byte CollectedSamples = 0;
  static float SampleAverage = 0;
  if( CollectedSamples < NUM_SAMPLES )
  {
    CollectedSamples ++;
    SampleAverage += (float)ADCValue;
  }
  else
  {
    // BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
    // BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((30.0 + 7.0) / 7.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 30k and 7K5
   BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + (float)BatteryOffset;  // Voltage dividor - 47k and 10k
    
    if( BatteryCurrentVoltage < BatteryMinVoltage )
    {
      if( BatteryCurrentVoltage > 1.6 ) // If the current voltage is 0, we are probably debugging
      {
        BatteryFlat = true;
      }
      else
      {
        BatteryFlat = false;
      }
    }
    else
    {
      BatteryFlat = false;
    } 
    BatteryPercent = map( (int)(BatteryCurrentVoltage * 10), (int)(BatteryMinVoltage * 10), (int)(BatteryMaxVoltage * 10), 1, 100 );
    CollectedSamples = 0;
    SampleAverage = 0;
  }

  // Reset back to the default position.
  LastCheck = millis(); 
  CurrentStatus = BM_STATUS_IDLE;
}

//------------------------------------ Process Serial Input -----------------------------
bool ProcessSerialInput()
{
  bool SerialDataAvailable = false;
  if( Serial.available() != 0 )
    SerialDataAvailable = true;
    
  if( !SerialDataAvailable ) return false; // Ignore when there is no serial input
  
  static byte CurrentBufferPosition = 0;

  while( Serial.available() > 0 )
  {
    char NextByte = 0;
    if( Serial.available() != 0 )
      NextByte = Serial.read();
    else
      NextByte = 0; //WTF is this happening??

    switch( NextByte )
    {
      case '#': // Starting new command
        CurrentBufferPosition = 0;
        break;
      case '$': // Ending command
        return true; // Jump out.. There's more data in the buffer, but we can read that next time around.
        break;
      case '?': // Presume help - Simulate DS
        SerialInputBuffer[0] = 'D';
        SerialInputBuffer[1] = 'S';
        return true;
        break;
      default: // Just some stuff coming through
        SerialInputBuffer[ CurrentBufferPosition ] = NextByte; // Insert into the buffer
        CurrentBufferPosition ++; // Move the place to the right
        if( CurrentBufferPosition >= SERIAL_INPUT_BUFFER_MAX ) CurrentBufferPosition = (SERIAL_INPUT_BUFFER_MAX - 1);  // Capture Overflows.
    }
  }

  return false;
}

//-------------------------------------------------- Process Serial Command Subroutine -------------------------------------------------
void ProcessSerialCommand()
{
  char CommandHeader[3]; // Place the header into this buffer
  // Copy it using a lazy way
  CommandHeader[0] = SerialInputBuffer[0];
  CommandHeader[1] = SerialInputBuffer[1];
  CommandHeader[2] = 0;


  // Full Power Command - FP
  if( (strcmp( CommandHeader, "FP" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    MotorSpeedFull = constrain( atoi( IntValue ), 30, 100 );
    TimeLastMotorSpeedChanged = millis();
    EEPROM.write( ADDR_MSF, MotorSpeedFull );
  }
  
  // Burst Size Command - BS
  if( (strcmp( CommandHeader, "BS" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );  
    EEPROM.write( ADDR_BURST, BurstSize );
  }

  // Burst Rate Command - BR
  if( (strcmp( CommandHeader, "BR" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    TargetDPSBurst = constrain( atoi( IntValue ), 1, 99 );
    EEPROM.write( ADDR_ROF_BURST, TargetDPSBurst );
  }

  // Full Auto Rate Command - FR
  if( (strcmp( CommandHeader, "FR" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    TargetDPSAuto = constrain( atoi( IntValue ), 1, 99 );
    EEPROM.write( ADDR_ROF_AUTO, TargetDPSAuto );
  }

  // Solenoid Pulse High Time
  if( (strcmp( CommandHeader, "PH" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    PulseOnTimeHigh = constrain( atoi( IntValue ), 1, 500 );
    EEPROM.put( ADDR_PULSE_HIGH, PulseOnTimeHigh );     
  }

  // Solenoid Pulse Low Time 
  if( (strcmp( CommandHeader, "PL" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    PulseOnTimeLow = constrain( atoi( IntValue ), 1, 500 );
    EEPROM.put( ADDR_PULSE_LOW, PulseOnTimeLow );     
  }

  // Solenoid Retract Time
  if( (strcmp( CommandHeader, "SR" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    PulseRetractTime = constrain( atoi( IntValue ), 1, 500 );
    EEPROM.put( ADDR_PULSE_RETRACT, PulseRetractTime );     
  }

  // Motor Accel Time
  if( (strcmp( CommandHeader, "MA" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[5] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], SerialInputBuffer[6], 0 };
    AccelerateTime = constrain( atoi( IntValue ), 0, 5000 );
    EEPROM.put( ADDR_ACCEL, AccelerateTime );     
  }

  // Motor Decel Time
  if( (strcmp( CommandHeader, "MD" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[5] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], SerialInputBuffer[6], 0 };
    DecelerateTime = constrain( atoi( IntValue ), 0, 5000 );
    EEPROM.put( ADDR_DECEL, DecelerateTime );     
  }

  // Motor Start Dwell Time
  if( (strcmp( CommandHeader, "UD" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[5] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], SerialInputBuffer[6], 0 };
    MotorStartDwellTime = constrain( atoi( IntValue ), 0, 5000 );
    EEPROM.put( ADDR_STARTD, MotorStartDwellTime );     
  }

  // Motor Step Dwell Time
  if( (strcmp( CommandHeader, "DD" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[5] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], SerialInputBuffer[6], 0 };
    MotorStopDwellTime = constrain( atoi( IntValue ), 0, 5000 );
    EEPROM.put( ADDR_STOPD, MotorStopDwellTime );     
  }  

  // Select Fire 1
  if( (strcmp( CommandHeader, "S1" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[2] = { SerialInputBuffer[3], 0 };
    ButtonActions[0] = constrain( atoi( IntValue ), 0, 7 );
    EEPROM.put( ADDR_BTN_SF1, ButtonActions[0] );     
  }

  // Select Fire 2
  if( (strcmp( CommandHeader, "S2" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[2] = { SerialInputBuffer[3], 0 };
    ButtonActions[1] = constrain( atoi( IntValue ), 0, 7 );
    EEPROM.put( ADDR_BTN_SF2, ButtonActions[1] );     
  }

  // Select Fire 3
  if( (strcmp( CommandHeader, "S3" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[2] = { SerialInputBuffer[3], 0 };
    ButtonActions[2] = constrain( atoi( IntValue ), 0, 7 );
    EEPROM.put( ADDR_BTN_SF3, ButtonActions[2] );     
  }  
  
  // Ramp DPS
  if( (strcmp( CommandHeader, "RD" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    RampDPS = constrain( atoi( IntValue ), 2, 9 );
    EEPROM.put( ADDR_RAMP_DPS, RampDPS );     
  }  
  
  // Ramp Threshold
  if( (strcmp( CommandHeader, "RT" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    RampThreshold = constrain( atoi( IntValue ), 2, 9 );
    EEPROM.put( ADDR_RAMP_THRESHOLD, RampThreshold );     
  }   

  // Solenoid High Voltage
  if( (strcmp( CommandHeader, "SH" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    SolenoidHighVoltage = constrain( atoi( IntValue ), 70, 180 );
    EEPROM.put( ADDR_RAMP_SOL_HI_VOLT, SolenoidHighVoltage );     
  }

  // Solenoid Low Voltage
  if( (strcmp( CommandHeader, "SL" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    SolenoidLowVoltage = constrain( atoi( IntValue ), 70, 180 );
    EEPROM.put( ADDR_RAMP_SOL_LO_VOLT, SolenoidLowVoltage );     
  }

  // Motor Spinup Lag
  if( (strcmp( CommandHeader, "ML" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    MotorSpinupLag = constrain( atoi( IntValue ), 0, 999 );
    EEPROM.put( ADDR_MOTOR_SPINUP_LAG, MotorSpinupLag );     
  }

  // Motor Compensation Bias
  if( (strcmp( CommandHeader, "MB" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    MotorCompensationBias = constrain( atoi( IntValue ), 0, 100 );
    EEPROM.put( ADDR_MOTOR_COMPENSATION_BIAS, MotorCompensationBias );     
  }
  
  // Battery Offset ---rn
  if( (strcmp( CommandHeader, "BO" ) == 0) && (SystemMode == SYSTEM_MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    BatteryOffset = constrain( atoi( IntValue ), -20, 20 );
    BatteryOffset = BatteryOffset / 10 ;        // move the decimal point
    //Serial.println (BatteryOffset);             // this is a floating point number -2.0 to +2.0
    EEPROM.put( ADDR_BATTERY_OFFSET, BatteryOffset );     
  }
  //---------------------------------------------------------------------------------------------------------------------------------------------
      
  // Query Device Command - QD
  if( strcmp( CommandHeader, "QD" ) == 0 )
  {
    Serial.println( F("#CB-OK$") );
  } 

  // Query Voltage Command - QV
  if( strcmp( CommandHeader, "QV" ) == 0 )
  {
    char VoltBuffer[5];
    sprintf( VoltBuffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    VoltBuffer[4] = 0;
    VoltBuffer[3] = VoltBuffer[2];
    VoltBuffer[2] = '.';
    Serial.println( VoltBuffer );
  }   

  // Display Settings - DS
  if( strcmp( CommandHeader, "DS" ) == 0 )
  {
    Serial.println( F("--------------------") );
    Serial.println( F("Blaster Settings:") );
    Serial.println( F("--------------------") );

    Serial.print( F("Full Trigger Power = ") );
    Serial.println( MotorSpeedFull );
    Serial.println( F("Change with #FP-xxx$  (xxx = 030 - 100)\n") );

    Serial.print( F("Burst Size = ") );
    Serial.println( BurstSize );
    Serial.println( F("Change with #BS-xx$  (xx = 01 - 99)\n") );

    Serial.print( F("ROF Burst = ") );
    Serial.println( TargetDPSBurst );
    Serial.println( F("Change with #BR-xx$  (xx = 01 - 99; Pusher Physical Limit Applies)\n") );

    Serial.print( F("ROF Auto = ") );
    Serial.println( TargetDPSAuto );
    Serial.println( F("Change with #FR-xx$  (xx = 01 - 99; Pusher Physical Limit Applies)\n") );

    Serial.print( F("Solenoid Pulse High Time = ") );
    Serial.println( PulseOnTimeHigh );
    Serial.println( F("Change with #PH-xxx$  (xxx = 001 - 500)\n") );

    Serial.print( F("Solenoid Pulse Low Time = ") );
    Serial.println( PulseOnTimeLow );
    Serial.println( F("Change with #PL-xxx$  (xxx = 001 - 500)\n") );

    Serial.print( F("Solenoid Retract Time = ") );
    Serial.println( PulseRetractTime );
    Serial.println( F("Change with #SR-xxx$  (xxx = 001 - 500)\n") );

    Serial.print( F("Motor Accel Time = ") );
    Serial.println( AccelerateTime );
    Serial.println( F("Change with #MA-xxxx$  (xxxx = 0000 - 5000)\n") );

    Serial.print( F("Motor Decel Time = ") );
    Serial.println( DecelerateTime );
    Serial.println( F("Change with #MD-xxxx$  (xxxx = 0000 - 5000)\n") );

    Serial.print( F("Motor Start Dwell Time = ") );
    Serial.println( MotorStartDwellTime );
    Serial.println( F("Change with #UD-xxxx$  (xxxx = 0000 - 5000)\n") );

    Serial.print( F("Motor Step Dwell Time = ") );
    Serial.println( MotorStopDwellTime );
    Serial.println( F("Change with #DD-xxxx$  (xxxx = 0000 - 5000)\n") );

    Serial.print( F("Select Fire Position 1 = ") );
    Serial.println( ButtonActions[0] );
    Serial.println( F("Change with #S1-x$  (x = 0 - 7: x = Fire Mode below)\n") );

    Serial.print( F("Select Fire Position 2 = ") );
    Serial.println( ButtonActions[1] );
    Serial.println( F("Change with #S2-x$  (x = 0 - 7: x = Fire Mode below)\n") );

    Serial.print( F("Select Fire Position 3 = ") );
    Serial.println( ButtonActions[2] );
    Serial.println( F("Change with #S3-x$  (x = 0 - 7: x = Fire Mode below)\n") );

    Serial.print( F("Ramp DPS = ") );
    Serial.println( RampDPS );
    Serial.println( F("Change with #RD-xx$  (xx = 02 - 09)\n") );

    Serial.print( F("Ramp Threshold = ") );
    Serial.println( RampThreshold );
    Serial.println( F("Change with #RT-xx$  (xx = 02 - 09)\n") );

    Serial.print( F("Solenoid High Voltage = ") );
    Serial.println( SolenoidHighVoltage );
    Serial.println( F("Change with #SH-xxx$  (xxx = 070 - 180 and xxx is measured voltage * 10 - use #QV$)\n") );

    Serial.print( F("Solenoid Low Voltage = ") );
    Serial.println( SolenoidLowVoltage );
    Serial.println( F("Change with #SL-xxx$  (xxx = 070 - 180 and xxx is measured voltage * 10 - use #QV$)\n") );

    Serial.print( F("Motor Spin-up Lag = ") );
    Serial.println( MotorSpinupLag );
    Serial.println( F("Change with #ML-xxx$  (xxx = 000 - 999 milliseconds)\n") );
 
    Serial.print( F("Motor Compensation Bias = ") );
    Serial.println( MotorCompensationBias );
    Serial.println( F("Change with #MB-xxx$  (xxx = 000 - 100 percent)\n") );
    
    Serial.print( F("Battery Offset = ") );
    Serial.println( BatteryOffset );
    Serial.println( F("Change with #BO-pxx$  (p= + or -)xx = -2.0 - +2.0 volts)\n") );
  
    Serial.println( F("--------------------") );
    Serial.println( F("Sel Fire Modes:") );
    Serial.println( F("--------------------") );
    Serial.println( F("0 = Single Shot") );
    Serial.println( F("1 = Burst Shot") );
    Serial.println( F("2 = Full Auto") );
    Serial.println( F("3 = Binary Trigger") );
    Serial.println( F("4 = Ramped Trigger") );
    Serial.println( F("5 = Zerg") );
    Serial.println( F("6 = Safety") );
    Serial.println( F("7 = No Action") );

    Serial.println( F("=====================") );
    Serial.println( F("Blaster Status:") );
    Serial.println( F("=====================") );
    Serial.print( F("Version = ") );
    Serial.println( VERSION );
    Serial.print( F("Battery Voltage = ") );
    char VoltBuffer[5];
    sprintf( VoltBuffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    VoltBuffer[4] = 0;
    VoltBuffer[3] = VoltBuffer[2];
    VoltBuffer[2] = '.';
    Serial.println( VoltBuffer );

    Serial.print( F("Motor Ramp Up Rate = ") );
    Serial.println( MotorRampUpPerMS );
    
    Serial.print( F("Motor Ramp Down Rate = ") );
    Serial.println( MotorRampDownPerMS );

    Serial.print( F("System Mode = ") );
    Serial.println( SystemMode );
    Serial.println( F("  0 = Normal, 1 = Lowbat") );
    
    Serial.print( F("Full Trigger State = ") );
    Serial.println( GET_FIRE_T );

    Serial.print( F("Select Fire A State = ") );
    Serial.println( GET_SEL_FIRE_A );

    Serial.print( F("Select Fire B State = ") );
    Serial.println( GET_SEL_FIRE_B );    

    Serial.print( F("Select Fire Mode = ") );
    Serial.println( CurrentFireMode );    

    Serial.print( F("Calculated Pulse Time = ") );
    Serial.println( map( (int)(BatteryCurrentVoltage*10), SolenoidHighVoltage, SolenoidLowVoltage, PulseOnTimeLow, PulseOnTimeHigh ) );

    Serial.println( F("--------------------\n") );
              
  }  
}

//--------------------------------------------- Calculate Ramp Frequency Subroutine -----------------------------------
void CalculateRampFrequency()
{
  RampFrequency = 1000 / RampDPS;
}

//--------------------------------------------- Perform the requested mapped function Subroutine --------------------
void PerformButtonAction( byte ActionIDX )
{
  switch( ActionIDX )
  {
    case BTN_SINGLE:
      CurrentFireMode = FIRE_MODE_SINGLE;
      break;
    case BTN_BURST:
      CurrentFireMode = FIRE_MODE_BURST;
      break;
    case BTN_AUTO:
      if( CurrentFireMode != FIRE_MODE_AUTO_LASTSHOT )
        CurrentFireMode = FIRE_MODE_AUTO;
      break;
    case BTN_SAFE:
      CurrentFireMode = FIRE_MODE_SAFE;
      break;
    case BTN_BINARY:
      CurrentFireMode = FIRE_MODE_BINARY;
      break;
    case BTN_RAMP:
      CurrentFireMode = FIRE_MODE_RAMPED;
      break;
    case BTN_ZERG:
      CurrentFireMode = FIRE_MODE_ZERG;
      break;
    case BTN_NOTHING: default:
      // Do nothing
      break;
  }
}
// THE END
