#ifndef _MY_FUNCTIONS_
#define _MY_FUNCTIONS_

#include "Arduino.h"
#include <driver/adc.h>


/*
     --------------------------------------------------------------------------
    |  NODE32S PIN  |  NODE32 I/O |    DB15 PIN     |       PURPOSE            |
    |---------------|-------------|-----------------|--------------------------|
    |      13       | Digital In  |    01 (LP)      |  Check LP status         |
    |      18       | Digital In  |    06 (GAS)     |  Check GAS status        |
    |      27       | Digital In  |    07 (ARM)     |  Check ARM status        |
    |      02       | Digital In  |    -----------  |  Comparator status       |
    |      21       | Digital Out |    15 (ARMExt)  |  Set ARM status          |
    |      22       | Digital Out |    14 (GASExt)  |  Set GAS status          |
    |      23       | Digital Out |    09 (LPExt)   |  Set LP status           |
    |      25       | Analog Out  |    05 (VAExt)   |  Set Wire Speed Level    |
    |      26       | Analog Out  |    04 (IREFExt) |  Set Current Level       |
    |      32       | Analog In   |    -----------  |  External Voltage Source |
    |      33       | Analog In   |    12 (IREF)    |  Check Current Ref Level |
    |      34       | Analog In   |    13 (VA)      |  Check Wire Speed Level  |
    |      35       | Analog In   |    02 (IAE)     |  Check Current Level     |
    |      04       | Analog In   |    03 (UAE)     |  Check Voltage Level     |
    |      GND      | Ground      |   10,11 (GND)   |  Common Ground           |
     --------------------------------------------------------------------------

     ------------------------------------------------------------------------------------------------------------------
    |                                                   CMD TABLE (IHM -> MCU)                                         | 
    |------------------------------------------------------------------------------------------------------------------|
    |                         DESCRIPTION                        |                         COMMAND                     |
    |------------------------------------------------------------|-----------------------------------------------------|
    | SET Process Control (Disable/Enable) value # (0 or 1)      | {SPC_#}                                             |
    | SET Estimating Plant Flag value # (0 or 1)                 | {SPE_#}                                             |
    | SET Process SetPoint Reference value # (0 to 65535)        | {SPR_#####}                                         |
    | SET Digital Output Power (LPExt) value # (0 or 1)          | {DOP_#}                                             |
    | SET Digital Output Wire (ARMExt) value # (0 or 1)          | {DOW_#}                                             |
    | SET Digital Output Gas (GASExt) value # (0 or 1)           | {DOG_#}                                             |
    | SET Analog Write Wire Speed (VAExt) value ### (000 to 255) | {AWW_###}                                           |
    | SET Analog Write Current (IREFExt) value ### (000 to 255)  | {AWC_###}                                           |
     ------------------------------------------------------------------------------------------------------------------

     -----------------------------------------------------------------------------------------------------------------
    |                                                  CMD TABLE (MCU -> IHM)                                         | 
    |-----------------------------------------------------------------------------------------------------------------|
    |                         DESCRIPTION                       |                         COMMAND                     |
    |-----------------------------------------------------------|-----------------------------------------------------|
    | GET Process Control Enabled Status value #                | {GPC_#}                                             |
    | GET Process SetPoint Reference value #                    | {GPR_#}                                             |
    | GET Process Control I/O value #                           | {GPO_#}                                             | ! To Do
    | GET Digital Input Power (LP) value #                      | {DIP_#}                                             |
    | GET Digital Input Wire (ARM) value #                      | {DIW_#}                                             |
    | GET Digital Input Gas (GAS) value #                       | {DIG_#}                                             |
    | GET Analog Read Wire Speed (VA) value ###                 | {ARW_###}                                           |
    | GET Analog Read SetPoint (IREF) value ###                 | {ARR_###}                                           |
    | GET Analog Read Current (IAE)  value ###                  | {ARC_###}                                           |
    | GET Analog Read Voltage (UAE)  value ###                  | {ARU_###}                                           |
    | GET Analog Read External Voltage (VAE)  value ###         | {ARV_###}                                           |
    | GET Full Report                                           | {GFR_#####;#;#;#;#;#;#;###;###;###;###;###;###;###} |    
     -----------------------------------------------------------------------------------------------------------------
*/

#pragma region CONSTANTS

#pragma region DIGITAL INPUTS
const uint8_t PIN_DI_LP                       = 13;             // DB15_01
const uint8_t PIN_DI_GAS                      = 18;             // DB15_06
const uint8_t PIN_DI_ARM                      = 27;             // DB15_07
const uint8_t PIN_DI_AMP                      = 02;             // HIGH when PIN_AI_EXT > PIN_AO_AMP_REF value or external reference
const uint8_t PIN_DI_AMP_DEBOUNCER_TIME_US    = 10000;          // (10000 means max of 100Hz trigerring) Minimum time in [us] to consider since last RISING EDGE
#pragma endregion // DIGITAL INPUTS

#pragma region DIGITAL OUTPUTS
const uint8_t PIN_DO_ARM                      = 21;             // DB15_15
const uint8_t PIN_DO_GAS                      = 22;             // DB15_14
const uint8_t PIN_DO_LP                       = 23;             // DB15_09
const bool PIN_DO_ARM_DEFAULT                 = LOW;            // SIGNAL STATE AT START
const bool PIN_DO_GAS_DEFAULT                 = LOW;            // SIGNAL STATE AT START
const bool PIN_DO_LP_DEFAULT                  = LOW;            // SIGNAL STATE AT START
#pragma endregion // DIGITAL OUTPUTS

#pragma region ANALOG_OUTPUTS // ANALOG OUTPUTS

#define USE_PWM_AS_DAC

#ifdef USE_PWM_AS_DAC
const uint16_t ANALOG_OUTPUT_MAX_VALUE       = 256;
#else
const uint16_t ANALOG_OUTPUT_MAX_VALUE       = 255;
#endif

#pragma region PWM OUTPUTS
const uint8_t  PWM_VA_CHANNEL                = 0;              // PWM CHANNEL
const uint32_t PWM_VA_FREQ                   = 78000;          // PWM FREQUENCY [Hz]
const uint8_t  PWM_VA_RESOLUTION             = 10;             // DUTY CICLE RESOLUTION FOR PWM
const uint8_t  PWM_VA_PIN                    = 25;             // PWM PIN
const uint16_t PWM_VA_DEFAULT                = 511;            // SIGNAL STATE AT START

const uint8_t  PWM_IREF_CHANNEL              = 1;              // PWM CHANNEL
const uint32_t PWM_IREF_FREQ                 = 78000;          // PWM FREQUENCY [Hz]
const uint8_t  PWM_IREF_RESOLUTION           = 10;             // DUTY CICLE RESOLUTION FOR PWM
const uint8_t  PWM_IREF_PIN                  = 26;             // PWM PIN
const uint16_t PWM_IREF_DEFAULT              = 511;            // SIGNAL STATE AT START

const uint8_t  PWM_VREF_CHANNEL              = 2;              // PWM CHANNEL
const uint32_t PWM_VREF_FREQ                 = 5000;           // PWM FREQUENCY [Hz]
const uint8_t  PWM_VREF_RESOLUTION           = 10;             // DUTY CICLE RESOLUTION FOR PWM
const uint8_t  PWM_VREF_PIN                  = 19;             // PWM PIN
const uint16_t PWM_VREF_DEFAULT              = 172;            // SIGNAL STATE AT START
#pragma endregion // PWM OUTPUTS 

#pragma region DAC OUTPUTS
const uint8_t PIN_AO_AMP_REF                 = 19;             // PWM to set AO_AMP reference
const uint8_t PIN_AO_VA                      = 25;             // DB15_05
const uint8_t PIN_AO_IREF                    = 26;             // DB15_04
const uint16_t PIN_AO_AMP_REF_DEFAULT        = 0;              // SIGNAL STATE AT START
const uint16_t PIN_AO_VA_DEFAULT             = 0;              // SIGNAL STATE AT START
const uint16_t PIN_AO_IREF_DEFAULT           = 0;              // SIGNAL STATE AT START
#pragma endregion // DAC OUTPUTS

#pragma endregion // ANALOG OUTPUTS

#pragma region ANALOG INPUTS
const adc1_channel_t PIN_AI_EXT              = ADC1_CHANNEL_4;   // ADC1_CH4 GPIO32 # External Voltage Source
const adc1_channel_t PIN_AI_IREF             = ADC1_CHANNEL_5;   // ADC1_CH5 GPIO33 # DB15_12
const adc1_channel_t PIN_AI_VA               = ADC1_CHANNEL_6;   // ADC1_CH6 GPIO34 # DB15_13
const adc1_channel_t PIN_AI_IAE              = ADC1_CHANNEL_7;   // ADC1_CH7 GPIO35 # DB15_02
const adc2_channel_t PIN_AI_UAE              = ADC2_CHANNEL_0;   // ADC2_CH0 GPIO04 # DB15_03
const adc_bits_width_t ADC_BITS_RESOLUTION   = ADC_WIDTH_BIT_9;  // ADC1 and ADC2 RESOLUTION
const uint8_t ADC_TIMER_CHANNEL              = 0;                // ADC TIMER CHANNEL (0 TO 2)
const uint16_t ADC_TIMER_PRESCALER           = 80;               // Set 80 to 1us tick ADC TIMER PRESCALER (1 to 65535)
const uint16_t ADC_TIMER_COUNTER_MAX         = 400;              // Set 400 to 400us period ADC TIMER TICKS COUNTER (1 to 65535)
const uint8_t ADC_FIFO_LENGTH                = 32;               // Recommended: 32 , Max 127
#pragma endregion // ANALOG INPUTS

#pragma region CONTROL TIMER
const uint8_t  PID_TIMER_CHANNEL             = 1;                // PID TIMER CHANNEL (0 to 2)
const uint16_t PID_TIMER_PRESCALER           = 8000;             // 0.1 ms precision tick (8000 = 80e6 [Hz] * 0.1e-3 [seconds per tick])
const uint16_t PID_TIMER_COUNTER_MAX         = 1500;             // 150 ms = 1500 ticks   (0.15 seconds = 1500[ticks] * 0.1e-3 [seconds per tick])
#pragma endregion // CONTROL TIMER

#pragma region PID CONTROL SETUP
const uint16_t PID_SAMPLE_TIME_MS            = 150;              // SAMPLING TIME IN [ms] , Min of 10Hz
const float    PID_KP                        = 300.0;            // PROPORTIONAL GAIN                         
const float    PID_KI                        = 0;                // INTEGRATOR GAIN
const float    PID_KD                        = 0;                // DERIVATIVE GAIN
const uint16_t PID_MIN_VALUE                 = 0;                // PID OUTPUT SATURATION, MIN VALUE
const uint16_t PID_MAX_VALUE                 = 255;              // PID OUTPUT SATURATION, MAX VALUE
#pragma endregion // PID CONTROL SETUP

#pragma region P CONTROL SETUP
const uint8_t TICKS_TO_BOOST                 = 20;               // IF NO SAMPLE ON N TIMER PERIODS, THEN GIVE OUTPUT A BOOST
const uint8_t TICKS_TO_DROP                  = 20;               // IF NO TICKS DURING INTERVAL< THEN GIVE OUTPUT A DROP
const float    P_KP                          = 300.0;            // PROPORTIONAL GAIN
//Kp = (Tau_ma/Tau_mf - 1)/Kma
//Kp = (1 - ESS) / ( ESS * Kma ) = (100 - ESS%) / (Kma * ESS%) -> 99 / Kma
//Kma = Freq / PWM_DUTY
#pragma endregion // P CONTROL SETUP

#pragma region COMMUNCATION SETUP
const uint16_t REPORT_EVERY_N_CONTROL_LOOPS  = 100;              // Every 100*150ms = 15 seconds
const uint8_t RX_BUFFER_SIZE                 = 100;              // BUFFER FOR INCOMING MESSAGES
const char START_OF_MESSAGE                  = '{';              // START OF FRAME MESSAGE
const char END_OF_MESSAGE                    = '}';              // END OF FRAME MESSAGE
const char COLUMN_SEPARATOR                  = ';';              // COLUMNS SEPARATOR
const uint32_t BAUDRATE                      = 256000;           // UART BAUD RATE
#pragma endregion //COMMUNCATION SETUP

#pragma endregion // CONSTANTS


// ------------------------


#pragma region FUNCTIONS

#pragma region FLAG HANDLERS
void checkFlags();                                              // CHECK FOR A NEW PERIOD
void refreshADC();                                              // GET A SAMPLE FROM ADCs
void iteratePID();                                              // CALCULATES AN ITERATION OF THE PID CONTROLLER
void iterateP();
#pragma endregion // INTERRUPT HANDLERS

#pragma region SETUP
void setupDigitalInputs();                                      // SETUP DIGITAL INPUTS
void setupDigitalOutputs();                                     // SETUP DIGITAL OUTPUTS
void setupAnalogInputs();                                       // SETUP ANALOG INPUTS
void setupDACOutputs();                                         // SETUP DAC ANALOG OUTPUTS
void setupPWMOutputs();                                         // SETUP PWM OUTPUTS
void setupAnalogOutputs();                                      // SETUP ANALOG OUTPUTS
void setupPIDControl();                                         // SETUP PID CONTROL
void setupIHMCommunication();                                   // SETUP IHM COMMUNICATION
#pragma endregion // SETUP

#pragma region IHM COMMUNICATION
void checkForIncommingMessages();                                // CHECK MESSAGES FROM IHM
void handleMessageFromIHM(String message);                       // HANDLE A MESSAGE FROM IHM
void giveProcessOutput();                                        // REPORT THE LAST OBSERVED INPUT WITH THE LAST OUTPUT
void giveFullReport();                                           // REPORT ALL I/O AND PROCESS VARIABLE TO IHM
#pragma endregion // IHM COMMUNICATION

void setVA(uint16_t value);                                      // SET ANALOG OUTPUT VALUE FOR VAExt
void setIREF(uint16_t value);                                    // SET ANALOG OUTPUT VALUE FOR IREFExt
void setVREF(uint16_t value);

#pragma region DEBUG ONLY
void debug_simulateOperation();                                  // TEST COMMUNICATION WITH IHM
void debug_dacCalibration();                                     // COLLECT DATA TO CALIBRATE THE CURVE DAC x VOLTAGE
void debug_adcCalibration();                                     // COLLECT DATA TO CALIBRATE THE CURVE ADC x DAC
void debug_dacTest();                                            // CHECK DAC ON OSCILOSCOPE
#pragma endregion // DEBUG ONLY

#pragma endregion // FUNCTIONS

#endif
