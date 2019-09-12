#include "myFunctions.h"

#pragma region VARIABLES

#pragma region ADC Multisampled Value
volatile uint16_t adcEXT                = 0;                              // ADC Value of PIN_AI_EXT with Multisampling
volatile uint16_t adcIREF               = 0;                              // ADC Value of PIN_AI_IREF with Multisampling
volatile uint16_t adcVA                 = 0;                              // ADC Value of PIN_AI_VA with Multisampling
volatile uint16_t adcIAE                = 0;                              // ADC Value of PIN_AI_IAE with Multisampling
volatile uint16_t adcUAE                = 0;                              // ADC Value of PIN_AI_UAE with Multisampling
#pragma endregion // ADC Multisampled Values

#pragma region DAC Values
volatile uint16_t  dacVA                 = PIN_AO_VA_DEFAULT;              // DAC Value of PIN_AO_VA set by PID Control
volatile uint16_t  dacIREF               = PIN_AO_IREF_DEFAULT;            // DAC Value of PIN_AO_IREF
#pragma endregion // DAC Values

#pragma region DI Values
volatile bool dinLP                     = false;                          // Actual Digital Value for the Input Pin PIN_DI_LP
volatile bool dinGAS                    = false;                          // Actual Digital Value for the Input Pin PIN_DI_GAS
volatile bool dinARM                    = false;                          // Actual Digital Value for the Input Pin PIN_DI_ARM
#pragma endregion // DI Values

#pragma region DO Values
volatile bool douARM                    = PIN_DO_ARM_DEFAULT;             // Actual Digital Value for the Output Pin PIN_DO_ARM
volatile bool douGAS                    = PIN_DO_GAS_DEFAULT;             // Actual Digital Value for the Output Pin PIN_DO_GAS
volatile bool douLP                     = PIN_DO_LP_DEFAULT;              // Actual Digital Value for the Output Pin PIN_DO_LP
#pragma endregion // DO Values

#pragma region ADC FIFOs for Multisampling
uint16_t ADC_BUFFER_EXT [ ADC_FIFO_LENGTH + 1 ];                          // FIFO Used for Multisampling the adcExt, First Index is used to calculate the sum of samples
uint16_t ADC_BUFFER_IREF[ ADC_FIFO_LENGTH + 1 ];                          // FIFO Used for Multisampling the adcIref, First Index is used to calculate the sum of samples
uint16_t ADC_BUFFER_VA  [ ADC_FIFO_LENGTH + 1 ];                          // FIFO Used for Multisampling the adcVa, First Index is used to calculate the sum of samples
uint16_t ADC_BUFFER_IAE [ ADC_FIFO_LENGTH + 1 ];                          // FIFO Used for Multisampling the adcIAE, First Index is used to calculate the sum of samples
uint16_t ADC_BUFFER_UAE [ ADC_FIFO_LENGTH + 1 ];                          // FIFO Used for Multisampling the adcUAE, First Index is used to calculate the sum of samples
hw_timer_t * adcTimer                   = NULL;                           // Timer used to Periodically get a new sample
volatile bool flagADCInterrupt          = false;                          // Flag that an interruption of adcTimer happen recently
#pragma endregion // ADC FIFOs for Multisampling

#pragma region PID
volatile bool flagIteratePID            = false;
volatile bool flagControlEnabled        = false;                          // Flag to determine if the PID is active
uint32_t lastMeasuredTime               = 0;                              // The time since the start of microcontroller and the last interruption
volatile uint32_t lastMeasuredPeriod    = 9999999;                        // The time since last interrupt in ms, it starts as max value to force PID to work
volatile uint32_t setPointPeriod        = 40;                             // The desired Period in [us] to PID Control
volatile double setPointFrequency       = 30;                             // The desired Frequency in [Hz]
int16_t pidOutput[3]                    = {0,0,0};                        // Last 3 Discrete Values of PID Output
int32_t pidError[3]                     = {0,0,0};                        // Last 3 Discrete Values of Error (SetPoint - actualValue)
hw_timer_t * pidTimer                   = NULL;                           // Timer used to Periodically iterate the PID Control
volatile bool flagDinAmpInterrupt       = false;                          // Flag that an interruption of PIN_DI_AMP happen recently
volatile uint8_t reportBoostCounter     = 0;                              // If this counter equals TICKS_TO_BOOST, setVA to max value
volatile uint8_t reportDropCounter      = 0;                              // If this counter equals TICKS_TO_DROP, setVA to min value
#pragma endregion // PID

#pragma region IHM COMMUNICATION
volatile bool flagGiveFullReport        = true;                           // REPORT ALL I/O
bool     stillBuffering                 = false;                          // SERVER IS STILL COMMUNICATING WITH THIS UNIT
String   rxBuffer                       = "";                             // BUFFER OF IHM COMMANDS CHARS
bool flagEstimatingPlant                = false;                          // If True report every handleInterruptDinAmp trigger on UART Serial
#pragma endregion // IHM COMMUNICATION

#pragma endregion // VARIABLES


// ------------------------

void startControl(){
  douGAS = true;
  digitalWrite( PIN_DO_GAS, douGAS );
  delay(1000);

  douLP = true;
  digitalWrite( PIN_DO_LP, douLP );
  delay(1000);

  while( adcIAE < 10 ){
    refreshADC();
    delay(1);
  }

  douARM = true;
  digitalWrite( PIN_DO_ARM, douARM );  
}

#pragma region FUNCTIONS

#pragma region INTERRUPT HANDLERS
// Interrupt Handler for PIN_DI_AMP Rising Edge
void IRAM_ATTR handleInterruptDinAmpRising() {
  if( flagDinAmpInterrupt ){ return; }                                    // Avoiding problems with the watch-dog timer
  uint32_t now = micros();                                                // Calculate the time since power on
  uint32_t aux = ( now - lastMeasuredTime );                              // Calculate the period since last interrupt
  if( aux > PIN_DI_AMP_DEBOUNCER_TIME_US ){                               // Ignore noise, bouncing and slew rate causing mistriggering
    reportBoostCounter = 0;
    lastMeasuredPeriod = aux;                                             // Sample for PID Control
    lastMeasuredTime = now;                                               // Update last interrupt time
    flagDinAmpInterrupt = true;                                           // Set the flag
  }
}

void IRAM_ATTR handleInterruptDinAmpFalling(){
  reportDropCounter = 0;
}

// Interrupt Handler for PIN_DI_LP Change
void IRAM_ATTR handleInterruptDinLP(){
  dinLP = digitalRead(PIN_DI_LP);
}

// Interrupt Handler for PIN_DI_GAS Change
void IRAM_ATTR handleInterruptDinGAS(){
  dinGAS = digitalRead(PIN_DI_GAS);
}
 
// Interrupt Handler for PIN_DI_ARM Change
void IRAM_ATTR handleInterruptDinARM(){
  dinARM = digitalRead(PIN_DI_ARM);
}

// Interrupt Handler for PID Timer
void handleTimerPID(){
  static uint16_t reportLoopsCounter = 0;
  
  // check if it is necessary to give a boost on output
  if( flagControlEnabled ){
    if( reportBoostCounter < TICKS_TO_BOOST ){
      reportBoostCounter++;
    }

    if( reportDropCounter < TICKS_TO_DROP ){
      reportDropCounter++;
    }
  }

  // flag it is time to report all data
  reportLoopsCounter++;
  if( reportLoopsCounter == REPORT_EVERY_N_CONTROL_LOOPS ){
    reportLoopsCounter = 0;
    flagGiveFullReport = true;
  }

  flagIteratePID = true;
}

// Interrupt Handler for ADC Multisampling Timer
void adcInterruptHandler(){
  flagADCInterrupt = true;
}

// Frequently check for interruption flags
void checkFlags(){
  // Check for the ADC Timer Interrupt
  if( flagADCInterrupt ){
    flagADCInterrupt = false;
    refreshADC();
  }

  // Check for PID Timer Interrupt
  if( flagIteratePID ){
    flagIteratePID = false;
    //iteratePID();

    if( !flagEstimatingPlant ){
      giveProcessOutput();
    }

    double auxVREF = 1.314717995151464*adcUAE -7.751779563719978;
    if( auxVREF < 0 ){ auxVREF = 0; }
    if( auxVREF > 1023 ){ auxVREF = 1023; }
    setVREF( (uint16_t)auxVREF );
  }

  // Check for the PIN_DI_AMP Interrupt
  if(flagDinAmpInterrupt)
  {
    flagDinAmpInterrupt = false;
    reportBoostCounter = 0;
    reportDropCounter = 0;

    if( flagEstimatingPlant ){
      Serial.println( lastMeasuredPeriod );
    }
    else{
      iterateP();
    }
  }
  else if( flagControlEnabled ){

    if( reportDropCounter == TICKS_TO_DROP-1 ){
      //reportDropCounter = 0;
      reportBoostCounter = 0;
      dacVA = 0;
      setVA( 0 );
    }
    else if(reportBoostCounter == TICKS_TO_BOOST-1 ){
      reportDropCounter = 0;
      //reportBoostCounter = 0;
      dacVA = ANALOG_OUTPUT_MAX_VALUE;
      setVA( ANALOG_OUTPUT_MAX_VALUE );
    }

  }

  // Report all Inputs and Outputs to IHM
  if( flagGiveFullReport ){
    flagGiveFullReport = false;

    if( !flagEstimatingPlant ){
      giveFullReport();
    }
  }


}

// Updates the ADC FIFOs and calculates the average value
void refreshADC(){ 
  // Starts the static counter variable for FIFO position
  static uint8_t counter = 1; // Index 0 is used to the sum
  static int adcReading = 0;

  // Gets a new sample >> Delete the oldest sample from the FIFO sum >> Sets the new sample to the FIFO >> Increase FIFO sum with the new sample
  adcReading = adc1_get_raw( PIN_AI_EXT   ); ADC_BUFFER_EXT [ 0 ] += adcReading - ADC_BUFFER_EXT [ counter ]; ADC_BUFFER_EXT [ counter ] = adcReading;
  adcReading = adc1_get_raw( PIN_AI_IREF  ); ADC_BUFFER_IREF[ 0 ] += adcReading - ADC_BUFFER_IREF[ counter ]; ADC_BUFFER_IREF[ counter ] = adcReading;
  adcReading = adc1_get_raw( PIN_AI_VA    ); ADC_BUFFER_VA  [ 0 ] += adcReading - ADC_BUFFER_VA  [ counter ]; ADC_BUFFER_VA  [ counter ] = adcReading;
  adcReading = adc1_get_raw( PIN_AI_IAE   ); ADC_BUFFER_IAE [ 0 ] += adcReading - ADC_BUFFER_IAE [ counter ]; ADC_BUFFER_IAE [ counter ] = adcReading;
  
  // Updates ADC2 Channels
  if( ESP_OK == adc2_get_raw( PIN_AI_UAE, ADC_BITS_RESOLUTION, &adcReading) ){
    ADC_BUFFER_UAE [ 0 ] += adcReading - ADC_BUFFER_UAE [ counter ]; ADC_BUFFER_UAE [ counter ] = adcReading;
  }

  // Updates FIFO position
  counter++;
  if( counter > ADC_FIFO_LENGTH ){ counter = 1; }

  // Updates ADC Outputs with Filtered Values
  adcEXT  = ADC_BUFFER_EXT [ 0 ] / ADC_FIFO_LENGTH;
  adcIREF = ADC_BUFFER_IREF[ 0 ] / ADC_FIFO_LENGTH;
  adcVA   = ADC_BUFFER_VA  [ 0 ] / ADC_FIFO_LENGTH;
  adcIAE  = ADC_BUFFER_IAE [ 0 ] / ADC_FIFO_LENGTH;
  adcUAE  = ADC_BUFFER_UAE [ 0 ] / ADC_FIFO_LENGTH;
}

// Calculates the PID Output
void iteratePID(){
  //C[k] = C[k-2] + E[k] * ( P + I*T/2 + D*2/T ) + E[k-1] * ( I*T - D*4/T ) + E[k-2] * ( -P + I*T/2 + D*2/T )
  static float E2_GAIN = ( PID_KP + PID_KI*PID_SAMPLE_TIME_MS*0.5 + PID_KD*2.0/PID_SAMPLE_TIME_MS );
  static float E1_GAIN = ( PID_KI*PID_SAMPLE_TIME_MS - PID_KD*4.0/PID_SAMPLE_TIME_MS );
  static float E0_GAIN = ( -PID_KP + PID_KI*PID_SAMPLE_TIME_MS*0.5 + PID_KD*2.0/PID_SAMPLE_TIME_MS );
  
  if( !flagControlEnabled )
  {
    // Only execute a control loop if required
    return;
  }

  // Update Error
  pidError[0] = pidError[1];
  pidError[1] = pidError[2];
  pidError[2] = (setPointPeriod - lastMeasuredPeriod); // [us]

  // Update Output
  pidOutput[0] = pidOutput[1];
  pidOutput[1] = pidOutput[2];
  pidOutput[2] = pidOutput[0] + pidError[2]*E2_GAIN + pidError[1]*E1_GAIN + pidError[0]*E0_GAIN;
  //pidOutput[2] = pidOutput[0] + pidError[2]*( PID_KP + PID_KI*PID_SAMPLE_TIME_MS*0.5 + PID_KD*2.0/PID_SAMPLE_TIME_MS ) +
  //                              pidError[1]*( PID_KI*PID_SAMPLE_TIME_MS - PID_KD*4.0/PID_SAMPLE_TIME_MS ) +
  //                              pidError[0]*( -PID_KP + PID_KI*PID_SAMPLE_TIME_MS*0.5 + PID_KD*2.0/PID_SAMPLE_TIME_MS );

  // Saturation  
  pidOutput[2] = (pidOutput[2] > ANALOG_OUTPUT_MAX_VALUE ) ? ANALOG_OUTPUT_MAX_VALUE : pidOutput[2];
  pidOutput[2] = (pidOutput[2] < PID_MIN_VALUE ) ? PID_MIN_VALUE : pidOutput[2];

  // Set Analog Output
  dacVA = pidOutput[2];
  setVA( dacVA );
}

// Proportional Control
void iterateP(){
  if( !flagControlEnabled )
  {
    // Only execute a control loop if required
    return;
  }

  double error = setPointFrequency - (1000000.0 / lastMeasuredPeriod );
  double output = 0;
  
  // APPROACH A
  //output = P_KP * error;
  
  // APPROACH B
  const double steps = (153 / 50.0);
  if( error > 0 ){
    output += steps;
  }else{
    output -= steps;
  }

  // APPROACH C
  //const double stepGain = (153 / 50.0);
  //output += error * stepGain;

  if( output > ANALOG_OUTPUT_MAX_VALUE ){ output = ANALOG_OUTPUT_MAX_VALUE; }
  if( output < 0 ){ output = 0; }
  dacVA = (uint16_t)output;
  setVA(dacVA);
}
#pragma endregion // Interrupt Handlers

#pragma region SETUP
// Setup Digital Inputs
void setupDigitalInputs(){
  // Digital Inputs Setup
  //gpio_set_pull_mode( PIN_DI_LP, GPIO_PULLUP_PULLDOWN );
  //gpio_set_pull_mode( PIN_DI_GAS, GPIO_PULLUP_PULLDOWN );
  //gpio_set_pull_mode( PIN_DI_ARM, GPIO_PULLUP_PULLDOWN );
  pinMode( PIN_DI_LP , INPUT ); dinLP  = digitalRead( PIN_DI_LP  );                           // Digital Input for Power Enabled Check
  pinMode( PIN_DI_GAS, INPUT ); dinGAS = digitalRead( PIN_DI_GAS );                           // Digital Input for Gas Enabled Check
  pinMode( PIN_DI_ARM, INPUT ); dinARM = digitalRead( PIN_DI_ARM );                           // Digital Input for Wire Enabled Check
  attachInterrupt(digitalPinToInterrupt(PIN_DI_LP) , handleInterruptDinLP , CHANGE);          // Setup interrupt handler for change on digital input
  attachInterrupt(digitalPinToInterrupt(PIN_DI_GAS), handleInterruptDinGAS, CHANGE);          // Setup interrupt handler for change on digital input
  attachInterrupt(digitalPinToInterrupt(PIN_DI_ARM), handleInterruptDinARM, CHANGE);          // Setup interrupt handler for change on digital input 
  pinMode( PIN_DI_AMP, INPUT );                                                               // Digital Input for Operational Amplifier Comparator
  attachInterrupt(digitalPinToInterrupt(PIN_DI_AMP), handleInterruptDinAmpRising , RISING);          // Setup interrupt handler for rising edge on PIN_DI_AMP
  attachInterrupt(digitalPinToInterrupt(PIN_DI_AMP), handleInterruptDinAmpFalling, FALLING);         // Setup interrupt handler for falling edge on PIN_DI_AMP
}

// Setup Digital Outputs
void setupDigitalOutputs(){
  // Digital Outputs Setup
  pinMode( PIN_DO_ARM, OUTPUT ); digitalWrite( PIN_DO_ARM, PIN_DO_ARM_DEFAULT );
  pinMode( PIN_DO_GAS, OUTPUT ); digitalWrite( PIN_DO_GAS, PIN_DO_GAS_DEFAULT );
  pinMode( PIN_DO_LP , OUTPUT ); digitalWrite( PIN_DO_LP , PIN_DO_LP_DEFAULT  );
}

// Setup Analog to Digital Outputs
void setupAnalogInputs(){
   // Analog Inputs Setup
   adc1_config_width( ADC_BITS_RESOLUTION );                                            // ADC1 RESOLUTION
   adc1_config_channel_atten(PIN_AI_EXT , ADC_ATTEN_DB_11);                             // AI_EXT  CONFIG
   adc1_config_channel_atten(PIN_AI_IREF, ADC_ATTEN_DB_11);                             // AI_IREF CONFIG
   adc1_config_channel_atten(PIN_AI_VA  , ADC_ATTEN_DB_11);                             // AI_VA   CONFIG
   adc1_config_channel_atten(PIN_AI_IAE , ADC_ATTEN_DB_11);                             // AI_IAE  CONFIG
   adc2_config_channel_atten(PIN_AI_UAE , ADC_ATTEN_DB_11);                             // AI_UAE  CONFIG
   for(uint8_t i=0; i<ADC_FIFO_LENGTH; i++)                                             // ADC FIFOs DEFINITION
   {
     ADC_BUFFER_EXT [ i ] = 0;
     ADC_BUFFER_IREF[ i ] = 0;
     ADC_BUFFER_VA  [ i ] = 0;
     ADC_BUFFER_IAE [ i ] = 0;
     ADC_BUFFER_UAE [ i ] = 0;
   }
   refreshADC();                                                                        // RUN ONCE BEFORE THE ALARM, FIRST RUN TAKES LONGER THAN USUAL
   adcTimer = timerBegin( ADC_TIMER_CHANNEL, ADC_TIMER_PRESCALER, true );               // ADC TIMER CREATION
   timerAttachInterrupt( adcTimer, &adcInterruptHandler, true );                        // ADC TIMER HANDLER SETUP
   timerAlarmWrite(adcTimer, ADC_TIMER_COUNTER_MAX, true);                              // ADC TIMER INTERRUPT SETUP
   timerAlarmEnable(adcTimer);                                                          // ADC TIMER ENABLE
}

// Setup Digital to Analog Outputs
void setupAnalogOutputs(){
  #ifdef USE_PWM_AS_DAC
  setupPWMOutputs();
  #else
  setupDACOutputs();
  #endif
}

#ifdef USE_PWM_AS_DAC
// Setup PWM Outputs
void setupPWMOutputs(){
  // VAExt
  ledcSetup(PWM_VA_CHANNEL, PWM_VA_FREQ, PWM_VA_RESOLUTION);
  ledcAttachPin(PWM_VA_PIN, PWM_VA_CHANNEL);
  ledcWrite(PWM_VA_CHANNEL, PWM_VA_DEFAULT);

  // IREFExt
  ledcSetup(PWM_IREF_CHANNEL, PWM_IREF_FREQ, PWM_IREF_RESOLUTION);
  ledcAttachPin(PWM_IREF_PIN, PWM_IREF_CHANNEL);
  ledcWrite(PWM_IREF_CHANNEL, PWM_IREF_DEFAULT);

  // VREF
  ledcSetup(PWM_VREF_CHANNEL, PWM_VREF_FREQ, PWM_VREF_RESOLUTION);
  ledcAttachPin(PWM_VREF_PIN, PWM_VREF_CHANNEL);
  ledcWrite(PWM_VREF_CHANNEL, PWM_VREF_DEFAULT);
}

#else

void setupDACOutputs(){
  // Analog Outputs Setup
  dacWrite( PIN_AO_VA  , PIN_AO_VA_DEFAULT   );
  dacWrite( PIN_AO_IREF, PIN_AO_IREF_DEFAULT );
}
#endif

// Setup PID Control
void setupPIDControl(){
  // PID Setup
  pidTimer = timerBegin( PID_TIMER_CHANNEL, PID_TIMER_PRESCALER, true );               // PID TIMER CREATION
  timerAttachInterrupt( pidTimer, &handleTimerPID, true );                                 // PID TIMER HANDLER SETUP
  timerAlarmWrite(pidTimer, PID_TIMER_COUNTER_MAX, true);                              // PID TIMER INTERRUPT SETUP
  timerAlarmEnable(pidTimer);                                                          // START TIMER
}

// Setup UART Communication with HOST
void setupIHMCommunication(){
  // IHM Communication Setup
  Serial.begin( BAUDRATE );
  
  // Define rxBuffer
  for( uint8_t i=0; i<RX_BUFFER_SIZE; i++){
    rxBuffer[i] = 0;
  }
}
#pragma endregion // SETUP

// Set Analog Output Value for VAExt
void setVA(uint16_t value){
  if( value > ANALOG_OUTPUT_MAX_VALUE ){
    value = ANALOG_OUTPUT_MAX_VALUE;
  }
  #ifdef USE_PWM_AS_DAC
  ledcWrite(PWM_VA_CHANNEL, value);
  #else
  dacWrite( PIN_AO_VA, value );
  #endif
}

// Set Analog Output Value for IREFExt
void setIREF(uint16_t value){
  if( value > ANALOG_OUTPUT_MAX_VALUE ){
    value = ANALOG_OUTPUT_MAX_VALUE;
  }
  #ifdef USE_PWM_AS_DAC
  ledcWrite(PWM_IREF_CHANNEL, value);
  #else
  dacWrite( PIN_AO_IREF, value );
  #endif
}

void setVREF(uint16_t value){
  /*
  if( value > ANALOG_OUTPUT_MAX_VALUE ){
    value = ANALOG_OUTPUT_MAX_VALUE;
  }
  */
  ledcWrite(PWM_VREF_CHANNEL, value);
}
#pragma region IHM Communication

// Give Process Control Output Value (lastMeasuredPeriod)
void giveProcessOutput(){
  String command = String(START_OF_MESSAGE);
  command      += "GPO_"                     +
                  String(lastMeasuredPeriod) + COLUMN_SEPARATOR + 
                  String( dacVA , DEC )      + END_OF_MESSAGE;
  Serial.println( command );
}

// Give Full Report Data
void giveFullReport(){
  //* {GFR_ 1/f , DI_LP, DI_GAS, DI_ARM, DO_LP, DO_GAS, DO_ARM, AI_VA, AI_IAE, AI_IREF, AI_UAE, AI_VAE, AO_VA, AO_IREF }
  //* {GFR_ #5c , #c   , #c    , #c    , #c   , #c    , #c    , #3c  , #3c   , #3c    , #3c   , #3c   , #3c  , #3c     }
  //#53 bytes = START_CHAR (1 CHAR) + TOPIC (4 CHARS) + PERIOD (UP TO 5 CHARS) + DIGITAL I/O (6 CHARS) + ANALOG I/O (7*3 CHARS) + COMMAS (13 CHARS) + END_CHAR (1 CHAR) + NEW_LINE (2 CHARS)

  String command = String(START_OF_MESSAGE);
  command      += "GFR_"          +
                  String(lastMeasuredPeriod) + COLUMN_SEPARATOR +
                  (char)(48 + dinLP  )       + COLUMN_SEPARATOR +
                  (char)(48 + dinGAS )       + COLUMN_SEPARATOR +
                  (char)(48 + dinARM )       + COLUMN_SEPARATOR +
                  (char)(48 + douLP  )       + COLUMN_SEPARATOR +
                  (char)(48 + douGAS )       + COLUMN_SEPARATOR +
                  (char)(48 + douARM  )      + COLUMN_SEPARATOR +
                  String( adcVA   )          + COLUMN_SEPARATOR +
                  String( adcIAE  )          + COLUMN_SEPARATOR +
                  String( adcIREF )          + COLUMN_SEPARATOR +
                  String( adcUAE  )          + COLUMN_SEPARATOR +
                  String( adcEXT  )          + COLUMN_SEPARATOR +
                  String( dacVA  , DEC )     + COLUMN_SEPARATOR +
                  String( dacIREF, DEC )     + String(END_OF_MESSAGE);
  Serial.println( command );
}

// Check for incoming messages from IHM
void handleMessageFromIHM(String message){
  if( message.length() < 7 ){
    return;
  }
  String topic = message.substring(1,4);

  // All chars from i=5 to i=(n-1) should be a number
  for(int i=5; i<message.length()-1; i++){
    if( ( message.charAt(i) < 48 ) || ( message.charAt(i) > 57 ) )
    {
      return;
    }
  }
  int value = message.substring(5, message.length()-1 ).toInt();
  String answer = String(START_OF_MESSAGE);

  if(      topic == "SPC" ){
    reportBoostCounter = 0;
    reportDropCounter = 0;
    flagControlEnabled = value;
    answer += "GPC_" + message.substring(5);
    if( flagControlEnabled ){

      startControl();

    }
  }
  else if( topic == "SPE" ){
    flagEstimatingPlant = value;
    
    if( flagEstimatingPlant ){
      flagControlEnabled = 0;
      answer += "GPC_0" + String(END_OF_MESSAGE);
    }else{
      answer = message;
    }
  }
  else if( topic == "SPR" ){
    setPointPeriod = value;
    setPointFrequency = 1000000.0 / setPointPeriod;
    answer += "GPR_" + message.substring(5);
  }
  else if( topic == "DOP" ){
    douLP = value;
    digitalWrite( PIN_DO_LP, douLP );
    delay(1);
    dinLP = digitalRead( PIN_DI_LP );
    answer +=  "DIP_";
    answer += dinLP ? "1" : "0";
    answer += END_OF_MESSAGE;

    if( !douLP ){
      digitalWrite( PIN_DO_GAS, LOW );
      digitalWrite( PIN_DO_ARM, LOW );
    }
  }
  else if( topic == "DOW" ){
    douARM = value;
    digitalWrite( PIN_DO_ARM, douARM );
    delay(1);
    dinARM = digitalRead( PIN_DI_ARM );
    answer +=  "DIW_";
    answer += dinARM ? "1" : "0";
    answer += END_OF_MESSAGE;
  }
  else if( topic == "DOG" ){
    douGAS = value;
    digitalWrite( PIN_DO_GAS, douGAS );
    delay(1);
    dinGAS = digitalRead( PIN_DI_GAS );
    answer +=  "DIG_";
    answer += dinGAS ? "1" : "0";
    answer += END_OF_MESSAGE;
  }
  else if( topic == "AWW" ){
    dacVA = value;
    setVA( dacVA );
    answer = message;
  }
  else if( topic == "AWC" ){
    dacIREF = value;
    setIREF( value );
    answer = message;
  }
  Serial.println( answer );
}

// Updates the rxBuffer
void checkForIncommingMessages()
{
  if (Serial.available() > 0)
  {
    stillBuffering = false;
    //------------------------------------------------------------
    //     READING UART VALUES AS VALID ASCII CHARACTERS ONLY
    //------------------------------------------------------------
    char buf1[RX_BUFFER_SIZE];
    uint8_t bufLen = Serial.readBytes(buf1, RX_BUFFER_SIZE);
    buf1[bufLen] = '\0';
    uint8_t i=0, j = 0;
    for (i = 0; i < bufLen; i++)
    {
      if ((buf1[i] == START_OF_MESSAGE) or (buf1[i] == END_OF_MESSAGE) or ((buf1[i] > 32) && (buf1[i] < 127)))
      {
        buf1[j++] = buf1[i];
      }
    }
    buf1[j] = '\0';

    rxBuffer += buf1;
    //------------------------------------------------------------
  }

  if ((rxBuffer.length() > 0) and (stillBuffering == false))
  {
    //FLUSHING CHARS UNTILL A VALID START CHAR IS FOUND
    while ((rxBuffer.indexOf(START_OF_MESSAGE) != 0) and (rxBuffer.length() > 0))
    {
      rxBuffer = rxBuffer.substring(1);
    }

    //WORD RECEIVED STARTS WITH A VALID START CHAR
    if( rxBuffer.indexOf(START_OF_MESSAGE) == 0 ) 
    {
      stillBuffering = true;
      
      //WORD RECEIVED HAS THE VALID END CHAR
      if (rxBuffer.indexOf(END_OF_MESSAGE) >= 4)
      {
        stillBuffering = false;
        handleMessageFromIHM( rxBuffer.substring( 0, rxBuffer.indexOf(END_OF_MESSAGE)+1 ) );

        //DELETING THE PART OF THE MSG THAT HAS ALREADY BEEN READ
        if (rxBuffer.indexOf(END_OF_MESSAGE) > 0)
        {
          rxBuffer = rxBuffer.substring(rxBuffer.indexOf(END_OF_MESSAGE) + 1);
        }
      }
    }
  }
}

#pragma endregion // IHM COMMUNICATION


// -------------------------------------------

#pragma region DEBUG ONLY

void debug_simulateOperation(){
  while( Serial.available() < 1 ){
    delay(15);
  }

  //--
  Serial.println(1e6 / 0.157388828111397,0);delay(1e3 * (3.818181818181818 - 0.636363636363636) );
  Serial.println(1e6 / 0.758314431462044,0);delay(1e3 * (5.090909090909091 - 3.818181818181818) );
  Serial.println(1e6 / 0.872284554822391,0);delay(1e3 * (7.318181818181818 - 5.090909090909091) );
  Serial.println(1e6 / 0.793854242604116,0);delay(1e3 * (11.136363636363637 - 7.318181818181818) );
  Serial.println(1e6 / 0.929656123521289,0);delay(1e3 * (14.636363636363637 - 11.136363636363637) );
  Serial.println(1e6 / 1.359911723666700,0);delay(1e3 * (18.136363636363637 - 14.636363636363637) );
  Serial.println(1e6 / 1.437843280162771,0);delay(1e3 * (21.000000000000000 - 18.136363636363637) );
  Serial.println(1e6 / 1.285461733498251,0);delay(1e3 * (24.500000000000000 - 21.0) );
  Serial.println(1e6 / 1.431378081256490,0);delay(1e3 * (27.045454545454547 - 24.5) );
  Serial.println(1e6 / 1.003427404834357,0);delay(1e3 * (29.909090909090910 - 27.045454545454547) );

  while(1){
    delay(15);
  }

  while( 1 ){
    for(uint8_t i=0; i<255; i++){

      String message = "" + String( START_OF_MESSAGE);
      switch (i)
      {
      // GPC
      case 0:
        message += "GPC_0" + String( END_OF_MESSAGE );
        break;
      
      case 1:
        message += "GPC_1" + String( END_OF_MESSAGE );
        break;

      case 2:
        message += "GPR_33333" + String( END_OF_MESSAGE );
        break;
        
      case 3:
        message += "GPR_25000" + String( END_OF_MESSAGE );
        break;

      case 4:
        message += "GPO_33333,127" + String( END_OF_MESSAGE );
        break;
        
      case 5:
        message += "GPO_25000,255" + String( END_OF_MESSAGE );
        break;
        
      case 6:
        message += "DIP_0" + String( END_OF_MESSAGE );
        break;
        
      case 7:
        message += "DIP_1" + String( END_OF_MESSAGE );
        break;

      case 8:
        message += "DIW_0" + String( END_OF_MESSAGE );
        break;
        
      case 9:
        message += "DIW_1" + String( END_OF_MESSAGE );
        break;

      case 10:
        message += "DIG_0" + String( END_OF_MESSAGE );
        break;
      
      case 11:
        message += "DIG_1" + String( END_OF_MESSAGE );
        break;

      case 12:
        message += "ARW_131" + String( END_OF_MESSAGE );
        break;

      case 13:
        message += "ARW_511" + String( END_OF_MESSAGE );
        break;
        
      case 14:
        message += "ARR_131" + String( END_OF_MESSAGE );
        break;

      case 15:
        message += "ARR_511" + String( END_OF_MESSAGE );
        break;
        
      case 16:
        message += "ARC_131" + String( END_OF_MESSAGE );
        break;

      case 17:
        message += "ARC_511" + String( END_OF_MESSAGE );
        break;

      case 18:
        message += "ARU_131" + String( END_OF_MESSAGE );
        break;

      case 19:
        message += "ARU_511" + String( END_OF_MESSAGE );
        break;

      case 20:
        message += "ARV_131" + String( END_OF_MESSAGE );
        break;

      case 21:
        message += "ARV_511" + String( END_OF_MESSAGE );
        break;
      
      default:
        lastMeasuredPeriod += 1;
        dinLP = !dinLP;
        dinGAS = !dinGAS;
        dinARM = !dinARM;
        douLP = !douLP;
        douGAS = !douGAS;
        douARM = !douARM;
        adcVA += 1;
        adcIAE += 1;
        adcIREF += 1;
        adcUAE += 1;
        adcEXT += 1;
        dacVA += 1;
        dacIREF += 1;
        giveFullReport();
        continue;
        break;
      }

      Serial.println( message );
      delay( 1000 );

    }
  }
}

void debug_adcCalibration(){
  while( Serial.available() < 1){
    delay(15);
  }
  Serial.println("DAC;EXT;IAE;IREF;VA;UAE");
  
  for(uint16_t dacOutput=0; dacOutput<ANALOG_OUTPUT_MAX_VALUE+1; dacOutput++){
    setVA( dacOutput );

    for(uint16_t j=0; j<ADC_FIFO_LENGTH; j++){
      refreshADC();
      delayMicroseconds(400);
    }

    for(uint8_t i=0; i<50; i++){
      Serial.print(dacOutput);Serial.print(";");
      Serial.print(adcEXT);Serial.print(";");
      Serial.print(adcIAE);Serial.print(";");
      Serial.print(adcIREF);Serial.print(";");
      Serial.print(adcVA);Serial.print(";");
      Serial.print(adcUAE);Serial.print(";");
      Serial.println();
      
      for(uint16_t j=0; j<ADC_FIFO_LENGTH; j++){
        refreshADC();
        delayMicroseconds(400);
      }
      
    }
  }
}

void debug_dacCalibration(){
  while( Serial.available() < 4){
    delay(15);
  }

  for(uint8_t dacOutput=0; dacOutput<ANALOG_OUTPUT_MAX_VALUE+1; dacOutput+=16){
    setVA( dacOutput );
    Serial.print(dacOutput,DEC);
    Serial.print(" , ");

    while( Serial.available() < 4){
      delay(15);
    }
    while( Serial.available() > 0){
      Serial.write( Serial.read() );
    }
  }
}

void debug_dacTest(){
  while( 1 ){
    for(uint16_t i=0; i<256; i++){
      setVA( i );
      setIREF( i );
      delay(3);
    }
    for(int16_t i=254; i>-1; i--){
      setVA( i );
      setIREF( i );
      delay(3);
    }
  }
}
#pragma endregion // DEBUG ONLY

#pragma endregion // FUNCTIONS
