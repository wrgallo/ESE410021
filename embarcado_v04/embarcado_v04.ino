#include "myFunctions.h"

void setup()
{
  // PINOUT Setup
  setupDigitalInputs();
  setupDigitalOutputs();
  setupAnalogInputs();
  setupAnalogOutputs();
  setupIHMCommunication();
  setupPIDControl();

  //debug_simulateOperation();
  //debug_dacTest(); // Devo observar uma curva dente de serra de 0 a 10V com periodo de 765 mili-segundos
}

void loop()
{
  checkFlags();
  checkForIncommingMessages();
}
