// Created by: Emiel Scholten
// Thermal control script for Earth mockup. Space Engineering Practical 2025.
// 
// The script is controlled via an external button, connected to pin 2. This starts
// the thermal control. If the temperature is below startTemp, the current is set
// to 3.3 amps (the maximum). The PID loop is only started once that temperature
// is reached, because the integral part might cause the temperature to oscillate
// otherwise. There is an overtemp protection which shuts down power if temperature
// is higher than maxTemp.

#include <SoftwareSerial.h>
#include <ArduPID.h>

// Power supply communications
SoftwareSerial rs232(7, 8); // RX, TX

// PID control settings
ArduPID controller;
double targetTemp = 44.0; // room temp + 20 deg
double current;
double prevcurrent;
double p = 0.75;
double i = 0.0001;
double d = 6;
int updateinterval_PID = 1000; // Update interval in milliseconds
unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long previousPIDMillis = 0;   // will store last time the current was updated
unsigned long previousTempMillis = 0;
const double outputLow = 1.52; // Set lower bound of PID to 1.52 amp to prevent relay clicking noise
const double outputHigh = 2.3; // upper boound of PID to 2.3 amps to prevent relay clicking noise

// Startup control
bool startup = true;
const double startTemp = 39; // Temperature to start PID control

// Temperature probe
#define sensorPin A0 // Define the analog pin, the TMP36's Vout pin is connected to
double currentTemp;
double maxTemp = 47.0; // Maximum temperature for overtemp protection
//running average for reducing noise in the temperature data
const int numSamples = 40; // Number of samples for running average
int readings[numSamples];  // Array to store samples
int index = 0;             // Current index in the array
long total = 0;            // Sum of the samples
int updateinterval_Temp = 100; // Update interval in milliseconds

// For interrupt handling
const byte interruptPin = 2; // Button pin
volatile bool state = LOW;
unsigned long lastISRmillis = 0L;
bool running = false;

void setup()
{
  // Open USB serial communications
  Serial.begin(9600);
  Serial.println("Thermal control");
  Serial.println("Press button to start");

  delay(1000); //Wait for power supply to start
  // set the data rate for the SoftwareSerial port
  rs232.begin(9600);

  // Initialise power supply
  sendRS232("RCL1"); // Recall settings from program M1
  sendRS232("OCP1"); // Turn on overcurrent protection (shuts down power in case of a short circuit)
  sendRS232("ISET1:0");
  sendRS232("ISET2:0.5");
  sendRS232("VSET1:0");
  sendRS232("VSET2:7.0");

  // Initialise start/stop button interrupt
  pinMode(2, INPUT_PULLUP);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR_button_pressed, FALLING);

  // Initialise PID
  controller.begin(&currentTemp, &current, &targetTemp, p, i, d);
  controller.setOutputLimits(outputLow, outputHigh);

  // Initialise temperature probe
  analogReference(INTERNAL); // Set reference voltage of ADC to 1.1V instead of 5V
  for (int i = 0; i < numSamples; i++) {
    readings[i] = 0; // Initialize readings to zero
  }
}

void loop()
{ 
  currentMillis = millis();
  if (state) {
    button_press();
  }
  if (running && currentMillis - previousPIDMillis >= updateinterval_PID) {
    run_PID();
    previousPIDMillis = currentMillis;
  }
  if (running && currentMillis - previousTempMillis >= updateinterval_Temp) {
    get_temp();
    previousTempMillis = currentMillis;
  }
}

String sendRS232(String command) {
  // Serial.println(command); // for debugging
  rs232.print(command); // Use print() to send the command string
  
  // Wait for response from RS232
  delay(15);
  
  // Read entire response
  String response = "";
  while (rs232.available()) {
    response += (char)rs232.read();
    delay(1); // Small delay to allow more characters to arrive
  }
  return response;
}

// Interrupt handling
void ISR_button_pressed(void)
{
  if (millis() > lastISRmillis + 200)//200 mS debounce time
  {
    state = true;//set flag
    lastISRmillis = millis();
  }
}

void button_press() 
{
  if (!running) {
      running = true;
      sendRS232("ISET1:0"); // Set current to start at 0
      sendRS232("VSET1:30"); // Set voltage to 30 Volts
      Serial.println("Starting temperature control");
  } else {
      running = false;
      sendRS232("ISET1:0"); // Set current to 0
      sendRS232("VSET1:0"); // Set voltage to 0
      Serial.println("Stopping temperature control");
  }
  state = false;//reset interrupt flag
}

void get_temp() {
  // Get the voltage reading from the TMP36
  int reading = analogRead(sensorPin);
  // Convert that reading into voltage
  double voltage = reading * (1.1 / 1024.0);

  // Convert the voltage into the temperature in Celsius
  double newReading = (voltage - 0.5) * 100; // why 0.5? no idea

  // running average
  total -= readings[index];   // Subtract the oldest value from the sum
  readings[index] = newReading; // Store new reading in the array
  total += newReading;        // Add the new value to the sum
  
  index = (index + 1) % numSamples; // Move index in circular manner
  
  currentTemp = total / (float)numSamples; // Compute the running average
  
  // Switch from high current starting mode to PID controlled mode at startTemp
  if (currentTemp > startTemp) {
    startup = false;
  } else {
    startup = true;
  }
}

void run_PID()
{
  // Print the temperature in Celsius
  Serial.println(currentTemp);

  // overtemp protection
  if (currentTemp > maxTemp) 
  {
    running = false;
    Serial.println("Overtemperature protection triggered.");
    sendRS232("OUT0"); // Turn off power supply output
  }
  else if (startup) {
    sendRS232("ISET1:3.3"); // In startup mode, deliver max power
  } else {
    // run PID
    controller.compute();
    sendRS232("ISET1:" + String(current, 3));  // Convert float to string with 3 decimal places

    // Uncomment to print PID debug
    // controller.debug(&Serial, "Controller", PRINT_INPUT    | // Can include or comment out any of these terms to print
    //                                         PRINT_OUTPUT   | // in the Serial plotter
    //                                         PRINT_SETPOINT |
    //                                         PRINT_BIAS     |
    //                                         PRINT_P        |
    //                                         PRINT_I        |
    //                                         PRINT_D);
  }
}