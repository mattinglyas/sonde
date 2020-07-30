#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include "Adafruit_FONA.h"
#include "SD.h"
#include "Config.h"
#include "WaterSondeTypeDefs.h"
#include "RTClib.h"

/* //////////////////////////////// PIN DEFINITIONS //////////////////////////////// */

#define INTERRUPT_PIN 2
#define FONA_PWR_PIN 6
#define FONA_RST_PIN 7
#define LED_PIN 13
#define EC_ENABLE_PIN 32
#define READY_TO_SLEEP_PIN 33
#define SOLENOID_VALVE_PIN 34

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
#define CS_PIN 4

/* ////////////////////////////////  UTILITIES //////////////////////////////// */

char ** strspl(char * inputString, char * delim, int splitNum) {                    // Split string into array of strings on delimiters
    int i;
    if (splitNum == 0) {                                                            // splitNum is zero - split on all delimiters
        for (i=0, splitNum=0; inputString[i]; i++) {                                // Iterate over list until we reach the null terminator
          splitNum += (inputString[i] == delim[0]);                                 // Count # of delimiters in inputString
        } 
    }
    char ** splitString = NULL;                                                     // Instantiate an array of pointers to strings to hold split parts of inputString
    splitString = malloc((splitNum+2)*sizeof(char *));                              // Allocate memory for array of pointers. Don't forget to free memory afterwards
    i = 0;
    for (char *p = strtok(inputString, delim); (i < splitNum+1) && p; i++) {        // split up inputString and assign parts to splitString
        splitString[i] = p;
        p = strtok(NULL, i == splitNum-1 ? "" : delim);
    }
    splitString[i] = NULL;                                                          // Add sentinel NULL value to terminate the list
    return splitString;
}

void blinkCountdown(unsigned long intervalSize, int numIntervals, unsigned long blinkInterval) {      // Execute a blink countdown to give a visual indication that the Arduino is getting ready to send the TPL5110 power timer the signal to power off.
  //    intervalSize - length of each countdown interval, in milliseconds.
  //    numIntervals - number of intervals to count down
  //    blinkInterval - interval between counting blinks, in milliseconds.

  unsigned long startMillis = millis();
  int intervalsLeft = numIntervals;
  int nextIntervalsLeft;
  
  while (intervalsLeft > 0) {
    nextIntervalsLeft = numIntervals - ((millis() - startMillis) / intervalSize);
    if (nextIntervalsLeft < intervalsLeft) {
      for (int b = nextIntervalsLeft; b > 0; b--) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(blinkInterval);
        digitalWrite(LED_BUILTIN, LOW);
        delay(blinkInterval);
      }
      intervalsLeft = nextIntervalsLeft;
    }
  }
}

// some log methods are used before logging section so header must be declared early
void logError(char * error);
void logEvent(char * eventDescription);
void logData(measurement * TEMPMeasurement, measurement * ECMeasurement, bool sampleRetrieved);

/* //////////////////////////////// RTC, TIME, AND SLEEP //////////////////////////////// */

RTC_DS1307 rtc;   // Real Time Clock object

unsigned long currentTime;
unsigned long modTime = 0;
unsigned long lastModTime = 0;
unsigned long nextMeasureTime = 0;

char * timeStamp;

ISR(WDT_vect) {
  // Dummy watchdog timer handler to prevent reset
}

void goToSleep() {                                                      // Settings various things to reduce power use by Arduino
//  Serial.println("Going to sleep...");
  delay(200);
  attachInterrupt(0, INTERRUPT_PIN, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
  power_adc_disable();        // Disable Analog/digital converter
//  Serial.println("Waking up!");
}

void setupSleepMode() {                                                 // Set up watchdog timer
  Serial.println(F("Initialising WDT..."));
  delay(100); //Allow for serial print to complete.

  /* Setup the interrupt pin direction. */
  pinMode(INTERRUPT_PIN, INPUT);
  
  /*** Setup the WDT ***/
  /*** Setup the Watch Dog Timer ***/
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);

  /* set new watchdog timeout prescaler value */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP0 | 0<<WDP1 | 0<<WDP2 | 1<<WDP3;
  WDTCSR |= _BV(WDIE);
  
  Serial.println(F("...WDT initialisation complete."));
  delay(100); //Allow for serial print to complete.
}

void pin2Interrupt(void) {                                            // Set up sleep interrupt pin
  /* This will bring us back from sleep. */
   /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  detachInterrupt(0);
}

void goLowPower() {         // Reduce power usage by setting unused analog outs to digital mode, and set to low output
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);
  pinMode(A1, OUTPUT);
  digitalWrite(A1, LOW);
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);
//  pinMode(A4, OUTPUT);   // In use for I2C communication to RTC
//  digitalWrite(A4, LOW);
//  pinMode(A5, OUTPUT);   // In use for I2C communication to RTC 
//  digitalWrite(A5, LOW);
  pinMode(A6, OUTPUT);
  digitalWrite(A6, LOW);
  pinMode(A7, OUTPUT);
  digitalWrite(A7, LOW);
  pinMode(A8, OUTPUT);
  digitalWrite(A8, LOW);
  pinMode(A9, OUTPUT);
  digitalWrite(A9, LOW);
  pinMode(A10, OUTPUT);
  digitalWrite(A10, LOW);
  pinMode(A11, OUTPUT);
  digitalWrite(A11, LOW);
  pinMode(A12, OUTPUT);
  digitalWrite(A12, LOW);
  pinMode(A13, OUTPUT);
  digitalWrite(A13, LOW);
  pinMode(A14, OUTPUT);
  digitalWrite(A14, LOW);
  pinMode(A15, OUTPUT);
  digitalWrite(A15, LOW);

  for (int i = 0; i <= 53; i++) {  // Set unused digital pins to output and low
    if ((i != 4) && (i != 10) && (i != 53) &&
        (i != EC_ENABLE_PIN) && 
        (i != SS) && 
        (i != CS_PIN) && 
        (i != LED_BUILTIN) && 
        (i != READY_TO_SLEEP_PIN) && 
        (i != SOLENOID_VALVE_PIN) && 
        (i != INTERRUPT_PIN) && 
        ((i < 10) || (i > 21)) && 
        ((i < 50 ) || (i > 53))) { // pins 10-21 and 50-53 in use for serial communication, pins 
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }

// All hell broke loose when I tried some of these :(
  power_adc_disable();        // Disable Analog/digital converter
//  power_spi_disable();        // Disable SPI
//  power_usart0_disable();   // Disable serial
//  power_usart2_disable();   // Disable serial
//  power_timer0_disable();   // Appears to be essential
//  power_timer1_disable();   // Appears to be essential
//  power_timer2_disable();   // Appears to be essential
//  power_timer3_disable();
//  power_timer4_disable();
//  power_timer5_disable();   // Appears to be essential
//  power_twi_disable();      
}

void powerDown() {                                            // Give a blink-countdown-warning then send a signal to the TPL5110 timer chip to power everything down
  Serial.println("ARD,Preparing to power down...");
  blinkCountdown(2000, 5, 100);  // Give the SD card 10 seconds to get its crap in order and avoid corrupting files by powering down in the middle of a file operation or something
  digitalWrite(READY_TO_SLEEP_PIN, HIGH);
  delay(5000);  // Wait for power down
  Serial.println("ARD,Arduino should be off now...so it must be powered via USB or barrel jack, rather than through timer chip. Restart Arduino with power thru timer chip to achieve power down.");
}

void getTimestamp(char * timestamp) {                             // Write a timestamp to the provided string pointer
    DateTime now = rtc.now();
    snprintf(timestamp, 32, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
}

void connectToRTC() {                             // Connect to real time clock on data logger shield
  if (! rtc.begin()) {      // Open connection to embedded RTC chip on data logger shield
    Serial.println("ARD,Error: Couldn't connect to RTC");
  } else {
    if (! rtc.isrunning()) {
      Serial.println("ARD,Error: RTC is NOT running!");
    } else {
      Serial.println("ARD,Connected to RTC");
    }
  }
}

/* //////////////////////////////// SD CARD //////////////////////////////// */

File dataFile;
const char * dataFileName = "data.txt";

// Settings variables and functions: In order to have settings and data that persist between boots, we need to be able to save and retrieve settings data from SD card.
File settingsFile;
const char * settingsFileName = "sets.wsd";

bool SDConnected = false;

void connectToSDCard() {                                // Open conection to data logger/SD card
  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);                                // make sure that the default chip select pin is set to output, even if you don't use it:
  SDConnected = false;
  int attempts = 5;
  while (!SDConnected && attempts > 0) {
    if (!SD.begin(CS_PIN, 11, 12, 13)) {
      attempts--;
      Serial.print("ARD,SD card connection failed, or not present. Attempts remaining: "); Serial.println(attempts);
      SDConnected = false;
      delay(100);
    } else {
      Serial.println("card initialized.");
      SDConnected = true;  
    }
  }
  if (!SD.begin(CS_PIN, 11, 12, 13)) {            // see if the card is present and can be initialized, using correct SD pins:
  } else {
  }
  Serial.flush();
}

void openDataFile(File * file, char * filename, byte mode) {            // Create/open for writing a file with the name given by argument. If argument is null string "", use default stored in variable dataFileName
  if (file) {
    (*file).close();
  }
  char message[128];
  if (!filename[0]) {
    filename = dataFileName;
  }
  if (*file) {                                  // If file is already open, close it
    (*file).close();
  }
  *file = SD.open(filename, mode);              // Attempt to open file
  if (!(*file)) {                               // If file fails to open, try creating it by opening it in write mode
    *file = SD.open(filename, FILE_WRITE);      // Create file
    (*file).close();
    *file = SD.open(filename, mode);            // Attempt to reopen file
  }
  if (!(*file)) {
    snprintf(message, 128, "ARD,Error opening file: %s", filename);
    Serial.println(message);
  } else {
//    sprintf(message, "Opened file: %s", filename);
//    Serial.println(message);
  }
}

void archiveDataFile(char * archiveFileName) {                      // Move data from current default data file to new file with given name, then delete default data file and recreate it blank.
  char message[128];
  if (!dataFile) {
    openDataFile(&dataFile, dataFileName, FILE_READ);           // If it isn't already open, open default data file
  }
  File archiveFile;
  openDataFile(&archiveFile, archiveFileName, FILE_WRITE);       // Open archive file
  if (dataFile && archiveFile) {
    char nextChar;
    while (true) {                                   // Copy default data file contents into archive file
      nextChar = dataFile.read();
      if (nextChar != -1) {
        archiveFile.write(nextChar);
      }
    }
    dataFile.close();                                // Close default data file
    archiveFile.close();                             // Close archive data file
    SD.remove(dataFileName);                         // Remove existing default data file
  } else {
      snprintf(message, 128, "Archive - could not open datafile: %s or %s", dataFileName, archiveFileName);
      logError(message);
  }
}

/* //////////////////////////////// SETTINGS //////////////////////////////// */

// Struct that holds a variety of settings that control the sonde's behavior
settings currentSettings = {
  false,    // sleepEnabled
  false,    // sampleRetrieved
  false,     // singleShotMeasurementOnBootMode
  1234UL,     // singleShotMeasurementDelay
  0UL,        // measurementInterval
  2345        // sampleDuration
};

// Default settings to fall back on
settings defaultSettings = {
  false,    // sleepEnabled
  false,    // sampleRetrieved
  true,     // singleShotMeasurementOnBootMode
  5000UL,     // singleShotMeasurementDelay
  0UL,        // measurementInterval
  1000        // sampleDuration
};

void settingsToBytes(settings * sIn, byte * bufOut) {         // Convert a settings struct to a byte array so it can be saved to the SD card
    memcpy(bufOut, sIn, sizeof(*sIn));
}

void bytesToSettings(byte * bufIn, settings * sOut) {         // Convert a byte array to a settings struct so the settings struct can be retrieved from the SD card and used
    memcpy(sOut, bufIn, sizeof(*sOut));
}

void saveSettingsToDisk(settings * settingsToSave) {          // Save the given settings struct to the SD card
  byte buf[sizeof(*settingsToSave)];
  settingsToBytes(settingsToSave, buf);
  openDataFile(&settingsFile, settingsFileName, O_WRITE | O_CREAT | O_TRUNC);
  settingsFile.write(buf, sizeof(*settingsToSave));
  settingsFile.flush();
  settingsFile.close();
}

void printCurrentSettings() {                                 // Send the current settings to the USB-connected PC
    Serial.println("*********** CHECKING CURRENT SETTINGS ************");
    Serial.print("sleepEnabled: "); Serial.println(currentSettings.sleepEnabled);
    Serial.print("sampleRetrieved: "); Serial.println(currentSettings.sampleRetrieved);
    Serial.print("singleShotMeasurementOnBootMode: "); Serial.println(currentSettings.singleShotMeasurementOnBootMode);
    Serial.print("singleShotMeasurementDelay: "); Serial.println(currentSettings.singleShotMeasurementDelay);
    Serial.print("measurementInterval: "); Serial.println(currentSettings.measurementInterval);
    Serial.print("sampleDuration: "); Serial.println(currentSettings.sampleDuration);
    Serial.println("*******************************************");
}

void saveCurrentSettingsToDisk() {                           // Save the currently used settings to the SD card
  saveSettingsToDisk(&currentSettings);
}

void saveDefaultSettingsToDisk() {                           // Save the default settings to the SD card
  saveSettingsToDisk(&defaultSettings);                  
}

void retrieveSettingsFromDisk() {                            // Load the saved settings on the SD card into the current settings struct
  byte buf[sizeof(defaultSettings)];
  openDataFile(&settingsFile, settingsFileName, FILE_READ);
  if (settingsFile) {
    settingsFile.read(buf, sizeof(defaultSettings));
    bytesToSettings(buf, &currentSettings);
    settingsFile.close();
  } else {
    Serial.println("ARD,Error, could not open settings file. Reverting to defaults.");
    currentSettings = defaultSettings;
  }
}

/* //////////////////////////////// FTP AND CELL //////////////////////////////// */

static const char * serverIP = FTP_SERVER_IP;
static const int serverPort = FTP_SERVER_PORT;
static const char * username = FTP_USERNAME;
static const char * password = FTP_PASSWORD;

HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

uint8_t readLine(char * buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0};

void simPowerOn() {
  digitalWrite(FONA_PWR_PIN, LOW);
  delay(750);
  digitalWrite(FONA_PWR_PIN, HIGH);
}

void simModuleSetup() {
  // Note: The SIM7000A baud rate seems to reset after being power cycled (SIMCom firmware thing)
  // SIM7000 takes about 3s to turn on but SIM7500 takes about 15s
  // Press reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset
  fonaSerial->begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  fonaSerial->println("AT+IPR=9600"); // Set baud rate
  delay(100);
  fonaSerial->begin(9600);
  if (!fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1); // Don't proceed if it couldn't find the device
  }

  // The commented block of code below is an alternative that will find the module at 115200
  // Then switch it to 9600 without having to wait for the module to turn on and manually
  // press the reset button in order to establish communication. However, once the baud is set
  // this method will be much slower.
  /*
  fonaSerial->begin(115200); // Default LTE shield baud rate
  fona.begin(*fonaSerial); // Don't use if statement because an OK reply could be sent incorrectly at 115200 baud
  Serial.println(F("Configuring to 9600 baud"));
  fona.setBaudrate(9600); // Set to 9600 baud
  fonaSerial->begin(9600);
  if (!fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find modem"));
    while(1); // Don't proceed if it couldn't find the device
  }
  */

  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L")); break;
    case SIM800H:
      Serial.println(F("SIM800H")); break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000A:
      Serial.println(F("SIM7000A (American)")); break;
    case SIM7000C:
      Serial.println(F("SIM7000C (Chinese)")); break;
    case SIM7000E:
      Serial.println(F("SIM7000E (European)")); break;
    case SIM7000G:
      Serial.println(F("SIM7000G (Global)")); break;
    case SIM7500A:
      Serial.println(F("SIM7500A (American)")); break;
    case SIM7500E:
      Serial.println(F("SIM7500E (European)")); break;
    default:
      Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // Needed for rare cases in which firmware on SIM7000 sets CFUN to 0 on start-up
  fona.setFunctionality(1); // Enable cellular (RF) with AT+CFUN=1
}

bool simNetStatus() {
  int n = fona.getNetworkStatus();
  
  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}

void connectToFTPServer() {
  // connect to FTP server
  Serial.println(F("Info: Connecting to FTP server..."));
  while (!fona.FTP_Connect(serverIP, serverPort, username, password)) {
    Serial.println(F("Error: Failed to connect to FTP server, retrying..."));
    delay(2000);
  }
}

void disconnectFromFTPServer() {
  if (!fona.FTP_Quit()) {
    Serial.println(F("Error: Failed to close FTP connection!"));
  }
}

/* //////////////////////////////// PROBES AND MEASUREMENT //////////////////////////////// */

bool measurementInProgress = false;                             // Is an automatic measurement in progress?
bool singleShotMeasurementComplete = false;                     // Flag to set when the single shot measurement has already been taken

// Structs used to collect and store measurement data from EC and TEMP probes
measurement ECMeasurement; 
measurement TEMPMeasurement; 

// Structs to collect messages received from PC, EC probe and TEMP probes as they arrive.
serialInput PCInput =   {"", 0, false};
serialInput SIMInput = {"", 0, false};
serialInput ECInput =   {"", 0, false};
serialInput TEMPInput = {"", 0, false};

void resetMeasurement (measurement * m) {                               // Reset a measurement struct back to zeros
  (*m).data[0] = 0;
  (*m).timestamp[0] = 0;
  (*m).voltage[0] = 0;
  (*m).lastPowerOff[0] = 0;
}

void zeroString(char * string) {
  for (int k = 0; string[k]; k++) {
    string[k] = 0;
  }
}

void resetInput (serialInput * input) {                                 // Reset an input struct back to empty values
  (*input).messageComplete = false;
  (*input).inputStringLoc = 0;
  (*input).inputString[0] = "\0";
  zeroString((*input).inputString);
}

void enableECProbe(bool enable) {   // Enable or disable EC probe
  digitalWrite(EC_ENABLE_PIN, (enable ? HIGH : LOW));
}

void getNextSignificantTEMPMessage(char * message, int maxLength) {                              // Blocking function that waits until a complete message has been received from the TEMP sensor
  while (true) {
    size_t nChar = Serial2.readBytesUntil('\r', message, maxLength);
    if (nChar > 0 && message[0] != '*') {
      message[nChar] = 0;
      break;   // This is not a response code (*OK, *WA, etc), it's a significant message.
    }
  }
}
void getNextSignificantECMessage(char * message, int maxLength) {                               // Blocking function that waits until a complete message has been received from the EC sensor
  while (true) {
    size_t nChar = Serial3.readBytesUntil('\r', message, maxLength);
    if (nChar > 0 && message[0] != '*') {
      message[nChar] = 0;
      break;   // This is not a response code (*OK, *WA, etc), it's a significant message.
    }
  }
}

void wakeTEMP() {                                  // Send a serial message to the TEMP sensor to wake up
  Serial.println("Wake TEMP");
  Serial2.print("x\r");                            // Wake TEMP, in case it's asleep (x is an invalid command, so if it's asleep, we'll get a singla *WA response, if it's awake, we'll get a single *ER response.
//  discardNextTEMPMessage();
}

void wakeEC() {                                    // Send a serial message to the EC sensor to wake up
  Serial.println("Wake EC");
  Serial3.print("x\r");                            // Wake EC, in case it's asleep (x is an invalid command, so if it's asleep, we'll get a singla *WA response, if it's awake, we'll get a single *ER response.
//  discardNextECMessage();
//  enableECProbe(true);
}

void sleepTEMP() {                                 // Send a serial message to the TEMP sensor to go to sleep
  Serial.println("Sleep TEMP");
  Serial2.print("SLEEP\r");
//  discardNextTEMPMessage();
//  discardNextTEMPMessage();
}

void sleepEC() {                                   // Send a serial message to the EC sensor to go to sleep
  Serial.println("Sleep EC");
  Serial3.print("SLEEP\r");  // Rather than set the EC circuit in sleep mode, we can just disable the inline voltage isolator for more power savings.
//  discardNextECMessage();
//  discardNextECMessage();
//  enableECProbe(false);   // This doesn't seem to work
}

void measureTEMP() {                              // Instruct the TEMP sensor to take a measurement, and block until the measurement has been received from the TEMP sensor
  wakeTEMP();
  delay(200);           // Give TEMP a chance to wake up
  Serial2.print("R\r");                                                           // Request a measurement from TEMP probe
  getNextSignificantTEMPMessage(TEMPMeasurement.data, 32);
  //  size_t nChar = Serial2.readBytesUntil('\r', TEMPMeasurement.data, 32);                                   // Blocking read TEMP probe messages (expected: string rep of float)
//  (TEMPMeasurement.data)[nChar] = 0;
//  discardNextTEMPMessage();
}

void measureEC() {                                // Instruct the EC sensor to take a measurement, and block until the measurement has been received from the EC sensor
  wakeEC();
  delay(200);           // Give EC a chance to wake up
  Serial3.print("R\r");                                                           // Request a measurement from EC probe
  getNextSignificantECMessage(ECMeasurement.data, 32);
//  size_t nChar = Serial3.readBytesUntil('\r', ECMeasurement.data, 32);                                     // Blocking read EC probe messages (expected: string rep of float)
//  (ECMeasurement.data)[nChar] = 0;
//  discardNextECMessage();
}

void updateTemperatureCompensation() {           // Send temperature measurement to the EC sensor to update its temperature compensation function that adjusts EC measurement to account for the effects of temperature.
  Serial3.print("T,"); Serial3.print(TEMPMeasurement.data); Serial3.print('\r');  // Update conductivity probe temperature compensation
//  discardNextECMessage();
}

void getTEMPStatus() {                          // Request TEMP sensor status, and block until it is received
  int maxLength = 32;
  char statusBuffer [maxLength];
  Serial2.print("STATUS\r");                                                                 // Request device status
  getNextSignificantTEMPMessage(statusBuffer, maxLength);
//  size_t nChar = Serial2.readBytesUntil('\r', statusBuffer, maxLength);                      // Blocking read EC probe messages (expected: status CSV string)
//  statusBuffer[nChar] = 0;
  Serial.print("TEMP status: "), Serial.println(statusBuffer);
  char ** splitString = strspl(statusBuffer, ",", 0);                                        // Split CSV values
  if (splitString[0] == 0 || splitString[1] == 0 || splitString[2] == 0) {                   // Something went wrong - status buffer didn't contain enough commas to be split as expected
    Serial.println("Error interpreting Temp status message:");
    Serial.println(statusBuffer);
    snprintf(TEMPMeasurement.voltage, 32, "%s", "ERR");                                      // Set temperature status voltage to ERR
    snprintf(TEMPMeasurement.lastPowerOff, 32, "%s", "ERR");                                 // Set temperature last poweroff to ERR
    return;                                                                                  // Exit function early
  } else {
    snprintf(TEMPMeasurement.voltage, 32, "%s", splitString[2]);
    snprintf(TEMPMeasurement.lastPowerOff, 32, "%s", splitString[1]);
  }
  free(splitString);                                                                         // Free up malloc'd splitString memory
//  discardNextTEMPMessage();
}

void getECStatus() {                                                                         // Request TEMP sensor status, and block until it is received
  int maxLength = 32;
  char statusBuffer [maxLength];
  Serial3.print("STATUS\r");                                                                 // Request device status
  getNextSignificantECMessage(statusBuffer, maxLength);
//  size_t nChar = Serial3.readBytesUntil('\r', statusBuffer, maxLength);                                     // Blocking read EC probe messages (expected: status CSV string)
//  statusBuffer[nChar] = 0;
  Serial.print("EC status: "), Serial.println(statusBuffer);
  char ** splitString = strspl(statusBuffer, ",", 0);                                        // Split CSV values
  if (splitString[0] == 0 || splitString[1] == 0 || splitString[2] == 0) {                   // Something went wrong - status buffer didn't contain enough commas to be split as expected
    Serial.println("Error interpreting EC status message:");
    Serial.println(statusBuffer);
    snprintf(ECMeasurement.voltage, 32, "%s", "ERR");                                      // Set temperature status voltage to ERR
    snprintf(ECMeasurement.lastPowerOff, 32, "%s", "ERR");                                 // Set temperature last poweroff to ERR
    return;                                                                                  // Exit function early
  } else {
    snprintf(ECMeasurement.voltage, 32, "%s", splitString[2]);
    snprintf(ECMeasurement.lastPowerOff, 32, "%s", splitString[1]);
  }
  free(splitString);                                                                         // Free up malloc'd splitString memory
//  discardNextECMessage();
}

void setContinuousMeasurements(unsigned long interval) {  //3600000          // Set mode to "continuous measurement", meaning repeated measurements without powering down. This is NOT the way the Sonde will probably be used in the field.
  if (interval < 10000 && interval != 0) {
    interval = 10000;
    Serial.println("ARD,Continuous measurement interval must be 0 or over 10000 ms");
  }
  if (interval != currentSettings.measurementInterval) {
    char message[128];
    if (interval) {
      Serial2.print("*OK,1\r");                                // Set response modes
      Serial3.print("*OK,1\r");   
      Serial2.print("S,C\r");                                 // Set temp units to celsius
      Serial3.print("O,EC,1\rO,TDS,0\rO,S,0\rO,SG,0\r");      // Enable conductivity measurement and disable all others
      delay(5000);                                            // Wait for responses to come in
      while(Serial2.available() || Serial3.available()) {     // Flush responses
        if (Serial2.available()) {
          Serial2.read();
        }
        if (Serial3.available()) {
          Serial3.read();
        }
      }
    }
    snprintf(message, 128, "Setting continuous measurement interval from %lu to %lu.", currentSettings.measurementInterval, interval);
    currentSettings.measurementInterval = interval;
    saveCurrentSettingsToDisk();
    logEvent(message);
    Serial.println(message);
  }
  if (!currentSettings.measurementInterval) {                  // Continuous measurements are currently off
    resetMeasurement(&ECMeasurement);
    resetMeasurement(&TEMPMeasurement);
  }
}

bool isItSampleTime(char * salinity) {                        // Check if it's time to take a sample or not based on the measured electrical conductivity (not implemented yet)
  return false;
}

void retrieveSample(int milliseconds) {                       // Open the solenoid valve for a certain amount of time to retrieve a sample
  // Open solenoid valve for the given # of milliseconds.
  // This is a blocking function
  digitalWrite(SOLENOID_VALVE_PIN, HIGH);   // Open solenoid valve
  delay(milliseconds);                    // Keep it open for some amount of time while the water enters the sample chamber
  digitalWrite(SOLENOID_VALVE_PIN, LOW);    // Close solenoid valve
  currentSettings.sampleRetrieved = true;
  saveCurrentSettingsToDisk();
}

void takeAndLogMeasurements() {
  resetMeasurement(&ECMeasurement);                               // Reset measurement tracking variables, just in case
  resetMeasurement(&TEMPMeasurement);
  measureTEMP();                                                  // Measure temperature
  Serial.print("Measured temp: "); Serial.println(TEMPMeasurement.data);
  getTimestamp(TEMPMeasurement.timestamp);                        // Record timestamp when measurement was received
  updateTemperatureCompensation();                                // Update temperature compensation for EC
  measureEC();                                                    // Measure conductivity
  Serial.print("Measured ec: "); Serial.println(ECMeasurement.data);
  getTimestamp(ECMeasurement.timestamp);                          // Record timestamp when measurement was received
  getTEMPStatus();                                                // Record TEMP device status
  getECStatus();                                                  // Record EC device status
  if (isItSampleTime(ECMeasurement.data)) {                       // Conductivity indicates we should take a sample, and we haven't taken one yet!
    retrieveSample(currentSettings.sampleDuration);                                             // Take a sample!
  }

  logData(&TEMPMeasurement, &ECMeasurement, currentSettings.sampleRetrieved);     // Log data to SD card
  resetMeasurement(&ECMeasurement);                               // Reset measurement tracking variables
  resetMeasurement(&TEMPMeasurement);
  sleepTEMP();
  sleepEC();  
}


/* //////////////////////////////// LOGGING AND DEBUG //////////////////////////////// */

char inChar;
const int targetCommandLength = 32;
char target[targetCommandLength];
char command[targetCommandLength];
char * commandName;

void logBootEvent() {                                         // Log some status info to the SD card
  if (SDConnected) {
    unsigned long milliseconds = millis();
    char timestamp[32];
    getTimestamp(timestamp);
  
    wakeTEMP();                                                     // Wake up TEMP in case it was sleeping
    wakeEC();                                                       // Wake up EC in case it was sleeping

    resetMeasurement(&ECMeasurement);
    resetMeasurement(&TEMPMeasurement);
    getTEMPStatus();
    getECStatus();

    openDataFile(&dataFile, dataFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("LOGTYPE,TIME_MS,TIMESTAMP,TEMP_VOLTAGE,TEMP_LASTPOWEROFF,EC_VOLTAGE,EC_LASTPOWEROFF");
      dataFile.print("BOOT,");
      dataFile.print(milliseconds, DEC); dataFile.print(","); 
      dataFile.print(timestamp); dataFile.print(",");
      dataFile.print(TEMPMeasurement.voltage); dataFile.print(",");
      dataFile.print(TEMPMeasurement.lastPowerOff); dataFile.print(",");
      dataFile.print(ECMeasurement.voltage); dataFile.print(",");
      dataFile.println(ECMeasurement.lastPowerOff);
    dataFile.flush();
    dataFile.close();
    }

  } else {
      Serial.println("ARD,Failed to log boot event - SD card not connected");
  }
}

void logCommunication(String message) {             // Make a log of a communication? Not sure what I was planning to use this for.
  if (SDConnected) {
    unsigned long milliseconds = millis();
    char timestamp[32];
    getTimestamp(timestamp);
  
    openDataFile(&dataFile, dataFileName, FILE_WRITE);
    dataFile.println("LOGTYPE,TIME_MS,TIMESTAMP,MESSAGE");
      dataFile.print("COMMUNICATION,");
      dataFile.print(milliseconds, DEC); dataFile.print(",");
      dataFile.print(timestamp); dataFile.print(",");
      dataFile.println(message);
    dataFile.flush();
    dataFile.close();
  } else {
      Serial.println("ARD,Failed to log communication - SD card not connected");
  }
}

void logData(measurement * TEMPMeasurement, measurement * ECMeasurement, bool sampleRetrieved) {    // Log a measurement to the SD card
  char dataEntry[120];
  snprintf(dataEntry, sizeof(dataEntry), "MEASUREMENT,%f,%s,%f,%s,%u,%f,%s,%f,%s", TEMPMeasurement->data, TEMPMeasurement->timestamp, ECMeasurement->data, ECMeasurement->timestamp, (sampleRetrieved ? 1 : 0), DEC, TEMPMeasurement->voltage, TEMPMeasurement->lastPowerOff, ECMeasurement->voltage, ECMeasurement->lastPowerOff);
  if (SDConnected) {
    openDataFile(&dataFile, dataFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("LOGTYPE,TEMPERATURE,TEMPERATURE_TIMESTAMP,SALINITY,SALINITY_TIMESTAMP,SAMPLE_RETRIEVED,TEMP_VOLTAGE,TEMP_LASTPOWEROFF,EC_VOLTAGE,EC_LASTPOWEROFF");
//    dataFile.print("MEASUREMENT,");
//        dataFile.print(TEMPMeasurement->data); dataFile.print(",");
//        dataFile.print(TEMPMeasurement->timestamp); dataFile.print(",");
//        dataFile.print(ECMeasurement->data); dataFile.print(",");
//        dataFile.print(ECMeasurement->timestamp); dataFile.print(",");
//        dataFile.print((sampleRetrieved ? 1 : 0), DEC); dataFile.print(",");
//        dataFile.print(TEMPMeasurement->voltage); dataFile.print(",");
//        dataFile.print(TEMPMeasurement->lastPowerOff); dataFile.print(",");
//        dataFile.print(ECMeasurement->voltage); dataFile.print(",");
//        dataFile.println(ECMeasurement->lastPowerOff);
      dataFile.println(dataEntry);
      dataFile.flush();
      dataFile.close();
    }
  } else {
      Serial.println("ARD,Failed to log data - SD card not connected");
  }
}

void logError(char * error) {    // Log an error to the SD card
  if (SDConnected) {
    unsigned long milliseconds = millis();
    char timestamp[32];
    getTimestamp(timestamp);
  
    openDataFile(&dataFile, dataFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("LOGTYPE,TIME_MS,TIMESTAMP,MESSAGE");
      dataFile.print("ERROR,"); 
      dataFile.print(milliseconds, DEC); dataFile.print(","); 
      dataFile.print(timestamp); dataFile.print(","); 
      dataFile.println(error);
    dataFile.close();
    }
  } else {
    Serial.println("ARD,Failed to log error - SD card not connected");
  }
}

void logEvent(char * eventDescription) {               // Log a generic event to the SD card
  if (SDConnected) {
    unsigned long milliseconds = millis();
    char timestamp[32];
    getTimestamp(timestamp);
    
    openDataFile(&dataFile, dataFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("LOGTYPE,TIME_MS,TIMESTAMP,MESSAGE");
      dataFile.print("EVENT,");
      dataFile.print(milliseconds, DEC); dataFile.print(","); 
      dataFile.print(timestamp); dataFile.print(","); 
      dataFile.println(eventDescription);
    dataFile.flush();
    dataFile.close();
    }
  } else {
    Serial.println("ARD,Failed to log event - SD card not connected");
  }
}

void logNote(char * noteText) {                       // Log a note to the SD card
  if (SDConnected) {
    unsigned long milliseconds = millis();
    char timestamp[32];
    getTimestamp(timestamp);
  
    openDataFile(&dataFile, dataFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("LOGTYPE,TIME_MS,TIMESTAMP,MESSAGE");
      dataFile.print("NOTE,"); 
      dataFile.print(milliseconds, DEC); dataFile.print(","); 
      dataFile.print(timestamp); dataFile.print(","); 
      dataFile.println(noteText);
    dataFile.close();
    }
  } else {
    Serial.println("ARD,Failed to log note - SD card not connected");
  }
}

void getCurrentDataFileContents() {                                // Dump entire SD card contents to the PC serial connection
  if (SDConnected) {
    openDataFile(&dataFile, dataFileName, FILE_READ);
    if (dataFile) {
      char c;
      while (dataFile.available()) {
        c = dataFile.read();
        Serial.print(c);
      }
      dataFile.close();
    } else {
      return "ERROR: No data file loaded";
    }
  } else {
    Serial.println("ARD,Failed to get data file contents - SD card not connected");
  }
}

/* //////////////////////////////// ARDUINO SKETCH //////////////////////////////// */

void setup() {
  Serial.begin(9600); // open USB connection to PC
  Serial2.begin(9600); // open SPI connection to TEMP probe
  Serial3.begin(9600); // open SPI connection to EC probe
    
  while (Serial.available()) {          // Flush incoming buffer from PC
    Serial.read();
  }
  while (Serial2.available()) {         // Flush incoming buffer from EC
    Serial.read();
  }
  while (Serial3.available()) {         // Flush incoming buffer from TEMP
    Serial.read();
  }

  /* PIN SETUP */

  pinMode(LED_PIN, OUTPUT);
  pinMode(FONA_RST_PIN, OUTPUT);
  pinMode(FONA_PWR_PIN, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(READY_TO_SLEEP_PIN, OUTPUT);
  pinMode(SOLENOID_VALVE_PIN, OUTPUT);
  pinMode(EC_ENABLE_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(FONA_RST_PIN, HIGH);
  digitalWrite(READY_TO_SLEEP_PIN, LOW);
  digitalWrite(SOLENOID_VALVE_PIN, LOW);

  /* SD SETUP */

  Serial.println(F("ARD,Setting up SD card"));

  connectToSDCard();
  retrieveSettingsFromDisk();
  printCurrentSettings();

  /* PROBE SETUP */

  Serial.println(F("ARD,Setting up probes"));

  enableECProbe(true);

  // wake both sensors
  wakeTEMP();
  wakeEC();

  // disable continuous reading and wait a second for that to register:
  Serial2.print("C,0\r");
  Serial3.print("C,0\r");
  delay(1000);

  Serial.println(F("ARD,Putting probes to sleep"));
  sleepTEMP();
  sleepEC();

  /* RTC SETUP */

  Serial.println(F("ARD,Setting up RTC"));
  connectToRTC();

  /* SLEEP SETUP */
  
  // Set up watchdog timer and interrupt pin to be ready for sleeping
  Serial.println(F("ARD,Setting up sleep mode"));
  setupSleepMode();

  /* SIM SETUP */
  Serial.println(F("ARD, Setting up SIM module"));

  simPowerOn();
  simModuleSetup();

  // set modem to full functionality (LTE data)
  fona.setFunctionality(1); //AT+CFUN=1
  fona.setNetworkSettings(F("hologram"));

  Serial.println(F("ARD,Connected to cell network!"));

  // disable data connection before attempting to connect
  fona.enableGPRS(false);

  while (!fona.enableGPRS(true)) {
    Serial.println(F("ARD,Failed to enable data, retrying..."));
    delay(2000);
  }

  Serial.println(F("ARD,Enabled data connection"));

  /* MISC SETUP */

  Serial.println(F("ARD,Setup complete"));
}

// Loop workflow:
// 1. If continuous recording is enabled, execute blocking commands to collect and log data.
// 2. If single shot measurement mode is on, and measurement is already complete, signal ready to shut down. Otherwise, take a measurement.
// 3. Check for PC commands
// 4. Check for messages from temp probe
// 5. Check for messages from EC probe
// 6. If PC commands exist, execute them
// 7. If EC messages exist, relay them to PC
// 8. If Temp messages exist, relay them to PC
void loop() {
  int maxMessageLength  = 128;
  char message[maxMessageLength];
  if (currentSettings.measurementInterval) {               // If we are in automatic measurement mode...
    long currentTime = millis();
    if (currentTime < nextMeasureTime) {                                                      // Check if the desired time interval has passed for taking measurements
      Serial.println("ARD,Beginning automatic measurement...");
      takeAndLogMeasurements();
      Serial.println("ARD,...automatic measurement complete!");
    }
    nextMeasureTime = currentTime + currentSettings.measurementInterval;
  }

  if (currentSettings.singleShotMeasurementOnBootMode && millis() > currentSettings.singleShotMeasurementDelay) {  // Are we in wake ==> measurement ==> power down mode? And has the delay passed?
    if (singleShotMeasurementComplete) {                                            // Single shot measurement is complete
      if (!Serial.available() && !Serial2.available() && !Serial3.available()) {    // We're not waiting on any serial messages, then signal readiness to power down
        powerDown();   // Signal the low power chip that we're ready to power down
      } else {
        Serial.println("Waiting for serial messages to finish coming in, can't power down yet...");
      }
    } else {                                                                        // Single shot measurement not complete yet - take it.
      Serial.println("ARD,Beginning single shot measurement...");
      takeAndLogMeasurements();
      singleShotMeasurementComplete = true;
      Serial.println("ARD,...single shot measurement complete!");
    }
  }

  if (Serial.available()) {      // Read messages from PC, if any
    while (!PCInput.messageComplete) {
      if (Serial.available()) {
        inChar = (char)Serial.read();                   // Read 1 char of serial data
//        Serial.print("|"); Serial.print(inChar); Serial.println("|");
        PCInput.messageBegun = true;
      } else {break;}                                   // No data in pipe, end serial read loop
      if (PCInput.inputStringLoc == 0 && inChar == ',') {  // We're detecting a initial dummy wake character - ignore it.
        // Do nothing
      } else if (inChar == '\n' || inChar == '\r') {           // Check if character is a line end character
        PCInput.messageComplete = true;                 // Set flag that message is complete
        PCInput.messageBegun = false;
        PCInput.inputString[PCInput.inputStringLoc] = 0;  // Null terminate string
      } else {
        PCInput.inputString[PCInput.inputStringLoc] = inChar;   // Add new character to the inputString at the append string index
        PCInput.inputStringLoc++;                               // Increment the append string index for the next character
      }
      if (PCInput.messageComplete) {
        Serial.println("PC msg complete:");
        Serial.println(PCInput.inputString);
        char ** splitString = strspl(PCInput.inputString, ",", 1);   // Message is complete! Split it into its CSV parts
        strcpy(target, splitString[0]);
        strcpy(command, splitString[1]);           // First CSV component is the command target, rest is the command
        Serial.println(target);
        Serial.println(command);
        free(splitString);                              // Free allocated memory for splitString
      }
    }
  }

  if (Serial1.available()) {    // Read messages from SIM7000 modem, if any
    while (true) {
      if (Serial1.available()) {
        inChar = (char)Serial1.read();                   // Read 1 char of serial data
//        Serial.println(inChar);
        SIMInput.messageBegun = true;
      } else {break;}                                   // No data in pipe, end serial read loop
      if (inChar == '\n' || inChar == '\r') {           // Check if character is a line end character
        SIMInput.messageComplete = true;                 // Set flag that message is complete
        SIMInput.messageBegun = false;
        SIMInput.inputString[SIMInput.inputStringLoc] = 0;  // Null terminate string
      } else {
        SIMInput.inputString[SIMInput.inputStringLoc] = inChar;   // Add new character to the inputString at the append string index
        SIMInput.inputStringLoc++;                               // Increment the append string index for the next character
      }
    }
  }

  if (Serial2.available()) {    // Read messages from temp probe, if any
    while (true) {
      if (Serial2.available()) {
        inChar = (char)Serial2.read();                   // Read 1 char of serial data
//        Serial.println("Recieved TEMP char:");
//        Serial.println(inChar);
        TEMPInput.messageBegun = true;
      } else {break;}                                   // No data in pipe, end serial read loop
      if (inChar == '\n' || inChar == '\r') {           // Check if character is a line end character
        TEMPInput.messageComplete = true;                 // Set flag that message is complete
        TEMPInput.messageBegun = false;
        TEMPInput.inputString[TEMPInput.inputStringLoc] = 0;  // Null terminate string
      } else {
        TEMPInput.inputString[TEMPInput.inputStringLoc] = inChar;   // Add new character to the inputString at the append string index
        TEMPInput.inputStringLoc++;                               // Increment the append string index for the next character
      }
    }
  }

  if (Serial3.available()) {      // Read messages from EC probe, if any
    while (true) {
      if (Serial3.available()) {
        inChar = (char)Serial3.read();                   // Read 1 char of serial data
        ECInput.messageBegun = true;
      } else {break;}                                   // No data in pipe, end serial read loop
      if (inChar == '\n' || inChar == '\r') {           // Check if character is a line end character
        ECInput.messageComplete = true;                 // Set flag that message is complete
        ECInput.messageBegun = false;
        ECInput.inputString[ECInput.inputStringLoc] = 0;  // Null terminate string
      } else {
        ECInput.inputString[ECInput.inputStringLoc] = inChar;   // Add new character to the inputString at the append string index
        ECInput.inputStringLoc++;                               // Increment the append string index for the next character
      }
    }
  }

  if (PCInput.messageComplete) {               // Execute command received from PC
    if (strcmp(target, "EC") == 0) {       // Command is targeted to conductivity probe
        Serial3.print(command); Serial3.print('\r');
    } else if (strcmp(target, "TEMP") == 0) {    // Command is targeted to temperature probe
        Serial.println("Sending command to TEMP:");
        Serial.println(command);
        Serial2.print(command); Serial2.print('\r');
        Serial.println("...done sending command to TEMP!");
    } else if (strcmp(target, "SAMP") == 0) {    // Command is targeted to water sampler
        char ** splitString = strspl(command, ",", 0);   // Message is complete! Split it into its CSV parts
        commandName = splitString[0];
        if (strcmp(commandName, "SAMPLE") == 0) {
          if (strcmp(splitString[1], "1") == 0) {
            Serial.println("SAMP,Retrieving sample...");
            retrieveSample(currentSettings.sampleDuration);
            Serial.println("SAMP,...sample retrieved!");
          } else if (strcmp(splitString[1], "0") == 0) {
            currentSettings.sampleRetrieved = false;
            saveCurrentSettingsToDisk();
            Serial.println("SAMP,Sample record reset");
          } else if (strcmp(splitString[1], "?") == 0) {
            Serial.print("SAMP,Sample retrieved: "); Serial.println(splitString[1] ? "Yes" : "No");
          } else {
            Serial.print("SAMP,ERROR - unrecognized error for SAMPLE command: "), Serial.println(splitString[1]);
          }
        } else if (strcmp(commandName, "VALVE") == 0) {
          if (strcmp(splitString[1], "0") == 0) {
            Serial.println("SAMP,Closing sample valve");
            digitalWrite(SOLENOID_VALVE_PIN, LOW);   // Close solenoid valve
          } else if (strcmp(splitString[1], "1") == 0) {
            Serial.println("SAMP,Opening valve. WARNING: Leaving valve open will draw a large amount of current from the valve battery. It is recommended that you do not leave the valve open for more than a few seconds.");
            digitalWrite(SOLENOID_VALVE_PIN, HIGH);   // Open solenoid valve
          }
        } else if (strcmp(commandName, "DURATION") == 0) {
          char * ptr;
          int newSampleDuration = strtol(splitString[1], &ptr, 10);
          if (newSampleDuration) {
            currentSettings.sampleDuration = (int) newSampleDuration;
            saveCurrentSettingsToDisk();
          } else {
            Serial.print("SAMP,Error: invalid sample duration: "); Serial.println(splitString[1]);
          }
        } else {
          Serial.print("SAMP,ERROR: command not recognized: "); Serial.println(commandName);
        }
    } else if (strcmp(target, "ARD") == 0) {     // Command is targeted to arduino
        char ** splitString = strspl(command, ",", 0);   // Message is complete! Split it into its CSV parts
        commandName = splitString[0];
      if        (strcmp(commandName, "SENDTIME") == 0) {
        Serial.println("ARD,REQUEST_TIME");
      } else if (strcmp(commandName, "DEFAULTS") == 0) {
        Serial.println("ARD,Restoring all settings to defaults.");
        saveDefaultSettingsToDisk();
        retrieveSettingsFromDisk();
      } else if (strcmp(commandName, "DUMPDATA") == 0) {
        Serial.println("Received command DUMPDATA");
        Serial.println("**** BEGIN FILE CONTENTS ****");
        getCurrentDataFileContents();
        Serial.println("****  END FILE CONTENTS  ****");
      } else if (strcmp(commandName, "STARTFILE") == 0) {
        Serial.print("ARD,STARTFILE not implemented yet");
      } else if (strcmp(commandName, "CLEARDATA") == 0) {
        SD.remove(dataFileName);
        Serial.print("ARD,Cleared data file");
      } else if (strcmp(commandName, "MILLIS") == 0) {
        char strMillis[16];
        snprintf(strMillis, 16, "%lu", millis());
        Serial.print("ARD,"); Serial.println(strMillis);
      } else if (strcmp(commandName, "PWRDOWN") == 0) {
        // Signal for power down. Ironically this can't really work, because if this message is received, then USB is connected, and timer isn't controlling power, so it can't power down. 
        powerDown();
      } else if (strcmp(commandName, "C") == 0) {
        if (strcmp(splitString[1], "?") == 0) {            // Check continuous measurement setting
          Serial.print("ARD,DATA_INTERVAL="); Serial.println(currentSettings.measurementInterval, DEC);
        } else {
          char *ptr;
          setContinuousMeasurements(strtol(splitString[1], &ptr, 10));
        }
      } else if (strcmp(commandName, "M") == 0) {
        Serial.println("ARD,Beginning measurement...");
        takeAndLogMeasurements();
        Serial.println("ARD,...measurement complete and logged");
      } else if (strcmp(commandName, "SSM") == 0) {
        if (strcmp(splitString[1], "?") == 0) {            // Check single shot measurement mode
          Serial.print("ARD,SSM="); Serial.println(currentSettings.singleShotMeasurementOnBootMode ? 1 : 0, DEC);
        } else {
          char * ptr;
          currentSettings.singleShotMeasurementOnBootMode = (strtol(splitString[1], &ptr, 10) == 1);
          saveCurrentSettingsToDisk();
          Serial.print("ARD,Setting SSM to "); Serial.println(currentSettings.singleShotMeasurementOnBootMode, DEC);
        }
      } else if (strcmp(commandName, "SSMDELAY") == 0) {
        char * ptr;
        int newSingleShotMeasurementDelay = strtol(splitString[1], &ptr, 10);
        if (newSingleShotMeasurementDelay) {
          currentSettings.singleShotMeasurementDelay = newSingleShotMeasurementDelay;
          saveCurrentSettingsToDisk();
          Serial.print("ARD,Setting single shot measurement delay to "); Serial.println(currentSettings.singleShotMeasurementDelay);
        } else {
          Serial.print("ARD,Error: invalid single shot measurement delay value: "); Serial.println(splitString[1]);
        }
      } else if (strcmp(commandName, "CONNECT") == 0) {
          Serial.print("ARD,CONNECT not implemented yet");
      } else if (strcmp(commandName, "TIMESTAMP") == 0) {  // Log timestamp from PC
        if (strcmp(splitString[1], "?") == 0) {
          char timestamp[64];
          getTimestamp(timestamp);
          Serial.print("ARD,TIME="); Serial.println(timestamp);
        } else if (strcmp(splitString[1], "S") == 0) {
          int splitStringCount;
          for (splitStringCount = 0; splitString[splitStringCount]; splitStringCount++) {   // Count the # of strings in splitString
          }
          Serial.println(splitStringCount);
          if (splitStringCount < 7) {
            Serial.println("ARD,Error: Not enough arguments for TIMESTAMP,S command");
          } else {
            char * ptr;
            int dateVals [6];
            for (int i = 0; i < 6; i++) {
              dateVals[i] = strtol(splitString[i+2], &ptr, 10);
              Serial.println(dateVals[i], DEC);
            }
            rtc.adjust(DateTime(dateVals[0], dateVals[1], dateVals[2], dateVals[3], dateVals[4], dateVals[5]));
            snprintf(message, maxMessageLength, "ARD,Set date: %02d/%02d/%02d %02d:%02d:%02d", dateVals[0], dateVals[1], dateVals[2], dateVals[3], dateVals[4], dateVals[5]);
            Serial.println(message);
            logEvent(message);
            message[0] = 0;
          }
        }
      } else if (strcmp(commandName, "NOTE") == 0) {
        logNote(splitString[1]);
//      } else if (strcmp(commandName, "") == 0) {
      } else if (strcmp(commandName, "EC_SLEEP") == 0) {
        Serial.println("ARD,Goodnight, EC probe");
        sleepEC();
      } else if (strcmp(commandName, "TEMP_SLEEP") == 0) {
        Serial.println("ARD,Goodnight, TEMP probe");
        sleepTEMP();
      } else if (strcmp(commandName, "ARD_SLEEP") == 0) {
        if (strcmp(splitString[1], "0") == 0) {
          Serial.println("ARD,Sleep mode off");
          currentSettings.sleepEnabled = false;
          saveCurrentSettingsToDisk();
        } else if (strcmp(splitString[1], "1") == 0) {
          Serial.println("ARD,Sleep mode on");
          currentSettings.sleepEnabled = true;
          saveCurrentSettingsToDisk();
        } else if (strcmp(splitString[1], "?") == 0) {
          Serial.print("ARD,Sleep mode: "); Serial.println((currentSettings.sleepEnabled ? "On" : "Off"));
        } else {
          Serial.print("ARD,Sleep command not recognized: "); Serial.println(splitString[1]);
        }
      } else if (strcmp(commandName, "PRINTSETTINGS") == 0) {
        Serial.println("ARD,Current settings:");
        printCurrentSettings();
      } else {
        Serial.print("ARD,Error: command not recognized: "); Serial.println(commandName);
      }
      free(splitString);
    } else {
      Serial.print("ARD,Error: target not recognized: "); Serial.println(target);
    }
    resetInput(&PCInput);
  }

  if (SIMInput.messageComplete) {             // Relay message received from SIM7000 modem
    Serial.print("SIM,"); Serial.println(SIMInput.inputString);
    Serial.flush();
    resetInput(&SIMInput);
  }

  if (TEMPInput.messageComplete) {             // Relay message received from TEMP probe
    Serial.print("TEMP,"); Serial.println(TEMPInput.inputString);
    Serial.flush();
//    if (TEMPMeasurement.waitingForData) {
////      char *ptr;
////      TEMPMeasurement.data = strtod(TEMPInput.inputString, &ptr);
//      strcpy(TEMPMeasurement.data, TEMPInput.inputString);
//      getTimestamp(TEMPMeasurement.timestamp);
//      TEMPMeasurement.waitingForData = false;
//      TEMPMeasurement.measurementAcquired = true;
//    }
    resetInput(&TEMPInput);
  }

  if (ECInput.messageComplete) {               // Relay message received from EC probe
    Serial.print("EC,"); Serial.println(ECInput.inputString);
    Serial.flush();
//    if (ECMeasurement.waitingForData) {
//      strcpy(ECMeasurement.data, ECInput.inputString);
//      getTimestamp(ECMeasurement.timestamp);
//      ECMeasurement.waitingForData = false;
//      ECMeasurement.measurementAcquired = true;
//    }
    resetInput(&ECInput);
  }

  if (currentSettings.sleepEnabled) {
    long currentTime = millis();
    if (!currentSettings.measurementInterval || ((currentTime + 8700) > nextMeasureTime) {  // Make sure the next 8 second sleep interval won't overrun the next measurement time.
      if (!PCInput.messageBegun && !ECInput.messageBegun && !TEMPInput.messageBegun && !Serial.available() && !Serial2.available() && !Serial3.available()) {  // Make sure no one is trying to talk to the Arduino before going to sleep
        /* Re-enter sleep mode. */
        goToSleep();  // Sleeps for 8 seconds or until a serial message is received from PC
      } else {
        delay(500); // There's a message incoming. Wait for rest of message to come in so we don't go to sleep between character transmissions
      }
    }
  }
}