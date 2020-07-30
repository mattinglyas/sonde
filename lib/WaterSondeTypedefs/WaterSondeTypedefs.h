typedef struct measurementObject {                                        // Structure to hold measurement data and track the progress of acquiring it
  char data [32];
  char timestamp [32];
  char voltage [32];
  char lastPowerOff [8];
} measurement;

typedef struct serialInputObject {                                        // Structure to hold information about and from serial connection
  char  inputString [32];
  int   inputStringLoc;
  bool  messageComplete;
  bool  messageBegun;
} serialInput;

typedef struct settingsObject {
  bool sleepEnabled;
  bool sampleRetrieved;
  bool singleShotMeasurementOnBootMode;                        // If true, the Arduino takes a single measurement on booting, then signals the low power timer chip that it's done and can be powered down again.
  unsigned long singleShotMeasurementDelay;                // Time to wait after boot to begin single shot measurement (this gives time to exit single shot mode if necessary)
  unsigned long measurementInterval;               // Period with which to take automatic measurements, ms. 0=do not take any.
  int sampleDuration;                                  // Time in ms for the solenoid valve to remain open to take water sample
} settings;