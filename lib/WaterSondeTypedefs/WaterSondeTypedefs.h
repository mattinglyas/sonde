typedef struct settingsObject {
  bool sleepEnabled;
  bool sampleRetrieved;
  bool singleShotMeasurementOnBootMode;         // If true, the Arduino takes a single measurement on booting, then signals the low power timer chip that it's done and can be powered down again.
  unsigned long singleShotMeasurementDelay;     // Time to wait after boot to begin single shot measurement (this gives time to exit single shot mode if necessary)
  unsigned long measurementInterval;            // Period with which to take automatic measurements, ms. 0=do not take any.
  unsigned long int sampleDuration;             // Time in ms for the solenoid valve to remain open to take water sample
  unsigned char measurementFailureCount;        // Current count of consequitively failed measurements
  unsigned char sampleFailureThreshold;         // Number of consequive failures that triggers a sample
  double measurementFailureThreshold;           // uS reading above which a measurement counts toward measurementFailureCount
  unsigned int crc;                             // CRC check for this structure (unimplemented)
} settings;