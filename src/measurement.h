// Class to hold measurement data and track the progress of acquiring it
class Measurement {
    public:
        char data[32];
        char timestamp[32];
        char voltage[32];
        char lastPowerOff[8];

        Measurement();
        void reset();
};
