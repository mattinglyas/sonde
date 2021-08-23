// Class to handle incomming data from serial devices

class cInput {
    public:
        char inputString[64];
        unsigned char inputStringLoc;
        bool messageComplete;
        bool messageBegun;

        cInput(Stream &port);

        void update();
        void reset();
        bool isComplete();
        void enableEcho();
        void disableEcho();

    private:
        Stream *m_port;
        bool m_echo;

        void appendChar(char c);
};