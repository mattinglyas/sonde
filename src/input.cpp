#include <Arduino.h>
#include "input.h"

cInput::cInput(Stream &port) {
    m_port = &port;
}

void cInput::update() {
    char inChar;

    while(m_port->available()) {
        inChar = (char)m_port->read();                      // Read 1 char of serial data

        if(m_echo)                                          // If echo enabled, send input character back to sender
            m_port->write(inChar);                          // This is nice for interactive interfaces

        if(!messageBegun) {
            if(isPrintable(inChar)) {                    // Await start of message...
                messageBegun = true;
                inputString[inputStringLoc++] = inChar;
                continue;
            }
        } else {
            // Check if character is a line end character, or end of buffer...
            if(inChar == '\n' || inChar == '\r' || inputStringLoc >= (sizeof(inputString) - 1)) {
                messageComplete = true;                     // Set flag that message is complete
                messageBegun = false;
                inputString[inputStringLoc] = 0;            // Null terminate string
                Serial.print("INPUT ["); Serial.print(inputString); Serial.println("]");
            } else {
                inputString[inputStringLoc++] = inChar;     // Append new character to the end of inputString
            }
        }
    }
}

void cInput::appendChar(char c) {
    inputString[inputStringLoc++] = c;
}

void cInput::reset() {
  messageComplete = false;
  messageBegun = false;
  inputStringLoc = 0;
  inputString[0] = 0;

  /* NOTE: This shouldn't really be necessary!
   * Any code that breaks because the buffer hasn't been cleared is
   * probably broken in the first place.
   */
  memset(inputString, 0, sizeof(inputString));
}

bool cInput::isComplete() {
    return messageComplete;
}

void cInput::enableEcho() {
    m_echo = true;
}

void cInput::disableEcho() {
    m_echo = false;
}