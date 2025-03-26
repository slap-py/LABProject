#include "rylr.h"


void rylr998::begin(Stream& serial, uint32_t baud){
    _serial = &serial;
    _serial->begin(baud);
}

void rylr998::sendCommand(String message, String target){
    int length = message.length();
    String command = "AT+SEND="+target+","+String(length)+","+message;
    _serial->println(command);
}

void rylr998::update() {
    if (_serial && _serial->available()) {
        String line = _serial->readStringUntil('\n');
        line.trim();

        if (line.startsWith("+RCV=")) {
            _lastResponse = line;
            rylr998::parseRCVStatement(line);
        } else if (line.indexOf("+OK") != -1 || line.indexOf("ERROR") != -1) {
            _lastResponse = line; // store response if needed
        }
    }
}

ReceivedMessage parseRCVStatement(String line){
    //address,length,data,rssi,snr
    
}