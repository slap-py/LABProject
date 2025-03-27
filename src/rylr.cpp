#include "rylr.h"
#include <sstream>
#include <string>


String split(String str, char delimiter, int index) {
    int startIndex = 0;
    int endIndex = str.indexOf(delimiter);
    int currentIndex = 0;
    
    while (endIndex >= 0) {
      if (currentIndex == index) {
        return str.substring(startIndex, endIndex);
      }
      startIndex = endIndex + 1;
      endIndex = str.indexOf(delimiter, startIndex);
      currentIndex++;
    }
    if (currentIndex == index) {
      return str.substring(startIndex);
    }
    return "";
  }


void rylr998::begin(Stream& serial, uint32_t baud){
    _serial = &serial;
    _serial->begin(baud);
}

void rylr998::sendMessage(String message, String target){
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
            _lastResponse = line;

        }else{
            //throw some kind of error
        }
    }
}

ReceivedMessage parseRCVStatement(String line){
    //address,length,data,rssi,snr
    ReceivedMessage parsed;
    parsed.sender = int(split(line,',',0));
    parsed.length = int(split(line,',',1));
    parsed.data = split(line,',',2);
    parsed.rssi = int(split(line,',',3))
    parsed.snr = int(split(line,',',4));
    return parsed;
}