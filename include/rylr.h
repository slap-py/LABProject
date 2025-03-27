#ifndef RYLR_H
#define RYLR_H
#include <Arduino.h>
#include <array>

class rylr998{
    public:
        //public variables and functions
        void begin(Stream& serial, uint32_t baud = 115200);
        
        struct ReceivedMessage{
            String content;
            int length;
            int sender;
            int rssi;
            int snr;
        }

        void update();
        void sendMessage(String message, String target); //240 bytes max message, target is int (id)
        void reset(); //pull reset low
        void factoryReset(); //AT+FACTORY
        void setMode(String mode); //AT+MODE
        void setFrequency(float frequency); //AT+BAND in MHz
        //TODO: bool setParameter(); //AT+PARAMETER
        void setDeviceAddress(int address); //AT+ADDRESS
        void setNetworkID(int networkID); //AT+NETWORK
        void setEncryptionKey(String cpin); //AT+CPIN (must be 8 chars 00000000 thru FFFFFFFF)
        void setTxStrength(int strength); //AT+CRFOP

        int getMode(); //AT+MODE? 1,2 or 3
        float getFrequency(); //AT+BAND? in MHz
        std::array<String,4> getParameter();
        int getDeviceAddress(); //AT+ADDRESS?
        int getNetworkID(); //AT+NETWORK?
        String getEncryptionKey(); // AT+CPIN?
        int getTxStrength(); //AT+CRFOP?

        String getFirmwareVersion(); //AT+VER?
        String getSerialNumber(); //AT+UID?

    private:
        Stream* _serial;
        int mode;
        float frequency;
        //ADD PARAMETER STUFF

        int deviceAddress;
        int networkID;
        String encryptionKey;
        int txStrength;
        String sendATCommand(String command, unsigned long timeout = 1000);

        ReceivedMessage parseRCVStatement(String line);

        String _lastMessage;
        String _lastSender;
        String _lastRSSI;
        String _lastSNR;

        







    
    private:
        //private variables and functions
        String _lastResponse;
}
#endif