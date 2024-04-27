
/**
 * Developer: Seyi R. Afolayan
 * Team: DekeShot 04/22/24
 * Work: Modified ZigBee functionality (JHU hockey tracking)
 **/
/****************************************************************************BEGIN CODE****************************************************************************/
// Import necessary library
#include <HardwareSerial.h>
#include <Arduino.h>

class XbeeCommunicator {
private:
    HardwareSerial& xbeeSerial = Serial1; // Reference to the serial port used by the XBee

public:
    // Constructor
    XbeeCommunicator(HardwareSerial& serial) : xbeeSerial(serial) {}

    // Initialize the Xbee communication
    void setupXbeeComm() {
        // Begin serial communications for both regular debugging and the Zigbee
        Serial.begin(115200);
        xbeeSerial.begin(115200);
        Serial.println("Waiting for match information...");
    }

    // Handle the communication in the main loop
    void handleXbeeComm() {
        // Send and receive data
        sendQuery(); // SEND the query to the Zigbee
        receiveData(); // RECEIVE the data from the Zigbee, if there is something to receive
    }

    // Send query to the XBee module
    void sendQuery() {
        static size_t prevQueryTime{millis()};
        if ((millis() - prevQueryTime) > 1000) {
            xbeeSerial.print('?');
            prevQueryTime = millis(); // Reset the time for the next query
        }
    }

    // Receive data from XBee module
    void receiveData() {
        if (xbeeSerial.available()) {
            String incomingXbeeData = xbeeSerial.readStringUntil('\n');
            incomingXbeeData.trim(); // Trim the whitespace
            if (incomingXbeeData.length() > 0)
                parseIncomingData(incomingXbeeData);
        }
    }

    // Parse the incoming data
    void parseIncomingData(const String& data) {
        // Check the first character of the data
        if (data.charAt(0) == '?') {
            Serial.println("No data. It has been more than 1 second.");
            return;
        } else if (data.charAt(0) == '/') {
            Serial.println("The checksums are not matching");
            return;
        }

        // Extracting parts of the data
        int indexFirstComma = data.indexOf(',');
        int indexSecondComma = data.indexOf(',', indexFirstComma + 1);
        int indexThirdComma = data.indexOf(',', indexSecondComma + 1);

        String matchByte = data.substring(0, indexFirstComma);
        String matchTime = data.substring(indexFirstComma + 1, indexSecondComma);
        String posX = data.substring(indexSecondComma + 1, indexThirdComma);
        String posY = data.substring(indexThirdComma + 1);

        if (posX == "---" || posY == "---") {
            Serial.println("Jhu Hockey Robot Location is not available.");
            return;
        }

        // Log data to serial monitor
        Serial.print("Match Status: "); Serial.println(matchByte == "1" ? "Ongoing" : "Not Active");
        Serial.print("Match Time: "); Serial.println(matchTime);
        Serial.print("Position X: "); Serial.println(posX);
        Serial.print("Position Y: "); Serial.println(posY);
    }
};
