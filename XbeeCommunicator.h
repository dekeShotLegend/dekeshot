
/**
 * Developer: Seyi R. Afolayan
 * Team: DekeShot 04/22/24
 * Work: Modified ZigBee functionality (JHU hockey tracking)
 **/
/****************************************************************************BEGIN CODE****************************************************************************/
// Structure for holding our data
#include <Arduino.h>
struct RobotData {
    int matchStatus;
    int matchTime;
    float posX;
    float posY;
};
RobotData robotData; 

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
        xbeeSerial.print('?');
    }

    // Receive data from XBee module
    void receiveData() {
        if (xbeeSerial.available()) {
            String incomingXbeeData = xbeeSerial.readStringUntil('\n');
            incomingXbeeData.trim(); // Trim the whitespace
            if (incomingXbeeData.length() > 0)
                parseIncomingData(incomingXbeeData);
        }
        else{
          Serial.println("no data found");
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

        robotData.matchStatus = data.substring(0, indexFirstComma).toInt();
        robotData.matchTime = data.substring(indexFirstComma + 1, indexSecondComma).toInt();
        String posXstr = data.substring(indexSecondComma + 1, indexThirdComma);
        String posYstr = data.substring(indexThirdComma + 1);

        if (posXstr == "---" || posYstr == "---") {
            Serial.println("Jhu Hockey Robot Location is not available.");
            return;
        }
        robotData.posX = posXstr.toFloat(); 
        robotData.posY = posYstr.toFloat();
        Serial.println(robotData.matchStatus);
        Serial.println(robotData.matchTime);
        Serial.println(robotData.posX);
        Serial.println(robotData.posY);
    }

    void updateRobotPosition(float x, float y){
      x = robotData.posX; 
      y = robotData.posY;
    }
    int available() {
      return xbeeSerial.available();
    }
};
