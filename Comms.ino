/**
* Developer: Team DekeShot 04/22/24
* Work: Modified ZigBee functionality (JHU hockey tracking)
**/
/***************************************************************************BEGIN CODE***********************************************************************/

// Referencing the Serial1 object 
HardwareSerial  &xbeeSerial = Serial1; 

/***************************************************************************MAIN FUNCTION********************************************************************/
void setup(){
  // Begin serial communications for both regular debugging and the Zigbee
  Serial.begin(115200); 
  xbeeSerial.begin(115200); 
  Serial.println("Waiting for match information...");
}

void loop(){
  /**
  The loop does two things - SEND AND RECIEVE DATA
  SEND DATA
  We query the Xbee Module by sending the character '?'
  */
  static size_t prevQueryTime{millis()}; 
  if ((millis() - prevQueryTime) > 1000) {
    xbeeSerial.write('?'); 
    prevQueryTime = millis(); // Update the query time which effectively resets the time for the next query. 
  }

  // RECEIVE DATA
  // Let's see if we got data coming in 
  if (xbeeSerial.available()){
    String incomingXbeeData = xbeeSerial.readStringUntil('\n'); 
    if (!incomingXbeeData.isEmpty())
    parseIncomingData(incomingXbeeData.trim());
  }
}

/***************************************************************************DATA PARSING********************************************************************/
// Based on the Summary of Expected Outputs, we parse in the incoming data the following way: 
void parseIncomingData(const String& data) {
  // Checking the first character of the data: 
  if (data.charAt(0) == '?') {
    Serial.println("No data. It is has been more than 1 second."); 
    return;
  }
  else if (data.charAt(0) == '/') {
    Serial.println("The checksums are not matching");
    return;
  }

  // Finding the indices of the first three commas to serve as reference point for extracting different parts of the match information 
  int indexFirstComma = data.indexOf(','); 
  int indexSecondComma = data.indexOf(',', indexFirstComma + 1); 
  int indexThirdComma = data.indexOf(',', indexSecondComma + 1);

  // Extracting the match byte, match time, and robot position in the jh hockey arena
  String matchByte = data.substring(0, indexFirstComma);
  String matchTime = data.substring(indexFirstComma + 1, indexSecondComma);
  String posX = data.substring(indexSecondComma + 1, indexThirdComma); 
  string posY = data.substring(indexThirdComma + 1);
  
  // Check if the 2D position of the robot is available: 
  if (posX == "___" || posY == "___"){
    Serial.println("Jhu Hockey Robot Location is not available."); 
    return;
  }

  // Otherwise, will log this to serial monitor 
  Serial.print("Match Status: "); Serial.println(matchByte == "1" ? "Ongoing" : "Not Active");
  Serial.print("Match Time: "); Serial.println(matchTime); 
  Serial.print("Position X: "); Serial.println(posX);
  Serial.print("Position Y: "); Serial.println(posY);
}
