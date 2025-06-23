#include <Arduino.h>
#include "BluetoothSerial.h"


/* Bluetooth enable check */
// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif
/* Bluetooth enable check */
















uint16_t rpmFinal;  // holds the calculated value of engine speed aka RPM
uint8_t speedFinal; // holds obd2 vehicle speed
uint8_t throttleFinal;  // holds throttle plate angle
String incomingData = "";  // holds the incoming data sent from the teensy serial connection 
String inputBuffer = "";  // holds bluetooth incoming transmissions
bool initComplete = false;  // whether the bluetooth obd handshake is complete or not
int protocol = 0;  // obd pid or canbus mode
BluetoothSerial SerialBT;  // bluetooth object
unsigned long timer;



void processSerial();
void sendFrame();
void printHexDump(uint8_t* data, size_t length);
void processBt();
void sendResponse(String response);
void handleCmd(String cmd);
void sendCanFrameWithoutDLC(uint16_t canId, const uint8_t* data, uint8_t len);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2 -> GPIO16, TX2 -> GPIO17

  SerialBT.begin("CANBUS-MALWARE-INJECTOR");
  //SerialBT.setPin("7406");

}

void loop() {

  processBt();
  /*
  processSerial();
  Serial.print("RPM: ");
  Serial.print(rpmFinal);
  Serial.print(" SPEED: ");
  Serial.print(speedFinal);
  Serial.print(" THROTTLE: ");
  Serial.println(throttleFinal);
  sendFrame();
  */

  if (initComplete && protocol == 1) {
    
    //rpmFinal = 1234;
    //speedFinal = 99;
    //throttleFinal = 55;
    
    
    uint8_t frame[4];
    frame[0] = (rpmFinal >> 8) & 0xFF;  // RPM high byte
    frame[1] = rpmFinal & 0xFF;         // RPM low byte
    frame[2] = speedFinal;
    frame[3] = throttleFinal;
    //frame[4] = 0x00;
    //frame[5] = 0x00;
    //frame[6] = 0x00;
    //frame[7] = 0x00;
    sendCanFrameWithoutDLC(0x123, frame, 4);


    /*
    char buffer[64];
    uint8_t data[9] = {0x60, 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};   
    for (int z = 0; z < 5; z++) {
      if (z == 0) { sprintf(buffer, "%03X", data[z]); }
      else { sprintf(buffer, "%02X", data[z]); }
      SerialBT.print(buffer);
    }
    SerialBT.print("\r");
    */



    /*
    char buffer[4]; // Enough to hold max 3 characters + null terminator
    uint8_t data[9] = {0x60, 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};

    for (int z = 0; z < 5; z++) {
      if (z == 0) {
        sprintf(buffer, "%03X", data[z]); // 3-digit hex for first byte
      } else {
        sprintf(buffer, "%02X", data[z]); // 2-digit hex for others
      }
      for (char* p = buffer; *p != '\0'; p++) {
        SerialBT.write(*p); // Write character-by-character
      }
    }
    SerialBT.write('\r');
    */







    //sendCanFrameWithoutDLC(0x123, data, 8);
    processSerial();
    //Serial.print("RPM: ");
    //Serial.print(rpmFinal);
    //Serial.print(" SPEED: ");
    //Serial.print(speedFinal);
    //Serial.print(" THROTTLE: ");
    //Serial.println(throttleFinal);
    //sendFrame();


  }
  
  delay(20);

}


void processBt() {
  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\r' || c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        handleCmd(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

void handleCmd(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  Serial.print("Received: " + cmd + " ");

  /* HANDLES OBD2 PID RESPONSES, INEFFECIENT BUT WORKS */
  if (cmd == "010C" || cmd == "0111" || cmd == "010B" || cmd == "010A" || cmd == "0105" || cmd == "015C" || cmd == "010F") {
    Serial.println("Received OBD command");

    if (cmd == "010C") {
      // Simulate 3000 RPM
      uint16_t rpm_raw = 3000 * 4; // 12000
      uint8_t A = (rpm_raw >> 8) & 0xFF;
      uint8_t B = rpm_raw & 0xFF;

      char buffer[32];
      sprintf(buffer, "41 0C %02X %02X", A, B);
      sendResponse(String(buffer));
    }
    else if (cmd == "0111") {
      // Simulate 50% throttle
      uint8_t A = 0x7F;  // ~50% throttle
      char buffer[16];
      sprintf(buffer, "41 11 %02X", A);
      sendResponse(String(buffer));
    }
    else if (cmd == "010B") {
      uint8_t A = 40;  // 40 kPa
      char buffer[16];
      sprintf(buffer, "41 0B %02X", A);
      sendResponse(String(buffer));
    }
    else if (cmd == "010A") {
      uint8_t A = 100;  // 100 × 3 = 300 kPa
      char buffer[16];
      sprintf(buffer, "41 0A %02X", A);
      sendResponse(String(buffer));
    }
    else if (cmd == "0105") {
      uint8_t A = 90 + 40;  // 130 = 0x82
      char buffer[16];
      sprintf(buffer, "41 05 %02X", A);
      sendResponse(String(buffer));
    }
    else if (cmd == "015C") {
      uint8_t A = 100 + 40;  // 140 = 0x8C
      char buffer[16];
      sprintf(buffer, "41 5C %02X", A);
      sendResponse(String(buffer));
    }
    else if (cmd == "010F") {
      uint8_t A = 35 + 40;  // 75 = 0x4B
      char buffer[16];
      sprintf(buffer, "41 0F %02X", A);
      sendResponse(String(buffer));
    }

  }
  /* THIS HANDLES THE HANDSHAKE AND AT COMMANDS, YOU JUST NEED TO RESPONSD WITH "OK\r>" */
  else {
    if (cmd == "ATZ") {
      Serial.println("[Init begin]");
      initComplete = false;
      protocol = 0;
      sendResponse("OK");
    }
    else if (cmd == "STP 33") {
      Serial.print("[Protocol: CANBUS] ");
      protocol = 1;
      sendResponse("OK");
    }
    else if (cmd == "ATSP00") {
      Serial.print("[Protocol: OBD2] ");
      protocol = 2;
      sendResponse("OK");
    }
    else if (cmd == "STM") {
      Serial.print("[Init complete] ");
      initComplete = 1;

      SerialBT.memrelease();
      SerialBT.begin();


      sendResponse("OK");
    }
    else if (cmd == "ATPC") {
      Serial.print("[Connection close] ");
      initComplete = false;
      protocol = 0;
      sendResponse("OK");
    }
    else if (cmd == "ATIGN") {
      Serial.print("[Connection close] ");
      initComplete = false;
      protocol = 0;
      sendResponse("WAKE");
    }
    else {
      sendResponse("OK");
    }
  }
}


void sendResponse(String response) {
  Serial.println("Sending " + response);
  SerialBT.print(response);
  SerialBT.print("\r>");
}

// make sure the can frame has no spaces, otherwise the app will ignore it and not parse
void sendCanFrameWithoutDLC(uint16_t canId, const uint8_t* data, uint8_t len) {
  long startTime = millis();
  timer = millis();
  char header[64];
  sprintf(header, "%d.%03d", (int)(timer/1000), (int)(timer % 1000)); Serial.print(header); Serial.print(" SENDING... ");
  



  char buffer[64];

  sprintf(buffer, "%03X", canId);
  SerialBT.print(buffer);
  //Serial.print("sent ");
  for (int i = 0; i < len; i++) {
    sprintf(buffer, "%02X", data[i]);
    SerialBT.print(buffer);
    //Serial.print(buffer);
  }
  //Serial.println();
  SerialBT.print("\r");
  timer = millis();
  //char header[64];
  sprintf(header, "%d.%03d", (int)(timer/1000), (int)(timer % 1000)); Serial.print(header); Serial.print(" SENT ");
  long elapsedTime = millis() - startTime;
  
  Serial.print(" Took: ");
  Serial.println(elapsedTime);

}

void sendFrame() {
  uint8_t frame[8];

  frame[0] = 0x60;
  frame[1] = (rpmFinal >> 8) & 0xFF;  // RPM high byte
  frame[2] = rpmFinal & 0xFF;         // RPM low byte
  frame[3] = speedFinal;
  frame[4] = throttleFinal;
  frame[5] = 0x00;
  frame[6] = 0x00;
  frame[7] = 0x00;

  printHexDump(frame, sizeof(frame));  // print before sending
  SerialBT.write(frame, sizeof(frame));
}

void printHexDump(uint8_t* data, size_t length) {
  Serial.print("Raw: ");
  for (size_t i = 0; i < length; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println( );
}

void processSerial() {
  if (Serial2.available ()) {
    incomingData = Serial2.readStringUntil('\n');
    incomingData.trim();

    //Serial.println(incomingData);

    int firstComma = incomingData.indexOf(',');
    int secondComma = incomingData.indexOf(',', firstComma + 1);

    rpmFinal = incomingData.substring(0, firstComma).toInt();
    speedFinal = incomingData.substring(firstComma + 1, secondComma).toInt();
    throttleFinal = incomingData.substring(secondComma + 1).toInt();
  }
}